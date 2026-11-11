#include "gimbal_cmd.h"
#include <stdlib.h>
#include <string.h>
#include "algorithm.h"
#include "arm.h"
#include "arm_auto.h"
#include "common.h"

#include "rc.h"
#include "robot_state.h"

/* 读取遥控器在线状态。 */
static uint8_t Rc_Online(void) { return (uint8_t)(!is_Offline(rc.monitor)); } // NOLINT(readability-identifier-naming)

static void Stop_Mode_Update(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Passive_Mode_Update(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Run_Mode_Update(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Set_Ctrl_Default(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Auto_Mode_Update(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Plan_Running_Check(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Ctrl_Src_Calc(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Motor_Angle_Limit(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Ctrl_Post_Process(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)
static void Arm_Ref_Set(Gimbal_cmd* obj); // NOLINT(readability-identifier-naming)

static void joint_src_follow(Gimbal_cmd* obj);
static Joint_motion_state joint_feedback_reference(const Gimbal_cmd* obj, int joint);
static void joint_store_reference(Gimbal_cmd* obj, int joint, Joint_motion_state state);

/* 创建命令状态、临时规划组和关节轨迹实例。 */
Gimbal_cmd* Gimbal_Cmd_Create(Arm* arm) {

    /* 机械臂或电机控制对象无效时拒绝创建。 */
    if (arm == NULL || arm->gimbal == NULL || arm->gimbal->profile == NULL) return NULL;

    Gimbal_cmd* obj = (Gimbal_cmd*)malloc(sizeof(Gimbal_cmd));
    /* 命令对象分配失败时返回空指针。 */
    if (obj == NULL) return NULL;
    memset(obj, 0, sizeof(Gimbal_cmd));
    obj->arm = arm;
    obj->gimbal = arm->gimbal;
    obj->temp_group = Plan_Group_Temp_Create(ARM_JOINT_NUM);
    /* 规划组分配失败时释放命令对象。 */
    if (obj->temp_group == NULL) {
        free(obj);
        return NULL;
    }

    if (!Joint_Post_Process_Init(obj)) {
        free((void*)obj->temp_group);
        free(obj);
        return NULL;
    }

    return obj;
}

/* 更新输入源、自动动作、限位和控制参考。 */
void Gimbal_Cmd_Update(Gimbal_cmd* obj) {
    /* 命令对象无效时结束本次更新。 */
    if (obj == NULL || obj->gimbal == NULL) return;

    /* 遥控器上线稳定后优先选择遥控工作模式。 */
    uint8_t rc_stable = HAL_GetTick() > 1000;
    obj->mode = Rc_Online() && rc_stable ? ((rc.sw_a == 1) ? arm_run : arm_stop)
                                         : obj->arm->arm_mode;


    /* 清空本周期速度、加速度和夹爪力矩默认值。 */
    Set_Ctrl_Default(obj);

    /* 按工作模式选择控制源更新流程。 */
    switch (obj->mode) {
    /* 运行模式读取遥控器或协议目标。 */
    case arm_run:
        Run_Mode_Update(obj);
        break;
    /* 阻尼模式使参考值跟随反馈。 */
    case arm_damping:
    /* 重力模式使参考值跟随反馈。 */
    case arm_only_gravity_comp:
        Passive_Mode_Update(obj);
        break;
    /* 停止模式取消自动动作并清空前馈。 */
    case arm_stop:
    /* 异常模式按停止流程处理。 */
    default:
        Stop_Mode_Update(obj);
        break;
    }

    /* 推进自动动作并覆盖相应控制源。 */
    Auto_Mode_Update(obj);

    /* 更新自动轨迹是否接管运动参考。 */
    Plan_Running_Check(obj);

    /* 从当前输入源生成关节求解目标。 */
    Ctrl_Src_Calc(obj);

    /* 按机型关节和夹爪范围限制目标。 */
    Motor_Angle_Limit(obj);

    /* 更新轨迹参考并衔接控制源状态。 */
    Ctrl_Post_Process(obj);

    /* 选择各轴参考来源并发布运动导数。 */
    Arm_Ref_Set(obj);

    /* 夹爪归零期间发布搜索或回退参考值。 */
    if (obj->initialization.home_active) {
        obj->gimbal->arm_control_data.gripper_angle = Arm_Gripper_From_Motor(
            obj->gimbal->profile, obj->initialization.home_target_deg);
        obj->gimbal->arm_control_data.gripper_torque = obj->initialization.home_torque;
    }
    obj->last_auto_mode = obj->auto_mode;

}

/* 取消自动动作并使关节和夹爪目标跟随反馈。 */
static void Stop_Mode_Update(Gimbal_cmd* obj) {

    obj->mode = arm_stop;
    obj->gimbal->arm_control_data.mode = arm_stop;

    obj->auto_mode = arm_auto_none;
    obj->auto_step = 0;

    joint_src_follow(obj);

    /* 停止模式使夹爪目标跟随反馈并清零力矩。 */
    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 被动模式跟随反馈并清空运动前馈。 */
static void Passive_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = obj->mode;

    joint_src_follow(obj);

    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 运行模式选择遥控器或协议控制目标。 */
static void Run_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = arm_run;

    /* 遥控器在线时使用摇杆增量控制。 */
    if (Rc_Online()) {

        /* 开关处于保持位置时维持当前目标。 */
        if (rc.sw_a == 0) {
            return;
        }

        const Arm_profile* profile = obj->gimbal->profile;
        const float channels[] = {rc.rx, rc.ry, rc.lx};
        const double increment = (double)profile->rc_speed_deg_s / ARM_CONTROL_FREQUENCY_HZ;
        for (int channel = 0; channel < 3; ++channel) {
            const int joint = profile->rc_joint[channel];
            if (joint >= 0 && joint < ARM_JOINT_NUM && fabsf(channels[channel]) > 0.1)
                obj->ctrl_src.joint_src.joint_angle[joint] += fsgn(channels[channel]) * increment;
        }
        return;
    }

    /* 读取当前机械臂的协议控制目标。 */
    const Robot_ctrl_cmd* cmd = Robot_State_Ctrl(obj->arm);
    /* 协议目标尚未有效时保持控制源。 */
    if (cmd == NULL || !cmd->valid) return;

    /* 逐轴加载协议位置目标。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->ctrl_src.joint_src.joint_angle[i] = cmd->joint_angle[i];
    }
    obj->gimbal->arm_control_data.gripper_angle = cmd->gripper_angle;
}

/* 按自动模式分派动作更新。 */
static void Auto_Mode_Update(Gimbal_cmd* obj) {
    /* 选择本周期执行的自动动作。 */
    switch (obj->auto_mode) {
    /* 复位模式推进关节复位步骤。 */
    case arm_auto_reset:
        Arm_Reset(obj);
        break;
    /* 空闲模式保持当前控制源。 */
    case arm_auto_none:
    /* 枚举边界值保持当前控制源。 */
    case ARM_AUTO_MODE_NUM:
    /* 其余模式保持当前控制源。 */
    default:
        break;
    }
}

/* 根据自动动作步骤更新规划运行标志。 */
static void Plan_Running_Check(Gimbal_cmd* obj) {
    // 检查是否为一键模式以及tempgoup是否正在运行
    obj->plan_running =
        (obj->auto_mode != arm_auto_none && obj->temp_group[0]->status == move_plan_run);
}

/* 清空本周期速度、加速度和夹爪力矩。 */
static void Set_Ctrl_Default(Gimbal_cmd* obj) {
    /* 逐轴清除运动前馈默认值。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->gimbal->arm_control_data.joint_vel[i] = 0;
        obj->gimbal->arm_control_data.joint_acc[i] = 0;
    }
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 将输入关节目标复制为限位前参考值。 */
static void Ctrl_Src_Calc(Gimbal_cmd* obj) {

    memcpy(obj->ctrl_src.joint_angle_solved, obj->ctrl_src.joint_src.joint_angle,
        sizeof(obj->ctrl_src.joint_angle_solved));

    /* 被动和停止模式使用当前反馈角。 */
    if (obj->mode != arm_run) {
        /* 逐轴使求解目标跟随反馈位置。 */
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            obj->ctrl_src.joint_angle_solved[i] =
                obj->gimbal->arm_feedback_data.joint_angle[i];
        }
    }
}

/* 对位置有效的关节和夹爪执行软限位。 */
static void Motor_Angle_Limit(Gimbal_cmd* obj) {

    /* 逐轴限制求解目标到机型允许范围。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        /* 位置尚未有效的关节保持当前求解目标。 */
        if (!obj->gimbal->arm_feedback_data.motor_inited[i]) continue;
        obj->ctrl_src.joint_angle_solved[i] = fclip(obj->ctrl_src.joint_angle_solved[i],
            obj->gimbal->profile->joints[i].minimum_deg, obj->gimbal->profile->joints[i].maximum_deg);
    }

    /* 将限位结果同步回输入目标。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->ctrl_src.joint_src.joint_angle[i] = obj->ctrl_src.joint_angle_solved[i];
    }

    /* 夹爪初始化完成后限制协议目标范围。 */
    if (obj->gimbal->arm_feedback_data.motor_inited[GRIPPER])
        obj->gimbal->arm_control_data.gripper_angle =
            fclip(obj->gimbal->arm_control_data.gripper_angle, obj->gimbal->profile->gripper.minimum,
                obj->gimbal->profile->gripper.maximum);
}

static Joint_motion_state joint_feedback_reference(const Gimbal_cmd* obj, int joint) {
    return (Joint_motion_state){obj->gimbal->arm_feedback_data.joint_angle[joint], 0, 0};
}

static void joint_store_reference(Gimbal_cmd* obj, int joint, Joint_motion_state state) {
    obj->post_processed_joint_angle[joint] = state.position;
    obj->post_processed_joint_vel[joint] = state.velocity;
    obj->post_processed_joint_acc[joint] = state.acceleration;
}

/* 选择参考来源、推进轨迹、校验结果并同步算法状态。 */
static void Ctrl_Post_Process(Gimbal_cmd* obj) {
    const struct Joint_post_process_ops* ops = obj->post_process;
    const uint8_t running = obj->mode == arm_run;
    const uint8_t use_algorithm = running && !obj->plan_running;

    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        Joint_post_process_ref* ref = &obj->joint_post_ref[i];
        if (use_algorithm) {
            if (!ref->initialized) {
                ref->state = joint_feedback_reference(obj, i);
                ref->initialized = 1;
                ops->reset(ref);
            }
            ref->status = ops->step(ref, &obj->gimbal->profile->joints[i],
                obj->ctrl_src.joint_angle_solved[i]);
            if (ref->status < 0) goto reference_failed;
        } else if (running) {
            ref->state = (Joint_motion_state){obj->ctrl_src.joint_angle_solved[i],
                obj->plan_joint_vel[i] * obj->gimbal->profile->joints[i].plan_velocity_scale,
                obj->plan_joint_acc[i]};
        } else {
            ref->state = joint_feedback_reference(obj, i);
        }

        if (!joint_reference_is_finite(ref->state)) {
            ref->status = JOINT_REFERENCE_INVALID;
            goto reference_failed;
        }
        if (!use_algorithm) {
            /* 使普通位置轨迹从当前发布的参考状态开始。 */
            ref->initialized = 1;
            ops->reset(ref);
        }
        joint_store_reference(obj, i, ref->state);
    }
    return;

reference_failed:
    /* 整臂回到反馈位置并进入阻尼，所有轴清零运动导数。 */
    obj->mode = arm_damping;
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        Joint_post_process_ref* ref = &obj->joint_post_ref[i];
        ref->state = joint_feedback_reference(obj, i);
        ref->initialized = 0;
        ops->reset(ref);
        joint_store_reference(obj, i, ref->state);
    }
}

/* 发布后处理阶段生成的关节参考。 */
static void Arm_Ref_Set(Gimbal_cmd* obj) {
    Arm_control_data* control = &obj->gimbal->arm_control_data;
    control->mode = obj->mode;
    memcpy(control->joint_angle, obj->post_processed_joint_angle, sizeof(control->joint_angle));
    memcpy(control->joint_vel, obj->post_processed_joint_vel, sizeof(control->joint_vel));
    memcpy(control->joint_acc, obj->post_processed_joint_acc, sizeof(control->joint_acc));
}

/* 使输入关节目标跟随反馈位置。 */
static void joint_src_follow(Gimbal_cmd* obj) {

    memcpy(obj->ctrl_src.joint_src.joint_angle, obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->ctrl_src.joint_src.joint_angle));
}
