#include "gimbal_cmd.h"
#include <stdlib.h>
#include <string.h>
#include "arm.h"
#include "arm_auto.h"
#include "common.h"

#include "rc.h"
#include "robot_state.h"

#define GIMBAL_CMD_PERIOD_S ARM_CONTROL_PERIOD_S

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
static void joint_angle_td_follow(Gimbal_cmd* obj, float ref[ARM_JOINT_NUM]);

/* 创建命令状态、临时规划组和关节跟踪器。 */
Gimbal_cmd* Gimbal_Cmd_Create(Arm* arm) {

    /* 机械臂或电机控制对象无效时拒绝创建。 */
    if (arm == NULL || arm->gimbal == NULL) return NULL;

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

    /* 按各关节配置初始化跟踪微分器。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        /* 配置关节跟踪速度和控制采样周期。 */
        TD_Config td_config = {
            .r = obj->gimbal->profile->joints[i].td_r,
            .h = GIMBAL_CMD_PERIOD_S,
            .h0 = GIMBAL_CMD_PERIOD_S * 2,
            .J = 0,
        };
        TD_Init(&obj->joint_angle_td[i], &td_config);
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

    /* 推进目标跟踪并衔接自动轨迹状态。 */
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

    obj->gimbal->arm_control_data.mode = arm_stop;

    obj->auto_mode = arm_auto_none;
    obj->auto_step = 0;

    /* 使停止参考位置跟随反馈并清空运动导数。 */
    memcpy(obj->gimbal->arm_control_data.joint_angle,
        obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->gimbal->arm_feedback_data.joint_angle));
    memset(obj->gimbal->arm_control_data.joint_vel, 0,
        sizeof(obj->gimbal->arm_control_data.joint_vel));
    memset(obj->gimbal->arm_control_data.joint_acc, 0,
        sizeof(obj->gimbal->arm_control_data.joint_acc));

    joint_src_follow(obj);

    joint_angle_td_follow(obj, obj->gimbal->arm_feedback_data.joint_angle);

    /* 停止模式使夹爪目标跟随反馈并清零力矩。 */
    obj->gimbal->arm_control_data.gripper_angle =
        obj->gimbal->arm_feedback_data.gripper_angle;
    obj->gimbal->arm_control_data.gripper_torque = 0;
}

/* 被动模式跟随反馈并清空运动前馈。 */
static void Passive_Mode_Update(Gimbal_cmd* obj) {
    obj->gimbal->arm_control_data.mode = obj->mode;

    memcpy(obj->gimbal->arm_control_data.joint_angle,
        obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->gimbal->arm_feedback_data.joint_angle));
    memset(obj->gimbal->arm_control_data.joint_vel, 0,
        sizeof(obj->gimbal->arm_control_data.joint_vel));
    memset(obj->gimbal->arm_control_data.joint_acc, 0,
        sizeof(obj->gimbal->arm_control_data.joint_acc));

    joint_src_follow(obj);
    joint_angle_td_follow(obj, obj->gimbal->arm_feedback_data.joint_angle);

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

        const float channels[] = {rc.rx, rc.ry, rc.lx};
        /* 按机型映射将摇杆输入分配给关节。 */
        for (int channel = 0; channel < 3; ++channel) {
            const int joint = obj->gimbal->profile->rc_joint[channel];
            /* 有效通道超出死区时累加关节目标。 */
            if (joint >= 0 && joint < ARM_JOINT_NUM && fabsf(channels[channel]) > 0.1f)
                obj->ctrl_src.joint_src.joint_angle[joint] += fsgn(channels[channel])
                    * obj->gimbal->profile->rc_speed_deg_s*GIMBAL_CMD_PERIOD_S;
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
    obj->plan_running = obj->auto_mode == arm_auto_reset && obj->auto_step > 0;
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

/* 更新关节跟踪器并同步自动规划状态。 */
static void Ctrl_Post_Process(Gimbal_cmd* obj) {

    /* 逐轴推进跟踪微分器。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {

        obj->joint_angle_td[i].ref = obj->ctrl_src.joint_angle_solved[i];
        TDFunction_independent(&obj->joint_angle_td[i]);
    }

    /* 自动轨迹运行时使跟踪器跟随规划位置。 */
    if (obj->plan_running) {
        joint_angle_td_follow(obj, obj->ctrl_src.joint_angle_solved);
    }
}

/* 更新单关节位置命令的连续运动参考。 */
static void Joint_Quintic_Ref_Update(Gimbal_cmd* obj, int joint) { // NOLINT(readability-identifier-naming)
    Joint_quintic_ref* ref = &obj->joint_quintic_ref[joint];
    const float target = obj->ctrl_src.joint_angle_solved[joint];
    /* 首次采样以当前反馈建立参考初值。 */
    if (!ref->initialized) {
        ref->position = obj->gimbal->arm_feedback_data.joint_angle[joint];
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
        ref->initialized = 1;
    }

    /* 目标变化或轨迹未激活时重建五次曲线。 */
    if (!ref->active || target != ref->target) {
        /* 以当前参考导数和新目标构造曲线边界。 */
        Quintic_spline_config config = {
            .x0 = 0.0f,
            .y0 = ref->position,
            .dy0 = ref->velocity,
            .ddy0 = ref->acc,
            .x1 = obj->gimbal->profile->quintic_duration_s,
            .y1 = target,
            .dy1 = 0.0f,
            .ddy1 = 0.0f,
            .xmin = 0.0f,
            .xmax = obj->gimbal->profile->quintic_duration_s,
        };
        Quintic_Spline_Reinit(&ref->curve, &config);
        ref->target = target;
        ref->elapsed = 0.0f;
        ref->active = 1;
    }

    /* 按命令控制周期推进曲线时间。 */
    ref->elapsed += GIMBAL_CMD_PERIOD_S;
    /* 到达轨迹终点后保持目标并清零导数。 */
    if (ref->elapsed >= obj->gimbal->profile->quintic_duration_s) {
        ref->elapsed = obj->gimbal->profile->quintic_duration_s;
        ref->position = ref->target;
        ref->velocity = 0.0f;
        ref->acc = 0.0f;
    } else { /* 在轨迹区间内计算位置及一、二阶导数。 */
        ref->position = Quintic_Spline_Calc_y(&ref->curve, ref->elapsed);
        ref->velocity = Quintic_Spline_Calc_dy(&ref->curve, ref->elapsed);
        ref->acc = Quintic_Spline_Calc_ddy(&ref->curve, ref->elapsed);
    }
}

/* 按控制来源生成位置、速度和加速度参考。 */
static void Arm_Ref_Set(Gimbal_cmd* obj) {
    Arm_control_data* control = &obj->gimbal->arm_control_data;
    const Robot_ctrl_cmd* cmd = Robot_State_Ctrl(obj->arm);
    const uint8_t use_external_velocity =
        (obj->mode == arm_run && !Rc_Online() && cmd != NULL && cmd->valid && cmd->has_vel);

    /* 逐轴选择参考来源并发布控制输出。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        float angle;
        float velocity;
        float acc;

        /* 自动规划运行时使用规划位置和导数。 */
        if (obj->plan_running) {
            angle = obj->ctrl_src.joint_angle_solved[i];
            velocity = obj->plan_joint_vel[i] * obj->gimbal->profile->joints[i].plan_velocity_scale;
            acc = obj->plan_joint_acc[i];
        } else if (use_external_velocity) { /* 带速度协议使用跟踪位置和协议速度。 */
            angle = obj->joint_angle_td[i].v1;
            velocity = cmd->joint_vel[i];
            acc = 0.0f;
        } else if (obj->mode == arm_run) { /* 普通位置控制使用连续曲线参考。 */
            Joint_Quintic_Ref_Update(obj, i);
            angle = obj->joint_quintic_ref[i].position;
            velocity = obj->joint_quintic_ref[i].velocity;
            acc = obj->joint_quintic_ref[i].acc;
        } else { /* 被动和停止模式使用跟踪器输出。 */
            angle = obj->joint_angle_td[i].v1;
            velocity = obj->joint_angle_td[i].v2;
            acc = obj->joint_angle_td[i].v3;
        }

        /* 被动和停止模式清零参考加速度。 */
        if (obj->mode != arm_run) acc = 0.0f;
        /* 参考值异常时切入阻尼并清除轨迹状态。 */
        if (!isfinite(angle) || !isfinite(velocity) || !isfinite(acc)) {
            obj->mode = arm_damping;
            control->mode = obj->mode;
            /* 逐轴恢复反馈位置并清零运动参考。 */
            for (int j = 0; j < ARM_JOINT_NUM; j++) {
                control->joint_angle[j] = obj->gimbal->arm_feedback_data.joint_angle[j];
                control->joint_vel[j] = 0.0f;
                control->joint_acc[j] = 0.0f;
                obj->joint_quintic_ref[j].initialized = 0;
                obj->joint_quintic_ref[j].active = 0;
            }
            return;
        }

        /* 将本轴选定的位置、速度和加速度写入执行目标。 */
        control->joint_angle[i] = angle;
        control->joint_vel[i] = velocity;
        control->joint_acc[i] = acc;
        /* 同步其他控制来源的状态以衔接位置轨迹。 */
        if (obj->mode != arm_run || obj->plan_running || use_external_velocity) {
            Joint_quintic_ref* ref = &obj->joint_quintic_ref[i];
            /* 被动和停止模式以反馈重置参考状态。 */
            if (obj->mode != arm_run) {
                ref->position = obj->gimbal->arm_feedback_data.joint_angle[i];
                ref->velocity = 0.0f;
                ref->acc = 0.0f;
            } else { /* 运行模式以实际发布参考建立衔接状态。 */
                ref->position = angle;
                ref->velocity = velocity;
                ref->acc = acc;
            }
            ref->initialized = 1;
            ref->active = 0;
        }
    }
}

/* 使输入关节目标跟随反馈位置。 */
static void joint_src_follow(Gimbal_cmd* obj) {

    memcpy(obj->ctrl_src.joint_src.joint_angle, obj->gimbal->arm_feedback_data.joint_angle,
        sizeof(obj->ctrl_src.joint_src.joint_angle));
}

/* 将跟踪微分器对齐给定位置并清零导数。 */
static void joint_angle_td_follow(Gimbal_cmd* obj, float ref[ARM_JOINT_NUM]) {
    /* 逐轴重置跟踪微分器状态。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->joint_angle_td[i].ref = ref[i];
        obj->joint_angle_td[i].v1 = obj->joint_angle_td[i].ref;
        obj->joint_angle_td[i].v2 = 0;
        obj->joint_angle_td[i].v3 = 0;
    }
}
