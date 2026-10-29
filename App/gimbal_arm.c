#include "gimbal_arm.h"
#include "arm.h"
#include "can_config.h"
#include "common.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* 调试器观测控制器数据。 */
float g_fdb_pos, g_fdb_speed, g_ref_pos, g_ref_speed;
float g_speed_comp, g_output, g_speed_output, g_ouput_comp;

/* 根据机型配置创建单臂电机和控制工作区。 */
Gimbal* Gimbal_Create_Profile(const Arm_profile* profile, uint8_t can_index) {
    /* 总线索引或机型配置无效时拒绝创建。 */
    if (can_index >= DEVICE_CAN_CNT || !Arm_Profile_Validate(profile)) return NULL;
    Gimbal* obj = calloc(1, sizeof(*obj));
    /* 控制对象分配失败时返回空指针。 */
    if (!obj) return NULL;
    obj->profile = profile;
    obj->can_index = can_index;
    /* 逐电机创建驱动并设置初始模式。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        Arm_motor* motor = &obj->motors[i];
        motor->config = &profile->motors[i];
        /* 在临界区内创建驱动并发布初始增益。 */
        const uint32_t interrupt_mask = Arm_Motors_Lock();
        const int created = motor->config->driver->create(motor, motor->config, can_index);
        /* 创建成功后初始化监控状态和停止增益。 */
        if (created) {
            motor->monitor->count = 0;
            motor->config->driver->mode(motor, 0, motor->config->kp, motor->config->kd);
        }
        Arm_Motors_Unlock(interrupt_mask);
        /* 任一电机创建失败时终止控制对象创建。 */
        if (!created) return NULL;
    }
    /* 加载各关节电机零点偏移，单位 deg。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i)
        obj->motor_pos_init[i] = profile->joints[i].position_zero_deg;
    return obj;
}

/* 根据板级绑定创建指定机械臂。 */
Gimbal* Gimbal_Create(uint8_t arm_index) {
    /* 机械臂索引越界时拒绝创建。 */
    if (arm_index >= ARM_NUM) return NULL;
    const Arm_binding* binding = &ARM_BINDINGS[arm_index];
    /* 检查各臂是否独占 CAN 总线。 */
    for (int i = 0; i < ARM_NUM; ++i)
        /* 总线绑定冲突时拒绝创建。 */
        if (i != arm_index && binding->can_index == ARM_BINDINGS[i].can_index) return NULL;
    return Gimbal_Create_Profile(binding->profile, binding->can_index);
}

/* 按运行模式统一设置 PID 和驱动增益。 */
static void apply_motor_mode(Gimbal* obj, Arm_mode mode) {
    /* 被动控制使用零 PID 增益配置。 */
    const PID_Config zero = {0};
    /* 逐电机发布使能状态、刚度和阻尼。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        Arm_motor* motor = &obj->motors[i];
        const Arm_motor_config* config = motor->config;
        const uint8_t passive = mode == arm_damping
            || (mode == arm_only_gravity_comp && i != GRIPPER);
        motor->control->pid_pos_data.config = passive ? zero : config->position_pid;
        motor->control->pid_speed_data.config = passive ? zero : config->speed_pid;
        const float kp = passive ? 0 : config->kp;
        const float kd = mode == arm_damping ? config->damping_kd
            : passive ? 0 : config->kd;
        config->driver->mode(motor, mode != arm_stop, kp, kd);
    }
}

/* 采集电机状态并换算关节和夹爪反馈。 */
static void get_feedback(Gimbal* obj) {
    Arm_feedback_data* feedback = &obj->arm_feedback_data;
    uint8_t online = 1;
    /* 读取各电机掉线状态和温度。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        Arm_motor* motor = &obj->motors[i];
        feedback->is_motor_lost[i] = is_Offline(motor->monitor);
        feedback->motor_temp[i] = motor->config->driver->temperature(motor);
        /* 任一电机掉线时清除全部在线标志。 */
        if (feedback->is_motor_lost[i]) online = 0;
    }
    /* 将电机反馈换算为关节角和关节速度。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        const Arm_joint_config* joint = &obj->profile->joints[i];
        controller* control = obj->motors[i].control;
        feedback->joint_angle[i] = (control->fdb_position-obj->motor_pos_init[i])
            * joint->direction/joint->reduction;
        feedback->joint_vel[i] = control->fdb_speed*joint->direction/joint->reduction;
        /* 绝对编码关节按在线状态标记位置有效性。 */
        if (joint->absolute_encoder)
            feedback->motor_inited[i] = !feedback->is_motor_lost[i];
    }
    /* 将夹爪电机位置转换为协议角度。 */
    feedback->gripper_angle = Arm_Gripper_From_Motor(obj->profile,
        obj->motors[GRIPPER].control->fdb_position);
    /* 全部电机在线时记录首次上线完成。 */
    if (online) obj->first_init_flag = 1;
    /* 按首次上线状态和掉线策略执行停止保护。 */
    if (!obj->first_init_flag || (!online && obj->profile->stop_on_motor_lost))
        obj->arm_mode = arm_stop;
}

/* 按当前模式计算关节逆动力学补偿。 */
static void dynamics_update(Gimbal* obj) {
    memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
    obj->dynamics_status = ARM_DYNAMICS_OK;
    /* 模型未配置或模式无需补偿时保持零力矩。 */
    if (!obj->profile->dynamics
        || (obj->arm_mode != arm_run && obj->arm_mode != arm_only_gravity_comp)) return;
    float q[ARM_JOINT_NUM], qd[ARM_JOINT_NUM], qdd[ARM_JOINT_NUM];
    /* 将角度、参考速度和加速度换算为 SI 单位。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        q[i] = obj->arm_feedback_data.joint_angle[i]*DEG2RAD;
        qd[i] = obj->arm_control_data.joint_vel[i]*DEG2RAD;
        qdd[i] = obj->arm_control_data.joint_acc[i]*DEG2RAD;
    }
    /* 按完整逆动力学或静态重力模式计算补偿。 */
    obj->dynamics_status = Arm_Dynamics_Compute(obj->profile->dynamics, q, qd, qdd,
        obj->profile->gravity, obj->arm_mode == arm_only_gravity_comp,
        obj->tau_comp, obj->dynamics_work, ARM_JOINT_NUM);
    /* 动力学计算失败时清空补偿并停止该臂。 */
    if (obj->dynamics_status != ARM_DYNAMICS_OK) {
        memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
        obj->arm_mode = arm_stop;
    }
}

/* 将关节和夹爪目标换算为电机参考值。 */
static void set_references(Gimbal* obj) {
    memset(obj->damp_comp, 0, sizeof(obj->damp_comp));
    /* 先使所有电机参考位置跟随反馈并清空前馈。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        controller* control = obj->motors[i].control;
        control->ref_position = isfinite(control->fdb_position) ? control->fdb_position : 0;
        control->speed_comp = 0;
        control->output_comp = 0;
    }
    /* 停止和阻尼模式使用已清空的前馈值。 */
    if (obj->arm_mode == arm_stop || obj->arm_mode == arm_damping) return;
    /* 逐轴换算参考角、速度和补偿力矩。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        const Arm_joint_config* joint = &obj->profile->joints[i];
        controller* control = obj->motors[i].control;
        /* 运行模式设置关节位置和速度前馈。 */
        if (obj->arm_mode == arm_run) {
            control->ref_position = obj->arm_control_data.joint_angle[i]
                * joint->direction*joint->reduction+obj->motor_pos_init[i];
            control->speed_comp = obj->arm_control_data.joint_vel[i]
                * joint->direction*joint->reduction*joint->velocity_scale;
            obj->damp_comp[i] = obj->arm_control_data.joint_vel[i]*DEG2RAD*joint->viscous_damping;
        }
        /* 将关节补偿换算为电机输出方向和力矩标定。 */
        control->output_comp = (obj->tau_comp[i]+obj->damp_comp[i])
            * joint->direction*joint->torque_scale;
    }
    /* 运行模式发布夹爪位置和力矩目标。 */
    if (obj->arm_mode == arm_run) {
        controller* grip = obj->motors[GRIPPER].control;
        grip->ref_position = Arm_Gripper_To_Motor(obj->profile, obj->arm_control_data.gripper_angle);
        grip->output_comp = obj->arm_control_data.gripper_torque*obj->profile->gripper.torque_scale;
    }
}

/* 更新反馈、动力学和本周期电机控制输出。 */
void Gimbal_Update(Gimbal* obj) {
    /* 控制对象无效时结束本次更新。 */
    if (!obj) return;
    obj->arm_mode = obj->arm_control_data.mode;
    /* 模式值超出有效范围时进入停止模式。 */
    if ((unsigned)obj->arm_mode > arm_only_gravity_comp) obj->arm_mode = arm_stop;
    get_feedback(obj);
    /* 设零期间强制使用停止模式。 */
    if (Arm_Zero_Busy(obj)) obj->arm_mode = arm_stop;
    dynamics_update(obj);
    /* 在临界区内同步发布电机参考值和模式。 */
    const uint32_t interrupt_mask = Arm_Motors_Lock();
    set_references(obj);
    /* 发布前逐电机校验参考值有效性。 */
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        const controller* c = obj->motors[i].control;
        /* 任一参考值异常时标记错误并停止该臂。 */
        if (!isfinite(c->ref_position) || !isfinite(c->speed_comp) || !isfinite(c->output_comp)) {
            obj->arm_mode = arm_stop;
            obj->dynamics_status = ARM_DYNAMICS_INVALID;
        }
    }
    /* 停止模式清空补偿并重建安全参考值。 */
    if (obj->arm_mode == arm_stop) {
        memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
        set_references(obj);
    }
    /* 发布增益、使能状态及实际反馈模式。 */
    apply_motor_mode(obj, obj->arm_mode);
    obj->arm_feedback_data.mode = obj->arm_mode;
    Arm_Motors_Unlock(interrupt_mask);

    /* 采样 A 臂 JOINT2 的反馈、参考值及控制输出。 */
    if (obj == g_arm[ARM_A].gimbal) {
        const controller* control = obj->motors[JOINT2].control;
        g_fdb_pos = control->fdb_position;
        g_fdb_speed = control->fdb_speed;
        g_ref_pos = control->ref_position;
        g_ref_speed = control->ref_speed;
        g_speed_comp = control->speed_comp;
        g_output = control->output;
        g_ouput_comp = control->output_comp;
        g_speed_output = control->pid_speed_data.output;
    }
}
