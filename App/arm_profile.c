#include "arm_profile.h"
#include "can_config.h"
#include <math.h>

/* 检查浮点参数数组是否全部为有限值。 */
static uint8_t finite_values(const float* values, size_t count) {
    /* 逐项检查参数有效性。 */
    for (size_t i = 0; i < count; ++i)
        /* 遇到非有限值时返回校验失败。 */
        if (!isfinite(values[i])) return 0;
    return 1;
}

/* 校验机型尺寸、参数、驱动和发送预算。 */
int Arm_Profile_Validate(const Arm_profile* profile) {
    /* 检查必要配置指针及枚举派生尺寸。 */
    if (!profile || !profile->motors || !profile->joints || !profile->initialize
        || profile->joint_count != ARM_JOINT_NUM || profile->motor_count != ARM_MOTOR_NUM
        || !ARM_JOINT_NUM || ARM_MOTOR_NUM > 255) return 0;
    /* 检查轨迹时长、设零超时和重力参数。 */
    if (!isfinite(profile->quintic_duration_s) || profile->quintic_duration_s <= 0
        || !isfinite(profile->init.reset_time_s) || profile->init.reset_time_s <= 0
        || !isfinite(profile->init.pose_tolerance_deg) || profile->init.pose_tolerance_deg < 0
        || !profile->init.zero_motor_ms || !profile->init.zero_total_ms
        || !profile->init.zero_disable_timeout_ms || !profile->init.zero_disable_retry_ms
        || !finite_values(profile->gravity, 3)) return 0;
    /* 收集初始化动作和遥控速度的有限值检查项。 */
    const float home_parameters[] = {profile->init.gripper_home_torque, profile->init.gripper_speed_deg_s,
        profile->init.gripper_backoff_deg, profile->init.stall_tolerance_deg,
        profile->init.gripper_follow_error_deg, profile->init.gripper_backoff_tolerance_deg,
        profile->rc_speed_deg_s};
    /* 检查夹爪初始化和遥控速度参数。 */
    if (!finite_values(home_parameters, sizeof(home_parameters)/sizeof(home_parameters[0]))) return 0;
    const Arm_gripper_mapping* mapping = &profile->gripper;
    /* 检查夹爪标定点、限位及双向映射回调。 */
    if (!finite_values(mapping->cmd, 2) || !finite_values(mapping->motor_deg, 2)
        || mapping->cmd[0] == mapping->cmd[1]
        || mapping->motor_deg[0] == mapping->motor_deg[1]
        || !isfinite(mapping->minimum) || !isfinite(mapping->maximum)
        || mapping->minimum >= mapping->maximum || !isfinite(mapping->torque_scale)
        || !!mapping->to_motor != !!mapping->from_motor) return 0;
    /* 检查动力学模型指针和关节数量。 */
    if (profile->dynamics && (!profile->dynamics->links
        || profile->dynamics->joint_count != ARM_JOINT_NUM)) return 0;
    /* 逐轴检查坐标换算、限位和复位目标。 */
    for (size_t i = 0; i < profile->joint_count; ++i) {
        const Arm_joint_config* joint = &profile->joints[i];
        /* 收集当前关节的换算与控制参数。 */
        const float values[] = {joint->direction, joint->reduction, joint->position_zero_deg,
            joint->minimum_deg, joint->maximum_deg, joint->reset_deg, joint->torque_scale,
            joint->viscous_damping, joint->velocity_scale, joint->plan_velocity_scale, joint->td_r};
        /* 拒绝无效方向、比例、跟踪参数或复位目标。 */
        if (!finite_values(values, sizeof(values)/sizeof(values[0]))
            || fabsf(joint->direction) != 1 || joint->reduction <= 0 || joint->td_r <= 0
            || joint->minimum_deg > joint->maximum_deg
            || joint->reset_deg < joint->minimum_deg || joint->reset_deg > joint->maximum_deg) return 0;
    }
    /* 逐电机检查驱动操作和控制量程。 */
    for (size_t i = 0; i < profile->motor_count; ++i) {
        const Arm_motor_config* motor = &profile->motors[i];
        const Arm_motor_driver* driver = motor->driver;
        /* 收集当前电机的编码量程和模式增益。 */
        const float ranges[] = {motor->position_max, motor->speed_max, motor->torque_max,
            motor->kp_max, motor->kd_max, motor->kp, motor->kd, motor->damping_kd};
        /* 检查必要驱动回调、发送相位及增益范围。 */
        if (!driver || !driver->create || !driver->mode || !driver->position_rad
            || !driver->temperature || !driver->disable || !driver->stopped
            || !driver->zero_start || !driver->zero_poll || !motor->tx_group
            || !finite_values(ranges, sizeof(ranges)/sizeof(ranges[0]))
            || motor->position_max <= 0 || motor->speed_max <= 0 || motor->torque_max <= 0
            || motor->kp_max <= 0 || motor->kd_max <= 0 || motor->kp < 0 || motor->kd < 0
            || motor->kp > motor->kp_max || motor->kd > motor->kd_max
            || motor->damping_kd < 0 || motor->damping_kd > motor->kd_max) return 0;
        /* 收集当前电机位置环和速度环的 PID 参数。 */
        const float pid_parameters[] = {motor->position_pid.KP, motor->position_pid.KI,
            motor->position_pid.KD, motor->position_pid.error_max, motor->position_pid.outputMax,
            motor->position_pid.Irange, motor->speed_pid.KP, motor->speed_pid.KI,
            motor->speed_pid.KD, motor->speed_pid.error_max, motor->speed_pid.outputMax,
            motor->speed_pid.Irange};
        /* 检查位置环和速度环 PID 参数有效性。 */
        if (!finite_values(pid_parameters, sizeof(pid_parameters)/sizeof(pid_parameters[0]))) return 0;
        /* 执行驱动专用的配置校验。 */
        if (driver->validate && !driver->validate(motor)) return 0;
        /* 与已检查电机比较标识符冲突。 */
        for (size_t j = 0; j < i; ++j) {
            const Arm_motor_config* previous = &profile->motors[j];
            /* 发送标识符重复时返回校验失败。 */
            if (motor->tx_id == previous->tx_id) return 0;
            /* 按驱动共享规则检查反馈标识符。 */
            if (motor->rx_id == previous->rx_id
                && (driver != previous->driver || !driver->shared_rx_id)) return 0;
        }
    }
    /* 每个相位预留一帧，用于驱动维护命令。 */
    /* 逐发送相位核算包含维护帧的总预算。 */
    for (unsigned phase = 0; phase < CAN_TX_GROUP_NUM; ++phase) {
        unsigned frames = 1;
        /* 累计本相位需要发送的电机帧数。 */
        for (size_t i = 0; i < profile->motor_count; ++i)
            frames += (profile->motors[i].tx_group >> phase) & 1u;
        /* 超过单周期发送预算时返回校验失败。 */
        if (frames > CAN_TX_MAX_FRAMES_PER_TICK) return 0;
    }
    return 1;
}

/* 将协议夹爪角映射为电机角，单位 deg。 */
float Arm_Gripper_To_Motor(const Arm_profile* profile, float cmd) {
    const Arm_gripper_mapping* m = &profile->gripper;
    /* 配置自定义回调时使用其正向映射。 */
    if (m->to_motor) return m->to_motor(cmd, m->context);
    /* 按两个标定点执行线性正向映射。 */
    return m->motor_deg[0] + (cmd-m->cmd[0])
        * (m->motor_deg[1]-m->motor_deg[0])/(m->cmd[1]-m->cmd[0]);
}

/* 将电机角映射为协议夹爪角，单位 deg。 */
float Arm_Gripper_From_Motor(const Arm_profile* profile, float motor_deg) {
    const Arm_gripper_mapping* m = &profile->gripper;
    /* 配置自定义回调时使用其反向映射。 */
    if (m->from_motor) return m->from_motor(motor_deg, m->context);
    /* 按两个标定点执行线性反向映射。 */
    return m->cmd[0] + (motor_deg-m->motor_deg[0])
        * (m->cmd[1]-m->cmd[0])/(m->motor_deg[1]-m->motor_deg[0]);
}
