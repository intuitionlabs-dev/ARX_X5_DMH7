#pragma once
#include "robot_def.h"
#include "arm_motor.h"
#include "arm_dynamics.h"
struct Gimbal_cmd_t;

/* 配置关节坐标换算、限位和轨迹控制参数。 */
typedef struct {
    float direction, reduction, position_zero_deg;
    float minimum_deg, maximum_deg, reset_deg;
    float torque_scale, viscous_damping, velocity_scale, plan_velocity_scale;
    float td_r;
    uint8_t absolute_encoder;
} Arm_joint_config;

/* 配置单电机夹爪的双向坐标和力矩映射。 */
typedef struct {
    /* 映射点使用协议夹爪角和电机角，单位 deg。 */
    float cmd[2], motor_deg[2];
    float torque_scale;
    float minimum, maximum;
    float (*to_motor)(float cmd, const void* context);
    float (*from_motor)(float motor_deg, const void* context);
    const void* context;
} Arm_gripper_mapping;

/* 配置关节复位、夹爪搜索和设零时限。 */
typedef struct {
    float reset_time_s, pose_tolerance_deg;
    uint32_t zero_total_ms, zero_motor_ms, zero_disable_timeout_ms, zero_disable_retry_ms;
    float gripper_home_torque, gripper_speed_deg_s, gripper_backoff_deg;
    float stall_tolerance_deg, gripper_follow_error_deg, gripper_backoff_tolerance_deg;
    uint32_t gripper_settle_ms;
    uint32_t stall_window_ms, stall_windows, seek_timeout_ms, backoff_timeout_ms;
} Arm_init_policy;

/* 集中提供机型参数、模型及初始化回调。 */
typedef struct Arm_profile { // NOLINT(readability-identifier-naming)
    const char* name;
    size_t joint_count, motor_count;
    const Arm_motor_config* motors;
    const Arm_joint_config* joints;
    Arm_gripper_mapping gripper;
    Arm_init_policy init;
    const Arm_dynamics_model* dynamics;
    float gravity[3];
    float quintic_duration_s;
    uint8_t stop_on_motor_lost;
    /* start=1 启动；返回 0 等待、1 完成、负值失败。 */
    int (*initialize)(struct Gimbal_cmd_t* cmd, uint8_t start);
    void (*cancel_initialize)(struct Gimbal_cmd_t* cmd);
    /* 遥控器通道 rx、ry、lx 的关节索引，负值表示通道未分配。 */
    int16_t rc_joint[3];
    float rc_speed_deg_s;
} Arm_profile;

/* 定义单臂机型配置与板级总线的绑定。 */
typedef struct {
    const Arm_profile* profile;
    uint8_t can_index;
} Arm_binding;

/* 提供内置机型配置和各臂板级绑定。 */
extern const Arm_profile ARM_PROFILE_METAL;
extern const Arm_profile ARM_PROFILE_ARX_X5;
extern const Arm_binding ARM_BINDINGS[ARM_NUM];

/* 校验机型参数和发送预算，合法时返回 1。 */
int Arm_Profile_Validate(const Arm_profile* profile);
/* 将协议夹爪角转换为电机角，单位 deg。 */
float Arm_Gripper_To_Motor(const Arm_profile* profile, float cmd);
/* 将电机角转换为协议夹爪角，单位 deg。 */
float Arm_Gripper_From_Motor(const Arm_profile* profile, float motor_deg);
