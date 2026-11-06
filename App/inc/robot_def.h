#pragma once

#include <stdint.h>

/* 提供通用声明属性和控制周期常量。 */
#define MAYBE_UNUSED __attribute__((unused))
#define CONSTEXPR static const
#define ARM_NUM 2
#define ARM_CONTROL_PERIOD_S 0.002f

/* 定义板上两台机械臂的数组索引。 */
typedef enum { ARM_A = 0, ARM_B = 1 } Arm_Index; // NOLINT(readability-identifier-naming)

typedef struct Arm Arm;

/* 定义停止、运行、阻尼和重力补偿模式。 */
typedef enum { arm_stop = 0, arm_run, arm_damping, arm_only_gravity_comp } Arm_mode;

/* 定义自动动作模式及枚举边界。 */
typedef enum {
    arm_auto_none = 0,
    arm_auto_reset,
    ARM_AUTO_MODE_NUM, // NOLINT(readability-identifier-naming)
} Arm_auto_mode;

/* 在 ARM_JOINT_NUM 前配置关节，夹爪索引由关节数派生。 */
enum Arm_Motor_Index { // NOLINT(readability-identifier-naming)
    JOINT1 = 0, // NOLINT(readability-identifier-naming)
    JOINT2, // NOLINT(readability-identifier-naming)
    JOINT3, // NOLINT(readability-identifier-naming)
    JOINT4, // NOLINT(readability-identifier-naming)
    JOINT5, // NOLINT(readability-identifier-naming)
    JOINT6, // NOLINT(readability-identifier-naming)
    ARM_JOINT_NUM, // NOLINT(readability-identifier-naming)
    GRIPPER = ARM_JOINT_NUM, // NOLINT(readability-identifier-naming)
    ARM_MOTOR_NUM // NOLINT(readability-identifier-naming)
};

/* 固定协议要求六个关节，其他轴数触发编译错误。 */
_Static_assert(ARM_JOINT_NUM == 6, "当前 H7 固件要求 ARM_JOINT_NUM == 6");

/* 按关节坐标保存模式、运动参考和夹爪目标。 */
typedef struct {
    Arm_mode mode;
    float joint_angle[ARM_JOINT_NUM];  // deg
    float joint_vel[ARM_JOINT_NUM];  // deg/s
    float joint_acc[ARM_JOINT_NUM];  // deg/s²
    float gripper_angle;  // 协议夹爪角，deg
    float gripper_torque;  // 夹爪力矩，N·m
} Arm_control_data;

/* 保存电机状态及关节、夹爪反馈。 */
typedef struct {
    Arm_mode mode;
    uint8_t is_motor_lost[ARM_MOTOR_NUM];
    uint8_t motor_inited[ARM_MOTOR_NUM];
    float joint_angle[ARM_JOINT_NUM];  // deg
    float joint_vel[ARM_JOINT_NUM];  // deg/s
    float gripper_angle;  // 协议夹爪角，deg
    uint8_t motor_temp[ARM_MOTOR_NUM];
} Arm_feedback_data;
