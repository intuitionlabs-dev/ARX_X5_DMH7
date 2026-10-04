#pragma once

/* 六轴机械臂(+夹爪)的枚举 / 结构体定义。
 *
 * 从 ec_code/engineer-wheel-leg-v-1/APP/robot_def.h 里挑出来的**臂相关部分**——
 * 那个文件同时装着底盘、裁判系统、UI、以太网一大堆东西，本工程用不到，没搬。
 *
 * 这个头里**只有类型，没有常量数组**，所以谁 include 都安全。板级标定量(电机方向、
 * 减速比、零点、前馈系数)在 App/inc/gimbal_arm_def.h，那个是"每个包含它的编译单元
 * 各得一份副本"，只给 gimbal_arm.c 用。
 *
 * 机械臂的指令来源：源工程走 pub_sub(register_pub / register_sub)，本工程没有这个
 * 框架，改由命令层(App/gimbal_cmd.c)每拍直接写 Gimbal.arm_control_data。
 */

#include <stdint.h>

/* 源工程里由 config/global_config.h 提供 */
#define MAYBE_UNUSED __attribute__((unused))
/* 源工程里 C++ 下展开成 constexpr、C 下展开成 static const */
#define CONSTEXPR static const

/* 机械臂任一电机掉线是否停机 */
#define ANY_MOTOR_LOST_STOP_ENABLE 1

/* 机械臂工作模式 */
typedef enum { arm_stop = 0, arm_run } Arm_mode;

/* 机械臂电机索引。GRIPPER 故意和 ARM_JOINT_NUM 同值：它是第 7 个电机，但不属于 6 个
 * 关节，所以以关节数结尾的循环不会碰到它，以电机数结尾的循环会。 */
// NOLINTBEGIN
enum Arm_Motor_Index {
    JOINT1 = 0,
    JOINT2,
    JOINT3,
    JOINT4,
    JOINT5,
    JOINT6,
    ARM_JOINT_NUM,            // 6，关节数
    GRIPPER = ARM_JOINT_NUM,  // 6，夹爪
    ARM_MOTOR_NUM             // 7，电机总数
};
// NOLINTEND

/* 云台 cmd -> arm：指令 */
typedef struct {
    Arm_mode mode;
    float joint_angle[ARM_JOINT_NUM];  // 关节角度，deg
    float joint_vel[ARM_JOINT_NUM];    // 关节速度
    float joint_acc[ARM_JOINT_NUM];    // 关节加速度
    float gripper_oc_angle;    // 夹爪开合程度，0~1，0为完全闭合，1为完全张开
} Arm_control_data;

/* arm -> 云台 cmd：反馈 */
typedef struct {
    Arm_mode mode;
    uint8_t is_motor_lost[ARM_MOTOR_NUM];  // 电机掉线标志
    uint8_t motor_inited[ARM_MOTOR_NUM];   // 初始化状态，0 未初始化 / 1 已初始化
    float joint_angle[ARM_JOINT_NUM];      // 关节角度，deg
    uint8_t motor_temp[ARM_MOTOR_NUM];     // 电机温度，℃
} Arm_feedback_data;
