#pragma once

/* CtrBoard-H7_ALL 上六轴机械臂的板级标定量：电机方向、减速比、零点、前馈系数。
 *
 * 原样搬自 ec_code/engineer-wheel-leg-v-1/APP/gimbal/gimbal_arm_def.h，另外把那个
 * 工程 robot_struct.h 里的 REDUCTION_DM2325 挪了进来。
 *
 * **只给 App/gimbal_arm.c include** —— 下面全是 static const 数组/标量，每个包含这个
 * 头的编译单元都会各得一份副本，到处 include 就是白占 Flash，还会冒一堆 unused 警告。
 * (类型定义在 robot_def.h，那个才是谁都能 include 的。)
 *
 * 这些数字是实机标定出来的，改机械结构就得重新量。
 */

#include "robot_def.h"
#include "x5_gravity.h" /* x5_model / x5_gravity_mode，给下面的重力补偿配置用 */

// 电机正方向在运动学坐标系中的方向
CONSTEXPR int8_t MOTOR_DIR[ARM_JOINT_NUM] = {1, 1, 1, 1, 1, 1};

/* motor_angle_init */
// 用于校准零点的位置下关节的运动学角度，deg
CONSTEXPR float JOINT_ANGLE_INIT[ARM_JOINT_NUM] = {
    0,
    0,
    0,
    0,
    0,
    0,
};
// 减速比
CONSTEXPR float MOTOR_REDUCTION[ARM_JOINT_NUM] = {1, 1, 1, 1, 1, 1};
/* motor_pos_init */
// 公式：pos_init = controller.fdb_position - MOTOR_DIR * MOTOR_REDUCTION * JOINT_ANGLE_INIT
#define get_pos_init(fdb_pos, dir, reduction, angle_init) \
    ((fdb_pos) - (dir) * (reduction) * (angle_init))
// 公式：motor_angle = (controller.fdb_position - pos_init) * MOTOR_DIR / MOTOR_REDUCTION
#define pos_to_angle(pos, pos_init, dir, reduction) \
    (((pos) - (pos_init)) * (dir) / (reduction))
// 公式：controller.fdb_position = MOTOR_DIR * MOTOR_REDUCTION * motor_angle + motor_pos_init
#define angle_to_pos(angle, pos_init, dir, reduction) \
    ((angle) * (reduction) * (dir) + (pos_init))
#define vel_to_speed(vel, dir, reduction) ((vel) * (reduction) * (dir))
#define speed_to_vel(speed, dir, reduction) ((speed) / (reduction) * (dir))

// 初始化位置下电机在 controller 里的 fdb_position
CONSTEXPR float JOINT1_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT1], MOTOR_REDUCTION[JOINT1], JOINT_ANGLE_INIT[JOINT1]);
CONSTEXPR float JOINT2_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT2], MOTOR_REDUCTION[JOINT2], JOINT_ANGLE_INIT[JOINT2]);
CONSTEXPR float JOINT3_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT3], MOTOR_REDUCTION[JOINT3], JOINT_ANGLE_INIT[JOINT3]);
CONSTEXPR float JOINT4_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT4], MOTOR_REDUCTION[JOINT4], JOINT_ANGLE_INIT[JOINT4]);
CONSTEXPR float JOINT5_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT5], MOTOR_REDUCTION[JOINT5], JOINT_ANGLE_INIT[JOINT5]);
CONSTEXPR float JOINT6_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT6], MOTOR_REDUCTION[JOINT6], JOINT_ANGLE_INIT[JOINT6]);

// 关节电机转矩转换系数，由转矩到 output
CONSTEXPR float JOINT_TORQUE_TRANS[ARM_JOINT_NUM] = {
    1,
    1,
    1,
    1,
    1,
    1,
};
// 关节电机转矩转换系数修正系数
CONSTEXPR float JOINT_TORQUE_TRANS_COEF[ARM_JOINT_NUM] = {
    1,
    1,
    1,
    1,
    1,
    1,
};
// 粘性摩擦系数 b (N·m·s/rad)
CONSTEXPR float JOINT_DAMPING[ARM_JOINT_NUM] = {0, 0, 0, 0, 0, 0};
// 速度前馈系数，仅用于 controller.speed_comp 的计算
CONSTEXPR float JOINT_SPEEDCOMP_COEF[ARM_JOINT_NUM] = {1, 1, 1, 1, 1, 1};

/* ==================== 重力补偿 ====================
 * libs/gravity_compensation 是纯 C99 / 零依赖的 **静态重力力矩** 模块(不算法速度/
 * 加速度惯性，也不算科里奥利)，正好顶替被注释掉的 Joint_Feedforward_Calc() ——
 * 那个函数原来调 arm_serial_link 的整机 Newton-Euler，本工程没移植。填的是
 * Gimbal.tau_comp[]，Set_Motor_ref() 再乘 JOINT_TORQUE_TRANS/_COEF 变成 output_comp。
 *
 * 三个型号**只有末端连杆质量不同**(x5/2023 = 0.580、master = 0.680、2025 = 0.570 kg)，
 * 前六段完全一样，所以选错的代价集中在 j4~j6。本臂按 2023 填。
 *
 * 模式选 SDK 是因为它和厂商 SDK 的重力模式**逐位一致**：G(q) 再乘
 * [0.8, 0.8, 0.8, 1.32, 1.32, 1.32] —— 那个系数就在
 * libs/gravity_compensation/c/x5_sdk_parameters.c 的 gravity_scale 字段里，
 * 和 PROVENANCE.md 里反汇编出来的 .rodata 常数 0.8 / 1.32 对得上。
 * 换成 X5_GRAVITY_PHYSICAL 会得到未缩放的 KDL G(q)，前三个关节小 20%、后三个小 24%。 */
#define JOINT_GRAVITY_COMP_ENABLE 1
#define JOINT_GRAVITY_COMP_MODEL X5_MODEL_2023
#define JOINT_GRAVITY_COMP_MODE X5_GRAVITY_SDK

/* 基座系重力方向。NULL 就是按 (0, 0, -9.81) —— 机械臂水平安装时用这个。
 * 底座要是歪着装(垫了角度、装在斜面上)，把实际的重力向量按基座系填进来。 */
#define JOINT_GRAVITY_BASE NULL
