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
// j4 由于存在多圈问题，故每次上电都重新设置为当前 fdb_position
CONSTEXPR float JOINT5_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT5], MOTOR_REDUCTION[JOINT5], JOINT_ANGLE_INIT[JOINT5]);
// j6 和 gripper 电机掉电不保存零点，motor_pos_init 在初始化时设置为当前 fdb_position
CONSTEXPR float JOINT6_POS_INIT =
    get_pos_init(0, MOTOR_DIR[JOINT6], MOTOR_REDUCTION[JOINT6], JOINT_ANGLE_INIT[JOINT6]);

// 达妙电机 ref_torque 会在驱动里做映射，只用考虑传动比
CONSTEXPR float MG6010E_TORQUE_CONSTANT = 0.16f;  // Nm/A
// 关节电机转矩转换系数，由转矩到 output
CONSTEXPR float JOINT_TORQUE_TRANS[ARM_JOINT_NUM] = {
    1,
    1,
    1,
    1 / MOTOR_REDUCTION[JOINT4],
    1,
    1 / MOTOR_REDUCTION[JOINT6],
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
