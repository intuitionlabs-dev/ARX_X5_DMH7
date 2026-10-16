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
#include "x5_dynamics.h"

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

/* arm_damping 模式下逐台电机的 kd（**含夹爪**，所以是 ARM_MOTOR_NUM 不是
 * ARM_JOINT_NUM）。
 *
 * **别和上面的 JOINT_DAMPING 搞混**：那个是"粘性摩擦前馈系数 b"，跑在
 * arm_run 里、算的是 output_comp 的一项、单位 N·m·s/rad、只有 6 个关节；
 * 这个表是**电机自己的 kd 字段**（obj->kp/obj->kd，达妙走 MIT、ENCOS 走它自己那套），
 * 阻尼模式下位置环 PID 全关，力矩就靠它 —— 单位是电机那边的力矩/速度比，
 * 和 b 不是一回事。
 *
 * 值**照抄** Gimbal_Create() 里给正常模式设的那两组（gimbal_arm.c:665-702 的
 * encos_kpkd[i][1] 与 pid_mit_kpkd[i][1]），也就是"阻尼模式和正常运行时的 kd 一样"。
 * 这样掉线兜底时的手感至少是已经验证过的那一档，不会引入没试过的数。
 *
 * TODO: 这是从正常模式抄来的，**没有单独标定过阻尼模式**。阻尼模式的期望行为是
 * "能被人推动、松手缓慢下沉"，标定方向是往下压 kd 到刚好不振荡。第一次上真机试
 * arm_damping 时要重新量这几个数（尤其 j2/j3 承重，50 可能偏大）。 */
CONSTEXPR float JOINT_DAMPING_MODE_KD[ARM_MOTOR_NUM] = {
    35, 50, 50, 0.8f, 0.8f, 1.0f, 0.1f,
};

/* 六轴动力学补偿使用 KDL RNE 与 X5 URDF 的质量、质心和惯量。
 * SDK 模式的重力项系数为 [0.8, 0.8, 0.8, 1.32, 1.32, 1.32]。
 * 基座系重力向量的单位为 m/s²，NULL 表示 (0, 0, -9.81)。
 */
#define JOINT_DYNAMICS_COMP_ENABLE 1
#define JOINT_DYNAMICS_COMP_MODEL X5_MODEL_2023
#define JOINT_DYNAMICS_GRAVITY_MODE X5_GRAVITY_SDK
#define JOINT_DYNAMICS_GRAVITY_BASE NULL
