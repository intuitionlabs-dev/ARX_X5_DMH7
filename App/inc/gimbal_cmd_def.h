#pragma once

/* 机械臂命令层的板级标定量(角度软限位)。
 *
 * 原样搬自 ec_code/engineer-wheel-leg-v-1/APP/gimbal_cmd/gimbal_cmd_def.h。那边一共
 * 126 行，但大半是自定义控制器(工作空间/映射比例)、支架干涉航点、拓展板磁铁这些本工程
 * 没有的东西，只把**软限位**这几项挑出来 —— 它们和机械结构有关，改机械结构就得重新量。
 *
 * **只给 App/gimbal_cmd.c include** —— 下面全是 static const(CONSTEXPR)，每个包含这个
 * 头的编译单元都会各得一份副本，到处 include 就是白占 Flash，还会冒一堆 unused 警告。
 * (同 App/inc/gimbal_arm_def.h 的路子。类型定义在 gimbal_cmd.h，那个才是谁都能 include 的。)
 *
 * 这几个数配的是**关节角**，即 Arm_control_data.joint_angle / Arm_feedback_data.joint_angle
 * 那一套 deg 坐标，不是电机角度。注意 j2/j3 的零点还没按 ENCOS 重标定(见 gimbal_arm_def.h
 * 的 JOINT2_POS_INIT / JOINT3_POS_INIT)，标定之前这两个关节的限位是不准的 —— 好在停机
 * 模式下目标跟随反馈，限位是空转，等遥控器输入接进来之前必须先标定完。
 */

#include "robot_def.h"

/* 角度限位，deg */
CONSTEXPR float JOINT_ANGLE_MIN[ARM_JOINT_NUM] = {
    -150,
    5.72,
    5.72,
    -90,
    -90,
    -112.87,
};

CONSTEXPR float JOINT_ANGLE_MAX[ARM_JOINT_NUM] = {
    180,
    206.26,
    171.88,
    90,
    90,
    119.17,
};
