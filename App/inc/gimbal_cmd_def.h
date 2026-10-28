#pragma once

/* Metal 关节空间软限位，单位 deg，与 SDK board_defs/metal_arm.py 一致。
 * 这些参数作用于 Arm_control_data / Arm_feedback_data 的关节角坐标。 */

#include "robot_def.h"
#include "common.h"  /* RAD2DEG */

/* 普通关节目标的五次样条过渡时长，s。 */
CONSTEXPR float JOINT_QUINTIC_DURATION_S = 0.16f;

/* 角度限位，deg */
CONSTEXPR float JOINT_ANGLE_MIN[ARM_JOINT_NUM] = {
    -160, -180, 0, -123, -85, -145,
};

CONSTEXPR float JOINT_ANGLE_MAX[ARM_JOINT_NUM] = {
    160, 0, 180, 81, 85, 145,
};

CONSTEXPR float GRIPPER_ANGLE_MIN = 0.0f;
CONSTEXPR float GRIPPER_ANGLE_MAX = 2.4f * RAD2DEG;  // 2.4 rad，约 137.50987 deg

/* 规划速度前馈系数，一键模式跑规划时用（Arm_Ref_Set()）。*/
CONSTEXPR float PLAN_SPEED_COMP_COEF[ARM_JOINT_NUM] = {
    1,
    1,
    1,
    1,
    1,
    1,
};
