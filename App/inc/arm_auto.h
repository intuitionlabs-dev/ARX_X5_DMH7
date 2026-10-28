#pragma once

#include "gimbal_cmd.h"

/* 六轴从反馈姿态到复位目标的五次样条时长，s。 */
#define ARM_AUTO_RESET_TIME_S 2

/* Metal 六轴复位姿态，单位 deg，全部为 0。
 * 初始化入口检查使用同一目标，允许偏差见 ROBOT_INIT_POSE_TOL_DEG。 */
extern const float ARM_RESET_ANGLE[ARM_JOINT_NUM];

/* CmdTask 每 2 ms 调用一次，非阻塞地启动并跟踪六轴规划。
 * 规划位置、速度、加速度由命令层传给驱动层；结束后 auto_mode 回到 arm_auto_none。
 * 夹爪保持已有目标角。初始化和 0x18 复位共用此动作。 */
void Arm_Reset(Gimbal_cmd* obj);
