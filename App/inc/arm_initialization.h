#pragma once
#include "gimbal_cmd.h"

/* 启动或推进保持夹爪位置的关节复位策略。 */
int Arm_Initialize_Hold(Gimbal_cmd* cmd, uint8_t start);
/* 启动或推进夹爪寻限位、回退和设零策略。 */
int Arm_Initialize_Home(Gimbal_cmd* cmd, uint8_t start);

/* 清除初始化接管状态并使夹爪跟随反馈。 */
void Arm_Initialization_Cancel(Gimbal_cmd* cmd);
