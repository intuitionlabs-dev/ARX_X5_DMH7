#pragma once
#include "arm.h"

/* 现有线格式使用六个关节槽位，夹爪使用独立槽位。 */
int Arm_Protocol_Control(Arm* arm, const Robot_rx_payload* payload, uint8_t has_velocity);
/* 将控制反馈编码为固定协议载荷。 */
void Arm_Protocol_Feedback(const Arm* arm, FramePktArmFeedback* feedback);
/* 将设零状态和逐电机结果编码为应答载荷。 */
void Arm_Protocol_Zero_Result(const Gimbal* arm, FramePktMotorSetZeroAck* result);
