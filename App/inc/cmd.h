#pragma once

#include "DM_motor.h"

/* CMD 线程的电机对象与命令处理。
 *
 * 目前只建一台电机(cmd_motor)，参数写在 App/cmd.c 的 cmd_motor_config 里，
 * 上位机 / 命令协议还没定，所以 Cmd_Loop() 先留空，等你往里填。
 *
 * 挂载方式：
 *   Cmd_Init();   // main.c 的 USER CODE BEGIN 2，必须在 BSP_CAN_Init() 和
 *                 // DMmotor_Driver_Init() 之后(它依赖 dm_motor_list 和 FDCAN)
 *   StartCmdTask 的循环里调 Cmd_Loop();
 */

/* 电机对象，Cmd_Init() 之后有效。
 * DMmotor_Create() 失败时(比如 bsp_can_index 超出 FDCAN 实例数)为 NULL，
 * 用之前自己判一下。 */
extern DM_motor *cmd_motor;

void Cmd_Init(void);
void Cmd_Loop(void);
