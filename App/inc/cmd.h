#pragma once

#include <stdint.h>

#include "DM_motor.h"

/* CMD 线程的电机对象。
 *
 * 电机：cmd_motor，参数写在 App/cmd.c 的 cmd_motor_config 里。
 * 遥控器输入不在这里 —— 它是独立驱动，在 libs/rc/(见 rc.h)，Cmd_Loop() 只是
 * 顺手带跑一遍 Rc_Loop()。
 *
 * 挂载方式：
 *   Cmd_Init();   // main.c 的 USER CODE BEGIN 2，必须在 BSP_CAN_Init() 和
 *                 // DMmotor_Driver_Init() 之后(它依赖 dm_motor_list 和 FDCAN)
 *   StartCmdTask 的循环里调 Cmd_Loop();(Cmd_Loop() 里已经带了 Rc_Loop())
 */

/* 电机对象，Cmd_Init() 之后有效。
 * DMmotor_Create() 失败时(比如 bsp_can_index 超出 FDCAN 实例数)为 NULL，
 * 用之前自己判一下。 */
extern DM_motor *cmd_motor;

void Cmd_Init(void);
void Cmd_Loop(void);
