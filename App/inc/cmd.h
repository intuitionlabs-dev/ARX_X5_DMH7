#pragma once

/* CMD 线程。
 *
 * 这里原来挂着 cmd_motor(单个达妙电机，参数写在 App/cmd.c 的 cmd_motor_config
 * 里)和 Cmd_Init()，机械臂云台接进来之后那段逻辑删掉了 —— 电机对象现在由
 * gimbal_arm.c 的 Gimbal_Create() 统一建，见 App/inc/gimbal_arm.h。
 *
 * 遥控器输入不在这里 —— 它是独立驱动，在 libs/rc/(见 rc.h)，Cmd_Loop() 只是
 * 顺手带跑一遍 Rc_Loop()。
 *
 * 挂载方式：
 *   gimbal = Gimbal_Create();  // freertos.c 的 StartCmdTask 里、进循环之前。
 *                              // 必须在 BSP_CAN_Init() 和 DMmotor_Driver_Init()
 *                              // 之后(它依赖 dm_motor_list 和 FDCAN)；
 *                              // 也必须在任务里而不是 main()（主栈只有 2 KB）。
 *   StartCmdTask 的循环里调 Cmd_Loop();(Cmd_Loop() 里已经带了 Rc_Loop())
 */

void Cmd_Loop(void);
