#pragma once

/* CMD 线程：500 Hz / 2 ms，跑机械臂命令层 + 机械臂驱动。
 *
 *   Cmd_Loop()
 *     ├─ Rc_Loop()            遥控器驱动(libs/rc)的服务：换算/判丢信号/喂狗
 *     ├─ Gimbal_Cmd_Update()  命令层(App/gimbal_cmd.c)：输入 → Arm_control_data
 *     └─ Gimbal_Update()      机械臂驱动(App/gimbal_arm.c)：按 Arm_control_data 跑电机
 *
 * 这里原来挂着 cmd_motor(单个达妙电机，参数写在 App/cmd.c 的 cmd_motor_config 里)和
 * Cmd_Init()，机械臂云台接进来之后那段逻辑删掉了 —— 电机对象现在由 gimbal_arm.c 的
 * Gimbal_Create() 统一建，见 App/inc/gimbal_arm.h。
 *
 * 遥控器的输入逻辑不在这里：本层只是"服务"遥控器驱动(下面那个 Rc_Loop())，把摇杆值
 * 用起来那部分(摇杆 → 关节角)按用户要求先不做，见 App/gimbal_cmd.c 的
 * Remote_Mode_Update()。
 *
 * 挂载方式：
 *   gimbal = Gimbal_Create();          // freertos.c 的 StartCmdTask 里、进循环之前。
 *                                      // 必须在 BSP_CAN_Init() 和 DMmotor_Driver_Init()
 *                                      // 之后(它依赖 dm_motor_list 和 FDCAN)；
 *                                      // 也必须在任务里而不是 main()（主栈只有 2 KB）。
 *   gimbal_cmd = Gimbal_Cmd_Create();  // 紧跟其后(它要读全局 gimbal)
 *   StartCmdTask 的循环里调 Cmd_Loop();
 *
 * 遥控器(rc)挂在 cmd 下，靠的就是下面那个 include：**谁 include 本头，谁就能直接访问
 * 全局 rc**(rc.rx / rc.ry / rc.lx / rc.sw_a / rc.link_ok / rc.monitor…)，不用再自己去
 * include libs/rc/rc.h。rc 本体是 libs/rc/rc.c 里的全局(rc_t rc;)，初始化时机没变
 * (Rc_Init() 在 Core/Src/main.c 的 USER CODE 2 里调)。
 */

#include "rc.h" /* 遥控器挂到 cmd 下：include 本头即可访问全局 rc(见上面那段) */

void Cmd_Loop(void);
