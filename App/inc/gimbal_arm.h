#pragma once

/* 六轴机械臂 + 夹爪(7 个电机)：j1~j3 是 ENCOS，j4~j6 和夹爪是达妙。
 *
 * 从 ec_code/engineer-wheel-leg-v-1/APP/gimbal/ 搬过来的，和当前框架的适配：
 *
 *   - **指令来源**：源工程走 pub_sub(arm_control_sub->getdata())，订阅队列空(len == -1)
 *     就把 arm_mode 打回 arm_stop(电机失能)。本工程没有这个框架，也**不再有独立的
 *     指令源**：命令层 App/gimbal_cmd.c 每拍直接写 obj->arm_control_data，
 *     Gimbal_Update() 直接读 data.mode 决定使能/失能，两边在同一个任务里、顺序执行，
 *     中间没有队列也没有"喂指令"的入口。源工程"掉线就停"的两条判据在本工程换成：
 *     输入链路(遥控器)由命令层自己判(未做)，电机掉线由 Motor_Lost_Handle() 判。
 *   - **服务(service.h)**：电机重初始化服务没接入，Arm_Reinit_Service_Handler 和相关
 *     字段都注释掉了。
 *   - **动力学(arm_serial_link)**：整机 Newton-Euler 没移植，那一块仍然是注释状态。
 *     但**静态重力项接上了**：Joint_Gravity_Comp_Calc() 每拍用反馈关节角调
 *     libs/gravity_compensation 的 x5_gravity_compute()，结果填 tau_comp[]。
 *     型号/输出模式/开关在 gimbal_arm_def.h 的"重力补偿"那一段。
 *     所以 tau_comp[] 不再是恒 0，位置/速度/摩擦前馈照旧。
 *   - **打印**：本工程 printf 是坏的(__io_putchar 没定义)，PRINT_ANGLE 关掉了。
 *   - **j1~j3 的电机驱动**：从达妙换成 ENCOS(ENCOS V1.20EAP，就是 ARX X5 的
 *     MotorType4)。配置散在 gimbal_arm.c 的 Gimbal_Create() 里，参数来源见那里。
 *     注意 ENCOS 的 `enable` 只是标志位（驱动**没有** Enable 接口，见
 *     libs/encos_motor/ENCOS_motor.h），真正的启停帧由 ENCOSmotor_Send_Step()
 *     在 MotorTask 里发；别在这里调 ENCOSmotor_Disable()，理由见
 *     Gimbal_Update() 里的注释。
 *
 * 板级标定量(电机方向、减速比、零点、前馈系数)在 App/inc/gimbal_arm_def.h。
 *
 * 挂载方式：
 *   g_arm[ARM_A].gimbal = Gimbal_Create(ARM_A);   // 必须在**任务里**调，别放 main() 的
 *   g_arm[ARM_B].gimbal = Gimbal_Create(ARM_B);   // USER CODE 2！它栈上要 ~4 KB
 *                               // (joint_controller_config[7])，而主栈
 *                               // _Min_Stack_Size 只有 2 KB。两台**顺序**建，峰值还是
 *                               // 4 KB（第一台的栈帧已经还了）。现在挂在 StartCmdTask
 *                               // 进循环之前。还要在 BSP_CAN_Init()/
 *                               // DMmotor_Driver_Init()/ENCOSmotor_Driver_Init() 之后。
 *   Cmd_Loop() 里逐台 Gimbal_Update(arm->gimbal);   // CmdTask 500 Hz / 2 ms
 */

#include <stdint.h>

#include "pid.h"  /* PID_Config */

#include "robot_def.h"  /* Arm_mode / Arm_*_data / JOINTn / ARM_*_NUM */
/* pub_sub.h / service.h 本工程没有，相关成员见下面的注释 */
/* BSP 层 */
/* HAL 层 */
#include "DM_motor.h"
#include "ENCOS_motor.h"
#include "arm_zero.h" /* Arm_zero —— 设零状态机的全部跨拍状态(per 台) */

/* 前三个关节是 ENCOS，后面 j4~j6 和夹爪还是达妙。
 * 两个驱动的电机类型不一样，数组下标也就分开算。 */
#define ARM_ENCOS_JOINT_NUM 3

typedef struct Gimbal_t {
    /* 外设 */
    ENCOS_motor* joint1;  // ENCOS(力位混合控制)
    ENCOS_motor* joint2;  // ENCOS
    ENCOS_motor* joint3;  // ENCOS
    DM_motor* joint4;   // 4310
    DM_motor* joint5;   // 4310
    DM_motor* joint6;   // DM2325
    DM_motor* gripper;  // 4310

    /* 这台臂走哪路 CAN。**就是臂号**（见 robot_def.h 的 Arm_Index / 帧里的 arm_id）：
     * Gimbal_Create(can_index) 灌进来，7 台电机的 config.bsp_can_index 全用它。
     * 两台臂完全同构，唯一的区别就是这个数 —— 也是达妙反馈路由必须按它过滤的原因
     * （两台臂的达妙 CAN_ID 都是 5/6/7/8、MST_ID 都是 0，不看总线就串台）。 */
    uint8_t can_index;

    /* 电机控制器 / 监视器数组，便于缩减代码冗余。
     * **per 台**：两台臂各一套，放全局数组的话第二台建完就把第一台的覆盖了。 */
    controller* motor_controller_list[ARM_MOTOR_NUM];
    monitor_item* motor_monitor_list[ARM_MOTOR_NUM];

    /* 「首次上电自动初始化」跑过没有。
     * **per 台**：原来是个函数内 static，两台臂共用的话后建的那台看到它已经是 1，
     * 永远不跑 Arm_Auto_Init()（motor_pos_init[]/motor_inited[] 就一直是 0）。 */
    uint8_t first_init_flag;

    PID_Config pid_cfgs[ARM_MOTOR_NUM][2];  // pid参数 //位置环0和速度环1参数
    float pid_mit_kpkd[ARM_MOTOR_NUM][2];   // 达妙 MIT 的 kp/kd —— 只用 j4/j5/j6/gripper 那几行
    /* ENCOS 内置的是"力位混合控制"(kp/kd 和 位置/速度/力矩 同帧下发)，和达妙的 MIT
     * 不是一回事，所以单独一组，别和上面的 pid_mit_kpkd 混用。值见 gimbal_arm.c 的
     * pid_init()，来源是 ARX X5 SDK 的 joint_parameters.position_kp/kd。 */
    float encos_kpkd[ARM_ENCOS_JOINT_NUM][2];  // [kp, kd]

    /* 初始化量 */
    float motor_pos_init[ARM_JOINT_NUM];  // 电机初始角度，与motor_controller->fdb_position是同等地位，deg,公式:motor_angle = MOTOR_DIR*(motor_controller->fdb_position - motor_pos_init) + motor_angle_init

    /* 状态量 */
    Arm_mode arm_mode;
    uint8_t ik_status;  // 逆解算状态，1为解算成功，0为失败(未实现)
    uint8_t is_load;    // 机械臂是否夹持，决定重力补偿的参数TODO:

    /* 控制量 */
    float damp_comp[ARM_JOINT_NUM];  // 前馈计算得到的阻尼补偿值
    /* 前馈计算得到的无阻尼力矩补偿值, N·m, 关节系(joint1..joint6)。
     * Joint_Gravity_Comp_Calc() 每拍写一次 = X5 静态重力力矩；整机动力学(惯量/
     * 科里奥利)没接入。arm_stop 下照样算，只是 Set_Motor_ref() 不会用它。 */
    float tau_comp[ARM_JOINT_NUM];

    Arm_feedback_data arm_feedback_data;  // 回传云台cmd的数据
    Arm_control_data arm_control_data;  // 指令，由命令层(App/gimbal_cmd.c)每拍直接写

    /* 「设零点」这条独立维护路径的全部状态(App/arm_zero.c 的状态机，**per 台**)。
     * 放这儿而不是当全局：两台臂各有各的设零进度，共用一份的话 A 台设到一半、B 台一触发
     * 就把它的进度整个覆盖。触发/结果都在这个成员里，读法见 App/inc/arm_zero.h。 */
    Arm_zero zero;
} Gimbal;

/* 建一台臂。**一台臂建一次，两台臂各建一次**：
 *
 *     g_arm[ARM_A].gimbal = Gimbal_Create(ARM_A);   // CAN1
 *     g_arm[ARM_B].gimbal = Gimbal_Create(ARM_B);   // CAN2
 *
 * can_index 就是臂号、也是 CAN 总线索引（见 robot_def.h 的 Arm_Index）。它决定这台臂
 * 的 7 台电机挂在哪条总线上；两条总线上的电机 ID 完全一样（ENCOS 1/2/4、达妙 5/6/7/8），
 * 靠总线号区分。
 *
 * **必须在任务里调，而且要在 BSP_CAN_Init()/DMmotor_Driver_Init()/
 * ENCOSmotor_Driver_Init() 之后**。返回 NULL 表示电机没建起来（配置不合法的典型是
 * ENCOS 的 tx_id != rx_id、或同一总线上同一个 rx_id 建了两次）。 */
Gimbal* Gimbal_Create(uint8_t can_index);
void Gimbal_Update(Gimbal* obj);
