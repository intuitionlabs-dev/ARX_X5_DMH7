#pragma once

/* 六轴机械臂和夹爪；CmdTask 每 2 ms 调用 Gimbal_Update()。
 * 运行模式使用 KDL 逆动力学计算关节前馈，纯重力模式计算静态支撑力矩。
 * 板级参数见 gimbal_arm_def.h。
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

/* 前三个关节是 ENCOS，后面 j4~j6 和夹爪是达妙。
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
     * **per 台**：两台臂共用的话后建的那台看到它已经是 1，
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
    float tau_comp[ARM_JOINT_NUM];  // 关节逆动力学力矩，N·m

    Arm_feedback_data arm_feedback_data;  // 回传云台cmd的数据
    Arm_control_data arm_control_data;  // 指令，由命令层(App/gimbal_cmd.c)每拍直接写

    /* 「设零点」这条独立维护路径的全部状态(App/arm_zero.c 的状态机，**per 台**)。
     * 放这儿而不是当全局：两台臂各有各的设零进度，共用一份的话 A 台设到一半、B 台一触发
     * 就把它的进度整个覆盖。触发/结果都在这个成员里，读法见 App/inc/arm_zero.h。 */
    Arm_zero zero;
    int dynamics_status;  // 动力学求解状态，0 表示成功
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
