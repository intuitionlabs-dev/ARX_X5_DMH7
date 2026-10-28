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
#include "arm_zero.h" /* Arm_zero —— 设零状态机的全部跨拍状态(per 台) */

/* 每台臂使用七个达妙 MIT 电机，CAN 总线分别为 CAN1 / CAN2。 */

typedef struct Gimbal_t {
    /* 外设 */
    DM_motor* joint1;
    DM_motor* joint2;
    DM_motor* joint3;
    DM_motor* joint4;
    DM_motor* joint5;
    DM_motor* joint6;
    DM_motor* gripper;

    /* 这台臂走哪路 CAN。**就是臂号**（见 robot_def.h 的 Arm_Index / 帧里的 arm_id）：
     * Gimbal_Create(can_index) 灌进来，7 台电机的 config.bsp_can_index 全用它。
     * 两台臂完全同构，唯一的区别就是这个数 —— 也是达妙反馈路由必须按它过滤的原因
     * （两台臂的 TX ID 均为 1～7、RX ID 均为 0x11～0x17）。 */
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
    float pid_mit_kpkd[ARM_MOTOR_NUM][2];  // 七个电机的 MIT [kp, kd]

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

/* 在 BSP_CAN_Init()/DMmotor_Driver_Init() 后按 CAN 索引创建一台臂。
 * 两台臂的 ID 和参数相同，反馈按 CAN 总线、仲裁 ID 和载荷电机 ID 路由。 */
Gimbal* Gimbal_Create(uint8_t can_index);
void Gimbal_Update(Gimbal* obj);
/* 原始输出轴位置，rad；应用反馈与设零复核共用。 */
float Gimbal_Motor_Position_Rad(const DM_motor* motor);
