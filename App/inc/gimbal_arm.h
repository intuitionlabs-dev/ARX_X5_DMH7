#pragma once

/* 六轴机械臂 + 夹爪(7 个电机)：j1~j3 是 ENCOS，j4~j6 和夹爪是达妙。
 *
 * 从 ec_code/engineer-wheel-leg-v-1/APP/gimbal/ 搬过来的，和当前框架的适配：
 *
 *   - **指令来源**：源工程走 pub_sub(arm_control_sub->getdata())，本工程没有这个
 *     框架，所以改成 Gimbal_Set_Control() 直接喂 —— 调用方每拍调一次。
 *     超过 ARM_CMD_TIMEOUT_MS 没新指令，Gimbal_Update() 会把 arm_mode 打回 arm_stop
 *     (电机失能)，这就是源工程"订阅队列空就 stop"那个语义。
 *   - **服务(service.h)**：电机重初始化服务没接入，Arm_Reinit_Service_Handler 和相关
 *     字段都注释掉了。
 *   - **动力学(arm_serial_link)**：Joint_Feedforward_Calc() 整个注释掉了，所以
 *     tau_comp[] 恒为 0。位置/速度/摩擦前馈不受影响。
 *   - **打印**：本工程 printf 是坏的(__io_putchar 没定义)，PRINT_ANGLE 关掉了。
 *   - **j1~j3 的电机驱动**：从达妙换成 ENCOS(ENCOS V1.20EAP，就是 ARX X5 的
 *     MotorType4)。配置散在 gimbal_arm.c 的 Gimbal_Create() 里，参数来源见那里。
 *     注意 ENCOS 的 `enable` 只是标志位，真正的启停帧由 ENCOSmotor_Calc_Send()
 *     在 MotorTask 里发；别在这里调 ENCOSmotor_Enable()/Disable()，理由见
 *     Gimbal_Update() 里的注释。
 *
 * 板级标定量(电机方向、减速比、零点、前馈系数)在 App/inc/gimbal_arm_def.h。
 *
 * 挂载方式：
 *   gimbal = Gimbal_Create();   // 必须在**任务里**调，别放 main() 的 USER CODE 2！
 *                               // 它栈上要 ~4 KB(kjoint_controller_config[7])，
 *                               // 而主栈 _Min_Stack_Size 只有 2 KB。
 *                               // 现在挂在 StartCmdTask 进循环之前。
 *                               // 还要在 BSP_CAN_Init()/DMmotor_Driver_Init()/
 *                               // ENCOSmotor_Driver_Init() 之后。
 *   Cmd_Loop() 里调 Gimbal_Update(gimbal);   // CmdTask 500 Hz / 2 ms
 */

#include <stdint.h>

#include "pid.h"  /* PID_Config */

#include "robot_def.h"  /* Arm_mode / Arm_*_data / JOINTn / ARM_*_NUM */
/* pub_sub.h / service.h 本工程没有，相关成员见下面的注释 */
/* BSP 层 */
/* HAL 层 */
#include "DM_motor.h"
#include "ENCOS_motor.h"

/* 多久没收到新指令就把 arm_mode 打回 arm_stop(ms)。
 * 和 CmdTask 的周期(2 ms)比足够宽，正常丢几帧也不会误停。 */
#define ARM_CMD_TIMEOUT_MS 100U

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
    float tau_comp[ARM_JOINT_NUM];   // 前馈计算得到的无阻尼力矩补偿值(动力学未接入，恒 0)

    /* 模块间通信(源工程走 pub_sub，本工程未接入) */
    // Publisher* arm_feedback_pub;
    Arm_feedback_data arm_feedback_data;  // 回传云台cmd的数据
    // Subscriber* arm_control_sub;
    Arm_control_data arm_control_data;  // 接收到的指令，由 Gimbal_Set_Control() 写
    // Server* reinit_server;           // 重新初始化服务

    /* Gimbal_Set_Control() 上次被调用的时刻，用来判指令超时(HAL_GetTick) */
    uint32_t last_cmd_ms;
} Gimbal;

/* 机械臂对象，Gimbal_Create() 之后有效。DMmotor_Create() 失败时是 NULL。 */
extern Gimbal* gimbal;

Gimbal* Gimbal_Create(void);
void Gimbal_Update(Gimbal* obj);

/* 喂指令：每拍调一次，data->mode 决定启动/停止。
 * 停手超过 ARM_CMD_TIMEOUT_MS 就自动回 arm_stop。 */
void Gimbal_Set_Control(Gimbal* obj, Arm_control_data* data);
