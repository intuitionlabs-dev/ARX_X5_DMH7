#pragma once

/* 机械臂命令层：把"输入"翻译成 Arm_control_data，喂给 App/gimbal_arm.c 的机械臂驱动。
 *
 * 从 ec_code/engineer-wheel-leg-v-1/APP/gimbal_cmd/gimbal_cmd.* 搬来的，只保留在本工程
 * 成立的那层逻辑。源工程那个 1700 行的 Gimbal_Cmd_Update() 骨架是：
 *
 *   通信/链路检查 → 电机掉线 → 目标同步 → Set_Ctrl_Default → Arm_Fdb_Calc
 *   → reset_time_calc → reset_j3_min_calc → 模式分派(Stop/Imagetran/Remote)
 *   → Custom_Update → 一键模式 → Plan_Running_Check → Ctrl_Src_Calc
 *   → Motor_Angle_Limit → Ctrl_Post_Process → Arm_Ref_Set → Rfid_Handle → Send_Data
 *
 * 斜的那些全删了 —— 它们分别是板间 socket/拓展板 uart、图传键鼠/自定义控制器、
 * RFID、一键模式(arm_auto)、正逆运动学(arm_kinematics_calc)、pub_sub/service，
 * 本工程一个都没有。剩下的这条管线是：
 *
 *     输入 → ctrl_src(关节控制源) → 软限位 → TD 平滑 → Gimbal.arm_control_data
 *          → (下一句) Gimbal_Update()
 *
 * **当前状态：输入侧是空的。** 遥控器的输入逻辑按用户要求先不做，所以
 * Gimbal_Cmd_Update() 每拍恒把 mode 定成 arm_stop —— 目标跟随反馈、机械臂不动，
 * 但上面这条管线每拍都在跑。将来接输入时，只需要在 Gimbal_Cmd_Update() 里改 mode
 * 的来源、在 Remote_Mode_Update() 里写摇杆→关节的映射，骨架不用动。
 *
 * 板级标定量(角度软限位)在 App/inc/gimbal_cmd_def.h，只给 gimbal_cmd.c include。
 *
 * 挂载方式：
 *   gimbal = Gimbal_Create();           // freertos.c 的 StartCmdTask 里
 *   gimbal_cmd = Gimbal_Cmd_Create();   // 紧跟其后(它要读全局 gimbal)
 *   Cmd_Loop() 里 Gimbal_Cmd_Update(gimbal_cmd) 再 Gimbal_Update(gimbal);
 *   —— 顺序不能反，理由见 App/cmd.c。
 */

#include <stdint.h>

#include "robot_def.h"   /* Arm_mode / Arm_control_data / Arm_feedback_data / JOINTn */
#include "adrc.h"        /* TD_t —— 关节角跟踪微分器 */
#include "gimbal_arm.h"  /* Gimbal —— 机械臂驱动本体 */

/* 机械臂控制源类型(源工程 robot_def.h 的 Arm_ctrl_type)。
 * 笛卡尔空间控制和自定义控制器腕点控制都要 arm_kinematics_calc(逆运动学)，本工程
 * 没移植，所以那两项**只留名字、注释掉**：Ctrl_Src_Calc() 里的 case 也一起注释着，
 * 将来把 arm_kinematics_calc 搬进来时按原名打开即可。 */
typedef enum {
    arm_ctrl_joint = 0,  // 关节空间控制
    /* arm_ctrl_cartesian,          笛卡尔空间控制 —— 不做 */
    /* arm_ctrl_custom_selfsaving,  自定义控制器腕点控制 —— 不做 */
} Arm_ctrl_type;

/* 机械臂控制数据源整合包(源工程 Arm_ctrl_data_src)。
 * 同样的理由，cartesian_src / custom_selfsaving_src 两项连字段一起删了 —— 留着字段
 * 会让人以为可以照着填。 */
typedef struct {
    struct Joint_ctrl_data_src_t {
        /* 目标值 */
        float joint_angle[ARM_JOINT_NUM];  // 关节角度目标值，deg
    } joint_src;  // 关节空间控制数据源
    /* struct Cartesian_ctrl_data_src_t { ... } cartesian_src; */
    /* struct Custom_selfsaving_ctrl_data_src_t { ... } custom_selfsaving_src; */
    float joint_angle_solved[ARM_JOINT_NUM];  // 最终解算关节角目标值，deg
} Arm_ctrl_data_src;

typedef struct Gimbal_cmd_t {
    Gimbal* gimbal;  // 机械臂驱动本体；NULL 则整层不干活

    /* 控制源 */
    Arm_ctrl_type ctrl_type;
    Arm_ctrl_data_src ctrl_src;

    /* 模式(源工程是 Robot_mode，那边还带底盘/裁判系统状态；本工程只需要 arm_stop/arm_run) */
    Arm_mode mode;

    /* 关节角跟踪微分器：Ctrl_Post_Process() 把解算出来的目标角过一遍 TD，
     * Arm_Ref_Set() 取它的 v1/v2 当位置/速度，免得阶跃目标直接砸到位置环上。 */
    TD_t joint_angle_td[ARM_JOINT_NUM];
} Gimbal_cmd;

/* 命令层对象，Gimbal_Cmd_Create() 之后有效；全局 gimbal 为 NULL 时它也是 NULL。 */
extern Gimbal_cmd* gimbal_cmd;

/* 建对象 + 初始化 TD。必须在 Gimbal_Create() 之后调，而且**要在任务里**调
 * (和 Gimbal_Create() 同一个理由：任务栈比主栈大)。 */
Gimbal_cmd* Gimbal_Cmd_Create(void);

/* 每拍调一次(CmdTask 500 Hz / 2 ms)。结果是**直接写进** obj->gimbal->arm_control_data
 * 的；调用方随后自己调 Gimbal_Update() 就会读到。obj 为 NULL 时直接返回。 */
void Gimbal_Cmd_Update(Gimbal_cmd* obj);
