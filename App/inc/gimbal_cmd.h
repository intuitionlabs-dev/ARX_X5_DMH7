#pragma once

#include <stdint.h>

#include "robot_def.h"
#include "gimbal_arm.h"

#include "ruckig_c.h"
#include "plan_group.h"

/* 保存输入关节目标和限位后的求解目标。 */
typedef struct {
    /* 保存控制来源提供的关节位置，单位 deg。 */
    struct Joint_ctrl_data_src_t {

        float joint_angle[ARM_JOINT_NUM];
    } joint_src;

    float joint_angle_solved[ARM_JOINT_NUM];
} Arm_ctrl_data_src;

/* 保存单关节连续位置参考；状态与单精度求解器一致，避免额外舍入触发重规划。 */
typedef struct {
    ruckig_c_context* planner;
    ruckig_c_state state;
    int status;
    uint8_t initialized;
} Joint_ruckig_ref;

/* 保存单臂命令处理、自动规划和初始化上下文。 */
typedef struct Gimbal_cmd_t {
    Gimbal* gimbal;

    Arm* arm;

    Arm_ctrl_data_src ctrl_src;

    Arm_mode mode;

    /* 记录自动模式切换和动作步骤。 */
    Arm_auto_mode auto_mode;
    Arm_auto_mode last_auto_mode;
    uint8_t auto_step;

    /* 保存自动规划句柄、时间基准及运动导数。 */
    Plan_group_temp temp_group;
    uint32_t reset_start_ms;
    uint8_t auto_failed;
    float plan_joint_vel[ARM_JOINT_NUM];
    float plan_joint_acc[ARM_JOINT_NUM];
    uint8_t plan_running;

    /* 后处理器 */
    Joint_ruckig_ref joint_ruckig_ref[ARM_JOINT_NUM];
    /* 后处理关节参考，单位 deg、deg/s、deg/s²。 */
    float post_processed_joint_angle[ARM_JOINT_NUM];
    float post_processed_joint_vel[ARM_JOINT_NUM];
    float post_processed_joint_acc[ARM_JOINT_NUM];

    /* 保存夹爪搜索、停滞检测、回退和稳定计时状态。 */
    struct {
        uint8_t step, home_active;
        float home_target_deg, home_torque;
        uint32_t start_ms, window_ms, stall_windows;
        float window_position, backoff_target;
    } initialization;
    /* 保存机型初始化回调的自定义状态。 */
    void* initialization_context;

} Gimbal_cmd;

/* 为指定机械臂创建命令处理对象。 */
Gimbal_cmd* Gimbal_Cmd_Create(Arm* arm);

/* 推进输入、轨迹、限位和参考值发布。 */
void Gimbal_Cmd_Update(Gimbal_cmd* obj);
