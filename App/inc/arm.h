#pragma once

#include <stdint.h>

#include "robot_def.h"

#include "gimbal_arm.h"
#include "gimbal_cmd.h"
#include "robot_state.h"



/* 保存单臂控制对象、通信队列和业务状态。 */
typedef struct Arm {

    /* 机械臂索引与电机、命令控制对象的关联。 */
    uint8_t id;

    Gimbal* gimbal;
    Gimbal_cmd* gimbal_cmd;

    /* 当前模式、状态迁移和初始化步骤。 */
    Arm_mode arm_mode;

    Robot_state state;
    Robot_state state_last;
    Robot_state init_entry;
    Arm_mode arm_mode_last;
    Robot_ctrl_cmd ctrl;
    uint8_t handshake_ok;
    uint32_t last_rx_ms;
    uint32_t fb_last_ms;
    uint8_t init_step;
    uint8_t reset_step;
    uint8_t zero_seen_busy;
    uint32_t zero_start_ms;

    /* 中断写入、命令任务消费的单臂收包环。 */
    Robot_rx_slot rx_slot[ROBOT_RX_SLOT_NUM];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;

    /* 供调试器读取的状态、反馈和命令统计。 */
    volatile uint32_t transition_cnt;
    volatile uint32_t fb_cnt;
    volatile uint32_t handshake_cnt;
    volatile uint32_t handshake_reject_cnt;
    volatile uint32_t init_reject_cnt;
    volatile uint32_t init_abort_cnt;
    volatile uint32_t reset_reject_cnt;
    volatile uint32_t reset_abort_cnt;
    volatile uint32_t ctrl_frame_cnt;
    volatile uint8_t handshake_ok_obs;

    uint32_t hs_last_ms;
} Arm;

/* 通过机械臂索引访问单臂状态。 */
extern Arm g_arm[ARM_NUM];
