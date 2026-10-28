#pragma once

/* 每台臂独立的达妙设零状态机，MotorTask 每毫秒推进。
 * 默认参与六关节，participate_override 可显式选择电机。
 * 设零期间暂停本臂常规发送；先失能，再逐台设零并用新反馈复核。
 * 失能后无反馈的电机会被短暂唤醒，复核后再次失能。
 */

#include <stdint.h>

#include "robot_def.h" /* ARM_MOTOR_NUM / JOINT1..6 / GRIPPER */

/* Gimbal 的完整定义在 App/inc/gimbal_arm.h，但那个头**反过来要 include 本头**(它要把
 * 下面的 Arm_zero 当成员嵌进去)，互相包含只会让其中一个拿到不完整的类型。本头里 Gimbal
 * 只出现在函数参数和指针里，前向声明足够。 */
typedef struct Gimbal_t Gimbal;

/* 状态机状态。JLink 读的就是这个值(声明成 uint8_t，一个字节)。 */
typedef enum {
    ARM_ZERO_IDLE = 0,     /* 空闲 */
    ARM_ZERO_DISABLE,      /* 瞬态：失能帧已发，正等确认(本模块里没有停留点) */
    ARM_ZERO_WAIT_DISABLE, /* ① 等所有参与电机的失能确认 */
    ARM_ZERO_SEND,         /* ② 给第 s_index 台发设零帧 */
    ARM_ZERO_CHECK,        /* ③ 等新反馈判成功，不成功则重发 */
    ARM_ZERO_DONE,         /* 全部参与电机都没报错，结束 */
    ARM_ZERO_TIMEOUT,      /* 有台没搞定(或失能没确认)，结束 */
} Arm_zero_state;

/* 每台的结果码，写进 Arm_zero::result[] */
typedef enum {
    ARM_ZERO_RES_NONE = 0,        /* 没走到 */
    ARM_ZERO_RES_UNCONFIRMED = 1, /* 达妙：帧发了、位置本来就在容差内，无法证明写进去了 */
    ARM_ZERO_RES_OK = 2,          /* 位置被拉到 0 */
    ARM_ZERO_RES_FAIL = 3,        /* 电机对象不可用或复核失败 */
    ARM_ZERO_RES_TIMEOUT = 4,     /* 反复重发到单台超时 */
    ARM_ZERO_RES_SKIP = 5,        /* 未参与本轮 */
} Arm_zero_result;

/* ==================== 一轮设零的全部状态 ====================
 *
 * 必须按台分开 —— 两台臂各有各的设零进度，共用一份的话 A 台设到一半、B 台一触发就把它的
 * 进度整个覆盖。整块收在这个结构体里、挂在 `Gimbal` 上(见 App/inc/gimbal_arm.h)，
 * **一台臂一个**。
 *
 * JLink 的读法：
 *     g_arm[0].gimbal->zero.state       // Arm_zero_state
 *     g_arm[0].gimbal->zero.result[6]   // 夹爪的结果码
 * 触发一轮：写 `g_arm[0].gimbal->zero.request = 1`。
 *
 * 字段分两段：前一半是"外面写/外面读"的入口与反馈，后一半是本模块自己的跨拍状态。 */
typedef struct {
    /* ---- 入口 ---- */
    /* 写 1 触发一轮；状态机在**非忙**状态下会消费它(清 0)并开跑。 */
    volatile uint8_t request;

    /* ---- 反馈(JLink 读这几个) ---- */
    volatile uint8_t state;                 /* Arm_zero_state */
    volatile uint8_t result[ARM_MOTOR_NUM]; /* Arm_zero_result，每台一个 */
    volatile uint8_t retry[ARM_MOTOR_NUM];  /* 每台重发了几次 */
    volatile uint32_t elapsed_ms;           /* 本轮耗时 */
    /* 失能确认是否拿到过(达妙反馈 errcode==0)。超时时看它就知道是卡在失能还是卡在设零。 */
    volatile uint8_t disable_confirmed;
    /* 哪些电机在"检查设零结果"时是靠先发使能帧(0xFC)唤醒才读到位位置的，bit = 电机索引。
     * 实测 j5 失能后不再发反馈，会落到这一位；全 0 说明参与电机失能后仍有反馈。 */
    volatile uint8_t wake_mask;

    /* 参与集覆盖：非 0 = 这一轮**只让这些位参与**(bit = Arm_Motor_Index)，0 = 照旧(整臂)。
     * 一键夹爪找零用它把一轮设零缩到夹爪一台(见 App/arm_auto.c 的 case 3)，用完整的设零
     * 机制(失能确认 / 唤醒读回 / 重发 / 喂狗)但不动 j1~j6。
     * **一次性**：触发那一轮一开始(start())就锁进参与集并把它清回 0，和 request 被消费
     * 一个路子 —— 所以不会留下"以后每次设零都只设夹爪"的雷。 */
    uint8_t participate_override;

    /* ---- 内部跨拍状态(静态零初始化即可) ---- */
    uint32_t start_ms; /* 本轮开始时刻 */
    uint32_t phase_ms; /* 当前阶段(等失能确认 / 当前这一台)的起点 */
    uint32_t send_ms;  /* 当前这一台最近一次"发出的那一帧"的时刻 */
    uint32_t retry_ms; /* 最近一次重发失能帧的时刻 */
    uint8_t index;     /* 正在处理的电机，ARM_MOTOR_NUM 表示跑完了 */
    uint8_t dm_woken;  /* 当前这台达妙是不是已经被使能帧唤醒过(读完要关回去) */
    float before_pos;  /* 发设零帧之前的位置：分辨"真被拉到 0"还是"本来就在 0" */
    /* 本轮参与集在 start() 中锁存，bit = Arm_Motor_Index。 */
    uint8_t participate_mask;

    /* 保留诊断字段；达妙使用位置复核，此值保持 0。 */
    uint8_t ack_missing_mask;
} Arm_zero;

/* ---- API ---- */
/* 状态机步进，在 MotorTask 里每拍调一次(1 kHz)，**每台臂各调一次**。
 * g 为 NULL(驱动没建起来)时直接返回：请求留着不消费，下一拍再来。 */
void Arm_Zero_Loop(Gimbal* g);

/* 设零进行中返回 1：此时**不许发任何常规控制帧**(它那条总线的发送循环要跳过)。
 * **按台判**：一台臂在设零，另一台的帧照发(两台各走一路 CAN，互不影响)。
 * g 为 NULL 时返回 0(没有驱动就没有帧要发)。 */
uint8_t Arm_Zero_Busy(Gimbal* g);
