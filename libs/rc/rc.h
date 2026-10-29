#pragma once

#include <stdint.h>

#include "edge_trigger_cont.h"
#include "monitor.h" /* monitor_item，本驱动自带一个"四路齐活"看门狗 */
#include "tim.h"     /* TIM_HandleTypeDef（Rc_CountWrap 的参数）—— 它是匿名结构体，
 * 没法前向声明，只能靠 include */

/* 四路 PWM 输入由 TIM1/TIM2 的通道对捕获，逻辑通道映射见 rc_config.h。 */

#define RC_CH_CNT 4U  /* 4 路输入捕获，下标 0..3 */
#define RC_TIM_CNT 2U /* 用到的定时器个数(TIM1 / TIM2) */

/* 捕获层里的一路(一根信号线占一对定时器通道) */
typedef struct {
    /* 中断里更新 */
    volatile uint32_t rise_raw;   /* 最近一次上升沿的计数值(16 位原始值，看波形用) */
    volatile uint32_t fall_raw;   /* 最近一次下降沿的计数值(16 位原始值) */
    volatile uint32_t width_raw;  /* 高电平宽度(计数)，= 本次下降沿 - 本次上升沿 */
    volatile uint32_t period_raw; /* 周期(计数)，= 本次上升沿 - 上次上升沿 */
    volatile uint32_t rise_ext;   /* 最近一次上升沿的 32 位扩展计数值(中断内部用) */
    volatile uint32_t rise_cnt;   /* 收到的上升沿个数 */
    volatile uint32_t edge_cnt;   /* 收到的边沿总数(上升+下降) */

    /* Rc_Loop() 里更新 */
    volatile uint32_t width_us;  /* 高电平宽度，单位 µs */
    volatile uint32_t period_us; /* 周期，单位 µs */
    volatile uint32_t freq_hz;   /* 频率 = 1000000 / period_us */
    volatile float value;        /* 归一化到 -1 ~ +1(RC_CENTER_US 对应 0) */
    volatile uint8_t valid;

    /* Rc_Loop() 内部用 */
    uint32_t rise_cnt_prev;
    uint32_t last_rise_ms;
} rc_channel_t;

/* 保存捕获数据、计数时基和通道有效状态。 */
typedef struct {
    rc_channel_t ch[RC_CH_CNT];

    /* 当前在动的那一路(0..3)，-1 = 四路都没信号。按"最新收到上升沿"选。 */
    volatile int8_t active_ch;
    volatile uint8_t any_valid; /* 1 = 至少一路 valid */

    /* 位 i 标记通道 i 本轮出现新边沿；全部通道刷新后清零并刷新看门狗，仅任务上下文写入。 */
    volatile uint8_t fb_mask;

    /* 初始化后的实际时基，用调试器核对用(Rc_Init 的目标是 1 计数 = 1 µs)。
 * 下标和 rc_config.h 的 RC_TIM_ARRAY 一致：0 = TIM1，1 = TIM2。 */
    volatile uint32_t tim_clk_hz[RC_TIM_CNT]; /* 定时器的输入时钟 */
    volatile uint32_t psc[RC_TIM_CNT];        /* 写进 PSC 的分频值 */
    volatile uint32_t tick_hz[RC_TIM_CNT];    /* 计数频率，正常 = 1000000 */

    /* 16 位计数器的回绕次数，中断里累加。有了它才能把两次边沿的间隔算准：
 * 光看 16 位差值分不清"18 ms"和"18 ms + 一圈 65.5 ms"。 */
    volatile uint32_t wrap[RC_TIM_CNT];
} rc_cap_t;

/* 每轴正负方向各一个滞回触发器；Etc_Peek 读取持续状态，Etc_Get 读取并清除边沿标志。 */
typedef struct {
    Edge_trigger_continuous *rx_right; /* 右摇杆推右(rc.rx 转正) */
    Edge_trigger_continuous *rx_left;  /* 右摇杆推左(rc.rx 转负) */
    Edge_trigger_continuous *ry_up;    /* 右摇杆推上(rc.ry 转正) */
    Edge_trigger_continuous *ry_down;  /* 右摇杆推下(rc.ry 转负) */
    Edge_trigger_continuous *lx_right; /* 左摇杆推右(rc.lx 转正) */
    Edge_trigger_continuous *lx_left;  /* 左摇杆推左(rc.lx 转负) */
} rc_triggers_t;

/* 保存逻辑通道、捕获状态和链路监控。 */
typedef struct {
    /* 逻辑通道映射到 rc_config.h 配置的捕获索引。 */

    /* 摇杆(死区之后)：-1 ~ +1，中位附近(±RC_AXIS_DEADBAND)已经清零。
 * 对应那一路没信号时恒为 0(看 valid_* 区分)。 */
    volatile float rx; /* 右摇杆左右 */
    volatile float ry; /* 右摇杆上下 */
    volatile float lx; /* 左摇杆左右 */

    /* 保存应用死区前的归一化摇杆值。 */
    volatile float rx_raw; /* 右摇杆左右的归一化值，未过死区 */
    volatile float ry_raw; /* 右摇杆上下的归一化值，未过死区 */
    volatile float lx_raw; /* 左摇杆左右的归一化值，未过死区 */

    volatile float sw_a_raw; /* swA 的归一化值(取死区之前的那一步)，调试用 */
    volatile uint8_t sw_a;

    volatile uint8_t valid_rx;
    volatile uint8_t valid_ry;
    volatile uint8_t valid_lx;
    volatile uint8_t valid_swa;
    volatile uint8_t link_ok; /* 1 表示全部配置通道有效。 */

    /* 由 Rc_Loop 更新六个触发器，信号丢失时输入回中。 */
    rc_triggers_t trig;

    /* 看门狗检查各路边沿刷新；link_ok 检查脉宽、周期和信号超时。 */
    monitor_item *monitor; /* Monitor_Register() 出来的，Rc_Init() 之后有效。
 * 喂狗的记账在 rc.cap.fb_mask 里。 */

    /* 捕获层(硬件)，子结构 */
    rc_cap_t cap;
} rc_t;

/* Rc_Init 建立对象，Rc_Loop 周期更新逻辑通道。 */
extern rc_t rc;

void Rc_Init(void);

/* 任务中周期调用，完成信号换算、有效性检查、触发器更新和看门狗刷新。 */
void Rc_Loop(void);

/* 在 HAL_TIM_PeriodElapsedCallback 中调用，累计参与捕获的定时器回绕次数。 */
void Rc_CountWrap(TIM_HandleTypeDef *htim);
