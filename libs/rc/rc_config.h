#pragma once

#include <stddef.h>

/* 配置遥控器定时器、捕获映射和信号判据。 */

#include "tim.h" /* htim1 / htim2 */

/* 定时器表项数须与 RC_TIM_CNT 一致。 */
#define RC_TIM_ARRAY_CNT 2U
#define RC_TIM_ARRAY     {&htim1, &htim2}

/* 每项为定时器索引及通道对；对 0 使用 CH1/CH2，对 1 使用 CH3/CH4，奇数通道捕获上升沿。 */
#define RC_CH_ARRAY_CNT 4U
#define RC_CH_ARRAY     {{0U, 0U}, {0U, 1U}, {1U, 0U}, {1U, 1U}}

/* 逻辑通道映射到捕获索引：rx→1，ry→0，lx→3，swA→2。 */
#define RC_RX_CAP  1U /* 右摇杆左右 <- 接收机 ch1 */
#define RC_RY_CAP  0U /* 右摇杆上下 <- 接收机 ch2 */
#define RC_LX_CAP  3U /* 左摇杆左右 <- 接收机 ch4 */
#define RC_SWA_CAP 2U /* swA        <- 接收机 ch5 */

/* 时基 */
#define RC_ARR 0xFFFFU             /* 16 位计数器拉满，65.535 ms 满量程 */
#define RC_TICK_TARGET_HZ 1000000U /* 目标计数频率 1 MHz -> 1 计数 = 1 µs */

/* 合理性窗口(µs)：超出就认为这路没接信号、或者接的不是遥控器 PWM */
#define RC_WIDTH_MIN_US 100U
#define RC_WIDTH_MAX_US 30000U
#define RC_PERIOD_MIN_US 500U   /* 比 2 kHz 还快的不像遥控器 */
#define RC_PERIOD_MAX_US 65000U /* 遥控信号周期的允许上限。 */

/* 脉宽中心映射为 0，中心两侧 RC_HALF_RANGE_US 分别映射为 ±1。 */
#define RC_CENTER_US 1500U
#define RC_HALF_RANGE_US 500U

/* 多久没收到上升沿算丢信号(ms) */
#define RC_LOST_TIMEOUT_MS 100U

/* 死区内输出为 0，死区外保留归一化幅值。 */
#define RC_AXIS_DEADBAND 0.05f

/* swA 的死区：归一化值的绝对值小于它就记 0，否则记 1 */
#define RC_SW_A_DEADBAND 0.1f

/* 触发器输入为 (value+BIAS)×SCALE，DEAD 为复位阈值，FULL 为触发阈值。 */
#define RC_TRIG_SCALE 1000.0f
#define RC_TRIG_BIAS 1.0f
#define RC_TRIG_CENTER 1000
#define RC_TRIG_DEAD 100
#define RC_TRIG_FULL 320

/* 看门狗计数单位为 Monitor_Loop 周期；100 Hz 下计数 10 对应 100 ms。 */
#define RC_MONITOR_LOST_COUNT 10

/* 配置掉线回调，NULL 表示仅更新监控标志。 */
#define RC_LOST_CALLBACK NULL
#define RC_LOST_CALLBACK_DATA NULL
