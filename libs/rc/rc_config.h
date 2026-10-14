#pragma once

#include <stddef.h> /* NULL */

/* CtrBoard-H7_ALL 的遥控器接线 + 调参。
 * 只有 libs/rc/rc.c 会 include 这个文件 —— 换块板子/换接收机接线只改这里，
 * rc.h/rc.c 不用动。(同 libs/can/can_config.h 的路子。) */

#include "tim.h" /* htim1 / htim2 */

/* ==================== 定时器 ====================
 * 参与捕获的定时器。项数必须和 rc.h 的 RC_TIM_CNT 一致 —— 下面 *_CNT 是显式写
 * 出来的个数，rc.c 里拿 _Static_assert 对着它，写错编译不过。 */
#define RC_TIM_ARRAY_CNT 2U
#define RC_TIM_ARRAY     {&htim1, &htim2}

/* ==================== 每一路捕获挂哪儿 ====================
 * 每一项 {定时器下标, 通道对}，定时器下标进 RC_TIM_ARRAY。
 * 通道对 0 = CH1/CH2(量 TI1 那根线)、1 = CH3/CH4(量 TI3 那根线)；
 * 每一对里的奇数通道抓上升沿，偶数通道抓同一根线上的下降沿(CubeMX 配的)。
 *
 *     rc.cap.ch[]  {定时器, 通道对}   定时器通道             引脚
 *     [0]          {0, 0}             TIM1_CH1 / CH2         PE9
 *     [1]          {0, 1}             TIM1_CH3 / CH4         PE13
 *     [2]          {1, 0}             TIM2_CH1 / CH2         PA0
 *     [3]          {1, 1}             TIM2_CH3 / CH4         PA2
 *
 * 个数也必须和 rc.h 的 RC_CH_CNT 一致。写短了会被静默补 0，让两路映射到同一个
 * rc.cap.ch[] 互相踩 —— 全靠 rc.c 里那个 _Static_assert 挡住。 */
#define RC_CH_ARRAY_CNT 4U
#define RC_CH_ARRAY     {{0U, 0U}, {0U, 1U}, {1U, 0U}, {1U, 1U}}

/* ==================== (实体) <- (接收机通道) <- (捕获通道) ====================
 * 值就是上面那张表里的 rc.cap.ch[] 下标。实测接线，改接线只改这四个：
 *
 *     实体        接收机   捕获通道                        引脚
 *     右摇杆左右  ch1      rc.cap.ch[1]  TIM1_CH3 / CH4    PE13
 *     右摇杆上下  ch2      rc.cap.ch[0]  TIM1_CH1 / CH2    PE9
 *     左摇杆上下  ch3      —— 没接
 *     左摇杆左右  ch4      rc.cap.ch[3]  TIM2_CH3 / CH4    PA2
 *     swA         ch5      rc.cap.ch[2]  TIM2_CH1 / CH2    PA0
 *     swB         ch6      —— 没接
 *
 * 上层只跟 rc.rx / rc.ry / rc.lx / rc.sw_a 打交道(名字按实体叫)，
 * 不用管接收机通道号和捕获通道号。 */
#define RC_RX_CAP  1U /* 右摇杆左右 <- 接收机 ch1 */
#define RC_RY_CAP  0U /* 右摇杆上下 <- 接收机 ch2 */
#define RC_LX_CAP  3U /* 左摇杆左右 <- 接收机 ch4 */
#define RC_SWA_CAP 2U /* swA        <- 接收机 ch5 */

/* ==================== 时基 ==================== */
#define RC_ARR 0xFFFFU             /* 16 位计数器拉满，65.535 ms 满量程 */
#define RC_TICK_TARGET_HZ 1000000U /* 目标计数频率 1 MHz -> 1 计数 = 1 µs */

/* 合理性窗口(µs)：超出就认为这路没接信号、或者接的不是遥控器 PWM */
#define RC_WIDTH_MIN_US 100U
#define RC_WIDTH_MAX_US 30000U
#define RC_PERIOD_MIN_US 500U   /* 比 2 kHz 还快的不像遥控器 */
#define RC_PERIOD_MAX_US 65000U /* 比 15 Hz 还慢的不像遥控器(扩成 32 位后能测更大，
                                 * 这里纯粹是按遥控信号的常识卡上限) */

/* 归一化：脉宽 RC_CENTER_US 算 0，±RC_HALF_RANGE_US 算 ±1。
 * 标准舵机 1000~2000 µs 中心 1500，先按这个填；量出实际上下限后自己改
 * (实测中位 1509~1517 µs，居中时归一化值在 ±0.04 以内)。 */
#define RC_CENTER_US 1500U
#define RC_HALF_RANGE_US 500U

/* 多久没收到上升沿算丢信号(ms) */
#define RC_LOST_TIMEOUT_MS 100U

/* 摇杆死区：归一化值落在 ±RC_AXIS_DEADBAND 里就记 0(中位附近的抖动静默掉)。
 * 只清零、不重标定，所以贴着死区边界出来就是 ±RC_AXIS_DEADBAND。 */
#define RC_AXIS_DEADBAND 0.05f

/* swA 的死区：归一化值的绝对值小于它就记 0，否则记 1 */
#define RC_SW_A_DEADBAND 0.1f

/* ==================== 摇杆边沿触发器 ====================
 * 3 个比例轴(rc.rx / rc.ry / rc.lx)各配一对触发器，一共 6 个，用法照源工程
 * hal_bsp/hal/driver/remote 的 DT7 —— 那边也是每个通道 pos/neg 各一个。
 *
 * 喂进触发器的**不是浮点，是归一化值加偏置再放大后的整数**：
 *     trig = (rc.rx + RC_TRIG_BIAS) * RC_TRIG_SCALE      // -1~+1 -> 0~2000
 * 带偏置是因为 rc.rx 有正有负，直接转无符号会把负半边折成一个大正数。
 * 口径和源工程的 CHx_BIAS(1024) 是一个路子，只是中心值不同。中位落在
 * RC_TRIG_CENTER，它必须等于 RC_TRIG_BIAS * RC_TRIG_SCALE。
 *
 * 两个阈值沿用源工程的量级(它在 1024 上下取 100 / 320)，换算成归一化值就是：
 *   RC_TRIG_DEAD 回中复位线 —— 归一化 ±0.10
 *   RC_TRIG_FULL 触发线     —— 归一化 ±0.32
 * 两个之间就是滞回带，防止摇杆停在阈值附近时反复触发。
 *
 * 注意 rc.rx/ry/lx 是**已经过 RC_AXIS_DEADBAND(±0.05)** 的值，比这里的
 * RC_TRIG_DEAD(±0.10)小，所以两者不打架，不用为了触发器改去看 *_raw。 */
#define RC_TRIG_SCALE 1000.0f
#define RC_TRIG_BIAS 1.0f
#define RC_TRIG_CENTER 1000
#define RC_TRIG_DEAD 100
#define RC_TRIG_FULL 320

/* ==================== 看门狗 ====================
 * count 的单位是 Monitor_Loop() 的调用周期，现在 10 ms(100 Hz，见 freertos.c 的
 * StartMonitorTask)：10 -> 100 ms。遥控周期实测 18.4 ms，100 ms 能容忍整帧丢失；
 * 和 RC_LOST_TIMEOUT_MS 同口径。
 * Monitor_Loop() 的频率改了，这里和 DM_motor.c 的 count 都要跟着改。 */
#define RC_MONITOR_LOST_COUNT 10

/* 掉线只置标志，不自动动作：上层自己查 is_Offline(rc.monitor)。
 * 想接自己的处理函数就填这里(签名 void (*)(void*))。 */
#define RC_LOST_CALLBACK NULL
#define RC_LOST_CALLBACK_DATA NULL
