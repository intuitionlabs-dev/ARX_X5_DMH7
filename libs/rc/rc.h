#pragma once

#include <stdint.h>

#include "edge_trigger_cont.h" /* Edge_trigger_continuous，rc.trig 下面那 6 个摇杆边沿触发器 */
#include "monitor.h" /* monitor_item，本驱动自带一个"四路齐活"看门狗 */
#include "tim.h"     /* TIM_HandleTypeDef（Rc_CountWrap 的参数）—— 它是匿名结构体，
                      * 没法前向声明，只能靠 include */

/* 遥控器 PWM 输入捕获。
 *
 * 硬件：CubeMX 把 TIM1/TIM2 的 CH1/CH2 和 CH3/CH4 配成了输入捕获，
 * 每个引脚用一对通道量：CH1/CH3 抓上升沿(DIRECTTI)，CH2/CH4 抓同一根线上的
 * 下降沿(INDIRECTTI)。所以一根信号线占掉两个通道，一共 4 路：
 *
 *     rc.cap.ch[0]   TIM1_CH1 / CH2   PE9
 *     rc.cap.ch[1]   TIM1_CH3 / CH4   PE13
 *     rc.cap.ch[2]   TIM2_CH1 / CH2   PA0
 *     rc.cap.ch[3]   TIM2_CH3 / CH4   PA2
 *
 * 哪根线接哪个定时器/引脚、哪一路对应哪个摇杆，都写在 libs/rc/rc_config.h 里
 * (驱动的私事，用这个驱动的人不用管，直接看下面 rc 的逻辑层成员)。
 *
 * 四路同时在量，所以：
 *   - 不用知道接收机接在哪一路，看 rc.cap.active_ch(最新收到信号的那一路)，
 *     或者一路一路看 rc.cap.ch[i].rise_cnt 是不是在涨；
 *   - 不用知道 PWM 频率，rc.cap.ch[i].freq_hz / period_us 是量出来的。
 *
 * 提醒：时基是 1 个计数 = 1 µs、ARR = 0xFFFF(满量程 65.535 ms)，.ioc 里已经配成
 * Prescaler = 239 / Counter Period = 65535；Rc_Init() 里还会再写一遍同样的值，
 * 为的是时钟树改了它自己跟得上。另外更新中断(UIE)在 CubeMX 里没得配，
 * 只能由 Rc_Init() 打开。细节见 libs/rc/rc.c 顶上的注释。
 *
 * 挂载方式：
 *   Rc_Init();  // main.c 的 USER CODE BEGIN 2，必须在 MX_TIM1_Init()/MX_TIM2_Init()
 *               // 和 Monitor_Init() 之后、调度器启动之前
 *   StartCmdTask 的循环里调 Cmd_Loop();(Cmd_Loop() 里已经带了 Rc_Loop())
 *   main.c 的 HAL_TIM_PeriodElapsedCallback() 里(Callback 1 区)调 Rc_CountWrap(htim);
 *               // TIM1/TIM2 的更新中断，用来数 16 位计数器的回绕
 */

#define RC_CH_CNT 4U  /* 4 路输入捕获，下标 0..3 */
#define RC_TIM_CNT 2U /* 用到的定时器个数(TIM1 / TIM2) */

/* 捕获层里的一路(一根信号线占一对定时器通道) */
typedef struct {
    /* ---- 中断里更新 ---- */
    volatile uint32_t rise_raw;   /* 最近一次上升沿的计数值(16 位原始值，看波形用) */
    volatile uint32_t fall_raw;   /* 最近一次下降沿的计数值(16 位原始值) */
    volatile uint32_t width_raw;  /* 高电平宽度(计数)，= 本次下降沿 - 本次上升沿 */
    volatile uint32_t period_raw; /* 周期(计数)，= 本次上升沿 - 上次上升沿 */
    volatile uint32_t rise_ext;   /* 最近一次上升沿的 32 位扩展计数值(中断内部用) */
    volatile uint32_t rise_cnt;   /* 收到的上升沿个数 */
    volatile uint32_t edge_cnt;   /* 收到的边沿总数(上升+下降) */

    /* ---- Rc_Loop() 里更新 ---- */
    volatile uint32_t width_us;  /* 高电平宽度，单位 µs */
    volatile uint32_t period_us; /* 周期，单位 µs */
    volatile uint32_t freq_hz;   /* 频率 = 1000000 / period_us */
    volatile float value;        /* 归一化到 -1 ~ +1(RC_CENTER_US 对应 0) */
    volatile uint8_t valid;      /* 1 = 这路现在有正常信号 */

    /* ---- Rc_Loop() 内部用 ---- */
    uint32_t rise_cnt_prev;
    uint32_t last_rise_ms;
} rc_channel_t;

/* 捕获层：硬件测量 + 时基。挂在 rc.cap 下面，上层一般不用看。 */
typedef struct {
    rc_channel_t ch[RC_CH_CNT];

    /* 当前在动的那一路(0..3)，-1 = 四路都没信号。按"最新收到上升沿"选。 */
    volatile int8_t active_ch;
    volatile uint8_t any_valid; /* 1 = 至少一路 valid */

    /* 四路里哪几路这一轮已经报过到了(bit i = rc.cap.ch[i])。攒满 (1<<RC_CH_CNT)-1
     * 才喂一次狗，喂完清零重数 —— 这就是"四路齐活"的记账，见 rc.monitor。
     * 只由 Rc_Loop()(CmdTask)写，中断里一个字都不碰，所以无锁安全。 */
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

/* 摇杆边沿触发器：3 个比例轴 × 2 个方向 = 6 个，挂在 rc.trig 下面。
 * 数值口径和两个阈值都在 rc_config.h 的"摇杆边沿触发器"那一段。
 *
 * **rc 只负责每拍 Etc_Update()，不转存结果** —— 和源工程 remote 一个做法，
 * 中间量(flag / triggered)就留在触发器对象里，上层自己挑读法：
 *   Etc_Peek(t)  现在是否还压着 —— 越过触发线就一直为 1，回到回中线才变 0；
 *   Etc_Get(t)   取"这一次按下去的边沿"，**取完自动清 0**，所以只能读一次，
 *                且读的时机要比 Rc_Loop() 快不了太多才不会漏。
 *
 * 方向名说的是"归一化值的正负"，不是"哪个方向的物理动作"：rx_right = rc.rx 转正、
 * ry_up = rc.ry 转正，以此类推。
 * **符号方向已确认(2026-10-05)：右摇杆、左摇杆都是向右、向上为正**，正好和
 * 这里的名字对上，不用换指针。但换接收机、改 rc_config.h 的通道映射、或者动
 * Rc_Normalize() 之后要重新确认一次 —— 拨着摇杆读 rc.rx_raw / rc.ry_raw 一眼
 * 就能看出来，反了把对应的两个指针换一下即可。
 *
 * 指针在 Rc_Init() 里建好；建失败(堆不够)时 6 个全是 NULL。 */
typedef struct {
    Edge_trigger_continuous *rx_right; /* 右摇杆推右(rc.rx 转正) */
    Edge_trigger_continuous *rx_left;  /* 右摇杆推左(rc.rx 转负) */
    Edge_trigger_continuous *ry_up;    /* 右摇杆推上(rc.ry 转正) */
    Edge_trigger_continuous *ry_down;  /* 右摇杆推下(rc.ry 转负) */
    Edge_trigger_continuous *lx_right; /* 左摇杆推右(rc.lx 转正) */
    Edge_trigger_continuous *lx_left;  /* 左摇杆推左(rc.lx 转负) */
} rc_triggers_t;

/* 遥控器对象。逻辑层在顶层 —— 上层只看上半部分；捕获层缩在下面的 cap 里。 */
typedef struct {
    /* ================= 逻辑层：(实体) <- (捕获通道) =================
     * 映射表在 rc_config.h(哪一路接收机接在哪个捕获通道上)。这里只放结果。 */

    /* 摇杆(死区之后)：-1 ~ +1，中位附近(±RC_AXIS_DEADBAND)已经清零。
     * 对应那一路没信号时恒为 0(看 valid_* 区分)。 */
    volatile float rx; /* 右摇杆左右 */
    volatile float ry; /* 右摇杆上下 */
    volatile float lx; /* 左摇杆左右 */

    /* 摇杆(死区之前)：就是捕获层的 value，原样搬过来的。
     * 上面三个加了死区，摇杆在死区内时恒为 0，中位到底偏了多少就看不出来了 ——
     * 标定 / 排查漂移时看这三个(比如居中不动却是 +0.04，说明中位偏了 20 µs)。
     * 和 sw_a_raw 是一个道理。 */
    volatile float rx_raw; /* 右摇杆左右的归一化值，未过死区 */
    volatile float ry_raw; /* 右摇杆上下的归一化值，未过死区 */
    volatile float lx_raw; /* 左摇杆左右的归一化值，未过死区 */

    volatile float sw_a_raw; /* swA 的归一化值(取死区之前的那一步)，调试用 */
    volatile uint8_t sw_a;   /* swA：|sw_a_raw| < RC_SW_A_DEADBAND ? 0 : 1 */

    volatile uint8_t valid_rx;
    volatile uint8_t valid_ry;
    volatile uint8_t valid_lx;
    volatile uint8_t valid_swa;
    volatile uint8_t link_ok; /* 1 = 上面四路全部有效 */

    /* 摇杆边沿触发器(6 个)。Rc_Loop() 每拍喂一次，结果留在触发器里，rc 不转存 ——
     * 上层直接拿 Etc_Get()/Etc_Peek() 读，详见上面 rc_triggers_t 的说明。
     * 丢信号时 rx/ry/lx 是 0(中位)，触发器跟着回中复位，不会卡在"一直触发"上。 */
    rc_triggers_t trig;

    /* ================= 链路看门狗 =================
     * 四路输入都刷新过一遍才喂一次狗，所以它判的是"四路齐活"，不是"任一路活着"：
     *   is_Offline(rc.monitor) 为真  <=>  最近 RC_MONITOR_LOST_COUNT 个 monitor
     *                                     周期(现在是 10 ms/拍，合 100 ms)内，
     *                                     四路至少有一路一个上升沿都没收到。
     * 和 link_ok 是互补的两件事，别混：
     *   link_ok = 0   某一路当前没有合法信号(脉宽/周期跑出窗口，或者断了 100 ms)
     *   monitor 掉线  某一路连边沿都没有了(断线、接收机掉电)
     * 也就是：一路被干扰到一直吐垃圾脉冲 -> 喂狗还活着，但 link_ok = 0。 */
    monitor_item *monitor; /* Monitor_Register() 出来的，Rc_Init() 之后有效。
                            * 喂狗的记账在 rc.cap.fb_mask 里。 */

    /* ================= 捕获层(硬件)，子结构 ================= */
    rc_cap_t cap;
} rc_t;

/* 遥控器对象，Rc_Init() 之后有效；Rc_Loop() 每跑一遍，逻辑层就是最新的。 */
extern rc_t rc;

/* 改时基 + 开四路输入捕获中断 + 建 6 个摇杆边沿触发器 + 注册看门狗。
 * 必须在 MX_TIM1_Init()/MX_TIM2_Init() 和 Monitor_Init() 之后、调度器启动之前调用。 */
void Rc_Init(void);

/* 把原始计数换算成 µs/频率/归一化值，判丢信号、选 active_ch、推逻辑层、
 * 喂 6 个摇杆边沿触发器、喂看门狗。周期调用(1 kHz 够了)，Cmd_Loop() 里已经调了。
 * 中断上下文不能用，也不要在比 RC_LOST_TIMEOUT_MS 还慢的地方调。 */
void Rc_Loop(void);

/* 数回绕：TIM1/TIM2 的更新事件里调(htim->Instance 不是这两个时什么都不做)。
 * 必须挂在 main.c 的 HAL_TIM_PeriodElapsedCallback() 里(CubeMX 生成的函数，
 * 往 USER CODE BEGIN Callback 1 区加一行即可，重新生成不会丢 —— 那个函数对
 * 每个定时器都会调一次，所以"不是我要的定时器"这条路是常态不是例外)。
 * 每个定时器每 65.536 ms 进一次，只有自增，开销可以忽略。 */
void Rc_CountWrap(TIM_HandleTypeDef *htim);
