#include "rc.h"

#include "rc_config.h" /* 板级接线 + 调参，只有这个文件看得到 */

#include "main.h" /* HAL_GetTick() / HAL_RCC_GetPCLKxFreq() / RCC */

#include <math.h> /* fabsf()，摇杆/swA 过死区用 */

/* ==================== 遥控器 PWM 输入捕获 ====================
 *
 * 硬件/接线表在 rc_config.h 里，这里只记"为什么"：
 *
 * 1. 时基要是 "1 个计数 = 1 µs + ARR = 0xFFFF(65.535 ms 满量程)"：
 *      - 分辨率 1 µs，对 1000~2000 µs 的遥控信号刚好；
 *      - 但**计数器自己会回绕**(65536 计数 = 65.5 ms，平均每 3.5 个周期一次)，
 *        所以光有 16 位差值分不清"间隔 18 ms"和"间隔 18 ms + 绕了一圈"。
 *        解决办法是开更新中断数回绕、把计数值扩成 32 位，见 Rc_ExtendCnt()。
 *    .ioc 里已经按这个配好了(Prescaler = 239 / Counter Period = 65535)，
 *    Rc_Init() 里那两行 __HAL_TIM_SET_PRESCALER/AUTORELOAD 是把同样的值再写一遍 ——
 *    看着多余，留着是因为 PSC 是按 HAL_RCC_GetPCLKxFreq() 现算的(见第 2 条)，
 *    哪天真改了时钟树、忘了同步 .ioc 里的 Prescaler，这里会自动纠正，
 *    量出来的 µs 也还是对的。
 *    注意 **CubeMX 里没有"更新中断"这个选项**，DIER 的 UIE 位只能由
 *    Rc_Init() 里的 __HAL_TIM_ENABLE_IT(TIM_IT_UPDATE) 打开 —— 这行删不得，
 *    删了回绕计数就没了。NVIC 那边的 TIM1_UP_IRQn / TIM2_IRQn 是 CubeMX 配的。
 *
 * 2. 240 MHz 是现算的(Rc_TimerClockHzOf)，不是写死，改时钟树也跟得上。
 *
 * 3. 四路同时在抓，所以接在哪一路都能认出来(看 rc.cap.active_ch)，频率也是量出来的
 *    (两次上升沿之差)，不用事先知道接收机是 50 Hz 还是几百 Hz。
 */

rc_t rc;

/* 哪一路捕获挂在哪个定时器/通道对上，来自 rc_config.h。
 * 两张表都由 *_CNT 宏给出项数并对着 rc.h 的个数断言 —— 写短了会被静默补 0，
 * 那会让两路捕获映射到同一个 rc.cap.ch[] 互相踩，必须编译期挡住。 */
static TIM_HandleTypeDef *const rc_htim[RC_TIM_CNT] = RC_TIM_ARRAY;
static const uint8_t rc_ch_cfg[RC_CH_CNT][2] = RC_CH_ARRAY;

/* 消息用 ASCII：GCC 打印 _Static_assert 消息时会把非 ASCII 字节转义成 \377 之类，
 * 中文在里面是一团乱码，等于没说。 */
_Static_assert(RC_TIM_ARRAY_CNT == RC_TIM_CNT, "RC_TIM_ARRAY count != RC_TIM_CNT");
_Static_assert(RC_CH_ARRAY_CNT == RC_CH_CNT, "RC_CH_ARRAY count != RC_CH_CNT");

/* 反向表：[定时器下标][通道对] -> rc.cap.ch 下标，0xFF = 这一格没配捕获。
 * 中断里手上只有 htim->Instance 和 HAL_TIM_ACTIVE_CHANNEL_x，查这张表最省事。
 * Rc_Init() 里填，之后只读。 */
static uint8_t rc_ch_lut[RC_TIM_CNT][2];

/* H7 的规则：APB 预分频为 1 时定时器时钟 = PCLK，否则 = 2×PCLK。
 * ppre 是 RCC_D2CFGR 里 D2PPRE1/D2PPRE2 那三位的原始值。 */
static uint32_t Rc_TimerClockHz(uint32_t pclk, uint32_t ppre) {
    return (ppre == 0U) ? pclk : (pclk * 2U);
}

/* TIM1/TIM8/TIM15/16/17 挂 APB2(D2PPRE2)，其余(含 TIM2)挂 APB1(D2PPRE1)。
 * 比"下标 0 就是 TIM1"那种硬编码结实：换定时器不用回来改这里。 */
static uint32_t Rc_TimerClockHzOf(TIM_TypeDef *inst) {
    if (inst == TIM1 || inst == TIM8 || inst == TIM15 || inst == TIM16 || inst == TIM17) {
        return Rc_TimerClockHz(HAL_RCC_GetPCLK2Freq(),
            (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos);
    }
    return Rc_TimerClockHz(HAL_RCC_GetPCLK1Freq(),
        (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos);
}

/* 计数 -> µs。Rc_Init() 之后计数频率就是 1 MHz，原样返回；写成乘除是为了万一
 * 以后时钟树变了也不至于算错(32 位放不下，用 64 位算)。 */
static uint32_t Rc_RawToUs(uint32_t raw, uint32_t ch_idx) {
    uint32_t hz = rc.cap.tick_hz[rc_ch_cfg[ch_idx][0]];
    if (hz == 0U) {
        return raw;
    }
    return (uint32_t)(((uint64_t)raw * 1000000ULL) / (uint64_t)hz);
}

static float Rc_Normalize(uint32_t width_us) {
    float v = ((float)width_us - (float)RC_CENTER_US) / (float)RC_HALF_RANGE_US;
    if (v > 1.0f) {
        v = 1.0f;
    }
    if (v < -1.0f) {
        v = -1.0f;
    }
    return v;
}

void Rc_Init(void) {
    /* 先把反向表全标成"没配"，再按 rc_config.h 那张表填 */
    for (uint32_t t = 0; t < RC_TIM_CNT; ++t) {
        rc_ch_lut[t][0] = 0xFFU;
        rc_ch_lut[t][1] = 0xFFU;
    }
    for (uint32_t i = 0; i < RC_CH_CNT; ++i) {
        rc_ch_lut[rc_ch_cfg[i][0]][rc_ch_cfg[i][1]] = (uint8_t)i;
    }

    for (uint32_t t = 0; t < RC_TIM_CNT; ++t) {
        uint32_t clk_hz = Rc_TimerClockHzOf(rc_htim[t]->Instance);
        uint32_t psc =
            (clk_hz >= RC_TICK_TARGET_HZ) ? ((clk_hz / RC_TICK_TARGET_HZ) - 1U) : 0U;

        rc.cap.tim_clk_hz[t] = clk_hz;
        rc.cap.psc[t] = psc;
        rc.cap.tick_hz[t] = clk_hz / (psc + 1U);

        /* .ioc 里配的就是这两个值(239 / 65535)，这里等于再写一遍。
         * 上面第 1 条注释里写了为什么留着。 */
        __HAL_TIM_SET_PRESCALER(rc_htim[t], psc);
        __HAL_TIM_SET_AUTORELOAD(rc_htim[t], RC_ARR);
        __HAL_TIM_SET_COUNTER(rc_htim[t], 0U);
        /* PSC 是带影子寄存器的，写进去要等一次更新事件才生效(PSC 没有"关掉预装载"
         * 的开关)。ARR 这版 CubeMX 配的是 AutoReloadPreload = DISABLE，写进去立即
         * 生效，本来不用等；但这里两个值一起改，触发一次 UG 最省心。 */
        HAL_TIM_GenerateEvent(rc_htim[t], TIM_EVENTSOURCE_UPDATE);

        /* 上面那次 UG 会把 UIF 置起来，先清掉，否则一开中断就白数一次回绕。 */
        __HAL_TIM_CLEAR_FLAG(rc_htim[t], TIM_FLAG_UPDATE);
        rc.cap.wrap[t] = 0U;
        /* 开更新中断，唯一目的是数 16 位计数器的回绕(每 65.536 ms 一次)。
         * 不数的话，两次边沿的间隔只能靠 16 位差值表达，绕圈时会折叠回来 ——
         * 详见 Rc_ExtendCnt() 的注释。 */
        __HAL_TIM_ENABLE_IT(rc_htim[t], TIM_IT_UPDATE);
    }

    /* 四路输入捕获中断全开。CH1/CH3 是上升沿，CH2/CH4 是同一根线上的下降沿，
     * 所以每路的脉宽 = 该路的 CH2/CH4 减去 CH1/CH3。 */
    for (uint32_t i = 0; i < RC_CH_CNT; ++i) {
        TIM_HandleTypeDef *ht = rc_htim[rc_ch_cfg[i][0]];
        uint8_t pair = rc_ch_cfg[i][1]; /* 0 = CH1/CH2，1 = CH3/CH4 */
        HAL_TIM_IC_Start_IT(ht, (pair == 0U) ? TIM_CHANNEL_1 : TIM_CHANNEL_3);
        HAL_TIM_IC_Start_IT(ht, (pair == 0U) ? TIM_CHANNEL_2 : TIM_CHANNEL_4);
    }

    /* 链路看门狗：四路输入都刷新过一遍才喂一次狗(在 Rc_Loop() 里喂)。
     * 必须在 Monitor_Init() 之后调 —— main.c 里就是那个顺序。 */
    rc.cap.fb_mask = 0U;
    rc.monitor = Monitor_Register(RC_LOST_CALLBACK, RC_MONITOR_LOST_COUNT, RC_LOST_CALLBACK_DATA);
}

/* 直接按通道号读 CCR，不用 HAL_TIM_ReadCapturedValue()：后者要 TIM_CHANNEL_x，
 * 而中断里拿到的是 htim->Channel(HAL_TIM_ACTIVE_CHANNEL_x)，两套枚举值不一样
 * (0x00/0x04/0x08/0x0C vs 0x01/0x02/0x04/0x08)，传错了会静默读到别的通道。 */
static uint32_t Rc_ReadCcr(TIM_TypeDef *inst, uint32_t ch) {
    switch (ch) {
    case 1U:
        return inst->CCR1;
    case 2U:
        return inst->CCR2;
    case 3U:
        return inst->CCR3;
    default:
        return inst->CCR4;
    }
}

/* 数回绕：挂在 main.c 的 HAL_TIM_PeriodElapsedCallback() 里。
 * ARR = 0xFFFF、1 MHz，所以每个定时器每 65.536 ms 进一次，只自增一下。
 * 那个回调对**每个**定时器都会调一次(TIM23 的系统 tick 走的就是这里)，
 * 所以"不是参与捕获的定时器"是常态。 */
void Rc_CountWrap(TIM_HandleTypeDef *htim) {
    for (uint32_t t = 0; t < RC_TIM_CNT; ++t) {
        if (htim->Instance == rc_htim[t]->Instance) {
            rc.cap.wrap[t]++;
            return;
        }
    }
}

/* 把 16 位计数值补上回绕次数，拼成 32 位(单位不变，仍是 1 计数 = 1 µs，
 * 要 71 分钟才绕一圈 —— 对遥控信号来说等于不会绕)。
 *
 * 为什么非要这么做：16 位计数器满量程 65536 计数 = 65.5 ms，而周期只有
 * ~18 ms，所以每隔 3.5 个周期就有一次"两次上升沿之间正好跨过 0xFFFF->0"
 * 的情况。只拿 16 位差值来看，"间隔 18440"和"间隔 18440 + 65536"完全长得
 * 一样 —— 信号断 84 ms 再回来，算出来还是 18440，Rc_Loop() 的范围检查
 * 会当成正常帧收下。扩成 32 位之后这两个值就分开了，84 ms 会老老实实
 * 报成 84000 µs 然后被范围检查挡掉。
 *
 * 这里除了读回绕计数，还顺手查一次 UIF，是防一种竞态：回绕和本次捕获
 * 挤在同一瞬间，更新中断还没轮到跑。就地补上，免得少算一次回绕(少算正是
 * 上面那个"把 84 ms 看成 18 ms"的错)。更新中断已经跑过的话 UIF 已清，
 * 不会重复计数；反过来多算一次只会让这一拍的间隔偏大 65.5 ms 而被判废，
 * 落在安全的一侧。 */
static uint32_t Rc_ExtendCnt(TIM_HandleTypeDef *htim, uint32_t tim_idx, uint32_t val) {
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
        rc.cap.wrap[tim_idx]++;
    }
    return (rc.cap.wrap[tim_idx] << 16) | val;
}

/* HAL 的输入捕获回调：TIM1_CC_IRQHandler / TIM2_IRQHandler -> HAL_TIM_IRQHandler()
 * -> 这里。这是全工程唯一的定义(HAL 里那个是 __weak)。
 *
 * 中断里只做两件事：记(扩展成 32 位的)计数、计数。
 * 换算 µs/频率、判有效、挑 active_ch 都放到 Rc_Loop() 里，中断保持短。 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    uint32_t tim_idx = RC_TIM_CNT;
    uint32_t ch;

    /* 比 Instance 不比 handle 指针：HAL 传进来的不一定是 rc_htim[] 里存的那个 */
    for (uint32_t t = 0; t < RC_TIM_CNT; ++t) {
        if (htim->Instance == rc_htim[t]->Instance) {
            tim_idx = t;
            break;
        }
    }
    if (tim_idx >= RC_TIM_CNT) {
        return; /* 不是这两路遥控输入，不管 */
    }

    switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
        ch = 1U;
        break;
    case HAL_TIM_ACTIVE_CHANNEL_2:
        ch = 2U;
        break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
        ch = 3U;
        break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
        ch = 4U;
        break;
    default:
        return;
    }

    /* CH1/CH2 量 TI1 那根线(通道对 0)，CH3/CH4 量 TI3(通道对 1)；
     * 奇数通道抓上升沿。 */
    uint8_t idx = rc_ch_lut[tim_idx][(ch >= 3U) ? 1U : 0U];
    if (idx == 0xFFU) {
        return; /* 这一对通道没配成捕获 */
    }

    rc_channel_t *c = &rc.cap.ch[idx];
    uint32_t val = Rc_ReadCcr(htim->Instance, ch);
    uint32_t ext = Rc_ExtendCnt(htim, tim_idx, val);

    c->edge_cnt++;

    /* 两次边沿的间隔都用扩展后的 32 位值相减，得到的就是真实计数差。
     * 千万不能退回去用 (val - rise) & 0xFFFF —— 那只是把跨圈的那一拍照样
     * 折叠成一个看着合理的假值，分不出"18 ms"和"18 ms + 绕了一圈"。 */
    if ((ch & 1U) != 0U) {
        /* 上升沿：和上一次上升沿的差就是周期。
         * 第一次进来 rise_ext 还是 0，算出来的周期是废的，
         * Rc_Loop() 用 rise_cnt >= 2 挡掉。 */
        c->rise_raw = val;
        c->period_raw = ext - c->rise_ext;
        c->rise_ext = ext;
        c->rise_cnt++;
    } else if (c->rise_cnt != 0U) {
        /* 下降沿：和本次上升沿的差就是高电平宽度 */
        c->fall_raw = val;
        c->width_raw = ext - c->rise_ext;
    }
}

/* ==================== 逻辑层：接收机通道 -> 实体 ====================
 *
 * 哪一路接收机接在哪个捕获通道上，写在 rc_config.h 的 RC_RX_CAP / RC_RY_CAP /
 * RC_LX_CAP / RC_SWA_CAP 里(那张接线表)。这里只做换算：
 *   - 摇杆：原样搬捕获层的 value((脉宽 - 1500) / 500，限幅 ±1)存进 *_raw，
 *           再过一个 ±RC_AXIS_DEADBAND 的死区给上层；
 *   - swA ：同一套算法算出 sw_a_raw，再用死区分成 0 / 1。
 * 没信号的那一路给 0 并让 valid_* = 0 —— 不要让上层把上一次的残留值当成
 * 当前位置，丢信号必须看得出来。 */

/* 取某一路的归一化值，没信号时给 0 */
static float Rc_CapValue(uint32_t cap) {
    return rc.cap.ch[cap].valid ? rc.cap.ch[cap].value : 0.0f;
}

/* 摇杆死区：只清零、不重标定 —— 出死区的第一个值就是 ±RC_AXIS_DEADBAND，
 * 不会在边界上跳。捕获层的 rc.cap.ch[].value 保持原始值不动，标定时还要看它。 */
static float Rc_AxisDeadband(float v) {
    return (fabsf(v) < RC_AXIS_DEADBAND) ? 0.0f : v;
}

/* 捕获层 -> 逻辑层。捕获层那一半在 Rc_Loop() 里已经刷完了。 */
static void Rc_UpdateLogic(void) {
    /* 先存死区前的原始值，再从它过死区 —— 原始值留给标定/调试看
     * (摇杆居中却读出 +0.04，就说明中位偏了 20 µs，光看死区后的 0 是看不出来的) */
    rc.rx_raw = Rc_CapValue(RC_RX_CAP);
    rc.ry_raw = Rc_CapValue(RC_RY_CAP);
    rc.lx_raw = Rc_CapValue(RC_LX_CAP);

    rc.rx = Rc_AxisDeadband(rc.rx_raw);
    rc.ry = Rc_AxisDeadband(rc.ry_raw);
    rc.lx = Rc_AxisDeadband(rc.lx_raw);

    /* sw_a_raw 留着是为了调试：区分"开关真在中间"和"这路没信号"，两者都是 0 */
    rc.sw_a_raw = Rc_CapValue(RC_SWA_CAP);
    rc.sw_a = (uint8_t)(fabsf(rc.sw_a_raw) < RC_SW_A_DEADBAND ? 0U : 1U);

    rc.valid_rx = rc.cap.ch[RC_RX_CAP].valid;
    rc.valid_ry = rc.cap.ch[RC_RY_CAP].valid;
    rc.valid_lx = rc.cap.ch[RC_LX_CAP].valid;
    rc.valid_swa = rc.cap.ch[RC_SWA_CAP].valid;
    rc.link_ok = (uint8_t)(rc.valid_rx && rc.valid_ry && rc.valid_lx && rc.valid_swa);
}

void Rc_Loop(void) {
    uint32_t now = HAL_GetTick();
    int8_t best = -1;
    uint32_t best_age = 0xFFFFFFFFU;
    uint8_t fresh = 0U; /* 这一拍刷新过的通道位图 */

    for (uint32_t i = 0; i < RC_CH_CNT; ++i) {
        rc_channel_t *c = &rc.cap.ch[i];

        /* 这一拍出现了新的上升沿？更新时间戳(丢信号判断靠它)，顺便记下是哪几路刷新的 */
        if (c->rise_cnt != c->rise_cnt_prev) {
            c->rise_cnt_prev = c->rise_cnt;
            c->last_rise_ms = now;
            fresh |= (uint8_t)(1U << i);
        }

        uint32_t width = Rc_RawToUs(c->width_raw, i);
        uint32_t period = Rc_RawToUs(c->period_raw, i);
        uint32_t age = now - c->last_rise_ms;

        /* 这几条都过了才算这路有正常信号 */
        uint8_t ok = (uint8_t)(c->rise_cnt >= 2U); /* 至少量到一个完整周期 */
        ok = (uint8_t)(ok && period >= RC_PERIOD_MIN_US && period <= RC_PERIOD_MAX_US);
        ok = (uint8_t)(ok && width >= RC_WIDTH_MIN_US && width <= RC_WIDTH_MAX_US);
        ok = (uint8_t)(ok && width < period); /* 占空比必须小于 100% */
        ok = (uint8_t)(ok && age < RC_LOST_TIMEOUT_MS);

        c->width_us = ok ? width : 0U;
        c->period_us = ok ? period : 0U;
        c->freq_hz = (ok && period != 0U) ? (1000000U / period) : 0U;
        c->value = ok ? Rc_Normalize(width) : 0.0f;
        c->valid = ok;

        if (ok && age < best_age) {
            best_age = age;
            best = (int8_t)i;
        }
    }

    rc.cap.active_ch = best;
    rc.cap.any_valid = (uint8_t)(best >= 0);

    /* 四路都刷新过一遍才喂一次狗。只要有一路不再更新，fb_mask 永远凑不满，
     * monitor->count 一路减到 0 -> is_Offline(rc.monitor) 变真。
     * 这里按字面取"有过新边沿"就算刷新，没有再套 valid/ok —— 那是 link_ok 管的事，
     * 两者互补：一路吐垃圾脉冲时喂狗还活着但 link_ok = 0(见 rc.h 的说明)。 */
    rc.cap.fb_mask = (uint8_t)(rc.cap.fb_mask | fresh);
    if (rc.cap.fb_mask == (uint8_t)((1U << RC_CH_CNT) - 1U)) {
        rc.cap.fb_mask = 0U; /* 下一轮从零重新数 */
        if (rc.monitor != NULL) {
            rc.monitor->reset(rc.monitor);
        }
    }

    /* 捕获层刷完，顺手把逻辑层也推一遍，Rc_Loop() 返回后 rc.rx/... 就是最新的 */
    Rc_UpdateLogic();
}
