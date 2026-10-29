#include "rc.h"

#include "rc_config.h" /* 板级接线 + 调参，只有这个文件看得到 */

#include "main.h"

#include <math.h> /* fabsf()，摇杆/swA 过死区用 */

/* 捕获时基为 1 MHz、16 位计数器，结合更新中断扩展为 32 位时间戳。 */

rc_t rc;

/* 通道映射表的项数须与定时器和捕获通道数量一致。 */
static TIM_HandleTypeDef *const rc_htim[RC_TIM_CNT] = RC_TIM_ARRAY;
static const uint8_t rc_ch_cfg[RC_CH_CNT][2] = RC_CH_ARRAY;

/* 消息用 ASCII：GCC 打印 _Static_assert 消息时会把非 ASCII 字节转义成 \377 之类，
 * 中文在里面是一团乱码，等于没说。 */
_Static_assert(RC_TIM_ARRAY_CNT == RC_TIM_CNT, "RC_TIM_ARRAY count != RC_TIM_CNT");
_Static_assert(RC_CH_ARRAY_CNT == RC_CH_CNT, "RC_CH_ARRAY count != RC_CH_CNT");

/* 将定时器和通道对映射为捕获通道索引，0xFF 表示未配置。 */
static uint8_t rc_ch_lut[RC_TIM_CNT][2];

/* APB 预分频为 1 时 TIMCLK=PCLK，否则 TIMCLK=2×PCLK。 */
static uint32_t Rc_TimerClockHz(uint32_t pclk, uint32_t ppre) {
    return (ppre == 0U) ? pclk : (pclk * 2U);
}

/* TIM1/8/15/16/17 使用 APB2 时钟，其余捕获定时器使用 APB1 时钟。 */
static uint32_t Rc_TimerClockHzOf(TIM_TypeDef *inst) {
    if (inst == TIM1 || inst == TIM8 || inst == TIM15 || inst == TIM16 || inst == TIM17) {
        return Rc_TimerClockHz(HAL_RCC_GetPCLK2Freq(),
            (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos);
    }
    return Rc_TimerClockHz(HAL_RCC_GetPCLK1Freq(),
        (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos);
}

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

/* 为三个摇杆轴分别创建正向和负向滞回触发器。 */

/* 将归一化值加偏置并缩放为非负触发器输入，中位值为 RC_TRIG_CENTER。 */
static int16_t Rc_TrigValue(float v) {
    return (int16_t)((v + RC_TRIG_BIAS) * RC_TRIG_SCALE);
}

static void Rc_Triggers_Create(void) {
    /* 正向阈值为 CENTER+DEAD、CENTER+FULL；负向阈值为 CENTER-FULL、CENTER-DEAD。 */
    const int16_t pos_low = (int16_t)(RC_TRIG_CENTER + RC_TRIG_DEAD);
    const int16_t pos_high = (int16_t)(RC_TRIG_CENTER + RC_TRIG_FULL);
    const int16_t neg_low = (int16_t)(RC_TRIG_CENTER - RC_TRIG_FULL);
    const int16_t neg_high = (int16_t)(RC_TRIG_CENTER - RC_TRIG_DEAD);

    rc_triggers_t *t = &rc.trig;
    t->rx_right = Edge_Trigger_Continuous_Create(cont_pos, pos_low, pos_high);
    t->rx_left = Edge_Trigger_Continuous_Create(cont_neg, neg_low, neg_high);
    t->ry_up = Edge_Trigger_Continuous_Create(cont_pos, pos_low, pos_high);
    t->ry_down = Edge_Trigger_Continuous_Create(cont_neg, neg_low, neg_high);
    t->lx_right = Edge_Trigger_Continuous_Create(cont_pos, pos_low, pos_high);
    t->lx_left = Edge_Trigger_Continuous_Create(cont_neg, neg_low, neg_high);

    /* 一个建不出来就说明堆不够，整体没戏 —— 6 个一起置 NULL，让"要么全在、要么
 * 全不在"这个不变式成立，Rc_Triggers_Update() 里就只用查第一个指针。 */
    if (t->rx_right == NULL || t->rx_left == NULL || t->ry_up == NULL ||
        t->ry_down == NULL || t->lx_right == NULL || t->lx_left == NULL) {
        t->rx_right = t->rx_left = t->ry_up = t->ry_down = t->lx_right = t->lx_left = NULL;
    }
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

        /* 将计数时基设置为 1 MHz，计数周期为 65536。 */
        __HAL_TIM_SET_PRESCALER(rc_htim[t], psc);
        __HAL_TIM_SET_AUTORELOAD(rc_htim[t], RC_ARR);
        __HAL_TIM_SET_COUNTER(rc_htim[t], 0U);
        /* 触发 UG 使预分频值立即生效，并同步计数周期。 */
        HAL_TIM_GenerateEvent(rc_htim[t], TIM_EVENTSOURCE_UPDATE);

        /* 清除 UG 产生的 UIF，避免误计一次回绕。 */
        __HAL_TIM_CLEAR_FLAG(rc_htim[t], TIM_FLAG_UPDATE);
        rc.cap.wrap[t] = 0U;
        /* 使能更新中断以累计 16 位计数器回绕次数。 */
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

    /* 在 Monitor_Init 后注册看门狗，由全部通道刷新触发复位。 */
    rc.cap.fb_mask = 0U;
    rc.monitor = Monitor_Register(RC_LOST_CALLBACK, RC_MONITOR_LOST_COUNT, RC_LOST_CALLBACK_DATA);

    /* 创建六个摇杆方向触发器。 */
    Rc_Triggers_Create();
}

/* 根据活动通道直接读取对应 CCR 捕获值。 */
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

/* 在定时器周期回调中累计参与捕获的定时器回绕次数。 */
void Rc_CountWrap(TIM_HandleTypeDef *htim) {
    for (uint32_t t = 0; t < RC_TIM_CNT; ++t) {
        if (htim->Instance == rc_htim[t]->Instance) {
            rc.cap.wrap[t]++;
            return;
        }
    }
}

/* 将回绕计数与 16 位捕获值组合为 32 位微秒时间戳；待处理的 UIF 用于补偿回调顺序。 */
static uint32_t Rc_ExtendCnt(TIM_HandleTypeDef *htim, uint32_t tim_idx, uint32_t val) {
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
        rc.cap.wrap[tim_idx]++;
    }
    return (rc.cap.wrap[tim_idx] << 16) | val;
}

/* 输入捕获中断仅保存边沿时间戳和计数，信号换算在 Rc_Loop 中执行。 */
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

    /* 边沿间隔采用扩展时间戳相减，保留跨回绕的真实间隔。 */
    if ((ch & 1U) != 0U) {
        /* 连续两次上升沿的间隔作为周期；首次采样由 rise_cnt 过滤。 */
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

/* 按配置映射摇杆和开关通道，应用归一化、死区及有效性判定。 */

/* 取某一路的归一化值，没信号时给 0 */
static float Rc_CapValue(uint32_t cap) {
    return rc.cap.ch[cap].valid ? rc.cap.ch[cap].value : 0.0f;
}

/* 死区内输出为 0，死区外保留归一化幅值。 */
static float Rc_AxisDeadband(float v) {
    return (fabsf(v) < RC_AXIS_DEADBAND) ? 0.0f : v;
}

/* 每周期更新正负触发器；信号丢失时使用中位输入复位触发状态。 */
static void Rc_Triggers_Update(void) {
    if (rc.trig.rx_right == NULL) {
        return; /* 6 个同生共死，查一个就够；见 Rc_Triggers_Create() 里的整理 */
    }

    const int16_t vx = Rc_TrigValue(rc.rx);
    const int16_t vy = Rc_TrigValue(rc.ry);
    const int16_t vl = Rc_TrigValue(rc.lx);

    Etc_Update(rc.trig.rx_right, vx);
    Etc_Update(rc.trig.rx_left, vx);
    Etc_Update(rc.trig.ry_up, vy);
    Etc_Update(rc.trig.ry_down, vy);
    Etc_Update(rc.trig.lx_right, vl);
    Etc_Update(rc.trig.lx_left, vl);
}

/* 捕获层 -> 逻辑层。捕获层那一半在 Rc_Loop() 里已经刷完了。 */
static void Rc_UpdateLogic(void) {
    /* 保留死区处理前的归一化值，供通道标定使用。 */
    rc.rx_raw = Rc_CapValue(RC_RX_CAP);
    rc.ry_raw = Rc_CapValue(RC_RY_CAP);
    rc.lx_raw = Rc_CapValue(RC_LX_CAP);

    rc.rx = Rc_AxisDeadband(rc.rx_raw);
    rc.ry = Rc_AxisDeadband(rc.ry_raw);
    rc.lx = Rc_AxisDeadband(rc.lx_raw);

    /* 保存开关通道的归一化值供诊断读取。 */
    rc.sw_a_raw = Rc_CapValue(RC_SWA_CAP);
    rc.sw_a = (uint8_t)(fabsf(rc.sw_a_raw) < RC_SW_A_DEADBAND ? 0U : 1U);

    rc.valid_rx = rc.cap.ch[RC_RX_CAP].valid;
    rc.valid_ry = rc.cap.ch[RC_RY_CAP].valid;
    rc.valid_lx = rc.cap.ch[RC_LX_CAP].valid;
    rc.valid_swa = rc.cap.ch[RC_SWA_CAP].valid;
    rc.link_ok = (uint8_t)(rc.valid_rx && rc.valid_ry && rc.valid_lx && rc.valid_swa);

    /* 使用本周期更新的逻辑通道值推进触发器。 */
    Rc_Triggers_Update();
}

void Rc_Loop(void) {
    uint32_t now = HAL_GetTick();
    int8_t best = -1;
    uint32_t best_age = 0xFFFFFFFFU;
    uint8_t fresh = 0U; /* 本周期出现新上升沿的通道位图。 */

    for (uint32_t i = 0; i < RC_CH_CNT; ++i) {
        rc_channel_t *c = &rc.cap.ch[i];

        /* 出现新上升沿时更新信号时间戳和通道刷新标志。 */
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

    /* 全部捕获通道出现新边沿后刷新看门狗；信号合法性由 link_ok 单独表示。 */
    rc.cap.fb_mask = (uint8_t)(rc.cap.fb_mask | fresh);
    if (rc.cap.fb_mask == (uint8_t)((1U << RC_CH_CNT) - 1U)) {
        rc.cap.fb_mask = 0U; /* 下一轮从零重新数 */
        if (rc.monitor != NULL) {
            rc.monitor->reset(rc.monitor);
        }
    }

    /* 捕获数据更新完成后刷新逻辑通道。 */
    Rc_UpdateLogic();
}
