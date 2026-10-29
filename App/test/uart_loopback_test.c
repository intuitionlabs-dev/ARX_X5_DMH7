#include "uart_loopback_test.h"

#include <string.h>

#include "bsp_uart.h"
#include "main.h"
#include "test_config.h"
#include "frame.h"

/* 启用测试时编译串口双向回环实现。 */
#if TEST_UART_LOOPBACK_ENABLE

/* 在编译期校验 USART2、USART3 的 BSP 索引。 */
_Static_assert((UART_LB_U2 == BSP_UART_U2) && (UART_LB_U3 == BSP_UART_U3),
    "UART_LB_U2/U3 必须和 BSP_UART_U2/U3 一致");

/* 累计测试帧收发及协议校验结果。 */
volatile uint32_t uart_lb_tx_cnt;
volatile uint32_t uart_lb_tx_fail_cnt;
volatile uint32_t uart_lb_rx_cnt;
volatile uint32_t uart_lb_ok_cnt;
volatile uint32_t uart_lb_bad_cnt;
volatile uint32_t uart_lb_lost_cnt;
volatile uint32_t uart_lb_crc8_err;
volatile uint32_t uart_lb_crc16_err;
volatile uint32_t uart_lb_encode_err;
volatile uint32_t uart_lb_rearm_fail;

/* 按用例和 BSP 端口保存分类统计。 */
volatile uint32_t uart_lb_case_ok[UART_LB_CASE_CNT];
volatile uint32_t uart_lb_dir_ok[BSP_UART_CNT];
volatile uint32_t uart_lb_hal_err[BSP_UART_CNT];

volatile uint32_t uart_lb_byte_dir[BSP_UART_CNT];
volatile uint32_t uart_lb_rx_dir[BSP_UART_CNT];
volatile uint32_t uart_lb_ok_dir[BSP_UART_CNT];
volatile uint32_t uart_lb_ore_cnt[BSP_UART_CNT];
volatile uint32_t uart_lb_offdir_cnt;
volatile uint32_t uart_lb_offdir_size;

/* 保存最近一次校验失败的实际值与期望值。 */
volatile uint32_t uart_lb_bad_dir;
volatile uint32_t uart_lb_bad_got_type;
volatile uint32_t uart_lb_bad_got_len;
volatile uint32_t uart_lb_bad_exp_type;
volatile uint32_t uart_lb_bad_exp_len;
volatile uint32_t uart_lb_bad_got_hdr;
volatile uint32_t uart_lb_bad_exp_hdr;
volatile uint32_t uart_lb_bad_wire;
volatile uint32_t uart_lb_bad_pay_ok;

/* 保存最近一次丢帧的解析进度和字节计数。 */
volatile uint32_t uart_lb_lost_state;
volatile uint32_t uart_lb_lost_len;
volatile uint32_t uart_lb_lost_got;
volatile uint32_t uart_lb_lost_need;
volatile uint32_t uart_lb_lost_case;

/* 按缺失字节数和解析状态累计丢帧分布。 */
#define UART_LB_SHORT_BINS 6U
volatile uint32_t uart_lb_lost_short[UART_LB_SHORT_BINS];
volatile uint32_t uart_lb_lost_state_hist[UART_LB_STATE_BINS];

/* 保存 BSP DMA 布局检查结果和缺失配置掩码。 */
volatile uint32_t uart_lb_dma_mem_bad;
volatile uint32_t uart_lb_no_dma;

/* 保存吞吐量、最近帧信息和运行轮次。 */
volatile uint32_t uart_lb_tx_bytes;
volatile uint32_t uart_lb_rx_bytes;
volatile uint8_t uart_lb_last_type_id;
volatile uint8_t uart_lb_last_datalen;
volatile uint32_t uart_lb_last_seq;
volatile uint32_t uart_lb_round;

/* 每个 BSP 端口持有独立解析器。 */
static FrameParser s_parser[BSP_UART_CNT];

/* 接收回调发布完整帧就绪标志。 */
static volatile uint8_t s_rx_ready;

/* 保存本轮期望载荷、编码缓冲和帧信息。 */
static FramePayload s_tx_payload;

static uint8_t s_tx_frame[FRAME_MAX_LEN];
static FrameTypeId s_tx_type_id;
static uint8_t s_tx_len;

/* 记录本轮接收端、待结算状态和轮次。 */
static uint8_t s_rx_idx;
static uint8_t s_pending;
static uint32_t s_round;

/* 保存本轮开始时的接收字节计数基线。 */
static uint32_t s_round_bytes_base;

/* 使用 0x7F 类型码测试最大载荷帧。 */
#define UART_LB_TYPE_ID_UNASSIGNED 0x7FU

/* 轮转测试各类空载荷业务报文。 */
static const FrameTypeId s_empty_ids[] = {
    FRAME_TYPE_ID_MOTOR_SET_ZERO,
    FRAME_TYPE_ID_ARM_START,
    FRAME_TYPE_ID_ARM_GRAVITY,
    FRAME_TYPE_ID_ARM_STOP,
    FRAME_TYPE_ID_ARM_DAMPING,
};
#define UART_LB_EMPTY_CNT (sizeof s_empty_ids / sizeof s_empty_ids[0])

/* 每两轮切换用例以覆盖两个传输方向。 */
static uint8_t UartLoopbackTest_CaseOf(uint32_t seq) {
    return (uint8_t)((seq / 2U) % UART_LB_CASE_CNT);
}

/* 根据轮次生成本轮报文类型和载荷。 */
static void UartLoopbackTest_Build(uint32_t seq) {
    FrameTypeId id = FRAME_TYPE_ID_NONE;
    uint8_t len = 0U;
    uint32_t i;

    /* 按用例选择版本、空包、控制或边界报文。 */
    switch (UartLoopbackTest_CaseOf(seq)) {
    /* 生成携带版本码的报文。 */
    case 0:
        s_tx_payload.version.code = FrameVersionCode();
        len = (uint8_t)sizeof(FramePktVersion);
        id = FRAME_TYPE_ID_VERSION;
        break;

    /* 从空载荷类型表中选择报文。 */
    case 1:
        id = s_empty_ids[(seq / (UART_LB_CASE_CNT * 2U)) % UART_LB_EMPTY_CNT];
        len = 0U;
        break;

    /* 生成包含逐电机结果的设零应答。 */
    case 2:
        s_tx_payload.motor_set_zero_ack.state = (uint8_t)(seq & 0xFFU);
        /* 逐电机填充与轮次相关的结果字节。 */
        for (i = 0U; i < FRAME_PKT_MOTOR_NUM; i++) {
            s_tx_payload.motor_set_zero_ack.result[i] = (uint8_t)(seq + i);
        }
        s_tx_payload.motor_set_zero_ack.elapsed_ms = 1000U + seq;
        len = (uint8_t)sizeof(FramePktMotorSetZeroAck);
        id = FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK;
        break;

    /* 生成关节位置和夹爪目标报文。 */
    case 3:
        /* 逐关节填充位置目标。 */
        for (i = 0U; i < FRAME_PKT_JOINT_NUM; i++) {
            s_tx_payload.arm_ctrl_pos.joint_angle[i] =
                (float)((int32_t)(seq % 200U) - 100) + (float)i * 0.25f;
        }
        s_tx_payload.arm_ctrl_pos.gripper_angle = (float)(seq % 90U) * 0.5f;
        len = (uint8_t)sizeof(FramePktArmCtrlPos);
        id = FRAME_TYPE_ID_ARM_CTRL_POS;
        break;

    /* 生成关节位置、速度和夹爪目标报文。 */
    case 4:
        /* 逐关节填充位置与速度目标。 */
        for (i = 0U; i < FRAME_PKT_JOINT_NUM; i++) {
            s_tx_payload.arm_ctrl_pos_vel.joint_angle[i] =
                (float)((int32_t)(seq % 360U) - 180) * 0.5f;
            s_tx_payload.arm_ctrl_pos_vel.joint_vel[i] =
                (float)((int32_t)(seq % 100U) - 50) * 1.5f;
        }
        s_tx_payload.arm_ctrl_pos_vel.gripper_angle = (float)(seq % 90U) * 0.5f;
        len = (uint8_t)sizeof(FramePktArmCtrlPosVel);
        id = FRAME_TYPE_ID_ARM_CTRL_POS_VEL;
        break;

    /* 生成占满最大载荷长度的边界报文。 */
    default:
        /* 逐字节填充与轮次相关的边界数据。 */
        for (i = 0U; i < FRAME_MAX_DATA; i++) {
            s_tx_payload.raw[i] = (uint8_t)(seq + i);
        }
        len = (uint8_t)FRAME_MAX_DATA;
        id = UART_LB_TYPE_ID_UNASSIGNED;
        break;
    }

    /* 保存类型和长度用于编码与接收比对。 */
    s_tx_type_id = id;
    s_tx_len = len;
}

/* 保存最近一次报文比对失败的诊断快照。 */
static void UartLoopbackTest_Snapshot(const FrameParser *p) {
    uint32_t got_hdr = 0U;
    uint32_t exp_hdr = 0U;
    uint32_t wire = 0U;

    /* 提取载荷前缀和原始帧头供调试器比较。 */
    memcpy(&got_hdr, p->buf.f.payload.raw, sizeof got_hdr);
    memcpy(&exp_hdr, s_tx_payload.raw, sizeof exp_hdr);
    memcpy(&wire, p->buf.raw, sizeof wire);

    uart_lb_bad_dir = s_rx_idx;
    uart_lb_bad_got_type = p->type_id;
    uart_lb_bad_got_len = p->data_len;
    uart_lb_bad_exp_type = s_tx_type_id;
    uart_lb_bad_exp_len = s_tx_len;
    uart_lb_bad_got_hdr = got_hdr;
    uart_lb_bad_exp_hdr = exp_hdr;
    uart_lb_bad_wire = wire;

    /* 记录期望载荷与实际载荷的字节比较结果。 */
    uart_lb_bad_pay_ok = ((s_tx_len == 0U) ||
                          (memcmp(s_tx_payload.raw, p->buf.f.payload.raw, s_tx_len) == 0U))
                             ? 1U
                             : 0U;
}

/* 比较帧类型、长度、原始载荷和结构化字段。 */
static int UartLoopbackTest_Match(const FrameParser *p) {
    uint32_t i;

    /* 类型或载荷长度不匹配时报告失败。 */
    if ((p->type_id != s_tx_type_id) || (p->data_len != s_tx_len)) {
        return 0;
    }
    /* 载荷字节不匹配时报告失败。 */
    if ((s_tx_len > 0U) && (memcmp(s_tx_payload.raw, p->buf.f.payload.raw, s_tx_len) != 0)) {
        return 0;
    }

    /* 按报文类型读取结构化字段进行复核。 */
    switch (p->type_id) {
    /* 核对版本字段。 */
    case FRAME_TYPE_ID_VERSION:
        return (p->buf.f.payload.version.code == s_tx_payload.version.code);

    /* 核对设零状态、耗时和逐电机结果。 */
    case FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK: {
        const FramePktMotorSetZeroAck *got = &p->buf.f.payload.motor_set_zero_ack;
        const FramePktMotorSetZeroAck *exp = &s_tx_payload.motor_set_zero_ack;
        /* 状态或耗时不匹配时报告失败。 */
        if ((got->state != exp->state) || (got->elapsed_ms != exp->elapsed_ms)) {
            return 0;
        }
        /* 逐电机比较设零结果。 */
        for (i = 0U; i < FRAME_PKT_MOTOR_NUM; i++) {
            /* 任一电机结果不匹配时报告失败。 */
            if (got->result[i] != exp->result[i]) {
                return 0;
            }
        }
        return 1;
    }

    /* 核对关节位置与夹爪目标。 */
    case FRAME_TYPE_ID_ARM_CTRL_POS: {
        const FramePktArmCtrlPos *got = &p->buf.f.payload.arm_ctrl_pos;
        const FramePktArmCtrlPos *exp = &s_tx_payload.arm_ctrl_pos;
        /* 逐关节比较位置字段。 */
        for (i = 0U; i < FRAME_PKT_JOINT_NUM; i++) {
            /* 关节位置不匹配时报告失败。 */
            if (got->joint_angle[i] != exp->joint_angle[i]) {
                return 0;
            }
        }
        return (got->gripper_angle == exp->gripper_angle);
    }

    /* 核对关节位置、速度与夹爪目标。 */
    case FRAME_TYPE_ID_ARM_CTRL_POS_VEL: {
        const FramePktArmCtrlPosVel *got = &p->buf.f.payload.arm_ctrl_pos_vel;
        const FramePktArmCtrlPosVel *exp = &s_tx_payload.arm_ctrl_pos_vel;
        /* 逐关节比较位置和速度字段。 */
        for (i = 0U; i < FRAME_PKT_JOINT_NUM; i++) {
            /* 位置或速度不匹配时报告失败。 */
            if ((got->joint_angle[i] != exp->joint_angle[i]) ||
                (got->joint_vel[i] != exp->joint_vel[i])) {
                return 0;
            }
        }
        return (got->gripper_angle == exp->gripper_angle);
    }

    /* 空包和边界帧使用前述字节比较结果。 */
    default:

        return 1;
    }
}

/* 在中断中解析本轮接收端的数据并发布就绪标志。 */
static void UartLoopbackTest_OnRx(uint8_t id, const uint8_t *data, uint16_t len) {
    uint32_t i;

    /* 其他端口的数据仅累计丢弃字节数。 */
    if (id != s_rx_idx) {
        uart_lb_offdir_cnt += (uint32_t)len;
        uart_lb_offdir_size = (uint32_t)len;
        return;
    }

    uart_lb_rx_bytes += (uint32_t)len;
    uart_lb_byte_dir[id] += (uint32_t)len;

    /* 逐字节推进对应端口的解析器。 */
    for (i = 0U; i < (uint32_t)len; i++) {
        FrameStatus st = FrameParserFeed(&s_parser[id], data[i]);

        /* 完整合法帧置位就绪标志并累计接收数。 */
        if (st == FRAME_OK) {
            s_rx_ready = 1U;
            uart_lb_rx_dir[id]++;
        } else if (st == FRAME_ERR_CRC8) { /* 帧头校验失败时累计 CRC8 错误。 */
            uart_lb_crc8_err++;
        } else if (st == FRAME_ERR_CRC16) { /* 整帧校验失败时累计 CRC16 错误。 */
            uart_lb_crc16_err++;
        } else { /* 帧尚未完整时等待后续字节。 */

        }
    }
}

/* 将 BSP 串口统计复制到测试观测量。 */
static void UartLoopbackTest_RefreshStat(void) {
    const BspUartStat *s2 = BSP_UART_GetStat(UART_LB_U2);
    const BspUartStat *s3 = BSP_UART_GetStat(UART_LB_U3);
    uint8_t i;

    /* 任一测试端口统计不可用时结束刷新。 */
    if ((s2 == NULL) || (s3 == NULL)) {
        return;
    }

    /* 复制两路 HAL 错误、溢出和 DMA 重装统计。 */
    uart_lb_hal_err[UART_LB_U2] = s2->hal_err;
    uart_lb_hal_err[UART_LB_U3] = s3->hal_err;
    uart_lb_ore_cnt[UART_LB_U2] = s2->ore;
    uart_lb_ore_cnt[UART_LB_U3] = s3->ore;
    uart_lb_rearm_fail = s2->rearm_fail + s3->rearm_fail;

    uart_lb_no_dma = 0U;
    /* 遍历全部 BSP 端口生成 DMA 缺失掩码。 */
    for (i = 0U; i < BSP_UART_CNT; i++) {
        const BspUartStat *st = BSP_UART_GetStat(i);
        /* 端口缺少 DMA 配置时置位对应标志。 */
        if ((st != NULL) && (st->no_dma != 0U)) {
            uart_lb_no_dma |= (1UL << i);
        }
    }
}

/* 结算上一轮的丢帧、内容校验和分类结果。 */
static void UartLoopbackTest_Verify(void) {
    const FrameParser *p;

    /* 没有待结算轮次时结束处理。 */
    if (!s_pending) {
        return;
    }
    s_pending = 0U;

    /* 完整帧尚未就绪时记录丢帧及接收进度。 */
    if (!s_rx_ready) {

        uart_lb_lost_case = UartLoopbackTest_CaseOf(s_round - 1U);
        uart_lb_lost_state = (uint32_t)s_parser[s_rx_idx].state;
        uart_lb_lost_len = (uint32_t)s_parser[s_rx_idx].len;
        uart_lb_lost_got = uart_lb_byte_dir[s_rx_idx] - s_round_bytes_base;
        uart_lb_lost_need = (uint32_t)s_tx_len + FRAME_OVERHEAD;
        uart_lb_lost_cnt++;
        /* 累计丢帧时解析器所在状态的分布。 */
        {
            uint32_t state_now = (uint32_t)s_parser[s_rx_idx].state;
            /* 状态索引有效时更新对应桶计数。 */
            if (state_now < (uint32_t)UART_LB_STATE_BINS) {
                uart_lb_lost_state_hist[state_now]++;
            }
        }
        /* 计算缺失字节数量并归入长度区间。 */
        {
            uint32_t short_by = (uart_lb_lost_need > uart_lb_lost_got)
                                    ? (uart_lb_lost_need - uart_lb_lost_got)
                                    : 0U;
            uint32_t bin;
            /* 接收字节已达到期望长度时归入桶 0。 */
            if (short_by == 0U) {
                bin = 0U;
            } else if (short_by == 1U) { /* 缺少 1 字节时归入桶 1。 */
                bin = 1U;
            } else if (short_by == 2U) { /* 缺少 2 字节时归入桶 2。 */
                bin = 2U;
            } else if (short_by <= 7U) { /* 缺少 3 至 7 字节时归入桶 3。 */
                bin = 3U;
            } else if (short_by <= 31U) { /* 缺少 8 至 31 字节时归入桶 4。 */
                bin = 4U;
            } else { /* 缺少至少 32 字节时归入桶 5。 */
                bin = 5U;
            }
            uart_lb_lost_short[bin]++;
        }
        return;
    }
    s_rx_ready = 0U;

    p = &s_parser[s_rx_idx];
    uart_lb_rx_cnt++;
    uart_lb_last_type_id = p->type_id;
    uart_lb_last_datalen = p->data_len;
    uart_lb_last_seq = s_round - 1U;

    /* 内容不匹配时保存快照并累计异常帧。 */
    if (!UartLoopbackTest_Match(p)) {
        UartLoopbackTest_Snapshot(p);
        uart_lb_bad_cnt++;
        return;
    }

    /* 内容匹配后更新用例和方向成功计数。 */
    uart_lb_ok_cnt++;
    uart_lb_case_ok[UartLoopbackTest_CaseOf(s_round - 1U)]++;
    uart_lb_dir_ok[s_rx_idx]++;
    uart_lb_ok_dir[s_rx_idx]++;
}

/* 交替选择方向并启动一轮异步发送。 */
static void UartLoopbackTest_RunOneRound(void) {

    uint8_t u3_turn = (uint8_t)(s_round & 1U);
    uint8_t rx_idx = (u3_turn != 0U) ? UART_LB_U3 : UART_LB_U2;
    uint8_t tx_idx = (u3_turn != 0U) ? UART_LB_U2 : UART_LB_U3;
    int32_t n;

    UartLoopbackTest_Build(s_round);

    /* 发送前发布接收端并复位解析和计数基线。 */
    s_rx_idx = rx_idx;
    FrameParserReset(&s_parser[rx_idx]);
    s_rx_ready = 0U;
    s_round_bytes_base = uart_lb_byte_dir[rx_idx];

    /* 根据类型、载荷和长度编码本轮测试帧。 */
    n = FrameEncode(s_tx_frame, FRAME_MAX_LEN, s_tx_type_id, 0U,
                        (s_tx_len > 0U) ? s_tx_payload.raw : NULL, s_tx_len);
    /* 编码失败时记录错误并推进轮次。 */
    if (n < 0) {
        uart_lb_encode_err++;
        uart_lb_round = ++s_round;
        return;
    }

    /* 发送受理成功后累计发送帧数和字节数。 */
    if (BSP_UART_Send(tx_idx, s_tx_frame, (uint16_t)n) == BSP_UART_OK) {
        uart_lb_tx_cnt++;
        uart_lb_tx_bytes += (uint32_t)n;
    } else { /* 发送请求失败时累计失败次数。 */

        uart_lb_tx_fail_cnt++;
    }

    /* 标记本轮待结算并递增累计轮次。 */
    s_pending = 1U;
    uart_lb_round = ++s_round;
}

/* 检查 DMA 布局、复位状态并登记两路回调。 */
void UartLoopbackTest_Init(void) {

    uart_lb_dma_mem_bad = BSP_UART_LayoutOk();

    FrameParserReset(&s_parser[UART_LB_U2]);
    FrameParserReset(&s_parser[UART_LB_U3]);

    /* 在 BSP 初始化完成后登记两路接收回调。 */
    BSP_UART_RegisterRxCallback(UART_LB_U2, UartLoopbackTest_OnRx);
    BSP_UART_RegisterRxCallback(UART_LB_U3, UartLoopbackTest_OnRx);

    s_rx_ready = 0U;
    s_pending = 0U;
    s_round = 0U;
    s_rx_idx = UART_LB_U2;
    s_tx_type_id = FRAME_TYPE_ID_NONE;
    s_tx_len = 0U;

    UartLoopbackTest_RefreshStat();
}

/* 按测试周期刷新统计、结算并启动下一轮。 */
void UartLoopbackTest_Loop(void) {
    static uint32_t last_tick;
    static uint8_t started;

    uint32_t now = HAL_GetTick();

    /* 测试周期未到时等待下一次调用。 */
    if (started && ((uint32_t)(now - last_tick) < UART_LB_PERIOD_MS)) {
        return;
    }
    last_tick = now;
    started = 1U;

    UartLoopbackTest_RefreshStat();
    UartLoopbackTest_Verify();
    UartLoopbackTest_RunOneRound();
}

#endif
