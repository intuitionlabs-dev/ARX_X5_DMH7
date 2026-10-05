#include "uart_loopback_test.h"

#include <string.h> /* memcmp */

#include "bsp_uart.h"   /* 串口传输层：DMA 收发 + .RAM_D1 缓冲都封在它里面 */
#include "main.h"       /* HAL_GetTick() */
#include "test_config.h" /* 测试例程开关 */
#include "uart_frame.h" /* 帧协议 + 自动解包 union */

#if TEST_UART_LOOPBACK_ENABLE

/* 本测试自己的编号和 BSP 的编号必须是一回事。写成 _Static_assert 而不是注释：
 * 对不上时整个测试的语义就反了(收的字节全被当成"非本轮接收方"丢掉)，
 * 而现象只是"ok_cnt 不涨"，很难往这上面想。 */
_Static_assert((UART_LB_U2 == BSP_UART_U2) && (UART_LB_U3 == BSP_UART_U3),
    "UART_LB_U2/U3 必须和 BSP_UART_U2/U3 一致");

/* ==================== 观测变量(JLink 直接读) ==================== */
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

volatile uint32_t uart_lb_case_ok[UART_LB_CASE_CNT];
volatile uint32_t uart_lb_dir_ok[2];
volatile uint32_t uart_lb_hal_err[2];

volatile uint32_t uart_lb_byte_dir[2];
volatile uint32_t uart_lb_rx_dir[2];
volatile uint32_t uart_lb_ok_dir[2];
volatile uint32_t uart_lb_ore_cnt[2];
volatile uint32_t uart_lb_offdir_cnt;
volatile uint32_t uart_lb_offdir_size;

/* 抓一次"对不上"的现场：收到的 vs 期望的。看一眼就知道是长度/类型错了还是内容错了。 */
volatile uint32_t uart_lb_bad_dir;
volatile uint32_t uart_lb_bad_got_type;
volatile uint32_t uart_lb_bad_got_len;
volatile uint32_t uart_lb_bad_exp_type;
volatile uint32_t uart_lb_bad_exp_len;
volatile uint32_t uart_lb_bad_got_hdr; /* 收到的 payload 前 4 字节 */
volatile uint32_t uart_lb_bad_exp_hdr; /* 期望的 payload 前 4 字节 */
volatile uint32_t uart_lb_bad_wire;    /* 收到的原始帧头: AA 55 tid len(按小端读成 32 位) */
volatile uint32_t uart_lb_bad_pay_ok;  /* payload 是否一致(0/1)，用来把"长度错"和"内容错"分开 */

/* 抓一次"丢帧"的现场：那一轮到底收进来多少字节、解析器停在哪。 */
volatile uint32_t uart_lb_lost_state; /* 判 lost 时解析器的状态(0=等AA 1=等55 2=收头 3=收数据 4=收CRC) */
volatile uint32_t uart_lb_lost_len;   /* 判 lost 时解析器已攒的字节数 */
volatile uint32_t uart_lb_lost_got;   /* 那一轮该路实际收到多少字节 */
volatile uint32_t uart_lb_lost_need;  /* 那个用例本来该有多少字节(整帧) */
volatile uint32_t uart_lb_lost_case;
/* 丢帧时"还差几个字节"的直方图。分桶: 0 / 1 / 2 / 3..7 / 8..31 / >=32。
 * 全挤在 [1] 就是帧尾被截(DE 放开早了那种)；铺开到各个桶就是结算早于收完。 */
#define UART_LB_SHORT_BINS 6U
volatile uint32_t uart_lb_lost_short[UART_LB_SHORT_BINS];
volatile uint32_t uart_lb_lost_state_hist[UART_LB_STATE_BINS];

volatile uint32_t uart_lb_dma_mem_bad; /* 非 0 = DMA 缓冲区没落在 DMA 够得着的 RAM 里，见 Init() */

volatile uint32_t uart_lb_tx_bytes;
volatile uint32_t uart_lb_rx_bytes;
volatile uint8_t uart_lb_last_type_id;
volatile uint8_t uart_lb_last_datalen;
volatile uint32_t uart_lb_last_seq;
volatile uint32_t uart_lb_round;

/* ==================== 内部状态 ====================
 *
 * 收发底座整体搬到了 libs/uart 的 BSP 层：DMA 缓冲、武装、重装、错误恢复、
 * .RAM_D1 段属性全在那边，这里只做"造帧 → 发 → 收字节 → 摆帧 → 比对"。
 *
 * 和上一版(自己管 DMA)最大的区别：**两路的接收从此是常驻武装的** ——
 * 收完一帧，BSP 在自己的回调里立刻重装，不再需要上层挑着时机 Arm()/ServiceRearm()，
 * 也不再有"换方向就 abort 掉另一路"这套动作(那套动作正是上一版假 RxEvent 的来源，
 * 见头文件"实测过的坑")。
 *
 * 闸门因此也变简单了：既然两路都在收，就用 s_rx_idx 指定"这一轮谁说的话算数"，
 * 另一路收到的字节一律丢弃并计 uart_lb_offdir_cnt(见 UartLoopbackTest_OnRx)。
 */

/* 每路一个解析器(UartFrameParser 约 270 字节)。**普通静态即可** —— 解析器是上层
 * 自己的变量，从来不参与 DMA，不需要任何段属性；只有 BSP 的收发缓冲才需要。 */
static UartFrameParser s_parser[2];

/* 本轮接收方刚解出一整帧。中断里置，任务里读并清。 */
static volatile uint8_t s_rx_ready;

/* 本轮发出去的东西，下一轮结算时拿来比。用 UartFramePayload 而不是裸数组，
 * 这样"期望值"和"收到的值"走的是同一个 union 视图，比较时两边都不用强转。 */
static UartFramePayload s_tx_payload;
/* 发送源。**普通静态**：BSP_UART_Send() 会把它拷进自己的 .RAM_D1 缓冲再发。 */
static uint8_t s_tx_frame[UART_FRAME_MAX_LEN];
static UartFrameTypeId s_tx_type_id;
static uint8_t s_tx_len;

static uint8_t s_rx_idx;  /* 本轮的接收方，0/1 */
static uint8_t s_pending; /* 有一轮发出去了还没结算 */
static uint32_t s_round;

/* 本轮开始时接收方那一路的字节计数。结算时相减 = 这一轮它到底收到多少字节，
 * 跟"该收多少"一比就能分开两种丢帧：字节没到齐(线/中断丢了) vs 字节到齐了但没凑成帧。 */
static uint32_t s_round_bytes_base;

/* 边界帧用的码：0x7F 现在是没分配的。选个没分配的码是**故意的** —— 它顺带证明帧层
 * 不认识 type_id：只要长度和 CRC 对就收，不因为码没见过就拒。 */
#define UART_LB_TYPE_ID_UNASSIGNED 0x7FU

/* 空包用例轮转用的码表。这几个包"语义全在 type_id 上"，payload 为空。 */
static const UartFrameTypeId s_empty_ids[] = {
    UART_FRAME_TYPE_ID_MOTOR_SET_ZERO,
    UART_FRAME_TYPE_ID_ARM_START,
    UART_FRAME_TYPE_ID_ARM_GRAVITY,
    UART_FRAME_TYPE_ID_ARM_STOP,
    UART_FRAME_TYPE_ID_ARM_DAMPING,
};
#define UART_LB_EMPTY_CNT (sizeof s_empty_ids / sizeof s_empty_ids[0])

/* ==================== 造帧 ====================
 *
 * 值全部用轮次 seq 播种：上一轮残留的帧(哪怕是合法的)对不上这一轮的值，
 * 不可能蒙混过关 —— 这是"丢了但没发现"唯一的防线，因为帧层没有序号，
 * 它自己发现不了丢帧(见 uart_frame.h 解析器那段说明)。
 * 结果全部写进 s_tx_type_id / s_tx_len / s_tx_payload 三个静态量，**不返回值** ——
 * 之前返回长度、由调用方再往 s_tx_len 里存，结果漏存了一次，所有帧都被判成错的。
 * 现在只有一处出口，漏不掉。 */
/* 第 seq 轮该用哪个用例。
 *
 * **除数为什么是 2**：方向每轮换一次(`s_round & 1`)，用例要是也每轮加一，
 * 那方向就和 `seq % 6` 锁死了 —— 偶数轮永远是 U2 收、奇数轮永远是 U3 收，
 * 于是 U2 一辈子只收到偶数码的包(版本/设零回复/位置+速度)，**262 字节的边界帧
 * 和空包永远轮不到 U2 收**，而这恰恰是配置出错时最先暴露的那个用例。
 * 除以 2 让每个用例被两个方向各收一次。 */
static uint8_t UartLoopbackTest_CaseOf(uint32_t seq) {
    return (uint8_t)((seq / 2U) % UART_LB_CASE_CNT);
}

static void UartLoopbackTest_Build(uint32_t seq) {
    UartFrameTypeId id = UART_FRAME_TYPE_ID_NONE;
    uint8_t len = 0U;
    uint32_t i;

    switch (UartLoopbackTest_CaseOf(seq)) {
    case 0: /* 版本匹配包：4 字节，唯一一个真去算哈希的用例 */
        s_tx_payload.version.code = UartFrameVersionCode();
        len = (uint8_t)sizeof(UartPktVersion);
        id = UART_FRAME_TYPE_ID_VERSION;
        break;

    case 1: /* 空包：datalen 必须是 0，整帧只剩 7 字节 */
        id = s_empty_ids[(seq / (UART_LB_CASE_CNT * 2U)) % UART_LB_EMPTY_CNT];
        len = 0U;
        break;

    case 2: /* 设零回复包：12 字节，里面有个 uint32_t，验非对齐的 4 字节访问 */
        s_tx_payload.motor_set_zero_ack.state = (uint8_t)(seq & 0xFFU);
        for (i = 0U; i < UART_FRAME_PKT_MOTOR_NUM; i++) {
            s_tx_payload.motor_set_zero_ack.result[i] = (uint8_t)(seq + i);
        }
        s_tx_payload.motor_set_zero_ack.elapsed_ms = 1000U + seq;
        len = (uint8_t)sizeof(UartPktMotorSetZeroAck);
        id = UART_FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK;
        break;

    case 3: /* 位置控制包：7 个 float，全落在非 4 倍数地址上 */
        for (i = 0U; i < UART_FRAME_PKT_JOINT_NUM; i++) {
            s_tx_payload.arm_ctrl_pos.joint_angle[i] =
                (float)((int32_t)(seq % 200U) - 100) + (float)i * 0.25f;
        }
        s_tx_payload.arm_ctrl_pos.gripper_angle = (float)(seq % 90U) * 0.5f;
        len = (uint8_t)sizeof(UartPktArmCtrlPos);
        id = UART_FRAME_TYPE_ID_ARM_CTRL_POS;
        break;

    case 4: /* 位置+速度控制包：13 个 float，最长的一包 */
        for (i = 0U; i < UART_FRAME_PKT_JOINT_NUM; i++) {
            s_tx_payload.arm_ctrl_pos_vel.joint_angle[i] =
                (float)((int32_t)(seq % 360U) - 180) * 0.5f;
            s_tx_payload.arm_ctrl_pos_vel.joint_vel[i] =
                (float)((int32_t)(seq % 100U) - 50) * 1.5f;
        }
        s_tx_payload.arm_ctrl_pos_vel.gripper_angle = (float)(seq % 90U) * 0.5f;
        len = (uint8_t)sizeof(UartPktArmCtrlPosVel);
        id = UART_FRAME_TYPE_ID_ARM_CTRL_POS_VEL;
        break;

    default: /* 边界帧：payload 顶满 255 字节，整帧 262 —— UartFrameBuf 的上限 */
        for (i = 0U; i < UART_FRAME_MAX_DATA; i++) {
            s_tx_payload.raw[i] = (uint8_t)(seq + i);
        }
        len = (uint8_t)UART_FRAME_MAX_DATA;
        id = UART_LB_TYPE_ID_UNASSIGNED;
        break;
    }

    /* 三样一起存：type_id、长度、payload。结算时拿它们跟收到的比，
     * 缺一样就会把对的帧判成错的 —— 加新用例时记得是**三**样。 */
    s_tx_type_id = id;
    s_tx_len = len;
}

/* ==================== 校验 ==================== */

/* 收到的帧和本轮发出去的对不对得上。两层：
 *   - 字节级(memcmp)：证明线上一字不差。
 *   - 按类型读字段：**这一层才是真正验非对齐访问的**。memcmp 走的还是字节，
 *     而包里的 float/uint32 落在帧偏移 5 这种非 4 倍数地址上，只有当成 float 读
 *     一遍才知道 pack(1) 有没有真的让 GCC 生成逐字节访问的代码。
 *     float 直接 == 比是故意的：同一份二进制值来回一趟必须逐位相等。 */
/* 记一次"对不上"的现场，供调试器看。只记最后一次，够定位了。 */
static void UartLoopbackTest_Snapshot(const UartFrameParser *p) {
    uint32_t got_hdr = 0U;
    uint32_t exp_hdr = 0U;
    uint32_t wire = 0U;

    memcpy(&got_hdr, p->buf.f.payload.raw, sizeof got_hdr);
    memcpy(&exp_hdr, s_tx_payload.raw, sizeof exp_hdr);
    memcpy(&wire, p->buf.raw, sizeof wire); /* AA 55 tid len */

    uart_lb_bad_dir = s_rx_idx;
    uart_lb_bad_got_type = p->type_id;
    uart_lb_bad_got_len = p->data_len;
    uart_lb_bad_exp_type = s_tx_type_id;
    uart_lb_bad_exp_len = s_tx_len;
    uart_lb_bad_got_hdr = got_hdr;
    uart_lb_bad_exp_hdr = exp_hdr;
    uart_lb_bad_wire = wire;
    /* 把"长度/类型错"和"内容错"分开：前者帧头就对不上，后者帧头对但字节不同 */
    uart_lb_bad_pay_ok = ((s_tx_len == 0U) ||
                          (memcmp(s_tx_payload.raw, p->buf.f.payload.raw, s_tx_len) == 0U))
                             ? 1U
                             : 0U;
}

static int UartLoopbackTest_Match(const UartFrameParser *p) {
    uint32_t i;

    if ((p->type_id != s_tx_type_id) || (p->data_len != s_tx_len)) {
        return 0;
    }
    if ((s_tx_len > 0U) && (memcmp(s_tx_payload.raw, p->buf.f.payload.raw, s_tx_len) != 0)) {
        return 0;
    }

    switch (p->type_id) {
    case UART_FRAME_TYPE_ID_VERSION:
        return (p->buf.f.payload.version.code == s_tx_payload.version.code);

    case UART_FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK: {
        const UartPktMotorSetZeroAck *got = &p->buf.f.payload.motor_set_zero_ack;
        const UartPktMotorSetZeroAck *exp = &s_tx_payload.motor_set_zero_ack;
        if ((got->state != exp->state) || (got->elapsed_ms != exp->elapsed_ms)) {
            return 0;
        }
        for (i = 0U; i < UART_FRAME_PKT_MOTOR_NUM; i++) {
            if (got->result[i] != exp->result[i]) {
                return 0;
            }
        }
        return 1;
    }

    case UART_FRAME_TYPE_ID_ARM_CTRL_POS: {
        const UartPktArmCtrlPos *got = &p->buf.f.payload.arm_ctrl_pos;
        const UartPktArmCtrlPos *exp = &s_tx_payload.arm_ctrl_pos;
        for (i = 0U; i < UART_FRAME_PKT_JOINT_NUM; i++) {
            if (got->joint_angle[i] != exp->joint_angle[i]) {
                return 0;
            }
        }
        return (got->gripper_angle == exp->gripper_angle);
    }

    case UART_FRAME_TYPE_ID_ARM_CTRL_POS_VEL: {
        const UartPktArmCtrlPosVel *got = &p->buf.f.payload.arm_ctrl_pos_vel;
        const UartPktArmCtrlPosVel *exp = &s_tx_payload.arm_ctrl_pos_vel;
        for (i = 0U; i < UART_FRAME_PKT_JOINT_NUM; i++) {
            if ((got->joint_angle[i] != exp->joint_angle[i]) ||
                (got->joint_vel[i] != exp->joint_vel[i])) {
                return 0;
            }
        }
        return (got->gripper_angle == exp->gripper_angle);
    }

    default:
        /* 空包和边界帧没有带类型的字段可读(空包 sizeof 为 0)，上面 memcmp 已经够了 */
        return 1;
    }
}

/* ==================== 收 ====================
 *
 * BSP 每收完一段(总线空闲)就调这里，**在中断上下文**：只做"闸门 + 喂解析器 + 计数"，
 * 绝不阻塞、绝不打印。data 指 BSP 内部的 DMA 缓冲，返回后随时被下一次接收覆盖 ——
 * 这里当场喂进解析器，不存指针。
 *
 * 闸门是 s_rx_idx(本轮指定谁收)，**不是**"谁武装了"。现在两路都常驻武装，发送方那一路
 * 也会看到总线上的字节(它自己的收发器 RE̅ 是否跟着 DE 走，取决于板子)；那些字节一律
 * 丢弃，既不喂解析器也不计进 byte_dir，否则发送方会把自己发的那帧也解出来 ——
 * 那正是上一版 offdir_cnt ≈ 轮数的现象。 */
static void UartLoopbackTest_OnRx(uint8_t id, const uint8_t *data, uint16_t len) {
    uint32_t i;

    if (id != s_rx_idx) {
        uart_lb_offdir_cnt += (uint32_t)len; /* 非本轮接收方：丢字节，不丢事件 */
        uart_lb_offdir_size = (uint32_t)len;
        return;
    }

    uart_lb_rx_bytes += (uint32_t)len;
    uart_lb_byte_dir[id] += (uint32_t)len;

    for (i = 0U; i < (uint32_t)len; i++) {
        UartFrameStatus st = UartFrameParserFeed(&s_parser[id], data[i]);

        if (st == UART_FRAME_OK) {
            s_rx_ready = 1U;
            uart_lb_rx_dir[id]++;
        } else if (st == UART_FRAME_ERR_CRC8) {
            uart_lb_crc8_err++;
        } else if (st == UART_FRAME_ERR_CRC16) {
            uart_lb_crc16_err++;
        } else {
            /* INCOMPLETE：绝大多数字节走这条，正常 */
        }
    }
}

/* ==================== 统计搬运 ====================
 *
 * BSP 的统计(JLink 读得到，但名字是 bsp_uart 那边的)每轮搬进本测试的 uart_lb_* 变量，
 * 这样观测习惯不变：出错时还是看 uart_lb_hal_err / uart_lb_ore_cnt / uart_lb_rearm_fail。
 * 10 ms 搬一次，对调试器来说够实时了。 */
static void UartLoopbackTest_RefreshStat(void) {
    const BspUartStat *s2 = BSP_UART_GetStat(UART_LB_U2);
    const BspUartStat *s3 = BSP_UART_GetStat(UART_LB_U3);

    if ((s2 == NULL) || (s3 == NULL)) {
        return;
    }

    uart_lb_hal_err[UART_LB_U2] = s2->hal_err;
    uart_lb_hal_err[UART_LB_U3] = s3->hal_err;
    uart_lb_ore_cnt[UART_LB_U2] = s2->ore;
    uart_lb_ore_cnt[UART_LB_U3] = s3->ore;
    uart_lb_rearm_fail = s2->rearm_fail + s3->rearm_fail;
}

/* ==================== 结算 ==================== */

static void UartLoopbackTest_Verify(void) {
    const UartFrameParser *p;

    if (!s_pending) {
        return;
    }
    s_pending = 0U;

    if (!s_rx_ready) {
        /* 丢帧现场：s_round 此时还没 ++，所以 s_round-1 就是刚结算的这一轮。
         *   got == need → 字节全到了，是解析器/标志的问题；
         *   got <  need → 真有字节没收到，看差多少：差 1 通常是帧头那个 AA 被打断。
         * state/len 是解析器停在哪儿：len==0 说明它根本没同步上(帧头没看见)。 */
        uart_lb_lost_case = UartLoopbackTest_CaseOf(s_round - 1U);
        uart_lb_lost_state = (uint32_t)s_parser[s_rx_idx].state;
        uart_lb_lost_len = (uint32_t)s_parser[s_rx_idx].len;
        uart_lb_lost_got = uart_lb_byte_dir[s_rx_idx] - s_round_bytes_base;
        uart_lb_lost_need = (uint32_t)s_tx_len + UART_FRAME_OVERHEAD;
        uart_lb_lost_cnt++;
        {
            uint32_t state_now = (uint32_t)s_parser[s_rx_idx].state;
            if (state_now < (uint32_t)UART_LB_STATE_BINS) {
                uart_lb_lost_state_hist[state_now]++;
            }
        }
        {
            uint32_t short_by = (uart_lb_lost_need > uart_lb_lost_got)
                                    ? (uart_lb_lost_need - uart_lb_lost_got)
                                    : 0U;
            uint32_t bin;
            if (short_by == 0U) {
                bin = 0U;
            } else if (short_by == 1U) {
                bin = 1U;
            } else if (short_by == 2U) {
                bin = 2U;
            } else if (short_by <= 7U) {
                bin = 3U;
            } else if (short_by <= 31U) {
                bin = 4U;
            } else {
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

    if (!UartLoopbackTest_Match(p)) {
        UartLoopbackTest_Snapshot(p);
        uart_lb_bad_cnt++;
        return;
    }

    uart_lb_ok_cnt++;
    uart_lb_case_ok[UartLoopbackTest_CaseOf(s_round - 1U)]++;
    uart_lb_dir_ok[s_rx_idx]++;
    uart_lb_ok_dir[s_rx_idx]++;
}

static void UartLoopbackTest_RunOneRound(void) {
    uint8_t rx_idx = (uint8_t)(s_round & 1U);
    uint8_t tx_idx = (uint8_t)(1U - rx_idx);
    int32_t n;

    UartLoopbackTest_Build(s_round);

    /* 顺序很重要：**先**把闸门和解析器摆好，**再**发。
     *
     * 接收方是常驻武装的(BSP 收完一帧自己重装)，所以这里不是"武装"，而是"宣布
     * 从现在起哪一路算数"：s_rx_idx 一翻，上一轮接收方后来才到的残帧就自动作废。
     * 反过来的话帧头那几个字节会在解析器还没清干净的时候进来，整帧报废 ——
     * 而且这事和接线对不对没关系。 */
    s_rx_idx = rx_idx;
    UartFrameParserReset(&s_parser[rx_idx]);
    s_rx_ready = 0U;
    s_round_bytes_base = uart_lb_byte_dir[rx_idx]; /* 这一轮"收到多少字节"从这个值起算 */

    n = UartFrameEncode(s_tx_frame, UART_FRAME_MAX_LEN, s_tx_type_id,
                        (s_tx_len > 0U) ? s_tx_payload.raw : NULL, s_tx_len);
    if (n < 0) {
        uart_lb_encode_err++;
        uart_lb_round = ++s_round;
        return;
    }

    /* DMA 发送。s_tx_frame 只是个普通静态数组，内存段的事 BSP 在内部处理
     * (它会把数据拷进自己的 .RAM_D1 缓冲)。DE 由 RS485 外设硬件控制，也不用管。
     *
     * 不阻塞：一帧最长 262 字节 ≈ 2.84 ms，而一轮 10 ms —— 发完不等，
     * 下一轮开头再结算。 */
    if (BSP_UART_Send(tx_idx, s_tx_frame, (uint16_t)n) == BSP_UART_OK) {
        uart_lb_tx_cnt++;
        uart_lb_tx_bytes += (uint32_t)n;
    } else {
        /* 上一轮还没发完(BSP_UART_ERR_BUSY)。一轮 10 ms、最长一帧 2.84 ms，
         * 正常不该出现；真出现就是 TC 中断被挡得太久了。 */
        uart_lb_tx_fail_cnt++;
    }

    s_pending = 1U;
    uart_lb_round = ++s_round;
}

void UartLoopbackTest_Init(void) {
    /* 这里**不再**动 USART2/3 的中断优先级。
     * 改 DMA 之前是有这一步的：那会儿是逐字节 HAL_UART_Receive_IT，优先级 5 正好等于
     * FreeRTOSConfig.h 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，临界区一挡
     * 就丢字节。现在字节由 DMA 搬，USART 中断只管 IDLE，晚几微秒不丢数据 ——
     * 所以那个提优先级的补丁不需要了(它还会去抢 CAN/定时器的中断，能不加就不加)。
     * 全工程其它用串口收包的地方如果不是 DMA，那条仍然成立。 */

    /* 兜底：确认 BSP 的收发缓冲真的落在 DMA 够得着的 RAM 里(链接脚本的 .RAM_D1 段要是
     * 被 CubeMX 重新生成时冲掉了，那些缓冲会变孤儿段、静默回到 DTCM)。变 1 的时候
     * 别再怀疑接线和 DMA 流号 —— 症状一定是 tx_cnt 几乎不涨 + hal_err 出现 0x10。 */
    uart_lb_dma_mem_bad = BSP_UART_LayoutOk();

    UartFrameParserReset(&s_parser[UART_LB_U2]);
    UartFrameParserReset(&s_parser[UART_LB_U3]);

    /* 注册回调。**必须在 BSP_UART_Init() 之后**(BSP 初始化会清回调表)，
     * 见 main.c 里的调用顺序。 */
    BSP_UART_RegisterRxCallback(UART_LB_U2, UartLoopbackTest_OnRx);
    BSP_UART_RegisterRxCallback(UART_LB_U3, UartLoopbackTest_OnRx);

    s_rx_ready = 0U;
    s_pending = 0U;
    s_round = 0U;
    s_rx_idx = UART_LB_U2;
    s_tx_type_id = UART_FRAME_TYPE_ID_NONE;
    s_tx_len = 0U;
}

void UartLoopbackTest_Loop(void) {
    static uint32_t last_tick;
    static uint8_t started;

    uint32_t now = HAL_GetTick();

    if (started && ((uint32_t)(now - last_tick) < UART_LB_PERIOD_MS)) {
        return;
    }
    last_tick = now;
    started = 1U;

    UartLoopbackTest_RefreshStat(); /* 先把 BSP 的统计搬进 uart_lb_* 观测变量 */
    UartLoopbackTest_Verify();      /* 再结算上一轮 */
    UartLoopbackTest_RunOneRound();
}

#endif /* TEST_UART_LOOPBACK_ENABLE */
