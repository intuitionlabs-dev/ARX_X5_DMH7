#include "can_loopback_test.h"

#include "bsp_can.h"
#include "fdcan.h" /* hfdcan1/2/3，用来读错误计数器 */
#include "main.h"  /* HAL_GetTick() */

/* BSP_CAN_Send/AddFilter 都用 can_id 直接索引 can_devices[]，越界会踩内存。
 * 这里在编译期挡一道：改了 CAN_LB_*_CAN 又超出实例数会直接编不过。 */
_Static_assert(CAN_LB_TX_CAN < DEVICE_CAN_CNT, "CAN_LB_TX_CAN 超出 FDCAN 实例数");
_Static_assert(CAN_LB_RX_CAN < DEVICE_CAN_CNT, "CAN_LB_RX_CAN 超出 FDCAN 实例数");

volatile uint32_t can_lb_tx_cnt;
volatile uint32_t can_lb_rx_cnt;
volatile uint32_t can_lb_bad_cnt;
volatile uint32_t can_lb_lost_cnt;
volatile uint8_t can_lb_last_rx[8];
volatile uint32_t can_lb_last_rx_id;
volatile uint8_t can_lb_last_seq;

volatile uint32_t can_lb_tec[2];
volatile uint32_t can_lb_rec[2];
volatile uint8_t can_lb_busoff[2];
volatile uint8_t can_lb_warn[2];

/* 索引 0/1/2 -> FDCAN1/2/3，和 libs/can/can_config.h 的 CAN_PORT_ARRAY 一致 */
static FDCAN_HandleTypeDef *CanLoopbackTest_Bus(uint8_t can_index) {
    switch (can_index) {
        case 0: return &hfdcan1;
        case 1: return &hfdcan2;
        case 2: return &hfdcan3;
        default: return 0;
    }
}

/* 每帧 8 字节: [0]=序号, [1]=序号取反, [2..7]=序号+i。
 * 这样既能认出"是自己的帧"，也能发现错位/丢字节。 */
static void CanLoopbackTest_FillFrame(uint8_t *data, uint8_t seq) {
    data[0] = seq;
    data[1] = (uint8_t)~seq;
    for (int i = 2; i < 8; ++i) {
        data[i] = (uint8_t)(seq + i);
    }
}

static int CanLoopbackTest_CheckFrame(uint32_t identifier, const uint8_t *data,
                                      uint32_t len) {
    if (identifier != CAN_LB_ID || len != 8) {
        return 0;
    }
    uint8_t expect[8];
    CanLoopbackTest_FillFrame(expect, data[0]);
    for (int i = 0; i < 8; ++i) {
        if (data[i] != expect[i]) {
            return 0;
        }
    }
    return 1;
}

/* 中断上下文调用：只做校验和计数，不做任何阻塞/打印。 */
static void CanLoopbackTest_RxCallback(uint8_t can_id, uint32_t identifier,
                                       uint8_t *data, basic_data_t len) {
    (void)can_id;

    can_lb_last_rx_id = identifier;
    for (int i = 0; i < 8; ++i) {
        can_lb_last_rx[i] = (i < (int)len) ? data[i] : 0;
    }

    if (CanLoopbackTest_CheckFrame(identifier, data, len)) {
        can_lb_last_seq = data[0];
        can_lb_rx_cnt++;
    } else {
        can_lb_bad_cnt++;
    }
}

void CanLoopbackTest_Init(void) {
    /* BSP_CAN_Init() 把全局滤波设成了 FDCAN_REJECT(见 libs/can/bsp_can.c:178)，
     * 所以不登记滤波器就一帧都收不到。注册函数会在 filters[] 里找第一个空槽。 */
    BSP_CAN_AddFilter(CAN_LB_RX_CAN, CAN_LB_ID);
    BSP_CAN_RegisterRxCallback(CAN_LB_RX_CAN, CanLoopbackTest_RxCallback);
}

void CanLoopbackTest_Loop(void) {
    static uint32_t last_tick = 0;
    static uint32_t rx_seen = 0;
    static int started = 0;

    uint32_t now = HAL_GetTick();
    if (started && (now - last_tick) < CAN_LB_PERIOD_MS) {
        return;
    }
    last_tick = now;

    /* 1. 先结算上一周期发出去的帧收到了没有 */
    if (started && can_lb_rx_cnt == rx_seen) {
        can_lb_lost_cnt++;
    }
    rx_seen = can_lb_rx_cnt;
    started = 1;

    /* 2. 刷新两个方向的总线健康度，供调试器观察 */
    static const uint8_t sides[2] = {CAN_LB_TX_CAN, CAN_LB_RX_CAN};
    for (int k = 0; k < 2; ++k) {
        FDCAN_HandleTypeDef *hfdcan = CanLoopbackTest_Bus(sides[k]);
        if (hfdcan == 0) {
            continue;
        }
        FDCAN_ProtocolStatusTypeDef ps = {0};
        FDCAN_ErrorCountersTypeDef ec = {0};
        HAL_FDCAN_GetProtocolStatus(hfdcan, &ps);
        HAL_FDCAN_GetErrorCounters(hfdcan, &ec);
        can_lb_tec[k] = ec.TxErrorCnt;
        can_lb_rec[k] = ec.RxErrorCnt;
        can_lb_busoff[k] = (uint8_t)ps.BusOff;
        can_lb_warn[k] = (uint8_t)ps.Warning;
    }

    /* 3. 再发下一帧 */
    uint8_t data[8];
    CanLoopbackTest_FillFrame(data, (uint8_t)can_lb_tx_cnt);
    BSP_CAN_Send(CAN_LB_TX_CAN, CAN_LB_ID, data, 8);
    can_lb_tx_cnt++;
}
