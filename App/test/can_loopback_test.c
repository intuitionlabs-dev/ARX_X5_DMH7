#include "can_loopback_test.h"

#include "bsp_can.h"
#include "fdcan.h"
#include "main.h"
#include "test_config.h"

/* 启用测试时编译 CAN 回环实现和观测量。 */
#if TEST_CAN_LOOPBACK_ENABLE

/* 在编译期校验发送与接收总线索引。 */
_Static_assert(CAN_LB_TX_CAN < DEVICE_CAN_CNT, "CAN_LB_TX_CAN 超出 FDCAN 实例数");
_Static_assert(CAN_LB_RX_CAN < DEVICE_CAN_CNT, "CAN_LB_RX_CAN 超出 FDCAN 实例数");

/* 累计收发、校验和丢帧计数并保存最近一帧。 */
volatile uint32_t can_lb_tx_cnt;
volatile uint32_t can_lb_rx_cnt;
volatile uint32_t can_lb_bad_cnt;
volatile uint32_t can_lb_lost_cnt;
volatile uint8_t can_lb_last_rx[8];
volatile uint32_t can_lb_last_rx_id;
volatile uint8_t can_lb_last_seq;

/* 保存发送端和接收端的 FDCAN 错误状态。 */
volatile uint32_t can_lb_tec[2];
volatile uint32_t can_lb_rec[2];
volatile uint8_t can_lb_busoff[2];
volatile uint8_t can_lb_warn[2];

/* 将 BSP 总线索引映射到 HAL FDCAN 句柄。 */
static FDCAN_HandleTypeDef *CanLoopbackTest_Bus(uint8_t can_index) {
    /* 按总线索引返回对应外设句柄。 */
    switch (can_index) {
        /* 索引 0 对应 FDCAN1。 */
        case 0: return &hfdcan1;
        /* 索引 1 对应 FDCAN2。 */
        case 1: return &hfdcan2;
        /* 索引 2 对应 FDCAN3。 */
        case 2: return &hfdcan3;
        /* 索引越界时返回空句柄。 */
        default: return 0;
    }
}

/* 根据序号构造含反码和递增字节的测试帧。 */
static void CanLoopbackTest_FillFrame(uint8_t *data, uint8_t seq) {
    data[0] = seq;
    data[1] = (uint8_t)~seq;
    /* 填充测试帧的其余字节。 */
    for (int i = 2; i < 8; ++i) {
        data[i] = (uint8_t)(seq + i);
    }
}

/* 检查回环帧标识符、长度和全部数据。 */
static int CanLoopbackTest_CheckFrame(uint32_t identifier, const uint8_t *data,
                                      uint32_t len) {
    /* 标识符或长度不匹配时报告校验失败。 */
    if (identifier != CAN_LB_ID || len != 8) {
        return 0;
    }
    uint8_t expect[8];
    CanLoopbackTest_FillFrame(expect, data[0]);
    /* 逐字节比较收到的数据与期望数据。 */
    for (int i = 0; i < 8; ++i) {
        /* 任一字节不匹配时报告校验失败。 */
        if (data[i] != expect[i]) {
            return 0;
        }
    }
    return 1;
}

/* 在接收回调中保存数据并累计校验结果。 */
static void CanLoopbackTest_RxCallback(uint8_t can_id, uint32_t identifier,
                                       uint8_t *data, basic_data_t len) {
    (void)can_id;

    can_lb_last_rx_id = identifier;
    /* 保存最近一帧数据，未接收字节补零。 */
    for (int i = 0; i < 8; ++i) {
        can_lb_last_rx[i] = (i < (int)len) ? data[i] : 0;
    }

    /* 合法帧更新最近序号和接收计数。 */
    if (CanLoopbackTest_CheckFrame(identifier, data, len)) {
        can_lb_last_seq = data[0];
        can_lb_rx_cnt++;
    } else { /* 校验失败帧累计异常计数。 */
        can_lb_bad_cnt++;
    }
}

/* 登记接收滤波器和回环接收回调。 */
void CanLoopbackTest_Init(void) {

    BSP_CAN_AddFilter(CAN_LB_RX_CAN, CAN_LB_ID);
    BSP_CAN_RegisterRxCallback(CAN_LB_RX_CAN, CanLoopbackTest_RxCallback);
}

/* 按配置周期检查丢帧、刷新总线状态并发送。 */
void CanLoopbackTest_Loop(void) {
    static uint32_t last_tick = 0;
    static uint32_t rx_seen = 0;
    static int started = 0;

    uint32_t now = HAL_GetTick();
    /* 发送周期未到时等待下一次调用。 */
    if (started && (now - last_tick) < CAN_LB_PERIOD_MS) {
        return;
    }
    last_tick = now;

    /* 上一周期没有新增合法帧时累计丢帧。 */
    if (started && can_lb_rx_cnt == rx_seen) {
        can_lb_lost_cnt++;
    }
    rx_seen = can_lb_rx_cnt;
    started = 1;

    /* 保存两端 BSP 总线索引。 */
    static const uint8_t sides[2] = {CAN_LB_TX_CAN, CAN_LB_RX_CAN};
    /* 分别读取两端协议状态和错误计数。 */
    for (int k = 0; k < 2; ++k) {
        FDCAN_HandleTypeDef *hfdcan = CanLoopbackTest_Bus(sides[k]);
        /* 句柄无效时跳过该端状态采集。 */
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

    /* 生成下一帧序号并发送测试数据。 */
    uint8_t data[8];
    CanLoopbackTest_FillFrame(data, (uint8_t)can_lb_tx_cnt);
    BSP_CAN_Send(CAN_LB_TX_CAN, CAN_LB_ID, data, 8, CAN_TX_GROUP_ALL);
    can_lb_tx_cnt++;
}

#endif
