#pragma once

#include <stdint.h>

#include "test_config.h"

/* CAN1 发、CAN2 收；同名 H/L 接线，两端各接 120 Ω 终端电阻。 */
#define CAN_LB_TX_CAN    0u
#define CAN_LB_RX_CAN    1u
#define CAN_LB_ID        0x123u
/* 配置测试帧发送周期，单位 ms；两路位速率须一致。 */
#define CAN_LB_PERIOD_MS 100u

/* 测试启用时提供观测变量及回环接口。 */
#if TEST_CAN_LOOPBACK_ENABLE

/* 统计发送、合法接收、异常帧和周期丢帧数量。 */
extern volatile uint32_t can_lb_tx_cnt;
extern volatile uint32_t can_lb_rx_cnt;
extern volatile uint32_t can_lb_bad_cnt;
extern volatile uint32_t can_lb_lost_cnt;
/* 保存最近接收数据、标识符和序号。 */
extern volatile uint8_t  can_lb_last_rx[8];
extern volatile uint32_t can_lb_last_rx_id;
extern volatile uint8_t  can_lb_last_seq;

/* 保存两端错误计数及告警状态；索引 0 为发送端，1 为接收端。 */
extern volatile uint32_t can_lb_tec[2];
extern volatile uint32_t can_lb_rec[2];
extern volatile uint8_t  can_lb_busoff[2];
extern volatile uint8_t  can_lb_warn[2];

/* 在 BSP CAN 初始化后登记滤波器和接收回调。 */
void CanLoopbackTest_Init(void);
/* 推进周期发送、丢帧检查和总线状态采集。 */
void CanLoopbackTest_Loop(void);
/* 测试开关为 0 时提供空调用入口。 */
#else

/* 测试停用时保留初始化空入口。 */
static inline void CanLoopbackTest_Init(void) {}
/* 测试停用时保留周期更新空入口。 */
static inline void CanLoopbackTest_Loop(void) {}

#endif
