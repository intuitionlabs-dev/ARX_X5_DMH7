#pragma once

#include <stdint.h>

#include "bsp_uart.h"
#include "test_config.h"

/* USART2 与 USART3 交替收发；RS485 的 A、B 分别同名连接。 */
#define UART_LB_U2 1u
#define UART_LB_U3 2u
/* 配置轮次周期，单位 ms；两路通信参数须一致。 */
#define UART_LB_PERIOD_MS 10u

/* 六类用例每两轮切换一次，覆盖两个传输方向。 */
#define UART_LB_CASE_CNT 6u

/* 测试启用时提供观测变量及双向回环接口。 */
#if TEST_UART_LOOPBACK_ENABLE

/* 累计发送、接收、内容匹配、丢帧和协议错误。 */
extern volatile uint32_t uart_lb_tx_cnt;
extern volatile uint32_t uart_lb_tx_fail_cnt;
extern volatile uint32_t uart_lb_rx_cnt;
extern volatile uint32_t uart_lb_ok_cnt;
extern volatile uint32_t uart_lb_bad_cnt;
extern volatile uint32_t uart_lb_lost_cnt;
extern volatile uint32_t uart_lb_crc8_err;
extern volatile uint32_t uart_lb_crc16_err;
extern volatile uint32_t uart_lb_encode_err;

/* 保存两路 BSP 接收 DMA 重装失败次数之和。 */
extern volatile uint32_t uart_lb_rearm_fail;

/* 保存 BSP DMA 缓冲区布局检查结果；0 表示有效。 */
extern volatile uint32_t uart_lb_dma_mem_bad;

/* 位 N 表示 BSP 端口 N 缺少 DMA 配置。 */
extern volatile uint32_t uart_lb_no_dma;

/* 保存各用例和接收端口的成功次数及 HAL 错误码。 */
extern volatile uint32_t uart_lb_case_ok[UART_LB_CASE_CNT];
extern volatile uint32_t uart_lb_dir_ok[BSP_UART_CNT];
extern volatile uint32_t uart_lb_hal_err[BSP_UART_CNT];

/* 按 BSP 端口索引记录字节、帧、匹配和 ORE 次数。 */
extern volatile uint32_t uart_lb_byte_dir[BSP_UART_CNT];
extern volatile uint32_t uart_lb_rx_dir[BSP_UART_CNT];
extern volatile uint32_t uart_lb_ok_dir[BSP_UART_CNT];
extern volatile uint32_t uart_lb_ore_cnt[BSP_UART_CNT];

/* 累计非本轮接收端的丢弃字节数及最近丢弃长度。 */
extern volatile uint32_t uart_lb_offdir_cnt;
extern volatile uint32_t uart_lb_offdir_size;

/* 保存丢帧轮次的解析状态、已收长度、预期长度和用例。 */
extern volatile uint32_t uart_lb_lost_state;
extern volatile uint32_t uart_lb_lost_len;
extern volatile uint32_t uart_lb_lost_got;
extern volatile uint32_t uart_lb_lost_need;
extern volatile uint32_t uart_lb_lost_case;

/* 缺失字节分桶为 0、1、2、3～7、8～31、至少 32。 */
#define UART_LB_SHORT_BINS 6U
extern volatile uint32_t uart_lb_lost_short[UART_LB_SHORT_BINS];

/* 按 FrameState 索引累计丢帧时的解析状态。 */
#define UART_LB_STATE_BINS 5U
extern volatile uint32_t uart_lb_lost_state_hist[UART_LB_STATE_BINS];

/* 保存吞吐字节数、最近帧信息及累计轮次。 */
extern volatile uint32_t uart_lb_tx_bytes;
extern volatile uint32_t uart_lb_rx_bytes;
extern volatile uint8_t uart_lb_last_type_id;
extern volatile uint8_t uart_lb_last_datalen;
extern volatile uint32_t uart_lb_last_seq;
extern volatile uint32_t uart_lb_round;

/* 在 BSP UART 初始化后登记两路回调并复位测试状态。 */
void UartLoopbackTest_Init(void);
/* 推进统计刷新、上一轮结算和下一轮发送。 */
void UartLoopbackTest_Loop(void);
/* 测试开关为 0 时提供空调用入口。 */
#else

/* 测试停用时保留初始化空入口。 */
static inline void UartLoopbackTest_Init(void) {}
/* 测试停用时保留周期更新空入口。 */
static inline void UartLoopbackTest_Loop(void) {}

#endif
