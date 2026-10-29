#pragma once

#include <stdint.h>

#include "bsp_uart.h"
#include "test_config.h"

/* USART1 原样回显接收数据；适配器 TX 接 PA10，RX 接 PA9，并共地。 */
#define UART_ECHO_PORT BSP_UART_U1

/* 保留最近接收数据的前 64 字节供调试器读取。 */
#define UART_ECHO_LAST_LEN 64u

/* 测试启用时提供回显统计和初始化接口。 */
#if TEST_UART_ECHO_ENABLE

/* 累计接收事件、字节数、发送结果及最大接收长度。 */
extern volatile uint32_t uart_echo_rx_events;
extern volatile uint32_t uart_echo_rx_bytes;
extern volatile uint32_t uart_echo_tx_ok;
extern volatile uint32_t uart_echo_tx_busy;
extern volatile uint32_t uart_echo_tx_fail;
extern volatile uint32_t uart_echo_max_len;

/* 保存最近一段接收数据的实际长度和内容前缀。 */
extern volatile uint32_t uart_echo_last_len;
extern volatile uint8_t uart_echo_last[UART_ECHO_LAST_LEN];

/* 在 BSP UART 初始化后登记原样回显回调。 */
void UartEchoTest_Init(void);
/* 测试开关为 0 时提供初始化空入口。 */
#else

/* 测试停用时保留初始化空入口。 */
static inline void UartEchoTest_Init(void) {}
#endif
