#pragma once

/* 配置串口实例及 DMA 缓冲容量。 */

#include "usart.h" /* huart1 / huart2 / huart3 / huart7 / huart10 */

/* 数组顺序定义 BSP 端口编号；各实例须配置 RX 和 TX DMA。 */
#define BSP_UART_CNT 3
#define BSP_UART_PORT_ARRAY {&huart1, &huart2, &huart3}

/* 接收容量大于常规帧长，以接收空闲事件提交数据。 */
#define BSP_UART_RX_BUF_LEN 294

/* 发送缓冲容量决定单次发送长度上限。 */
#define BSP_UART_TX_BUF_LEN 262
