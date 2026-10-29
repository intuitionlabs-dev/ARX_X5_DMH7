#pragma once

#include <stdint.h>

#include "uart_config.h"

/* UART DMA 字节传输层；BSP 持有位于 .RAM_D1 的收发缓冲。 */

/* 端口编号等于 BSP_UART_PORT_ARRAY 中的索引。 */
#define BSP_UART_U1 0u /* USART1  PA9/PA10，普通异步口（不是 RS485） */
#define BSP_UART_U2 1u
#define BSP_UART_U3 2u

#define BSP_UART_OK        0
#define BSP_UART_ERR_PARAM (-1) /* id 越界 / 指针为空 / 长度为 0 / 超过发送缓冲 */
#define BSP_UART_ERR_BUSY  (-2) /* 上一笔还没发完，或 HAL 非 OK；本层不排队 */

/* 接收回调运行在中断上下文，须非阻塞；data 仅在回调期间有效。 */
typedef void (*bsp_uart_rx_cb)(uint8_t id, const uint8_t *data, uint16_t len);

/* 在串口硬件初始化完成后、调度器启动前初始化 DMA 接收和统计。 */
void BSP_UART_Init(void);

/* 本层管着几路（= uart_config.h 的 BSP_UART_CNT）。 */
uint8_t BSP_UART_Count(void);

/* 登记某一路的收包回调。传 NULL 可以注销。id 越界时什么都不做。 */
void BSP_UART_RegisterRxCallback(uint8_t id, bsp_uart_rx_cb cb);

/* 复制数据后异步发送，返回后调用方可复用数据；繁忙时返回 ERR_BUSY，由上层重试。 */
int32_t BSP_UART_Send(uint8_t id, const uint8_t *data, uint16_t len);

/* 按路的统计，给调试器（JLink）读。 */
typedef struct {
    uint32_t rx_events;  /* 进过多少次接收事件回调 */
    uint32_t rx_bytes;   /* 收到的总字节数 */
    uint32_t tx_ok;      /* 发出去且 HAL 返回 OK 的次数 */
    uint32_t tx_fail;    /* 没发出去：上一笔没发完，或 HAL 非 OK */
    uint32_t rx_over;    /* 一次收到了不低于缓冲长度的字节，尾部可能被截断 */
    uint32_t rearm_fail; /* 重新武装接收失败的次数 */
    uint32_t hal_err;    /* 见过的 HAL 错误码，按位 OR 累积 */
                         /* 0x01 PE 校验错  0x02 NE 噪声  0x04 FE 帧错  0x08 ORE 溢出 */
    uint32_t ore;        /* overrun error 次数（是计数，不是累积 —— 一个就毁掉一整帧） */
    uint32_t no_dma;     /* 1 = 这一路没有 DMA 句柄（CubeMX 里没给 RX/TX 配流） */
} BspUartStat;

/* id 越界返回 NULL。 */
const BspUartStat *BSP_UART_GetStat(uint8_t id);

/* 检查 DMA 缓冲地址，0 表示有效；DMA 句柄配置由 no_dma 标志检查。 */
uint8_t BSP_UART_LayoutOk(void);
