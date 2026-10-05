#pragma once

#include <stdint.h>

#include "bsp_uart.h" /* BSP_UART_U1 */
#include "test_config.h"

/* USART1 <-> PC 连通性(回显)测试
 *
 * 拓扑:本板 USART1 通过 USB 转 TTL(或 J-Link 自带的 VCOM)接 PC 的一个 tty 口。
 *
 *   接线(两头的名字都按"信号方向"看，不是按名字对接):
 *       适配器/调试器的 TX  ->  单片机 PA10(USART1_RX)
 *       适配器/调试器的 RX  <-  单片机 PA9 (USART1_TX)
 *       GND <-> GND(必须共地，只接 TX/RX 会收到一堆乱码)
 *   USART1 是**普通异步口**，不是 RS485 —— 没有 A/B、没有 DE，别当 RS485 接。
 *
 * 前提:两头波特率必须一致。当前 CubeMX 配置是 921600 8N1、无硬件流控
 * (见 Core/Src/usart.c 的 MX_USART1_UART_Init)。
 *
 * 做什么:PC 发什么，单片机原样发回去。**一个回环就把两个方向都验了** ——
 * 收到的字节和发出去的一模一样，就说明"PC 发->MCU 收"和"MCU 发->PC 收"
 * 两条线都通、波特率也对得上(对不上的话收到的会是乱码，不是原样)。
 *
 * 为什么需要这个测试:USART1 平时没有任何代码会在它上面发东西(BSP 只把它常驻
 * 武装在收)，光看"PC 发过去 MCU 收到了"只能证明一个方向。
 *
 * 用法:
 *   BSP_UART_Init();      // 必须先调(main.c 的 USER CODE BEGIN 2)
 *   UartEchoTest_Init();  // 紧跟其后 —— BSP_UART_Init() 会清空回调表，
 *                         // 注册回调必须在它之后。调度器启动前调用。
 * 没有循环体、没有任务:回显是在 BSP 的接收回调里当场发回去的(中断上下文)。
 *
 * 结果不走串口(串口本身就是被测对象)，全部记在下面的全局变量里，用调试器读。
 * 判据:
 *   uart_echo_rx_events/rx_bytes   PC 那头发了多少次/多少字节，这边就该收多少
 *   uart_echo_tx_ok                回发成功的次数(正常应该 == rx_events)
 *   uart_echo_last_len / last[]    最近一次收到的长度和内容，**直接对着 PC 发的比**
 *   uart_echo_tx_busy/tx_fail      必须恒 0(非 0 说明 PC 发得太密，上一笔还没发完)
 *   BSP_UART_GetStat(BSP_UART_U1)  -> hal_err / rx_over / no_dma 都要是 0
 *     (hal_err 里出现 0x04 = FE 帧错，通常是波特率不匹配或线太长；
 *      出现 0x02 = NE 噪声错，通常是没共地；出现 0x08 = ORE 溢出)
 */

#define UART_ECHO_PORT BSP_UART_U1 /* USART1 在 BSP 里的端口号 */

/* 最近一次收到的内容只留前这么多字节(够对着 PC 发的看一眼就行，
 * 不是接收缓冲 —— 接收缓冲在 BSP 里，有 BSP_UART_RX_BUF_LEN 那么长)。 */
#define UART_ECHO_LAST_LEN 64u

#if TEST_UART_ECHO_ENABLE

/* ---- 观测变量(JLink 直接读) ------------------------------------------- */
extern volatile uint32_t uart_echo_rx_events; /* PC -> MCU:收到多少次(一次 IDLE 事件算一次) */
extern volatile uint32_t uart_echo_rx_bytes;  /* PC -> MCU:收到多少字节 */
extern volatile uint32_t uart_echo_tx_ok;     /* MCU -> PC:成功回发多少次 */
extern volatile uint32_t uart_echo_tx_busy;   /* 回发时上一笔还没发完，这一次没回成 */
extern volatile uint32_t uart_echo_tx_fail;   /* 回发时 HAL 返回非 OK */
extern volatile uint32_t uart_echo_max_len;   /* 收到过的最大一次长度(看有没有顶到 RX 缓冲) */

extern volatile uint32_t uart_echo_last_len; /* 最近一次收到的字节数 */
extern volatile uint8_t uart_echo_last[UART_ECHO_LAST_LEN]; /* 最近一次收到的内容(前 64 字节) */

void UartEchoTest_Init(void);
#else
/* 该测试已关闭：接口退化成空函数，main.c 的调用点不用加 #if。
 * 观测变量 uart_echo_* 此时不存在，调试器里读不到。 */
static inline void UartEchoTest_Init(void) {}
#endif /* TEST_UART_ECHO_ENABLE */
