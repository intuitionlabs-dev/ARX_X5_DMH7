#pragma once

/* 本工程自己的 UART 配置（libs/uart 这一层）。
 *
 * 照 libs/can/can_config.h 的路子：板级相关的宏收在这里，只有 bsp_uart.c include 它。
 * 换板子、加减串口只改这个文件。 */

#include "usart.h" /* huart1 / huart2 / huart3 / huart7 / huart10 */

/* 本层管几路，以及分别是谁。数组顺序就是 BSP_UART_U2/U3 这些下标。
 *
 * **只列挂了 DMA 的两路**：USART2 = DMA1_Stream1(RX)/Stream2(TX)，
 * USART3 = DMA2_Stream0(RX)/Stream1(TX)（见 Core/Src/usart.c 里的 __HAL_LINKDMA）。
 * 本层收发都走 DMA，没挂流的串口放进来只会一直 HAL_ERROR。
 *
 * 以后要接 USART1 / UART7 / USART10：先在 CubeMX 里给那一路的 RX 和 TX 各配一条 DMA 流
 * （NORMAL 模式、Byte 对齐、优先级随意），重新生成代码，再把名字加进下面这个数组。 */
#define BSP_UART_CNT 2
#define BSP_UART_PORT_ARRAY {&huart2, &huart3}

/* 接收落地缓冲的长度。
 *
 * **比最长的一帧长一截是故意的**：用 ReceiveToIdle 收包时，"把缓冲收满"和"总线空闲"
 * 是两种不同的事件，都会进回调。留出余量之后，正常帧永远填不满缓冲，一次收帧就只会走
 * IDLE 这一条路，不用去分"是收满了还是收完了"。
 *
 * 这里的 294 是照着 libs/uart_frame 最长的帧(262)填的 —— 本层其实不认识帧协议，
 * 这个数只影响"多大的包会被截断"。改小不会出错，只会多出一种"收满"事件。 */
#define BSP_UART_RX_BUF_LEN 294

/* 发送缓冲长度。BSP_UART_Send() 会把调用方的数据**拷进来**再发，所以这里要放得下
 * 上层最大的那一帧。262 = libs/uart_frame 的最大帧长。 */
#define BSP_UART_TX_BUF_LEN 262
