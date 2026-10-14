#pragma once

#include <stdint.h>

#include "uart_config.h"

/* 串口传输层（BSP）。只做一件事：把 DMA 收发这套容易出错的东西封起来。
 *
 * ---- 为什么要单独一层 ----
 *
 * STM32H7 的 DMA1/DMA2 在 D2 域、是总线主设备，**物理上访问不到 DTCM/ITCM**
 * （TCM 只挂在 Cortex-M7 核上，不在 AXI/AHB 总线矩阵上）。而本工程的链接脚本把
 * .data/.bss 全放在 DTCM(0x20000000) —— 也就是说，谁自己开一块缓冲去调
 * HAL_UART_Transmit_DMA() / HAL_UARTEx_ReceiveToIdle_DMA()，谁就得记得给它挂
 * __attribute__((section(".RAM_D1")))。
 *
 * 忘了会怎样：HAL 会正常返回 HAL_OK（它只负责把
 * DMA 支起来），但流一使能，第一次访存就报传输错 → HAL_UART_ERROR_DMA(0x10)，
 * 收发全废，而**接线和 DMA 流号全是对的**。
 *
 * 这一层把缓冲收进来自己持有（见 .c 里的 BSP_UART_DMA_MEM），对外只给字节 ——
 * **调用方的数组在哪都无所谓**：栈上的、普通静态的、malloc 出来的都能直接传进来。
 *
 * ---- 分层 ----
 *
 * 本层不认识帧协议。收到的字节原样推给上层（回调），上层自己拿去喂
 * libs/frame 的 FrameParserFeed()；要发帧也是上层自己 FrameEncode()
 * 完再调 BSP_UART_Send()。解析器属于上层、是上层自己的普通变量 —— 它从来不参与
 * DMA，不需要任何段属性。（libs/frame 也因此保持不依赖 HAL。）
 *
 * ---- 用法 ----
 *
 *     // main.c 的 USER CODE 2，在 MX_USARTx_UART_Init() 之后、调度器启动之前
 *     BSP_UART_Init();                        // 把 BSP_UART_CNT 路接收全常驻武装起来
 *     BSP_UART_RegisterRxCallback(BSP_UART_U2, OnRx);
 *
 * 结果和统计都记在 BSP_UART_GetStat() 里（不用串口打印，JLink 直接读）。
 *
 * ---- 加新串口要做什么 ----
 *
 * 1. CubeMX 里给那一路上 RX+TX 两条 DMA 流（NORMAL / 逐字节 / MINC / FIFO disable /
 *    优先级 5），重新生成；2. 写进 uart_config.h 的 BSP_UART_PORT_ARRAY（加在后面）；
 * 3. 在 bsp_uart.h 里给它一个编号宏。**步骤 1 漏了不会编译报错** —— 那一路的
 *    `stat.no_dma` 会是 1、`rearm_fail` 一路涨，现象是"这一路收发全无"。
 */

/* 各路的编号 = 它在 uart_config.h 的 BSP_UART_PORT_ARRAY 里的下标。
 * **加口就是加在数组后面**，别插到中间 —— 插进去会把已有编号全挪位，
 * 所有按编号索引的东西（比如回环测试那些 uart_lb_*_dir[] 数组）都会静默错位。 */
#define BSP_UART_U1 0u /* USART1  PA9/PA10，普通异步口（不是 RS485） */
#define BSP_UART_U2 1u /* USART2  RS485 */
#define BSP_UART_U3 2u /* USART3  RS485 */

#define BSP_UART_OK        0
#define BSP_UART_ERR_PARAM (-1) /* id 越界 / 指针为空 / 长度为 0 / 超过发送缓冲 */
#define BSP_UART_ERR_BUSY  (-2) /* 上一笔还没发完，或 HAL 非 OK；本层不排队 */

/* 收到一段字节时调用。**在中断上下文里**：不要阻塞、不要打印、不要调 FreeRTOS API。
 *
 * data 指向本层内部的 DMA 缓冲，**回调返回之后内容随时可能被下一次接收覆盖** ——
 * 要留就当场拷走、或者直接喂进解析器，别把这个指针存下来。
 *
 * len 是一次事件收到的全部字节。总线上帧和帧之间必然有一段空闲，所以一次 IDLE 事件
 * 通常正好对应一个完整的包；但如果对端发了个超长的东西，也可能收到半截
 * （那种情况 stat.rx_over 会涨）。 */
typedef void (*bsp_uart_rx_cb)(uint8_t id, const uint8_t *data, uint16_t len);

/* 初始化：把两路的 DMA 接收武装起来，并清零统计。
 * 必须在 MX_USARTx_UART_Init() 之后、osKernelStart() 之前调。 */
void BSP_UART_Init(void);

/* 本层管着几路（= uart_config.h 的 BSP_UART_CNT）。 */
uint8_t BSP_UART_Count(void);

/* 登记某一路的收包回调。传 NULL 可以注销。id 越界时什么都不做。 */
void BSP_UART_RegisterRxCallback(uint8_t id, bsp_uart_rx_cb cb);

/* 发一段原始字节。
 *
 * 数据会被**拷进本层自己的 DMA 缓冲**再发，所以传进来的数组在哪都行，函数返回后
 * 就可以随便改。不阻塞：只把 DMA 支起来就返回，发完不等（一帧最长 262 字节
 * ≈ 2.84 ms @921600）。
 *
 * 上一笔还没发完就返回 BSP_UART_ERR_BUSY —— 本层不排队，重试还是丢弃由上层决定。
 *
 * DE 方向不用管：RS485 收发器的 DE 由外设硬件控制（见 Core/Src/usart.c 的
 * HAL_RS485Ex_Init），发送期间自动拉高、发完最后一个停止位自动放开。 */
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
                         /*   0x01 PE 校验错  0x02 NE 噪声  0x04 FE 帧错  0x08 ORE 溢出 */
    uint32_t ore;        /* overrun error 次数（是计数，不是累积 —— 一个就毁掉一整帧） */
    uint32_t no_dma;     /* 1 = 这一路没有 DMA 句柄（CubeMX 里没给 RX/TX 配流） */
} BspUartStat;

/* id 越界返回 NULL。 */
const BspUartStat *BSP_UART_GetStat(uint8_t id);

/* 0 = 收发缓冲确实落在 DMA 够得着的 RAM 里。
 *
 * 非 0 的时候，上面那些 tx_fail 满天飞、hal_err 里出现 0x10 都是**必然的**，
 * 别去查接线、也别去换 DMA 流号 —— 是链接脚本里手加的 .RAM_D1 段没了
 * （CubeMX 重新生成链接脚本会把它冲掉）。补回来就好，见 STM32H723xG_flash.ld 里
 * 那段注释，或者本文件上面"为什么要单独一层"那段。
 *
 * 这个只管"缓冲区在不在 DMA 够得着的 RAM 里"，**不管有没有 DMA 流** —— 后者看每路的
 * `stat.no_dma`。两个都可能让串口全哑，症状却不一样：这个出问题 hal_err 里有 0x10，
 * no_dma 出问题则连 HAL 都进不去。 */
uint8_t BSP_UART_LayoutOk(void);
