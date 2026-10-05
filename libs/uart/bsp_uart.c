#include "bsp_uart.h"

#include <string.h> /* memcpy */

/* ==================== 收发缓冲 ====================
 *
 * 这两个数组就是这一层存在的理由：它们必须落在 DMA 够得着的 RAM 里(见 bsp_uart.h
 * 开头那段)。放在这里之后，**调用方传什么数组进来都行** —— 栈上的、普通静态的、
 * 甚至 malloc 出来的，BSP_UART_Send() 会拷进来再发，上层再也不用背内存段这件事。
 *
 * .RAM_D1 段在 STM32H723xG_flash.ld 里(RAM_D1 / AXI SRAM, 0x24000000)。该段是 NOLOAD，
 * 启动代码不清零它 —— 两块缓冲每次收发前都会被 DMA 重新填，不依赖初值，没关系。 */
#define BSP_UART_DMA_MEM __attribute__((section(".RAM_D1"), aligned(32)))

BSP_UART_DMA_MEM static uint8_t s_rx_dma[BSP_UART_CNT][BSP_UART_RX_BUF_LEN];
BSP_UART_DMA_MEM static uint8_t s_tx_frame[BSP_UART_CNT][BSP_UART_TX_BUF_LEN];

/* ==================== 每口一份的状态 ==================== */

/* uart_config.h 是唯一 include usart.h 的地方，&huart2 / &huart3 从这里来。 */
static UART_HandleTypeDef *const s_port[BSP_UART_CNT] = BSP_UART_PORT_ARRAY;

static bsp_uart_rx_cb s_rx_cb[BSP_UART_CNT];
static BspUartStat s_stat[BSP_UART_CNT];

/* DMA 接收缓冲区所在的最小地址(RAM_D1 + D2 域 SRAM1/2/3，都在 AXI/AHB 上，
 * 也就是 DMA 够得着的那些)。DTCM/ITCM 在 0x20000000/0x00000000，比它低。 */
#define BSP_UART_DMA_ADDR_MIN 0x24000000UL

static int32_t BspUart_Find(const UART_HandleTypeDef *huart) {
    uint8_t i;

    for (i = 0U; i < BSP_UART_CNT; i++) {
        if (s_port[i] == huart) {
            return (int32_t)i;
        }
    }
    return -1; /* 不是本层管的串口(huart1/huart7/huart10)，一律不管 */
}

/* 武装(或重新武装)某一路的接收。
 *
 * 一帧收完(IDLE)之后 HAL 会自己把接收停掉 —— 它会清 DMAR、把 RxState 打回 READY、
 * 顺手 abort 掉 DMA 流(见 Drivers/.../stm32h7xx_hal_uart.c 里 IDLE 那一段)。所以
 * **每收完一帧都必须重装一次**，不是可选的"保险"。收满整块缓冲(TC)那条路也一样，
 * 收完同样是 READY。 */
static void BspUart_Arm(uint8_t idx) {
    UART_HandleTypeDef *huart = s_port[idx];

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_dma[idx], BSP_UART_RX_BUF_LEN) != HAL_OK) {
        s_stat[idx].rearm_fail++;
        return;
    }

    /* 半传输中断(收满一半)对我们没有意义：缓冲比最长帧长一截，正常帧永远灌不满，
     * 只会在 IDLE 上报。留着它只会让每帧多进一次回调，喂进去半帧残渣。
     *
     * 这里是安全的：HAL_DMA_Start_IT() 只在 XferHalfCpltCallback 非空时开 HT 中断，
     * 而 UART_Start_Receive_DMA() 正好给它装了一个，所以必须关。关之前不可能已经进过
     * 一次 HT —— 半传输要收够 147 字节，在 921600 下是 1.6 ms，而这几行是同一个
     * 中断上下文里的顺序代码(DMA 中断优先级和我们一样是 5，抢不进来)。 */
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void BSP_UART_Init(void) {
    uint8_t i;

    for (i = 0U; i < BSP_UART_CNT; i++) {
        s_rx_cb[i] = NULL;
        memset(&s_stat[i], 0, sizeof s_stat[i]);
        BspUart_Arm(i);
    }
}

uint8_t BSP_UART_Count(void) {
    return (uint8_t)BSP_UART_CNT;
}

void BSP_UART_RegisterRxCallback(uint8_t id, bsp_uart_rx_cb cb) {
    if (id >= BSP_UART_CNT) {
        return;
    }
    s_rx_cb[id] = cb;
}

int32_t BSP_UART_Send(uint8_t id, const uint8_t *data, uint16_t len) {
    UART_HandleTypeDef *huart;

    if ((id >= BSP_UART_CNT) || (data == NULL) || (len == 0U) ||
        ((uint32_t)len > BSP_UART_TX_BUF_LEN)) {
        return BSP_UART_ERR_PARAM;
    }

    huart = s_port[id];

    /* 上一笔还没发完就直说。**不排队** —— 排队要动态内存或者环形队列，
     * 而"发不出去"这件事上层比本层更清楚该怎么处理(丢帧 / 下轮重发 / 报错)。 */
    if (huart->gState != HAL_UART_STATE_READY) {
        s_stat[id].tx_fail++;
        return BSP_UART_ERR_BUSY;
    }

    /* 拷进自己的 DMA 缓冲再发。这一步就是"调用方不用管内存段"的代价：
     * 一帧最多 262 字节，memcpy 几十纳秒，换掉一整类上机才炸的 bug，值。 */
    memcpy(s_tx_frame[id], data, (size_t)len);

    if (HAL_UART_Transmit_DMA(huart, s_tx_frame[id], len) != HAL_OK) {
        s_stat[id].tx_fail++;
        return BSP_UART_ERR_BUSY;
    }

    s_stat[id].tx_ok++;
    return BSP_UART_OK;
}

const BspUartStat *BSP_UART_GetStat(uint8_t id) {
    if (id >= BSP_UART_CNT) {
        return NULL;
    }
    return &s_stat[id];
}

uint8_t BSP_UART_LayoutOk(void) {
    uint8_t i;

    for (i = 0U; i < BSP_UART_CNT; i++) {
        if (((uintptr_t)s_rx_dma[i] < BSP_UART_DMA_ADDR_MIN) ||
            ((uintptr_t)s_tx_frame[i] < BSP_UART_DMA_ADDR_MIN)) {
            return 1U;
        }
    }
    return 0U;
}

/* ==================== HAL 回调 ====================
 *
 * 这两个是 HAL 的全局弱回调(Drivers 里是 __weak)，本层覆盖它们。
 * **本层因此占住了"ReceiveToIdle + DMA"这条接收路径** —— 以后别的模块要用
 * huart1/huart7/huart10 走同样的接收方式，得把口加进 uart_config.h 的
 * BSP_UART_PORT_ARRAY(而不是再定义一个同名回调，那会重复符号)。
 *
 * 都在中断上下文：只做统计、回调、重装，绝不阻塞、绝不打印。 */

/* 收完一段字节时进来。Size 是这一次实际收了多少(正常就是整帧)。
 *
 * 顺序**必须**是"先给上层、后重装"：反过来的话 DMA 会在回调还在读 s_rx_dma 的时候
 * 就开始往里写，上层拿到的是被踩过的数据。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    int32_t found = BspUart_Find(huart);
    uint8_t idx;

    if (found < 0) {
        return;
    }
    idx = (uint8_t)found;

    s_stat[idx].rx_events++;
    s_stat[idx].rx_bytes += (uint32_t)Size;
    if ((uint32_t)Size >= BSP_UART_RX_BUF_LEN) {
        /* 收满了整块缓冲才停 —— 对端发的东西比缓冲还长，尾部可能已经被截掉。
         * 正常帧(≤262)灌不满 294 字节的缓冲，所以这一条正常恒为 0。 */
        s_stat[idx].rx_over++;
    }

    if (s_rx_cb[idx] != NULL) {
        s_rx_cb[idx](idx, s_rx_dma[idx], Size);
    }

    BspUart_Arm(idx);
}

/* 出错时进来。DMA 接收模式下**任何**错误都是阻塞型的：HAL 会把接收整个停掉
 * (清 DMAR、RxState 回 READY、abort 掉 DMA 流)，然后才从 DMA 的 abort 中断里回调
 * 这里 —— 所以此刻重装是安全且必要的，不然这一路从此再也收不到东西，
 * 现象跟"对端根本没发/线没接"一模一样，最容易误判。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    int32_t found = BspUart_Find(huart);
    uint8_t idx;
    uint32_t err;

    if (found < 0) {
        return;
    }
    idx = (uint8_t)found;

    err = HAL_UART_GetError(huart);
    s_stat[idx].hal_err |= err; /* 按位累积：读出来就知道每一路见过什么错误 */
    if ((err & HAL_UART_ERROR_ORE) != 0U) {
        s_stat[idx].ore++; /* ORE 是计数不是位：一个就毁掉一整帧 */
    }

    /* 按 RxState 判断要不要重装，不按错误码：ORE/RTO 以及 DMA 模式下的一切错误
     * 都会把接收停掉(那时 RxState 已经不是 BUSY_RX，要重装)。 */
    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        BspUart_Arm(idx);
    }
}
