#include "bsp_uart.h"

#include <string.h>

/* DMA 收发缓冲放在 .RAM_D1；NOLOAD 段初值由每次收发填充。 */
#define BSP_UART_DMA_MEM __attribute__((section(".RAM_D1"), aligned(32)))

BSP_UART_DMA_MEM static uint8_t s_rx_dma[BSP_UART_CNT][BSP_UART_RX_BUF_LEN];
BSP_UART_DMA_MEM static uint8_t s_tx_frame[BSP_UART_CNT][BSP_UART_TX_BUF_LEN];

/* 每口一份的状态 */

static UART_HandleTypeDef *const s_port[BSP_UART_CNT] = BSP_UART_PORT_ARRAY;

static bsp_uart_rx_cb s_rx_cb[BSP_UART_CNT];
static BspUartStat s_stat[BSP_UART_CNT];

#define BSP_UART_DMA_ADDR_MIN 0x24000000UL

static int32_t BspUart_Find(const UART_HandleTypeDef *huart) {
    uint8_t i;

    for (i = 0U; i < BSP_UART_CNT; i++) {
        if (s_port[i] == huart) {
            return (int32_t)i;
        }
    }
    return -1;
}

/* 每次接收结束或发生接收错误后重新启动 DMA 接收。 */
static void BspUart_Arm(uint8_t idx) {
    UART_HandleTypeDef *huart = s_port[idx];

    /* 缺少 DMA 句柄时记录重装失败并结束。 */
    if (huart->hdmarx == NULL) {
        s_stat[idx].rearm_fail++;
        return;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_dma[idx], BSP_UART_RX_BUF_LEN) != HAL_OK) {
        s_stat[idx].rearm_fail++;
        return;
    }

    /* 关闭半传输中断，仅在接收结束时提交数据。 */
    if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void BSP_UART_Init(void) {
    uint8_t i;

    for (i = 0U; i < BSP_UART_CNT; i++) {
        s_rx_cb[i] = NULL;
        memset(&s_stat[i], 0, sizeof s_stat[i]);

        /* 记录各端口是否配置了接收和发送 DMA 句柄。 */
        if ((s_port[i]->hdmarx == NULL) || (s_port[i]->hdmatx == NULL)) {
            s_stat[i].no_dma = 1U;
        }

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

    /* 发送繁忙时立即返回，由上层决定重试策略。 */
    if (huart->gState != HAL_UART_STATE_READY) {
        s_stat[id].tx_fail++;
        return BSP_UART_ERR_BUSY;
    }

    /* 复制到 .RAM_D1 发送缓冲后启动 DMA。 */
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

/* HAL 中断回调仅更新统计、分派接收数据并重装 DMA。 */

/* 先调用上层处理数据，再重装接收，保证回调期间缓冲内容稳定。 */
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
        /* 达到接收缓冲容量时累计可能截断的接收事件。 */
        s_stat[idx].rx_over++;
    }

    if (s_rx_cb[idx] != NULL) {
        s_rx_cb[idx](idx, s_rx_dma[idx], Size);
    }

    BspUart_Arm(idx);
}

/* 接收错误后重装 DMA，恢复后续接收。 */
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
