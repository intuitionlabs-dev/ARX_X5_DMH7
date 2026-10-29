#include "uart_echo_test.h"

#include "bsp_uart.h"
#include "test_config.h"

/* 启用测试时编译 USART1 回显实现。 */
#if TEST_UART_ECHO_ENABLE

/* 在编译期校验测试端口与 BSP 端口一致。 */
_Static_assert(UART_ECHO_PORT == BSP_UART_U1, "UART_ECHO_PORT 必须是 BSP_UART_U1");

/* 保存接收量、发送结果和最近一段数据。 */
volatile uint32_t uart_echo_rx_events;
volatile uint32_t uart_echo_rx_bytes;
volatile uint32_t uart_echo_tx_ok;
volatile uint32_t uart_echo_tx_busy;
volatile uint32_t uart_echo_tx_fail;
volatile uint32_t uart_echo_max_len;

volatile uint32_t uart_echo_last_len;
volatile uint8_t uart_echo_last[UART_ECHO_LAST_LEN];

/* 在接收中断中保存数据并提交原样回显。 */
static void UartEchoTest_OnRx(uint8_t id, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    int32_t r;

    /* 端口不匹配时结束回调。 */
    if (id != UART_ECHO_PORT) {
        return;
    }

    /* 空接收事件结束本次回显。 */
    if (len == 0U) {
        return;
    }

    uart_echo_rx_events++;
    uart_echo_rx_bytes += len;
    /* 刷新历史最大接收长度。 */
    if ((uint32_t)len > uart_echo_max_len) {
        uart_echo_max_len = len;
    }

    uart_echo_last_len = len;
    /* 复制本段数据的前缀供调试器读取。 */
    for (i = 0U; (i < len) && (i < UART_ECHO_LAST_LEN); i++) {
        uart_echo_last[i] = data[i];
    }

    /* 在回调返回前将原始数据复制到 BSP 发送缓冲。 */
    r = BSP_UART_Send(UART_ECHO_PORT, data, len);
    /* 发送受理成功时累计成功次数。 */
    if (r == BSP_UART_OK) {
        uart_echo_tx_ok++;
    } else if (r == BSP_UART_ERR_BUSY) { /* 发送忙时累计本次回显丢弃次数。 */

        uart_echo_tx_busy++;
    } else { /* 其余发送错误累计失败次数。 */
        uart_echo_tx_fail++;
    }
}

/* 清空回显观测量并注册接收回调。 */
void UartEchoTest_Init(void)
{
    uint16_t i;

    uart_echo_rx_events = 0U;
    uart_echo_rx_bytes  = 0U;
    uart_echo_tx_ok     = 0U;
    uart_echo_tx_busy   = 0U;
    uart_echo_tx_fail   = 0U;
    uart_echo_max_len   = 0U;
    uart_echo_last_len  = 0U;
    /* 逐字节清空最近接收数据。 */
    for (i = 0U; i < UART_ECHO_LAST_LEN; i++) {
        uart_echo_last[i] = 0U;
    }

    /* 在 BSP 初始化完成后登记回调。 */
    BSP_UART_RegisterRxCallback(UART_ECHO_PORT, UartEchoTest_OnRx);
}

#endif
