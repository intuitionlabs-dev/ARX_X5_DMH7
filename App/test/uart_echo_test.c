#include "uart_echo_test.h"

#include "bsp_uart.h"    /* 串口传输层：DMA 收发 + .RAM_D1 缓冲都封在它里面 */
#include "test_config.h" /* 测试例程开关 */

#if TEST_UART_ECHO_ENABLE

/* 本测试的端口号和 BSP 的编号必须是一回事。写成 _Static_assert 而不是注释：
 * 对不上的话最可能的现象是"PC 发过去什么都没回来"(注册到别的口上了)，
 * 而人第一反应一定是去查线和波特率。 */
_Static_assert(UART_ECHO_PORT == BSP_UART_U1, "UART_ECHO_PORT 必须是 BSP_UART_U1");

/* ==================== 观测变量(JLink 直接读) ==================== */
volatile uint32_t uart_echo_rx_events;
volatile uint32_t uart_echo_rx_bytes;
volatile uint32_t uart_echo_tx_ok;
volatile uint32_t uart_echo_tx_busy;
volatile uint32_t uart_echo_tx_fail;
volatile uint32_t uart_echo_max_len;

volatile uint32_t uart_echo_last_len;
volatile uint8_t uart_echo_last[UART_ECHO_LAST_LEN];

/* BSP 的接收回调。**在中断上下文里跑**(USART1 的 IDLE 中断)，所以这里不阻塞、
 * 不打印、不做需要临界区的事。
 *
 * 调用时机很关键：BSP 是"先把字节推给上层、返回前才重新武装接收"，所以此刻
 * DMA 还没开始往接收缓冲里写下一笔 —— data 在这一整个函数里都是稳的，可以直接读。
 * 但**函数返回后缓冲立刻被下一次接收复用**，所以要留内容只能当场拷走
 * (下面的 last[] 就是当场拷的；BSP_UART_Send() 也是当场把数据拷进它自己的发送缓冲)。
 *
 * 回显直接在中断里发:BSP_UART_Send() 只把数据拷贝进 BSP 的 .RAM_D1 发送缓冲，
 * 然后交给 HAL 的 DMA 去搬，本身不阻塞，在中断上下文里调用是安全的。 */
static void UartEchoTest_OnRx(uint8_t id, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    int32_t r;

    /* 回调是按口注册的，正常情况下 id 恒等于 UART_ECHO_PORT；留着这行是为了
     * 万一以后有人把同一个回调也注册到别的口上，不至于把那些口的数据也回显出去。 */
    if (id != UART_ECHO_PORT) {
        return;
    }

    /* 0 字节的事件没有内容可回，不计进统计(正常不该出现 —— 一次 IDLE 事件
     * 至少搬进来过一个字节)。 */
    if (len == 0U) {
        return;
    }

    uart_echo_rx_events++;
    uart_echo_rx_bytes += len;
    if ((uint32_t)len > uart_echo_max_len) {
        uart_echo_max_len = len;
    }

    /* 只留前 UART_ECHO_LAST_LEN 个字节给人看，长度照实记(超了也记真实长度)。 */
    uart_echo_last_len = len;
    for (i = 0U; (i < len) && (i < UART_ECHO_LAST_LEN); i++) {
        uart_echo_last[i] = data[i];
    }

    r = BSP_UART_Send(UART_ECHO_PORT, data, len);
    if (r == BSP_UART_OK) {
        uart_echo_tx_ok++;
    } else if (r == BSP_UART_ERR_BUSY) {
        /* 上一笔还没发完 —— 说明 PC 发得太密(每一笔都要等它自己发完再由 PC 发下一笔，
         * 就不会出现这个)。不排队，这一笔就丢了。 */
        uart_echo_tx_busy++;
    } else {
        uart_echo_tx_fail++;
    }
}

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
    for (i = 0U; i < UART_ECHO_LAST_LEN; i++) {
        uart_echo_last[i] = 0U;
    }

    /* 必须在 BSP_UART_Init() 之后调 —— BSP 的初始化会清空回调表。 */
    BSP_UART_RegisterRxCallback(UART_ECHO_PORT, UartEchoTest_OnRx);
}

#endif /* TEST_UART_ECHO_ENABLE */
