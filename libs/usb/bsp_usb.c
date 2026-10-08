#include "bsp_usb.h"

#include <string.h> /* memset */

#include "usbd_def.h"     /* USBD_HandleTypeDef / USBD_STATE_CONFIGURED */
#include "usbd_cdc_if.h"  /* CDC_Transmit_HS() */

/* USB 设备句柄。定义在 USB_DEVICE/App/usb_device.c，CubeMX 生成的 usb_device.h 里
 * 没有它的 extern 声明，所以自己声明一份（上游 bsp_usb.c 也是这么干的）。 */
extern USBD_HandleTypeDef hUsbDeviceHS;

/* ==================== 状态 ====================
 *
 * bsp_uart 那边是"每口一份"的数组（s_rx_cb[BSP_UART_CNT] / s_stat[BSP_UART_CNT]），
 * 这里只有一个 USB 口，所以就是两个普通变量。上游用 cvector 存一串回调，
 * 本工程不需要"多个订阅者"这件事，照 bsp_uart 的普通赋值更省事，
 * 也顺带没有"忘了调 Init 就注册 → 解引用空列表"这种坑。 */
static bsp_usb_rx_cb s_rx_cb;
static BspUsbStat s_stat;

void BSP_USB_Init(void) {
    s_rx_cb = NULL;
    memset(&s_stat, 0, sizeof s_stat);
}

uint8_t BSP_USB_IsConfigured(void) {
    return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

void BSP_USB_RegisterRxCallback(uint8_t id, bsp_usb_rx_cb cb) {
    if (id != BSP_USB_PORT) {
        return;
    }
    s_rx_cb = cb;
}

int32_t BSP_USB_Send(uint8_t id, const uint8_t *data, uint16_t len) {
    if ((id != BSP_USB_PORT) || (data == NULL) || (len == 0U)) {
        return BSP_USB_ERR_PARAM;
    }

    /* 没枚举出来就得挡住，**这不是"丢一帧"的事**：CDC_Transmit_HS() 上来就
     * `hUsbDeviceHS.pClassData->TxState`，而没枚举时 pClassData 是 NULL —— 直接是硬件异常。
     * 上电顺序上这事真会发生：Robot_State 的任务一启动就开始按 250 Hz 发反馈帧，
     * 那时候主机多半还没打开这个口（甚至线都没插）。 */
    if (!BSP_USB_IsConfigured() || (hUsbDeviceHS.pClassData == NULL)) {
        s_stat.not_ready++;
        return BSP_USB_ERR_NOT_READY;
    }

    /* 交给 USB 栈自己的路径：它先查 TxState（上一笔没发完就 USBD_BUSY），
     * 再 USBD_CDC_TransmitPacket() —— 那个函数会处理"长度正好是端点包长整数倍"
     * 时要补一个 ZLP 的情况。上游手搓 TxState + USBD_LL_Transmit 把这层跳过了，
     * 这里不必再重复一遍。 */
    if (CDC_Transmit_HS((uint8_t *)data, len) != USBD_OK) {
        s_stat.tx_fail++;
        return BSP_USB_ERR_BUSY;
    }

    s_stat.tx_ok++;
    return BSP_USB_OK;
}

void BSP_USB_Recv_Callback(uint8_t *data, uint16_t len) {
    s_stat.rx_events++;
    s_stat.rx_bytes += len;

    if (s_rx_cb != NULL) {
        s_rx_cb(BSP_USB_PORT, data, len);
    }
}

const BspUsbStat *BSP_USB_GetStat(uint8_t id) {
    if (id != BSP_USB_PORT) {
        return NULL;
    }
    return &s_stat;
}
