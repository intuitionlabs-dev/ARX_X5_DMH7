#include "bsp_usb.h"

#include <string.h>

#include "usbd_def.h"     /* USBD_HandleTypeDef / USBD_STATE_CONFIGURED */
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

/* 单 USB 端口独立保存接收回调和统计。 */
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

    /* USB 配置完成后才调用 CDC 发送接口，保证类数据指针有效。 */
    if (!BSP_USB_IsConfigured() || (hUsbDeviceHS.pClassData == NULL)) {
        s_stat.not_ready++;
        return BSP_USB_ERR_NOT_READY;
    }

    /* 通过 CDC 发送接口处理忙状态及端点整包长度对应的 ZLP。 */
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
