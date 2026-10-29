#pragma once

#include <stdint.h>

/* USB CDC 字节传输层；接收数据由上层解析，发送缓冲由调用方持有。 */

/* 单 USB 端口的固定索引。 */
#define BSP_USB_PORT 0u

#define BSP_USB_OK          0
#define BSP_USB_ERR_PARAM   (-1) /* id 不是 BSP_USB_PORT / 指针为空 / 长度为 0 */
#define BSP_USB_ERR_BUSY    (-2) /* 上一笔还没发完（主机没读走）；本层不排队 */
#define BSP_USB_ERR_NOT_READY (-3) /* 还没枚举/没配置好，这时候发不出去 */

/* 接收回调运行在中断上下文，须非阻塞；data 仅在回调期间有效，数据可能跨帧或包含多帧。 */
typedef void (*bsp_usb_rx_cb)(uint8_t id, const uint8_t *data, uint16_t len);

/* 清统计、清回调登记。没有硬件要配（USB 外设由 CubeMX 生成的 MX_USB_DEVICE_Init() 管）。 */
void BSP_USB_Init(void);

/* 登记收包回调。传 NULL 可以注销。id 不是 BSP_USB_PORT 时什么都不做。 */
void BSP_USB_RegisterRxCallback(uint8_t id, bsp_usb_rx_cb cb);

/* 零拷贝异步发送；缓冲须保持有效且内容不变直至传输完成，繁忙时由上层重试。 */
int32_t BSP_USB_Send(uint8_t id, const uint8_t *data, uint16_t len);

/* 主机把口配好了没有（= USB 栈处于 CONFIGURED）。1 = 可以发。 */
uint8_t BSP_USB_IsConfigured(void);

/* 仅由 USB CDC 的接收回调转交收到的字节。 */
void BSP_USB_Recv_Callback(uint8_t *data, uint16_t len);

/* 统计，给调试器（JLink）读。 */
typedef struct {
    uint32_t rx_events; /* 进过多少次接收回调 */
    uint32_t rx_bytes;  /* 收到的总字节数 */
    uint32_t tx_ok;     /* 交出去且 USB 栈接了的次数 */
    uint32_t tx_fail;   /* 没发出去：上一笔没发完（主机没读走） */
    uint32_t not_ready; /* 没发出去：还没枚举出来 */
} BspUsbStat;

/* id 不是 BSP_USB_PORT 返回 NULL。 */
const BspUsbStat *BSP_USB_GetStat(uint8_t id);
