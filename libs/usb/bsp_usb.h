#pragma once

#include <stdint.h>

/* USB CDC 传输层（BSP）。形状照着 libs/uart/bsp_uart.h 抄 —— 同一个上层
 * (App/robot_state.c) 要在"走串口"和"走 USB"之间编译期二选一，两边的接口
 * 如果不一样，上层就得写两套代码，那就白抽象了。
 *
 * 对照表（左 bsp_uart，右本层）：
 *
 *     BSP_UART_U3                            BSP_USB_PORT（恒 0，USB 只有一路）
 *     BSP_UART_Init()                        BSP_USB_Init()
 *     void (*bsp_uart_rx_cb)(uint8_t id,     void (*bsp_usb_rx_cb)(uint8_t id,
 *                             const uint8_t*, uint16_t)                       const uint8_t*, uint16_t)
 *     BSP_UART_RegisterRxCallback(id, cb)    BSP_USB_RegisterRxCallback(id, cb)
 *     int32_t BSP_UART_Send(id, buf, len)    int32_t BSP_USB_Send(id, buf, len)
 *     BSP_UART_OK / ERR_PARAM / ERR_BUSY     BSP_USB_OK / ERR_PARAM / ERR_BUSY / ERR_NOT_READY
 *     BSP_UART_GetStat(id)                   BSP_USB_GetStat(id)
 *
 * ---- 分层 ----
 *
 * 和 libs/uart 一样，**本层不认识帧协议**：收到的字节原样推给上层（回调），上层自己拿去喂
 * libs/frame 的 FrameParserFeed()；要发帧也是上层自己 FrameEncode() 完再调
 * BSP_USB_Send()。（libs/frame 也因此保持不依赖 HAL、不依赖 USB 栈。）
 *
 * ---- 和 bsp_uart 不一样的三件事（用之前必须知道）----
 *
 * 1. **发送是零拷贝的**。bsp_uart 会把数据拷进它自己的 DMA 缓冲；本层直接把指针交给
 *    USB 栈（CDC_Transmit_HS() 只是把它存进 hcdc->TxBuffer），**函数返回后那块内存
 *    必须一直有效到传输完成**。改它、或者传个栈上/临时数组，都会发出乱码。
 *    唯一的调用方 App/robot_state.c 用的是静态 s_tx_buf，所以是安全的。
 *
 * 2. **多了一个返回码 BSP_USB_ERR_NOT_READY**：USB 还没被主机枚举出来的那段时间，
 *    发是发不出去的（而且直接发会踩空指针，见 BSP_USB_Send() 里的注释）。uart 那边
 *    没有对应的状态 —— 串口引脚在那儿就是在那儿。
 *
 * 3. 本层**不需要 .RAM_D1 段**。USB 的收发 FIFO 是 CPU 直接读写的外设寄存器，
 *    不像 UART 那样挂 DMA，所以 DTCM 里的缓冲一样能用。
 *
 * ---- 用法 ----
 *
 *     // main.c 的 USER CODE 2
 *     BSP_USB_Init();
 *     BSP_USB_RegisterRxCallback(BSP_USB_PORT, OnRx);
 *
 * 结果和统计都记在 BSP_USB_GetStat() 里（不用打印，JLink 直接读）。 */

/* USB 只有一路，这个宏纯粹是为了和 BSP_UART_U3 那种写法同形。 */
#define BSP_USB_PORT 0u

#define BSP_USB_OK          0
#define BSP_USB_ERR_PARAM   (-1) /* id 不是 BSP_USB_PORT / 指针为空 / 长度为 0 */
#define BSP_USB_ERR_BUSY    (-2) /* 上一笔还没发完（主机没读走）；本层不排队 */
#define BSP_USB_ERR_NOT_READY (-3) /* 还没枚举/没配置好，这时候发不出去 */

/* 收到一段字节时调用。**在中断上下文里**（OTG_HS_IRQn，优先级 5）：不要阻塞、不要打印、
 * 不要调 FreeRTOS API。
 *
 * data 指向 USB 栈的接收缓冲，**回调返回之后内容随时可能被下一次接收覆盖** ——
 * 要留就当场拷走、或者直接喂进解析器，别把这个指针存下来。（和 bsp_uart_rx_cb 同样的约定。）
 *
 * len 是一次 OUT 传输收到的字节数。和串口不一样，这里**没有"帧与帧之间的空闲"可以依赖**：
 * 主机一次可以只发半帧、也可以把两帧挤在同一个包边界里，都由 USB 的包大小决定。
 * 好在帧协议自己带帧头 + CRC，逐字节喂 FrameParserFeed() 本来就不关心切在哪。 */
typedef void (*bsp_usb_rx_cb)(uint8_t id, const uint8_t *data, uint16_t len);

/* 清统计、清回调登记。没有硬件要配（USB 外设由 CubeMX 生成的 MX_USB_DEVICE_Init() 管）。 */
void BSP_USB_Init(void);

/* 登记收包回调。传 NULL 可以注销。id 不是 BSP_USB_PORT 时什么都不做。 */
void BSP_USB_RegisterRxCallback(uint8_t id, bsp_usb_rx_cb cb);

/* 发一段原始字节。**零拷贝**，见上面第 1 条 —— data 得活到发完为止。
 *
 * 不阻塞：支起 IN 传输就返回。上一笔还没发完返回 BSP_USB_ERR_BUSY —— 本层不排队，
 * 重试还是丢弃由上层决定。 */
int32_t BSP_USB_Send(uint8_t id, const uint8_t *data, uint16_t len);

/* 主机把口配好了没有（= USB 栈处于 CONFIGURED）。1 = 可以发。 */
uint8_t BSP_USB_IsConfigured(void);

/* USB 栈收到一包 OUT 数据时调它。**这是本层唯一的接线口** —— 由 CubeMX 生成的
 * USB_DEVICE/App/usbd_cdc_if.c 的 CDC_Receive_HS() 调用，别在别处调。 */
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
