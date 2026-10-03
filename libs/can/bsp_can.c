/**
 ******************************************************************************
 * 文件名          : bsp_can.c
 * 创建时间        : 2021.12.07
 * 作者            : 陈迅
 ******************************************************************************
 * 1.本代码基于STM32F407IGH6TR开发，编译环境为vscode，基于FreeRTOS进行开发
 * 2.本代码包含大量中文注释，请以UTF-8编码格式打开
 * 3.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
 *
 * Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
 ******************************************************************************
 * 说明：
 ******************************************************************************
 */
#include "bsp_can.h"
#include "main.h" // HAL_GetTick()

#if _BSP_CAN_ENABLE && _BSP_FDCAN_ENABLE
#error
#endif

#if _BSP_CAN_ENABLE && DEVICE_CAN_CNT
#include "can.h" // IWYU pragma: keep
#include "cvector.h"

typedef struct Bsp_can_type_def_t {
    CAN_HandleTypeDef *device;  // 自定义总线编号
    uint32_t tx_mailbox;
    cvector *call_backs;
    cvector *frame_callbacks;
    uint32_t bank_prefix;              // 不同can对应的过滤器相关参数值
    uint16_t filters[FILTER_MAX_CNT];  // 按标准帧ID定义
} Bsp_can_type_def_t;

Bsp_can_type_def_t can_devices[DEVICE_CAN_CNT];  // 定义对应数量的can外设，即代码中定义can总线

void BSP_CAN_Init(void) {
    CAN_HandleTypeDef* can_ports[DEVICE_CAN_CNT]  = CAN_PORT_ARRAY;
    uint32_t prefix_array[DEVICE_CAN_CNT] = CAN_PREFIX_ARRAY;
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        can_devices[i].device = can_ports[i];
        can_devices[i].bank_prefix = prefix_array[i];
        can_devices[i].call_backs = cvector_create(sizeof(can_rx_callback));
        can_devices[i].frame_callbacks = cvector_create(sizeof(Can_frame_rx_callback_t));

    }
    for (size_t d = 0; d < DEVICE_CAN_CNT; ++d) {
        for (size_t i = 0; i < FILTER_MAX_CNT; ++i) {
            can_devices[d].filters[i] = ID_NOTSET;  // 初始化can过滤器
        }
    }
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        HAL_CAN_Start(can_devices[i].device);                                              // 将HAL库的对应功能函数封装，实现映射和隔离
        HAL_CAN_ActivateNotification(can_devices[i].device, CAN_IT_RX_FIFO0_MSG_PENDING);  // 检测can是否报错
        HAL_CAN_ActivateNotification(can_devices[i].device, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

//  仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t *data,
                  basic_data_t len) {
    #if !_BSP_DEBUG_CAN_NOT_SEND
    CAN_TxHeaderTypeDef txconf;  //配置发送包头
    txconf.StdId = identifier;
    txconf.IDE = CAN_ID_STD;
    txconf.RTR = CAN_RTR_DATA;
    txconf.DLC = len;
    basic_data_t can_tx_start_time = BSP_sys_time_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(can_devices[can_id].device) == 0) {  // 获取空闲邮箱数量
        // HAL_CAN_AbortTxRequest
        basic_data_t can_tx_now_time = BSP_sys_time_ms();
        // 2ms还没有发送出去，放弃发送
        if (can_tx_now_time > can_tx_start_time + 2) {
            HAL_CAN_AbortTxRequest(can_devices[can_id].device, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            __NOP();
            break;
        }
    }
    HAL_CAN_AddTxMessage(can_devices[can_id].device, &txconf, data,  // 上次信息发送成功，传输新信息
                         &can_devices[can_id].tx_mailbox);
    #else
    #warning "Config CAN to not send!!"
    #endif
}

void Update_Filter(uint8_t can_id, uint32_t filter_index) {
    uint32_t group_index = filter_index / 4;
    CAN_FilterTypeDef tmp;
    tmp.FilterMode = CAN_FILTERMODE_IDLIST;
    tmp.FilterScale = CAN_FILTERSCALE_16BIT;
    // tmp.FilterFIFOAssignment = can_devices[can_id].fifo;
    tmp.FilterFIFOAssignment = (group_index & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;  // 4个一组，按照组别设置FIFO，平衡负载
    tmp.SlaveStartFilterBank = 14;
    uint16_t *filter_data = can_devices[can_id].filters;  // 与can总线保持一致
    tmp.FilterIdLow = filter_data[group_index * 4] << 5;
    tmp.FilterIdHigh = filter_data[group_index * 4 + 1] << 5;
    tmp.FilterMaskIdLow = filter_data[group_index * 4 + 2] << 5;
    tmp.FilterMaskIdHigh = filter_data[group_index * 4 + 3] << 5;
    tmp.FilterBank = group_index + can_devices[can_id].bank_prefix;
    uint8_t id_all_notset = 1;
    for (size_t i = 0; i < 4; ++i) {
        if (filter_data[group_index * 4 + i] != ID_NOTSET) id_all_notset = 0;
    }
    if (id_all_notset)
        tmp.FilterActivation = CAN_FILTER_DISABLE;
    else
        tmp.FilterActivation = CAN_FILTER_ENABLE;
    HAL_CAN_ConfigFilter(can_devices[can_id].device, &tmp);  // 配置can接收过滤器
}


void BSP_CAN_FifoMsg_Callback(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (hcan == can_devices[i].device) {
            uint8_t bsp_can_rxbuf[8];
            CAN_RxHeaderTypeDef rxconf;
            HAL_CAN_GetRxMessage(hcan, fifo, &rxconf, bsp_can_rxbuf);
            for (size_t j = 0; j < can_devices[i].call_backs->cv_len; ++j) {
                can_rx_callback funcnow = *(can_rx_callback *)cvector_val_at(can_devices[i].call_backs, j);
                funcnow(i, rxconf.StdId, bsp_can_rxbuf, rxconf.DLC);
            }
            can_frame_callback(i, rxconf.StdId, bsp_can_rxbuf, rxconf.DLC);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    BSP_CAN_FifoMsg_Callback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    BSP_CAN_FifoMsg_Callback(hcan, CAN_RX_FIFO1);
}

void BSP_ADD_Filters(uint8_t can_id) {
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT; ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == ID_NOTSET) {
            continue;
        }
        Update_Filter(can_id, filter_index);
    }
}

#endif

#if _BSP_FDCAN_ENABLE

#include "fdcan.h" // IWYU pragma: keep
#include "cvector.h"

typedef struct Bsp_fd_can_type_def_t {
    FDCAN_HandleTypeDef *device;  //自定义总线编号
    uint32_t tx_mailbox;
    cvector *call_backs;
    uint32_t bank_prefix;              //不同can对应的过滤器相关参数值
    uint16_t filters[FILTER_MAX_CNT];  //按标准帧ID定义
} Bsp_fd_can_type_def;

Bsp_fd_can_type_def can_devices[DEVICE_CAN_CNT];  //定义对应数量的can外设，即代码中定义can总线

void BSP_CAN_Init(void) {
    FDCAN_HandleTypeDef* can_ports[DEVICE_CAN_CNT]  = CAN_PORT_ARRAY;
    uint32_t prefix_array[DEVICE_CAN_CNT] = CAN_PREFIX_ARRAY;
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        can_devices[i].device = can_ports[i];
        can_devices[i].bank_prefix = prefix_array[i];
        can_devices[i].call_backs = cvector_create(sizeof(can_rx_callback));
    }

    for (size_t d = 0; d < DEVICE_CAN_CNT; ++d) {
        for (size_t i = 0; i < FILTER_MAX_CNT; ++i) {
            can_devices[d].filters[i] = ID_NOTSET;
        }
    }

    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (HAL_FDCAN_ConfigGlobalFilter(can_devices[i].device, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
            return;
        }
        HAL_FDCAN_Start(can_devices[i].device);  // 将HAL库的对应功能函数封装，实现映射和隔离
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_FULL, 0);
        // HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_FULL, 0);
        // HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    }
}

//  仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参
void BSP_CAN_Send(uint8_t fdcan_id, uint16_t identifier, uint8_t *data, basic_data_t len) {
        #if !_BSP_DEBUG_CAN_NOT_SEND
    FDCAN_TxHeaderTypeDef txconf;
    switch (len) {
        case 0:
            txconf.DataLength = FDCAN_DLC_BYTES_0; /* 数据长度:0 */
            break;
        case 1:
            txconf.DataLength = FDCAN_DLC_BYTES_1; /* 数据长度:1 */
            break;
        case 2:
            txconf.DataLength = FDCAN_DLC_BYTES_2; /* 数据长度:2 */
            break;
        case 3:
            txconf.DataLength = FDCAN_DLC_BYTES_3; /* 数据长度:3 */
            break;
        case 4:
            txconf.DataLength = FDCAN_DLC_BYTES_4; /* 数据长度:4 */
            break;
        case 5:
            txconf.DataLength = FDCAN_DLC_BYTES_5; /* 数据长度:5 */
            break;
        case 6:
            txconf.DataLength = FDCAN_DLC_BYTES_6; /* 数据长度:6 */
            break;
        case 7:
            txconf.DataLength = FDCAN_DLC_BYTES_7; /* 数据长度:7 */
            break;
        case 8:
            txconf.DataLength = FDCAN_DLC_BYTES_8; /* 数据长度:8 */
            break;
        default:
            /* 其他长度就退出函数 */
            return;
    }    
    txconf.Identifier = identifier;  // fdcan id
    txconf.IdType = FDCAN_STANDARD_ID;
    txconf.TxFrameType = FDCAN_DATA_FRAME;
    txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txconf.BitRateSwitch = FDCAN_BRS_OFF;
    txconf.FDFormat = FDCAN_CLASSIC_CAN;
    // txconf.DataLength = FDCAN_DLC_BYTES_8;
    txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txconf.MessageMarker = 0;

    /* 若当前 Tx FIFO 没有空闲则等待，最多 1ms；太久还没发出去就放弃本次发送 */
    uint32_t tx_start_time = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(can_devices[fdcan_id].device) == 0) {
        if (HAL_GetTick() - tx_start_time >= 1) {
            // clang-format off
            HAL_FDCAN_AbortTxRequest(can_devices[fdcan_id].device,
                FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2 | FDCAN_TX_BUFFER3 | FDCAN_TX_BUFFER4);
            // clang-format on
            /* TODO: 接入本工程日志后恢复
            LOG_WARN(DEBUG_WARN, "CAN%u,id0x%X TX abort\r in %s\n", fdcan_id, identifier,__func__);
            */
            return;
        }
    }
    
    HAL_FDCAN_AddMessageToTxFifoQ(can_devices[fdcan_id].device, &txconf, data);
#endif//_BSP_DEBUG_CAN_NOT_SEND
}

void Update_Filter(uint8_t fdcan_id, uint32_t filter_index) {
    FDCAN_FilterTypeDef tmp;
    uint16_t id = can_devices[fdcan_id].filters[filter_index];

    tmp.IdType = FDCAN_STANDARD_ID;      /* 只接收标准帧ID */
    tmp.FilterIndex = filter_index;      /* 一个滤波元素对应 filters[] 的一个槽位 */
    tmp.FilterType = FDCAN_FILTER_RANGE; /* 范围滤波；FilterID1 == FilterID2 时即精确匹配该ID */

    if (id == ID_NOTSET) {
        /* 该槽位没登记ID，关掉这个滤波元素。
         * 注意 ID_NOTSET(0x800) 超过标准帧ID上限 0x7FF，不能直接写进 FilterID1。 */
        tmp.FilterConfig = FDCAN_FILTER_DISABLE;
        tmp.FilterID1 = 0;
        tmp.FilterID2 = 0;
    } else {
        /* 注意：
         *  1.FDCAN_HandleTypeDef对象句柄的成员RxFifo1ElmtsNbr设置大于0时，表示启用RXFIFO1.
         *  3.如果启用RXFIFO0,那么滤波器必须关联到RXFIFO0,即FilterConfig必须赋值FDCAN_FILTER_TO_RXFIFO0.
         *  4.同理,如果启动RXFIFO1的话,滤波器必须关联到RXFIFO1,即FilterConfig必须赋值FDCAN_FILTER_TO_RXFIFO1. */
        tmp.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        tmp.FilterID1 = id;
        tmp.FilterID2 = id;
    }

    HAL_FDCAN_ConfigFilter(can_devices[fdcan_id].device, &tmp);
}

static const uint8_t DL_CTO_BYTES[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static void bsp_can_fifo_msg_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo, uint32_t RxFifo0ITs) {
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (hfdcan == can_devices[i].device) {
            uint8_t bsp_can_rxbuf[8];
            FDCAN_RxHeaderTypeDef rxconf;
            HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxconf, bsp_can_rxbuf); /* 提取FIFO0中接收到的数据 */
                                                                          /* 更新RXFIFO的状态（了解FIFO是否有剩余报文） */
            for (size_t j = 0; j < can_devices[i].call_backs->cv_len; ++j) {
                can_rx_callback funcnow = *(can_rx_callback *)cvector_val_at(can_devices[i].call_backs, j);
                funcnow(i, rxconf.Identifier, bsp_can_rxbuf, DL_CTO_BYTES[rxconf.DataLength >> 16]);
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    bsp_can_fifo_msg_callback(hfdcan, FDCAN_RX_FIFO0, RxFifo0ITs);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    bsp_can_fifo_msg_callback(hfdcan, FDCAN_RX_FIFO1, RxFifo1ITs);
}
#endif

#if DEVICE_CAN_CNT

// 注册新的接收过滤器
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT;
         ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == ID_NOTSET) {
            can_devices[can_id].filters[filter_index] = filter;
            Update_Filter(can_id, filter_index);
            return;
        }
    }
}

// 注销过滤器
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT;
         ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == filter) {
            can_devices[can_id].filters[filter_index] = ID_NOTSET;
            Update_Filter(can_id, filter_index);
        }
    }
}

// 注册回调函数
void BSP_CAN_RegisterRxCallback(uint8_t fdcan_id, can_rx_callback func) {
    cvector_pushback(can_devices[fdcan_id].call_backs, (void*)(void**)&func);
}
#endif
