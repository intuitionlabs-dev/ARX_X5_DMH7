#include "bsp_can.h"
#include "bsp_supervise.h"
#include "cmsis_os2.h"
#if _BSP_CAN_ENABLE && _BSP_FDCAN_ENABLE
    #error
#endif

/* 每个电机任务周期锁存一次发送相位，周期内保持一致。 */
static uint8_t s_tx_group_phase = 0u;

void BSP_CAN_TxGroupLatch(void) {
    /* 发送分组使用 RTOS 调度时钟。 */
    s_tx_group_phase = (uint8_t)(osKernelGetTickCount() % CAN_TX_GROUP_NUM);
}

uint8_t BSP_CAN_TxGroupMatch(uint8_t tx_group) {
    uint8_t pass = (tx_group & (uint8_t)(1u << s_tx_group_phase)) != 0u;
    return pass;
}

#if _BSP_CAN_ENABLE && DEVICE_CAN_CNT
    #include "can.h"  // IWYU pragma: keep
    #include "cvector.h"

typedef struct Bsp_can_type_def_t {
    CAN_HandleTypeDef *device;  /* 自定义总线编号 */
    uint32_t tx_mailbox;
    cvector *call_backs;
    cvector *frame_callbacks;
    uint32_t bank_prefix;              /* 不同can对应的过滤器相关参数值 */
    uint16_t filters[FILTER_MAX_CNT];  /* 按标准帧ID定义 */
} Bsp_can_type_def_t;

Bsp_can_type_def_t can_devices[DEVICE_CAN_CNT];

void BSP_CAN_Init(void) {
    CAN_HandleTypeDef *can_ports[DEVICE_CAN_CNT] = CAN_PORT_ARRAY;
    uint32_t prefix_array[DEVICE_CAN_CNT] = CAN_PREFIX_ARRAY;
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        can_devices[i].device = can_ports[i];
        can_devices[i].bank_prefix = prefix_array[i];
        can_devices[i].call_backs = cvector_create(sizeof(can_rx_callback));
        can_devices[i].frame_callbacks = cvector_create(sizeof(Can_frame_rx_callback_t));
    }
    for (size_t d = 0; d < DEVICE_CAN_CNT; ++d) {
        for (size_t i = 0; i < FILTER_MAX_CNT; ++i) {
            can_devices[d].filters[i] = ID_NOTSET;
        }
    }
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        HAL_CAN_Start(can_devices[i].device);  /* 将HAL库的对应功能函数封装，实现映射和隔离 */
        HAL_CAN_ActivateNotification(can_devices[i].device,
            CAN_IT_RX_FIFO0_MSG_PENDING);  /* 检测can是否报错 */
        HAL_CAN_ActivateNotification(can_devices[i].device, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

/* 仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参 */
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t *data, basic_data_t len,
    uint8_t tx_group) {
    /* 发送分组由驱动在控制计算前检查。 */
    (void)tx_group;
    #if !_BSP_DEBUG_CAN_NOT_SEND
    CAN_TxHeaderTypeDef txconf;  /* 配置发送包头 */
    txconf.StdId = identifier;
    txconf.IDE = CAN_ID_STD;
    txconf.RTR = CAN_RTR_DATA;
    txconf.DLC = len;
    basic_data_t can_tx_start_time = BSP_sys_time_ms();
    while (HAL_CAN_GetTxMailboxesFreeLevel(can_devices[can_id].device) == 0) {  /* 获取空闲邮箱数量 */

        basic_data_t can_tx_now_time = BSP_sys_time_ms();
        /* 2ms还没有发送出去，放弃发送 */
        if (can_tx_now_time > can_tx_start_time + 2) {
            HAL_CAN_AbortTxRequest(can_devices[can_id].device,
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            __NOP();
            break;
        }
    }
    HAL_CAN_AddTxMessage(can_devices[can_id].device, &txconf,
        data,  /* 上次信息发送成功，传输新信息 */
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

    tmp.FilterFIFOAssignment = (group_index & 1)
        ? CAN_RX_FIFO0
        : CAN_RX_FIFO1;  /* 4个一组，按照组别设置FIFO，平衡负载 */
    tmp.SlaveStartFilterBank = 14;
    uint16_t *filter_data = can_devices[can_id].filters;  /* 与can总线保持一致 */
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
    HAL_CAN_ConfigFilter(can_devices[can_id].device, &tmp);  /* 配置can接收过滤器 */
}

void BSP_CAN_FifoMsg_Callback(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (hcan == can_devices[i].device) {
            uint8_t bsp_can_rxbuf[8];
            CAN_RxHeaderTypeDef rxconf;
            HAL_CAN_GetRxMessage(hcan, fifo, &rxconf, bsp_can_rxbuf);
            for (size_t j = 0; j < can_devices[i].call_backs->cv_len; ++j) {
                can_rx_callback funcnow =
                    *(can_rx_callback *)cvector_val_at(can_devices[i].call_backs, j);
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

    #include "cvector.h"
    #include "fdcan.h"  // IWYU pragma: keep

typedef struct Bsp_fd_can_type_def_t {
    FDCAN_HandleTypeDef *device;  /* 自定义总线编号 */
    uint32_t tx_mailbox;
    cvector *call_backs;
    uint32_t bank_prefix;              /* 不同can对应的过滤器相关参数值 */
    uint16_t filters[FILTER_MAX_CNT];  /* 按标准帧ID定义 */
} Bsp_fd_can_type_def;

Bsp_fd_can_type_def can_devices[DEVICE_CAN_CNT];

    /* 每轮发送后按 DWT 周期等待最小帧间隔；cycles/us 由 HAL 毫秒时基校准。 */
    #define BSP_CAN_TX_MIN_GAP_US 200u

    #if BSP_CAN_TX_MIN_GAP_US
static uint32_t tx_gap_cycles_per_us; /* 0 表示周期计数器尚未校准。 */
static uint32_t tx_gap_last_cyc;

static void tx_gap_calibrate(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u; /* 解锁 Cortex-M7 的 DWT 寄存器。 */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t t = HAL_GetTick();
    while (HAL_GetTick() == t) {}
    uint32_t c0 = DWT->CYCCNT;
    t = HAL_GetTick();
    while (HAL_GetTick() == t) {}
    uint32_t c1 = DWT->CYCCNT;
    tx_gap_cycles_per_us = (c1 - c0) / 1000u;
    tx_gap_last_cyc = DWT->CYCCNT;
}

/* 每轮各总线发送后等待配置的最小帧间隔。 */
void BSP_CAN_TxGapWait(void) {
    if (!tx_gap_cycles_per_us) {
        tx_gap_calibrate();
    }
    uint32_t need = BSP_CAN_TX_MIN_GAP_US * tx_gap_cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - tx_gap_last_cyc) < need) {}
    tx_gap_last_cyc = DWT->CYCCNT;
}
    #else
/* 间隔控制停用时提供空实现。 */
void BSP_CAN_TxGapWait(void) {}
    #endif

    /* 错误被动或连续 FIFO 超时的总线暂停发送，并周期性恢复探测。 */
    #define CAN_TX_TIMEOUT_BLOCK_N 2u /* 连续等 FIFO 超时几次就关这一路 */
    #define CAN_TX_PROBE_MS 50u       /* 关掉之后每多久重开一次试 */

    #define CAN_TX_ABORT_ALL 0xFFFFFFFFu

typedef struct {
    uint32_t drop_cnt;    /* 关着期间丢掉的帧数 */
    uint32_t timeout_cnt; /* 连续超时次数；发送成功或暂停发送时清零。 */
    uint8_t blocked;      /* 1 = 这一路暂停发送 */
    uint32_t probe_ms;    /* blocked 期间上一次探测的时刻 */
    uint32_t recover_cnt; /* 关着门期间做过几次探测(中止 FIFO + 清 INIT) */
    /* 保存上次读取的 FDCAN 协议状态。 */
    FDCAN_ProtocolStatusTypeDef proto_status;
} Can_tx_health;

static Can_tx_health s_tx_health[DEVICE_CAN_CNT];

void BSP_CAN_TxHealthStep(void) {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        Can_tx_health *health = &s_tx_health[i];
        /* 错误被动或连续 FIFO 超时达到阈值时暂停该总线。 */
        HAL_FDCAN_GetProtocolStatus(can_devices[i].device, &health->proto_status);
        if (!health->proto_status.ErrorPassive
            && health->timeout_cnt < CAN_TX_TIMEOUT_BLOCK_N) {
            health->blocked = 0u;
            continue;
        }
        health->blocked = 1u;
        if ((uint32_t)(now - health->probe_ms) < CAN_TX_PROBE_MS) {
            continue;
        }
        health->probe_ms = now;
        /* 恢复前中止全部排队命令，避免发送过期目标。 */
        HAL_FDCAN_AbortTxRequest(can_devices[i].device, CAN_TX_ABORT_ALL);
        /* 依次 Stop、Start，使 HAL 状态允许清除 INIT 并启动总线恢复。 */
        HAL_FDCAN_Stop(can_devices[i].device);
        HAL_FDCAN_Start(can_devices[i].device);
        health->recover_cnt++;
        health->timeout_cnt = 0u;
        /* 探测周期允许发送，以便成功发送后退出错误被动态。 */
        health->blocked = 0u;
    }
}

void BSP_CAN_Init(void) {
    FDCAN_HandleTypeDef *can_ports[DEVICE_CAN_CNT] = CAN_PORT_ARRAY;
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
        if (HAL_FDCAN_ConfigGlobalFilter(can_devices[i].device, FDCAN_REJECT,
                FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)
            != HAL_OK) {
            return;
        }
        HAL_FDCAN_Start(can_devices[i].device);  /* 将HAL库的对应功能函数封装，实现映射和隔离 */
        HAL_FDCAN_ActivateNotification(can_devices[i].device,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_FULL, 0);

        HAL_FDCAN_ActivateNotification(can_devices[i].device,
            FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_FULL, 0);
    }
}


/* 仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参 */
void BSP_CAN_Send(uint8_t fdcan_id, uint16_t identifier, uint8_t *data, basic_data_t len) {
    #if !_BSP_DEBUG_CAN_NOT_SEND
    /* 本函数不等最小帧间隔：等待由调用方按"轮"做，见 BSP_CAN_TxGapWait() */
    Can_tx_health *health = &s_tx_health[fdcan_id];
    /* 总线被健康检查暂停时立即丢弃待发帧。 */
    if (health->blocked) {
        health->drop_cnt++;
        return;
    }
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
    txconf.Identifier = identifier;
    txconf.IdType = FDCAN_STANDARD_ID;
    txconf.TxFrameType = FDCAN_DATA_FRAME;
    txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txconf.BitRateSwitch = FDCAN_BRS_OFF;
    txconf.FDFormat = FDCAN_CLASSIC_CAN;

    txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txconf.MessageMarker = 0;

    /* 等待 Tx FIFO 空闲，超过 1 ms 后中止本次发送。 */
    uint32_t tx_start_time = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(can_devices[fdcan_id].device) == 0) {
        if (HAL_GetTick() - tx_start_time >= 1) {
            // clang-format off
            HAL_FDCAN_AbortTxRequest(can_devices[fdcan_id].device,
                FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2 | FDCAN_TX_BUFFER3 | FDCAN_TX_BUFFER4);
            // clang-format on
            health->timeout_cnt++; /* 这一路整整 1ms 一帧都没出去 */
            return;
        }
    }
    health->timeout_cnt = 0u;

    HAL_FDCAN_AddMessageToTxFifoQ(can_devices[fdcan_id].device, &txconf, data);
    #endif
}

void Update_Filter(uint8_t fdcan_id, uint32_t filter_index) {
    FDCAN_FilterTypeDef tmp;
    uint16_t id = can_devices[fdcan_id].filters[filter_index];

    tmp.IdType = FDCAN_STANDARD_ID; /* 只接收标准帧ID */
    tmp.FilterIndex = filter_index; /* 一个滤波元素对应 filters[] 的一个槽位 */
    tmp.FilterType = FDCAN_FILTER_RANGE; /* 范围滤波；FilterID1 == FilterID2 时即精确匹配该ID */

    if (id == ID_NOTSET) {
        /* 该槽位没登记ID，关掉这个滤波元素。
 * 注意 ID_NOTSET(0x800) 超过标准帧ID上限 0x7FF，不能直接写进 FilterID1。 */
        tmp.FilterConfig = FDCAN_FILTER_DISABLE;
        tmp.FilterID1 = 0;
        tmp.FilterID2 = 0;
    } else {
        /* 滤波器目标 FIFO 必须具有已配置的接收元素。 */
        tmp.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        tmp.FilterID1 = id;
        tmp.FilterID2 = id;
    }

    HAL_FDCAN_ConfigFilter(can_devices[fdcan_id].device, &tmp);
}

static const uint8_t DL_CTO_BYTES[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static void bsp_can_fifo_msg_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo,
    uint32_t RxFifo0ITs) {
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (hfdcan == can_devices[i].device) {
            /* 按最大 CAN FD 载荷预留 64 字节接收空间。 */
            uint8_t bsp_can_rxbuf[64];
            FDCAN_RxHeaderTypeDef rxconf;
            HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxconf,
                bsp_can_rxbuf); /* 提取FIFO0中接收到的数据 */

            for (size_t j = 0; j < can_devices[i].call_backs->cv_len; ++j) {
                can_rx_callback funcnow =
                    *(can_rx_callback *)cvector_val_at(can_devices[i].call_backs, j);
                /* HAL 返回 0～15 的原始 DLC，直接索引 DLCtoBytes。 */
                funcnow(i, rxconf.Identifier, bsp_can_rxbuf,
                    DL_CTO_BYTES[rxconf.DataLength & 0x0FU]);
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

/* 注册新的接收过滤器 */
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter) {
    /* 已经登记过的 ID 直接返回：一条总线上多台电机用同一个接收 ID 是常见情形
 * (达妙四台的反馈帧仲裁ID 都是 MST_ID=0)，重复占槽位没有任何作用 */
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT; ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == filter) {
            return;
        }
    }
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT; ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == ID_NOTSET) {
            can_devices[can_id].filters[filter_index] = filter;
            Update_Filter(can_id, filter_index);
            return;
        }
    }
}

/* 注销过滤器 */
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter) {
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT; ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == filter) {
            can_devices[can_id].filters[filter_index] = ID_NOTSET;
            Update_Filter(can_id, filter_index);
        }
    }
}

/* 注册回调函数 */
void BSP_CAN_RegisterRxCallback(uint8_t fdcan_id, can_rx_callback func) {
    cvector_pushback(can_devices[fdcan_id].call_backs, (void *)(void **)&func);
}
#endif
