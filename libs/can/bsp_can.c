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
#include "bsp_supervise.h"
#include "cmsis_os2.h"
#if _BSP_CAN_ENABLE && _BSP_FDCAN_ENABLE
    #error
#endif

/* 本拍的组相位。**每拍由 BSP_CAN_TxGroupLatch() 锁存一次**，见 bsp_can.h 里那段注释：
 * 一拍的发送是轮转做的(见 Core/Src/freertos.c 的 StartMotorTask)，一拍要跨 600~800us，
 * 现读时钟的话毫秒边沿会落在轮转中间，同一拍里排在后面的电机会被"下一组的门"整拍挡掉。 */
static uint8_t s_tx_group_phase = 0u;

void BSP_CAN_TxGroupLatch(void) {
    /* 时基必须用 osKernelGetTickCount()，不能是 HAL_GetTick()：门控要和任务调度
     * (SysTick)同相，两者实测差 43 ppm，混用会让掩码对不上频率。 */
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
    CAN_HandleTypeDef *device;  // 自定义总线编号
    uint32_t tx_mailbox;
    cvector *call_backs;
    cvector *frame_callbacks;
    uint32_t bank_prefix;              // 不同can对应的过滤器相关参数值
    uint16_t filters[FILTER_MAX_CNT];  // 按标准帧ID定义
} Bsp_can_type_def_t;

Bsp_can_type_def_t can_devices[DEVICE_CAN_CNT];  // 定义对应数量的can外设，即代码中定义can总线

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
            can_devices[d].filters[i] = ID_NOTSET;  // 初始化can过滤器
        }
    }
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        HAL_CAN_Start(can_devices[i].device);  // 将HAL库的对应功能函数封装，实现映射和隔离
        HAL_CAN_ActivateNotification(can_devices[i].device,
            CAN_IT_RX_FIFO0_MSG_PENDING);  // 检测can是否报错
        HAL_CAN_ActivateNotification(can_devices[i].device, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

//  仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t *data, basic_data_t len,
    uint8_t tx_group) {
    /* 这里**不再**重复判分组：判一次是不够、判两次是错的。
     * 需要分组的调用者(两个电机驱动)在进循环体之前就判过了；
     * 命令帧传的是 CAN_TX_GROUP_ALL，每一位都命中，判了永远是真。
     * 而判两次用的又是两次独立的 HAL_GetTick() —— 只要毫秒边沿落在
     * "驱动判过"和"这里再判"之间，这一帧就被静默丢掉，纯粹白丢。
     * 参数留着是为了调用点可读（以及将来要在这层加分组时不用改所有调用点）。 */
    (void)tx_group;
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
            HAL_CAN_AbortTxRequest(can_devices[can_id].device,
                CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
            __NOP();
            break;
        }
    }
    HAL_CAN_AddTxMessage(can_devices[can_id].device, &txconf,
        data,  // 上次信息发送成功，传输新信息
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
    tmp.FilterFIFOAssignment = (group_index & 1)
        ? CAN_RX_FIFO0
        : CAN_RX_FIFO1;  // 4个一组，按照组别设置FIFO，平衡负载
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
    FDCAN_HandleTypeDef *device;  //自定义总线编号
    uint32_t tx_mailbox;
    cvector *call_backs;
    uint32_t bank_prefix;              //不同can对应的过滤器相关参数值
    uint16_t filters[FILTER_MAX_CNT];  //按标准帧ID定义
} Bsp_fd_can_type_def;

Bsp_fd_can_type_def can_devices[DEVICE_CAN_CNT];  //定义对应数量的can外设，即代码中定义can总线

    /* ---- 最小帧间隔(按轮等) --------------------------------------------------
 * 把**我们自己**的发送节奏摊平成"每帧至少隔 N 微秒"：一 tick 里连着发好几帧
 * （多台电机指令帧挨着发 → 几乎同时回反馈）会把总线的空档挤没。
 * 注意：它只改"我们发帧"的间隔，**改不了电机的反馈节奏**（那是电机自己发的回应）。
 * 计时用 DWT->CYCCNT，开机后用 HAL_GetTick() 的 1ms 边沿自校准 cycles/us ——
 * 不猜主频：H7 的 CPUCLK 和 HCLK 不是一个数，SystemCoreClock 给的是 HCLK。
 *
 * 等的那一下由 MotorTask 在轮转循环末尾做(见 BSP_CAN_TxGapWait 的声明)，不在
 * BSP_CAN_Send() 里：**一轮里两路各发一帧、一轮只等这一次**，计时器被两条总线
 * 共用，避免逐帧等时每路的帧率砍半(5000 -> 2500 帧/s)。
 * 实测：CAN1 上达妙反馈 500 Hz、j3 正常。 */
    #define BSP_CAN_TX_MIN_GAP_US 200u

    #if BSP_CAN_TX_MIN_GAP_US
static uint32_t tx_gap_cycles_per_us; /* 0 = 还没校准 */
static uint32_t tx_gap_last_cyc;

static void tx_gap_calibrate(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u; /* Cortex-M7 要解锁才写得进 DWT */
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

/* 距上一帧不足 N us 就原地等 —— 调用者(MotorTask)会被拖住，这是代价。
 * 由 MotorTask 每"轮"调一次(BSP_CAN_TxGapWait)，不在 BSP_CAN_Send 里调。 */
void BSP_CAN_TxGapWait(void) {
    if (!tx_gap_cycles_per_us) {
        tx_gap_calibrate();
    }
    uint32_t need = BSP_CAN_TX_MIN_GAP_US * tx_gap_cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - tx_gap_last_cyc) < need) {}
    tx_gap_last_cyc = DWT->CYCCNT;
}
    #else
/* 关掉间隔控制时空实现：调用方(MotorTask)不用写 #if */
void BSP_CAN_TxGapWait(void) {}
    #endif

    /* ---- 发送健康：一条发不出去的总线不要拿它拖累另一条 ----------------------
 * BSP_CAN_Send() 里"满了就等最多 1ms"那段等待**照旧保留** —— 一个周期里要连发几帧，
 * 靠它把帧塞进 FIFO —— 但只对还能把帧发出去的总线保留。等在一路发不出去的总线上，
 * 换来的是另一条总线上电机的控制频率被拖下来(控制律在发送步骤里算，两条总线共用
 * MotorTask，见 DMmotor_Send_Step)，而那一帧本来也出不去。
 *
 * 判据见 BSP_CAN_TxHealthStep()：硬件报 error-passive(总线上已经没有能应答的节点了)，
 * 或这一路等 FIFO 等超时攒够了次数。两条在健康总线上都恒不成立，所以正常运行时这门一帧都不影响。 */
    #define CAN_TX_TIMEOUT_BLOCK_N 2u /* 连续等 FIFO 超时几次就关这一路 */
    #define CAN_TX_PROBE_MS 50u       /* 关掉之后每多久重开一次试 */
    /* Tx FIFO 的全部 32 个元素(深度见 Core/Src/fdcan.c 的 TxFifoQueueElmtsNbr=32，
     * TxBuffersNbr=0 ⇒ 元素 0~31 就是整个 Tx FIFO)。 */
    #define CAN_TX_ABORT_ALL 0xFFFFFFFFu

typedef struct {
    uint32_t drop_cnt;    /* 关着期间丢掉的帧数 */
    uint32_t timeout_cnt; /* 连续超时次数；成功发出或关门时清零 */
    uint8_t blocked;      /* 1 = 这一路暂停发送 */
    uint32_t probe_ms;    /* blocked 期间上一次探测的时刻 */
    uint32_t recover_cnt; /* 关着门期间做过几次探测(中止 FIFO + 清 INIT) */
    /* 上一拍 HAL 读到的硬件状态。"门是被哪条证据关的"看它就够：
     * blocked=1 且 ErrorPassive=1 ⇒ ①；blocked=1 且 ErrorPassive=0 ⇒ ②，看 timeout_cnt。 */
    FDCAN_ProtocolStatusTypeDef proto_status;
} Can_tx_health;

static Can_tx_health s_tx_health[DEVICE_CAN_CNT];

void BSP_CAN_TxHealthStep(void) {
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        Can_tx_health *health = &s_tx_health[i];
        /* 关门(这一路停发)有两条证据，任一成立就关：
         *   ① 硬件报 error-passive(TEC 或 REC ≥ 128) —— 总线上已经没有能应答我们帧的节点了
         *      (拔线、电机断电)。这条不靠等：那一路的 FIFO 永远排不空，"等 FIFO 空闲"
         *      等不出名堂来。
         *      拔线这一路硬件就停在 error-passive 上不动：TEC 涨到 128 之后 ISO 11898-1 的
         *      例外规则不再让它涨(error-passive 的节点只碰到 ACK 错、发 passive error flag
         *      时又没看到显性位 ⇒ TEC 不变)，于是 TEC 冻在 128、`PSR.BO` 恒 0
         *      (真机实测 ECR 恒 0x80、PSR 恒 0x77B：帧在反复重试，错误计数一个都不动)。
         *   ② 这一路等 FIFO 等超时攒够了次数 —— 硬件不报错，但帧就是塞不进队列。 */
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
        /* 先把挂在 Tx FIFO 里的旧指令**全部**中止掉：清 INIT 之后硬件会把它们立刻重发一遍，
         * 而这一路刚被判死、必然还是发不出去 —— 刚走完的恢复序列立刻又被打断，门立刻又关上。
         * 一帧都不能留：关门那一刻 FIFO 里堆着多少帧取决于怎么死的(死总线上每帧都发不出去、
         * DAR 又开着，元素只进不出，要等发送路径那段 1ms 超时按 5 个一组往外放)，
         * 堆到 30 以上时只清前 5 个就不够。 */
        HAL_FDCAN_AbortTxRequest(can_devices[i].device, CAN_TX_ABORT_ALL);
        /* bus-off 之后硬件置 CCCR.INIT 而没人清，硬件停在原地不动 —— 电缆插回来也回不去。
         * 硬件只在 INIT 被清掉之后才开始 bus-off 恢复序列(在总线上数够 129 次 11 个连续
         * 隐性位，数完自己清 PSR.BO 和 ECR 的两个计数)，所以"恢复"就是清 CCCR.INIT。
         * HAL 里是 Stop/Start 这一对：Stop 置 INIT、Start 清 INIT，而 Start 只认 Stop 之后
         * 的 READY 态，两个必须配对(单独 Start 直接返回 HAL_ERROR，一个寄存器都不写)。
         * 这一对只写 CCCR，不等总线，也不碰滤波器/RX 通知。 */
        HAL_FDCAN_Stop(can_devices[i].device);
        HAL_FDCAN_Start(can_devices[i].device);
        health->recover_cnt++;
        health->timeout_cnt = 0u;
        /* 探测这一下必须**真的把门放开**：判据①(error-passive)只有等一次发送成功(TEC 减 1)
         * 才消得掉，关着门就永远消不掉 —— 电缆插回来了门也只在 50ms 一次的探测里开着又关上。
         * 放开这一拍就够它试一次：FIFO 刚被清空、发帧不用等，只要总线上还有节点应答，这一帧
         * 就成功、EP 当拍就清，下一拍门自然开着；没人应答就下一拍再关上。 */
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
        HAL_FDCAN_Start(can_devices[i].device);  // 将HAL库的对应功能函数封装，实现映射和隔离
        HAL_FDCAN_ActivateNotification(can_devices[i].device,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_FULL, 0);
        // HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device,
            FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_FULL, 0);
        // HAL_FDCAN_ActivateNotification(can_devices[i].device, FDCAN_IT_RX_FIFO1_WATERMARK, 0);
    }
}

FPS_t j1_tx_fps;
//  仅对HAL库的can发送函数进行了形式上的封装，实际参数依然依赖外部传参
void BSP_CAN_Send(uint8_t fdcan_id, uint16_t identifier, uint8_t *data, basic_data_t len) {
    #if !_BSP_DEBUG_CAN_NOT_SEND
    /* 本函数不等最小帧间隔：等待由调用方按"轮"做，见 BSP_CAN_TxGapWait() */
    Can_tx_health *health = &s_tx_health[fdcan_id];
    /* 这一路已判定"发不出去"(见 BSP_CAN_TxHealthStep)：丢掉这一帧，不读寄存器也不等。
     * 在这里等的结果本来就是下面那条超时分支(abort + 丢帧)，帧一样出不去，白等。 */
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
            health->timeout_cnt++; /* 这一路整整 1ms 一帧都没出去 */
            return;
        }
    }
    health->timeout_cnt = 0u;

    HAL_FDCAN_AddMessageToTxFifoQ(can_devices[fdcan_id].device, &txconf, data);
    #endif  //_BSP_DEBUG_CAN_NOT_SEND
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
static void bsp_can_fifo_msg_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo,
    uint32_t RxFifo0ITs) {
    for (int i = 0; i < DEVICE_CAN_CNT; ++i) {
        if (hfdcan == can_devices[i].device) {
            /* 必须 >= 64 字节：HAL_FDCAN_GetRxMessage() 是按 DLCtoBytes[DataLength] 拷的，
             * DLC 最大 15 对应 64 字节。本工程三路都是 classic CAN、元素也配的 8 字节，
             * 正常最多 8 字节；但只要对端发一帧 FD 帧且 ID 命中滤波器，8 字节的栈缓冲
             * 就会被拷爆(在中断里，栈是 MSP)。留够 64。 */
            uint8_t bsp_can_rxbuf[64];
            FDCAN_RxHeaderTypeDef rxconf;
            HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rxconf,
                bsp_can_rxbuf); /* 提取FIFO0中接收到的数据 */
                                /* 更新RXFIFO的状态（了解FIFO是否有剩余报文） */
            for (size_t j = 0; j < can_devices[i].call_backs->cv_len; ++j) {
                can_rx_callback funcnow =
                    *(can_rx_callback *)cvector_val_at(can_devices[i].call_backs, j);
                /* 本工程的 HAL(见 stm32h7xx_hal_fdcan.c:3084)在 HAL_FDCAN_GetRxMessage()
                 * 里已经把 DLC 右移过了(掩码 0x000F0000)，rxconf.DataLength 就是 0~15 的
                 * 裸 DLC 值(FDCAN_DLC_BYTES_8 == 8)。这里再 >>16 会恒得 0，导致所有回调
                 * 收到的 len 都是 0、一个字节都拷不出去。直接拿它当表下标。 */
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

// 注册新的接收过滤器
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
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

// 注销过滤器
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter) {
    // uint32_t filter_index = can_devices[can_id].filter_cnt++;
    for (size_t filter_index = 0; filter_index < FILTER_MAX_CNT; ++filter_index) {
        if (can_devices[can_id].filters[filter_index] == filter) {
            can_devices[can_id].filters[filter_index] = ID_NOTSET;
            Update_Filter(can_id, filter_index);
        }
    }
}

// 注册回调函数
void BSP_CAN_RegisterRxCallback(uint8_t fdcan_id, can_rx_callback func) {
    cvector_pushback(can_devices[fdcan_id].call_backs, (void *)(void **)&func);
}
#endif
