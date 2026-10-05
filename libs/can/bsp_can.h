#pragma once

#include "can_config.h"
#include "stdint.h"

/* 原框架里由 osal/osal.h 提供 */
typedef uint32_t basic_data_t;

typedef void (*can_rx_callback)(uint8_t can_id, uint32_t identifier, uint8_t* data,
    basic_data_t len);

void BSP_CAN_Init(void);
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter);
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter);
/* 发送一帧。*/
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t* data, basic_data_t len);
/* 掩码里有没有"当前组"（当前组 = osKernelGetTickCount() % CAN_TX_GROUP_NUM）。
 * 驱动拿它把"算控制量"和"发帧"卡在同一个节拍上：算之前在循环体外面问一次，
 * 不匹配就整拍跳过 —— 否则 PID 会按任务周期(1 kHz)跑而不是按发送周期(500 Hz)，
 * 而 libs/controller 的 PID 没有 dt，增益是按调用次数算的，KI 会凭空翻倍。
 * 纯读 osKernelGetTickCount()，无副作用，随便调。 */
uint8_t BSP_CAN_TxGroupMatch(uint8_t tx_group);
void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
void BSP_ADD_Filters(uint8_t can_id);
