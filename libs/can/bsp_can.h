#pragma once

#include "stdint.h"
#include "can_config.h"

/* 原框架里由 osal/osal.h 提供 */
typedef uint32_t basic_data_t;

typedef void (*can_rx_callback)(uint8_t can_id, uint32_t identifier, uint8_t* data,
                                basic_data_t len);

void BSP_CAN_Init(void);
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter);
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter);
/* 发送一帧。tx_group 是组掩码(见 can_config.h 的 CAN_TX_xxxHZ_Gn / CAN_TX_GROUP_ALL)：
 * 第 n 位 = 1 表示这一帧属于第 n 组。总线按 HAL_GetTick() % CAN_TX_GROUP_NUM 判定当前组，
 * **掩码里没有当前组的这一次调用会被直接丢弃**（不发送，无声返回）。
 * 想"每拍都发"就传 CAN_TX_GROUP_ALL；设零/使能这类必须送达的命令帧也用它。 */
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t* data,
                  basic_data_t len, uint8_t tx_group);
/* 掩码里有没有"当前组"（当前组 = HAL_GetTick() % CAN_TX_GROUP_NUM）。
 * 驱动拿它把"算控制量"和"发帧"卡在同一个节拍上：算之前在循环体外面问一次，
 * 不匹配就整拍跳过 —— 否则 PID 会按任务周期(1 kHz)跑而不是按发送周期(500 Hz)，
 * 而 libs/controller 的 PID 没有 dt，增益是按调用次数算的，KI 会凭空翻倍。
 * 纯读 HAL_GetTick()，无副作用，随便调。 */
uint8_t BSP_CAN_TxGroupMatch(uint8_t tx_group);
void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
void BSP_ADD_Filters(uint8_t can_id);
