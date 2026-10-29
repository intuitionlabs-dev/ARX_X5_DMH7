#pragma once

#include "can_config.h"
#include "stdint.h"

typedef uint32_t basic_data_t;

typedef void (*can_rx_callback)(uint8_t can_id, uint32_t identifier, uint8_t* data,
    basic_data_t len);

void BSP_CAN_Init(void);
void BSP_CAN_AddFilter(uint8_t can_id, uint16_t filter);
void BSP_CAN_RemoveFilter(uint8_t can_id, uint16_t filter);
/* 发送一帧。 */
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t* data, basic_data_t len);
/* 检查锁存相位是否命中组掩码；驱动据此同步控制计算和发送频率。 */
uint8_t BSP_CAN_TxGroupMatch(uint8_t tx_group);

/* 每个电机任务周期调用一次，锁存 osKernelGetTickCount() 对应的发送相位。 */
void BSP_CAN_TxGroupLatch(void);

/* 每轮各总线发送后忙等最小帧间隔；间隔为 0 时为空实现。 */
void BSP_CAN_TxGapWait(void);

/* 每周期检查发送健康状态，暂停异常总线并按配置间隔恢复探测。 */
void BSP_CAN_TxHealthStep(void);

void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
void BSP_ADD_Filters(uint8_t can_id);
