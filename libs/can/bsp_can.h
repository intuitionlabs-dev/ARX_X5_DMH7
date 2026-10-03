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
void BSP_CAN_Send(uint8_t can_id, uint16_t identifier, uint8_t* data,
                  basic_data_t len);
void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
void BSP_ADD_Filters(uint8_t can_id);
