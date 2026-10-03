#pragma once

/* 本工程自己的 CAN 配置。
 * 这些宏原本散在框架的 config/board_def.h 和 config/mcu_config/RT_H723.h 里，
 * 为了 libs/ 不再依赖嵌入式框架，收拢到这里。
 * 值按 CtrBoard-H7_ALL 的实际情况定：三个 FDCAN 实例，CubeMX 里 StdFiltersNbr = 16。 */

#include "fdcan.h"  /* hfdcan1 / hfdcan2 / hfdcan3 */

#define _BSP_FDCAN_ENABLE       1
#define _BSP_CAN_ENABLE         0  /* 本工程只有 FDCAN；两者同时为 1 会触发 bsp_can.c 的 #error */
#define _BSP_DEBUG_CAN_NOT_SEND 0

#define DEVICE_CAN_CNT 3

/* 与 Bsp/can_bsp.c 顺序一致：索引 0/1/2 -> FDCAN1/2/3 */
#define CAN_PORT_ARRAY  {&hfdcan1, &hfdcan2, &hfdcan3}
#define CAN_PREFIX_ARRAY {0, 0, 0}

/* 必须 <= CubeMX 的 StdFiltersNbr，否则 HAL_FDCAN_ConfigFilter 会写到消息 RAM 之外 */
#define FILTER_MAX_CNT 16

#define ID_NOTSET 0x800  /* 大于标准帧 ID 上限 0x7FF，用作“该槽位空闲”的标记 */
