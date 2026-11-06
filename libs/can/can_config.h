#pragma once

/* 配置 FDCAN 实例、滤波容量和发送相位。 */

#include "fdcan.h"  /* hfdcan1 / hfdcan2 / hfdcan3 */

#define _BSP_FDCAN_ENABLE       1
#define _BSP_CAN_ENABLE         0  /* CAN 和 FDCAN 后端须互斥启用。 */
#define _BSP_DEBUG_CAN_NOT_SEND 0

#define DEVICE_CAN_CNT 3

/* 与 Bsp/can_bsp.c 顺序一致：索引 0/1/2 -> FDCAN1/2/3 */
#define CAN_PORT_ARRAY  {&hfdcan1, &hfdcan2, &hfdcan3}
#define CAN_PREFIX_ARRAY {0, 0, 0}

/* 必须 <= CubeMX 的 StdFiltersNbr，否则 HAL_FDCAN_ConfigFilter 会写到消息 RAM 之外 */
#define FILTER_MAX_CNT 16

#define ID_NOTSET 0x800  /* 大于标准帧 ID 上限 0x7FF，用作“该槽位空闲”的标记 */

/* RTOS 调度频率为 1 kHz，8 个相位对应单相位 125 Hz；驱动按掩码决定控制和发送频率。 */
/* 限制每周期每条总线的发送帧数，含驱动维护命令。 */
#define CAN_TX_MAX_FRAMES_PER_TICK 4u

#define CAN_TX_GROUP_NUM 8u
#define CAN_TX_GROUP_ALL 0xFFu /* 所有组：每拍都发(1000 Hz)，也可当"不过滤"用 */

#define CAN_TX_1000HZ CAN_TX_GROUP_ALL

#define CAN_TX_500HZ_G0 0x55u /* 组 0,2,4,6 */
#define CAN_TX_500HZ_G1 0xAAu /* 组 1,3,5,7 */

#define CAN_TX_250HZ_G0 0x11u /* 组 0,4 */
#define CAN_TX_250HZ_G1 0x22u /* 组 1,5 */
#define CAN_TX_250HZ_G2 0x44u /* 组 2,6 */
#define CAN_TX_250HZ_G3 0x88u /* 组 3,7 */

#define CAN_TX_125HZ_G0 0x01u
#define CAN_TX_125HZ_G1 0x02u
#define CAN_TX_125HZ_G2 0x04u
#define CAN_TX_125HZ_G3 0x08u
#define CAN_TX_125HZ_G4 0x10u
#define CAN_TX_125HZ_G5 0x20u
#define CAN_TX_125HZ_G6 0x40u
#define CAN_TX_125HZ_G7 0x80u
