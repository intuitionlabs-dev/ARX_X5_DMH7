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

/* ---- 发送分组（2026-10-05）----------------------------------------------
 * 每次 BSP_CAN_Send() 要多带一个 8 bit 的组掩码：“这一帧属于哪些组”。
 * 总线自己按 `HAL_GetTick() % CAN_TX_GROUP_NUM` 算出**当前组**，
 * 掩码里没有当前组就直接把这次调用丢掉（不发送，也不报错）。
 * 于是**发送频率由掩码决定，驱动里不再需要 fre_rel / frequency_cnt 那种分频计数**。
 *
 * 8 组 ⇒ 单个组 = 125 Hz。要更高频率就把多个组或起来；名字里的 **G 是相位**：
 * 频率相同、G 不同的两路帧会落在不同的 tick 上，天然错峰，不会挤在同一拍里发。
 *
 * 对照表（每一行都标了频率和它占的组号）：
 *   1000 Hz : 全部 8 组 (0..7)                    —— 每拍都发
 *    500 Hz : G0 = 组 0,2,4,6 ；G1 = 组 1,3,5,7   —— 两个相位
 *    250 Hz : G0 = 组 0,4 ；G1 = 1,5 ；G2 = 2,6 ；G3 = 3,7
 *    125 Hz : G0..G7 = 组 0..7                    —— 八个相位
 *
 * 用法示例：4 台达妙都要 500 Hz，但不想同时发 —— 两台用 CAN_TX_500HZ_G0，
 * 另外两台用 CAN_TX_500HZ_G1。改频率只改配置里那一个宏。
 * 设零/使能/查询这类"必须发出去"的命令帧请用 CAN_TX_GROUP_ALL，别被分组吃掉。
 *
 * 驱动里要**先问一次 `BSP_CAN_TxGroupMatch(config.tx_group)` 再进循环体**
 * (见 libs/dm_motor、libs/encos_motor)：只发不算是浪费，只算不发则 PID 会按
 * 任务周期(1 kHz)跑而不是发送周期(500 Hz)，而 libs/controller 的 PID 没有 dt，
 * 增益按调用次数算 —— KI 会凭空翻倍。 */
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
