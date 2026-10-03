#pragma once

#include <stdint.h>

/* CAN1 -> CAN2 自机通信(回环)测试
 *
 * 拓扑:本板 FDCAN1(索引 0) 发，FDCAN2(索引 1) 收。
 * 需要 CAN1、CAN2 的收发器挂在同一条总线上(板外连线或板上跳线):
 *     CAN1_H <-> CAN2_H,  CAN1_L <-> CAN2_L, 总线两端各 120Ω 终端电阻。
 *
 * 前提:CAN1 与 CAN2 的位速率必须完全一致。
 * 当前 CubeMX 配置三路 FDCAN 都是 1.000 Mbit/s
 * (内核时钟 120 MHz，Prescaler=6，Seg1=15，Seg2=4，采样点 80%)，满足该前提。
 *
 * 用法:
 *   CanLoopbackTest_Init();   // 调度器启动前调用(main.c 的 USER CODE BEGIN 2,
 *                             // 紧跟 BSP_CAN_Init() 之后)
 *   循环体里: CanLoopbackTest_Loop();  tick += 1;  osDelayUntil(tick);
 *             (uint32_t tick = osKernelGetTickCount(); 在循环外初始化一次)
 *
 * 结果不走串口，全部记在下面的全局变量里，直接用调试器(JLink/gdb)读地址。
 * 正常时 can_lb_tx_cnt 和 can_lb_rx_cnt 同步增长，bad/lost 恒为 0，
 * 四个总线健康度变量恒为 0。
 */

/* ---- 可调参数 ---------------------------------------------------------- */
#define CAN_LB_TX_CAN    0u    /* 发送方 BSP_CAN 索引 -> FDCAN1 */
#define CAN_LB_RX_CAN    1u    /* 接收方 BSP_CAN 索引 -> FDCAN2 */
#define CAN_LB_ID        0x123u /* 测试帧使用的标准帧 ID */
#define CAN_LB_PERIOD_MS 100u   /* 发送周期, ms */

/* ---- 收发统计 ---------------------------------------------------------- */
extern volatile uint32_t can_lb_tx_cnt;     /* 已发出帧数 */
extern volatile uint32_t can_lb_rx_cnt;     /* 已正确收到的帧数 */
extern volatile uint32_t can_lb_bad_cnt;    /* 收到了但 ID/长度/内容不对 */
extern volatile uint32_t can_lb_lost_cnt;   /* 发出去了但没收到(每周期最多 +1) */
extern volatile uint8_t  can_lb_last_rx[8]; /* 最近一次收到的原始数据 */
extern volatile uint32_t can_lb_last_rx_id; /* 最近一次收到的 ID */
extern volatile uint8_t  can_lb_last_seq;   /* 最近一次收到的帧序号(数据第 0 字节) */

/* ---- 总线健康度(每周期从 HAL 刷新，k=0 -> TX 方, k=1 -> RX 方) --------- */
extern volatile uint32_t can_lb_tec[2];     /* 发送错误计数 TEC, >96 危险, >255 BUSOFF */
extern volatile uint32_t can_lb_rec[2];     /* 接收错误计数 REC */
extern volatile uint8_t  can_lb_busoff[2];  /* 1 = 已经 Bus-Off */
extern volatile uint8_t  can_lb_warn[2];    /* 1 = 错误计数已到 96 的警告线 */

void CanLoopbackTest_Init(void);
void CanLoopbackTest_Loop(void);
