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
/* 掩码里有没有"本拍的组"（本拍的组 = BSP_CAN_TxGroupLatch() 锁存下来的相位）。
 * 驱动拿它把"算控制量"和"发帧"卡在同一个节拍上：算之前在循环体外面问一次，
 * 不匹配就整拍跳过 —— 否则 PID 会按任务周期(1 kHz)跑而不是按发送周期(500 Hz)，
 * 而 libs/controller 的 PID 没有 dt，增益是按调用次数算的，KI 会凭空翻倍。
 * 纯读一个变量，无副作用，随便调。 */
uint8_t BSP_CAN_TxGroupMatch(uint8_t tx_group);

/* **每拍开跑前调一次**，把"本拍属于哪个组"锁存下来(相位 = osKernelGetTickCount() % CAN_TX_GROUP_NUM)。
 *
 * 为什么要锁存而不是每次现读时钟：一拍的发送是**轮转**做的(见 freertos.c 的 StartMotorTask)，
 * 一个来回要等 200us 最小帧间隔，4 轮就是 600~800us —— 循环跑到后半截时**毫秒边沿
 * 已经跨过去了**，现读时钟就会让同一拍里的电机用两个不同相位：排在轮转后面的那台
 * 被"下一个组"的门挡掉，那一拍整拍不发。锁存之后同一拍内相位恒定。
 * (这条是原理上的必要性 —— **不是**在解释"某个 ENCOS 掩码 0x55 却只有 250 Hz 反馈"那个现象：
 * 那个已查明是电机侧应答率上不去 ~250 帧/s(问 500 答 250)，与门控相位无关。)
 *
 * 没调过这个函数时相位初值是 0(上电第一拍就是相位 0)；
 * **非 MotorTask 的发送方(测试帧等)都用 CAN_TX_GROUP_ALL，不受相位影响**。 */
void BSP_CAN_TxGroupLatch(void);

/* 「距上一帧至少 N 微秒」的忙等(BSP_CAN_TX_MIN_GAP_US，见 bsp_can.c)。
 *
 * **不再是 BSP_CAN_Send() 内部自己等**：那样等于"每帧都等一次"，两台臂各走一路 CAN
 * 之后会让两条总线去抢同一个计时器，把每路的帧率砍半。现在的用法是**调用方
 * (MotorTask)按"轮"等**：一轮里两条总线各发一帧，一轮结束才等这一次 —— 于是
 * 同一个 200us 的节拍被两条总线共用，各自不额外付等待。
 *
 * 副作用(有意接受)：那些"必须立刻发出去"的命令帧(达妙的使能/失能/设零)不再被节流，
 * 它们本来就在"整条总线关门"期间发。
 *
 * BSP_CAN_TX_MIN_GAP_US 配 0 时本函数是空实现，调用方不用写 #if。 */
void BSP_CAN_TxGapWait(void);
void BSP_CAN_RegisterRxCallback(uint8_t can_id, can_rx_callback func);
void BSP_ADD_Filters(uint8_t can_id);
