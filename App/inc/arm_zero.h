#pragma once

/* 机械臂「设零点」维护路径 —— 一个**独立的全局状态机**，不属于常规控制管线。
 *
 * 为什么单独一个模块，而不是往 Arm_mode(robot_def.h) 里加一个状态：
 *   设零点指令(`DMmotor_Save_Zero()` 的 FF..FE / `ENCOSmotor_Save_Zero()` 的 0x7FF ..03)
 *   和常规控制帧是**互斥**的，设零期间必须一条控制帧都不发。而常规控制帧有两处：
 *     - `DMmotor_Calc_Send()`(MotorTask 1kHz)：stop 模式下 500Hz 发零输出 MIT 帧，
 *       而且**每 300ms 无条件给所有达妙发一次使能帧** —— 它会直接把"全部失能"抵消掉；
 *     - `ENCOSmotor_Calc_Send()`：stop 模式下 500Hz 发停止帧。
 *   这两处都在 MotorTask 里按拍跑，跟 arm_mode 无关；把它们关掉只能在调用它们的地方关。
 *   所以本模块跑在 **MotorTask**、和两个 Calc_Send() 同一个任务，用 Arm_Zero_Busy()
 *   在设零期间把两个 Calc_Send() 一起跳过(见 Core/Src/freertos.c 的 StartMotorTask)。
 *   一条总线被关掉之后**每一条总线**都不发帧（不做"只关某条总线"）。
 *
 * 流程（用户指定）：
 *   得到设零请求 → ①所有参与电机失能 → ②等失能确认 → ③依次发设零指令 →
 *   ④检查是否成功，不成功就反复(重发) → 直到成功或单台/整体超时 →
 *   成功和超时两种情况都通过下面的全局变量给出反馈。
 *
 * 实测两个坑(细节见 arm_zero.c 文件头的记录)：
 *   - 达妙**失能之后不一定还发反馈**：j4 会继续发(errcode 报 0)，j5 收到 0xFD 之后就
 *     彻底不发了。所以"失能确认"是 errcode==0 或"安静够久"二者之一；"检查设零是否
 *     成功"对后者要先用使能帧(0xFC)把它唤醒才读得到位置，读完再关回去。
 *   - 设零期间总线故意安静，会把 20ms 掉线看门狗骗到，进而清掉 motor_inited[JOINT6]，
 *     所以设零期间主动喂狗(keep_monitors_alive)。
 *
 * 参与集：**全部 7 台**，但 j1~j3(ENCOS) 目前只占位 —— CAN3 现在不通、零点也没标定，
 * 由 arm_zero_encos_enable(默认 0)控制：为 0 时它们不参与失能、不发帧，结果记
 * ARM_ZERO_RES_SKIP；翻成 1 后自动变成真的 7 台。
 * 另外 arm_zero_participate_override 能把一轮**缩到指定的几台**(一键夹爪找零就这么用)：
 * 那一轮只失能/设零被点名的电机，其余电机虽然同样收不到帧(见上面"整条总线都不发帧")，
 * 但不会被失能 —— 对"只想设夹爪"这种用法，这就是避免整条臂松掉的唯一办法。
 * 注意**一轮设零的价值全在参与集**：finish() 只按参与集判成败，没点名的记 SKIP。
 *
 * 用法（JLink 直接读写内存，本工程就是这么验的）：
 *   写 arm_zero_request = 1  → 目标跑几百 ms → 读 arm_zero_state / arm_zero_result[]
 *   / arm_zero_retry[] / arm_zero_elapsed_ms / arm_zero_disable_confirmed。
 *   结束后 arm_zero_request 会被自动清 0；再写 1 可以再来一轮。
 *
 * 警告：设零把"当前位姿"定义成零点，会**覆盖电机里的原零点**。每次触发之前必须先把
 * 机械臂摆到基准位姿；摆的姿态若和 gimbal_arm_def.h 里 JOINTn_POS_INIT 标定的姿态不同，
 * 设零后要跟着改那些宏，否则关节角整体偏一个常量。
 */

#include <stdint.h>

#include "robot_def.h" /* ARM_MOTOR_NUM / JOINT1..6 / GRIPPER */

/* 状态机状态。JLink 读的就是这个值(声明成 uint8_t，一个字节)。 */
typedef enum {
    ARM_ZERO_IDLE = 0,     /* 空闲 */
    ARM_ZERO_DISABLE,      /* 瞬态：失能帧已发，正等确认(本模块里没有停留点) */
    ARM_ZERO_WAIT_DISABLE, /* ① 等所有参与电机的失能确认 */
    ARM_ZERO_SEND,         /* ② 给第 s_index 台发设零帧 */
    ARM_ZERO_CHECK,        /* ③ 等新反馈(达妙)/等 ACK(ENCOS)，判成功，不成功则重发 */
    ARM_ZERO_DONE,         /* 全部参与电机都没报错，结束 */
    ARM_ZERO_TIMEOUT,      /* 有台没搞定(或失能没确认)，结束 */
} Arm_zero_state;

/* 每台的结果码，arm_zero_result[] 的元素 */
typedef enum {
    ARM_ZERO_RES_NONE = 0,        /* 没走到 */
    ARM_ZERO_RES_UNCONFIRMED = 1, /* 达妙：帧发了、位置本来就在容差内，无法证明写进去了 */
    ARM_ZERO_RES_OK = 2,          /* 成功(达妙：位置被拉到 0；ENCOS：ACK 报成功) */
    ARM_ZERO_RES_FAIL = 3,        /* ENCOS：ACK 报失败 */
    ARM_ZERO_RES_TIMEOUT = 4,     /* 反复重发到单台超时 */
    ARM_ZERO_RES_SKIP = 5,        /* 占位跳过(当前是 j1~j3) */
} Arm_zero_result;

/* ---- 入口 ---- */
/* 写 1 触发一轮；状态机在**非忙**状态下会消费它(清 0)并开跑。 */
extern volatile uint8_t arm_zero_request;

/* ---- 反馈(JLink 读这几个) ---- */
extern volatile uint8_t arm_zero_state;                      /* Arm_zero_state */
extern volatile uint8_t arm_zero_result[ARM_MOTOR_NUM];      /* Arm_zero_result */
extern volatile uint8_t arm_zero_retry[ARM_MOTOR_NUM];       /* 每台重发了几次 */
extern volatile uint32_t arm_zero_elapsed_ms;                /* 本轮耗时 */
/* 失能确认是否拿到过(达妙反馈 errcode==0)。超时时看它就知道是卡在失能还是卡在设零。 */
extern volatile uint8_t arm_zero_disable_confirmed;
/* 哪些电机在"检查设零结果"时是靠先发使能帧(0xFC)唤醒才读到位位置的，bit = 电机索引。
 * 实测 j5 失能后不再发反馈，会落到这一位；全 0 说明 4 台失能后都还在汇报。 */
extern volatile uint8_t arm_zero_wake_mask;

/* j1~j3 是否真的参与(0=占位，只标记 SKIP)。CAN3 通了、零点标定完之后再翻这个。
 * **在触发那一轮开始时锁定**：运行途中翻它不影响本轮参与集，只影响下一轮。 */
extern uint8_t arm_zero_encos_enable;

/* 参与集覆盖：非 0 = 这一轮**只让这些位参与**(bit = Arm_Motor_Index)，0 = 照旧(整臂)。
 * 一键夹爪找零用它把一轮设零缩到夹爪一台(见 App/arm_auto.c 的 case 3)，用完整的设零
 * 机制(失能确认 / 唤醒读回 / 重发 / 喂狗)但不动 j1~j6。
 * **一次性**：触发那一轮一开始(start())就锁进参与集并把它清回 0，和 arm_zero_request
 * 被消费一个路子 —— 所以不会留下"以后每次设零都只设夹爪"的雷。 */
extern uint8_t arm_zero_participate_override;

/* ---- API ---- */
/* 状态机步进，在 MotorTask 里每拍调一次(1 kHz)。 */
void Arm_Zero_Loop(void);
/* 设零进行中返回 1：此时**不许发任何常规控制帧**(两个 Calc_Send() 都要跳过)。 */
uint8_t Arm_Zero_Busy(void);
