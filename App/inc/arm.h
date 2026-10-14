#pragma once

/* 一块板子上的**一台**机械臂 —— 把整条控制链路的状态收在一个对象里。
 *
 * 为什么要有这个类型：各层之间靠**显式持有**互相找对方，一台臂一个 —— 谁需要谁就顺着
 * `arm->gimbal` / `arm->gimbal_cmd` 拿到自己那一份。
 *
 * **静态分配、开机即存在**：收包回调可能在任务建 Gimbal 之前就打进来，收包环必须已经在
 * 那儿(否则 ISR 里要一路判 NULL)。用结构体数组而不是指针数组，就没有"指向空"这个状态。
 * JLink 里也直接读得到(`g_arm[1].gimbal->joint1` 这样)。
 *
 * 加字段一律**追加在末尾**，别插到中间 —— 调试器里存着偏移。
 */

#include <stdint.h>

#include "robot_def.h" /* ARM_NUM / Arm_Index / Arm_mode */

#include "gimbal_arm.h"  /* Gimbal —— 驱动层实例(一台臂 7 台电机 + PID + 反馈/控制数据) */
#include "gimbal_cmd.h"  /* Gimbal_cmd —— 命令层实例(输入 → 控制源 → 软限位 → TD) */
#include "robot_state.h" /* 总模式状态机的类型与收包环(Robot_state / Robot_ctrl_cmd / …) */

/* 臂号 == CAN 索引这条约定要成立，两台臂就各占一条总线，所以 ARM_NUM 不能超过总线条数。
 * (DEVICE_CAN_CNT 来路：DM_motor.h → bsp_can.h → can_config.h。)
 * 超了的话 `g_arm[i].id = i` 会是一个 BSP_CAN_Send 认不出来的索引。 */
_Static_assert(ARM_NUM <= DEVICE_CAN_CNT, "臂数不能超过 CAN 总线数(臂号 == 总线索引)");

typedef struct Arm {
    /* 臂号。**== CAN 索引 == 帧里的 arm_id**，见 robot_def.h 的 Arm_Index。
     * 两台臂完全同构，唯一的区别就是这个数。 */
    uint8_t id;

    /* 驱动层 / 命令层实例。Gimbal_Create() / Gimbal_Cmd_Create() 建，
     * 建失败(堆不够)时是 NULL —— 用之前判空。 */
    Gimbal* gimbal;
    Gimbal_cmd* gimbal_cmd;

    /* 出力模式(Arm_mode)，由总模式状态机推导：App/robot_state.c 每拍写，命令层读它决定
     * Gimbal.arm_control_data.mode(遥控器在线时命令层改用自己的判据，见 gimbal_cmd.c)。
     * 放这儿是为了让"命令层读哪台臂的模式"这件事有个显式的落点，而不是一个全局函数。 */
    Arm_mode arm_mode;

    /* ==================== 总模式状态机 ====================
     *
     * 必须按台分开：**每臂各自握手、各自 500 ms 看门狗、各自状态迁移**，共用一份的话 A 台
     * 掉线会把 B 台一起判掉线。
     *
     * 收包环(rx_slot / rx_head / rx_tail)之所以也在结构体里、而不是当"共享链路的资源"：
     * 中断解出一整帧后是**按帧里的 arm_id 分流**的，帧进哪台臂的环由 arm_id 决定 ——
     * 所以环是 per 台的。**共享的只有解析器**，因为字节流只有一路(一条 USB/串口)。
     *
     * 中断里会碰的成员只有 rx_*(单字节或"中断写、任务读"，读写天然原子)；其余成员
     * 全在任务上下文里碰，见 robot_state.c 里收包环那段注释。 */
    Robot_state state;      /* 当前总模式 */
    Robot_state state_last; /* 观测量：上一拍的总模式 */
    Robot_state init_entry; /* 0x02 是从哪个总模式发起的(见 robot_state.c) */
    Arm_mode arm_mode_last; /* 观测量：上一拍的出力模式 */
    Robot_ctrl_cmd ctrl;    /* 上位机最新控制帧 */
    uint8_t handshake_ok;   /* 握手通过 */
    uint32_t last_rx_ms;    /* 最后一次收到有效帧的时刻 */
    uint32_t fb_last_ms;    /* 上一次发反馈帧的时刻（4 ms 门控 = 250 Hz） */
    uint8_t init_step;      /* 初始化流程的子步骤 */
    uint8_t reset_step;     /* 一键复位的子步骤 */
    uint8_t zero_seen_busy; /* 设零请求"已经见过忙"的锁存(见 robot_state.c) */
    uint32_t zero_start_ms; /* 写设零请求的时刻，用来判"迟迟没人认领" */

    /* 本台臂的收包环：中断写 head、任务写 tail。 */
    Robot_rx_slot rx_slot[ROBOT_RX_SLOT_NUM];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;

    /* ---- 观测量（JLink 读；按台读 g_arm[N].xxx）----
     * 链路级的观测量(解析错误、发送失败的计数)不在这里 —— 那是**共享链路**的，见
     * robot_state.c 里“链路级观测量”那一段。 */
    volatile uint32_t transition_cnt;       /* 状态迁移次数 */
    volatile uint32_t fb_cnt;               /* 发出去的反馈帧数(250/s) */
    volatile uint32_t handshake_cnt;        /* 握手成功次数 */
    volatile uint32_t handshake_reject_cnt; /* 因版本码不符被拒 */
    volatile uint32_t init_reject_cnt;      /* 0x02 被拒次数 */
    volatile uint32_t init_abort_cnt;       /* 0x02 中途被打断 */
    volatile uint32_t reset_reject_cnt;     /* 0x10 被拒次数 */
    volatile uint32_t reset_abort_cnt;      /* 0x10 中途被打断 */
    volatile uint32_t ctrl_frame_cnt;       /* 收到的控制帧数 */
    volatile uint8_t handshake_ok_obs;      /* = handshake_ok，给调试器看 */

    /* 没握手之前那帧 0x01 的时刻（10 Hz 门控，见 robot_state.c 的 Handshake_Step）。
     * **故意写在最后**：加在中间会把后面所有成员的偏置推走，而 JLink 那套观察量是按偏置
     * 读的。放这儿只吃掉末尾的填充，原有成员一个都不动。 */
    uint32_t hs_last_ms;
} Arm;

/* 两台臂。索引就是臂号(Arm_Index)，**越界索引会直接写到别的变量上**，用之前保证 < ARM_NUM */
extern Arm g_arm[ARM_NUM];
