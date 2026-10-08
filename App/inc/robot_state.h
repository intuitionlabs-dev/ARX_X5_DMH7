#pragma once

/* 机器人**总模式**状态机 —— 上位机串口这条链路的落点。
 *
 * ==================== 它和 Arm_mode 是两个层次 ====================
 *
 *   Robot_state(本模块，7 个值)   机器人"现在该干什么"：握手 / 等电机 / 等初始化 /
 *                                 正在初始化 / 正常运行 / 掉线 / 校准零点
 *   Arm_mode(robot_def.h，4 个值) **电机怎么出力**：stop(松劲) / run(位控) /
 *                                 damping(被动力矩) / only_gravity_comp(只顶重力)
 *
 * 总模式是 Arm_mode 的**上游决策者**，不替换它。两者的关系：
 *
 *     上位机串口 ──→ Robot_state ──→(Robot_State_Arm_Mode())──→ gimbal_cmd->mode
 *                       │                                            │
 *                       │                                            ↓
 *                       └── 也直接写 gimbal_cmd->auto_mode     Gimbal.arm_control_data.mode
 *                          (初始化流程的两步一键动作)               │
 *                                                                  ↓
 *                                                        Gimbal_Update() 读 mode
 *
 * 每个总模式对应的 Arm_mode 在 .c 里的 Robot_State_Enter() 里定死（表就那一处）。
 *
 * ==================== 遥控器优先 ====================
 *
 * **遥控器在线时覆盖所有串口指令**（用户指定的行为）：本模块照常收包、照常跑状态机、
 * 照常发 250 Hz 反馈，但它的两个输出都被闸住 ——
 *
 *   - 模式：App/gimbal_cmd.c 的模式判定里，遥控器在线时**根本不调**
 *     Robot_State_Arm_Mode()，改用 rc.sw_a；
 *   - 动作：初始化 / 校准零点在这里就**拒绝进入**（停在待初始化模式），
 *     控制帧也不写入。
 *
 * 于是遥控器在线时没有任何一条串口指令能作用到电机上，等价于"串口不存在"，而且不
 * 需要在状态机里维护"冻结/解冻"的第二套状态。
 *
 * ==================== 三条时间约定 ====================
 *
 *   - 握手通过后**无论什么状态都以 250 Hz 发反馈帧**(0x80)，直到复位；
 *   - 握手通过后 **500 ms 收不到有效帧** → 进通信离线模式、机械臂 damping；
 *     之后**收到任意有效帧**就回"离线等待模式"（**不要求重新握手**）；
 *   - 反馈帧用 HAL_GetTick() 做 4 ms 门控（Robot_State_Loop 自己是 500 Hz）。
 *
 * ==================== 挂载方式 ====================
 *
 *   Robot_State_Init();   // main.c 的 USER CODE 2，**必须在 BSP_UART_Init() 之后**
 *                         // (那个函数会清空 BSP 的回调表)，调度器启动之前
 *   Robot_State_Loop();   // Cmd_Loop() 开头，500 Hz
 *
 * 本模块**只管一路串口**(ROBOT_UART_PORT = BSP_UART_U3)，这也是"共享状态不用临界区"
 * 的前提 —— 收包回调只有这一路会进来。
 *
 * 注意 U3 上原本挂着自机回环测试 App/test/uart_loopback_test.c，两者**抢同一路**的回调。
 * 那个测试现在是关着的(TEST_UART_LOOPBACK_ENABLE = 0)；就算打开也不冲突，因为它的
 * UartLoopbackTest_Init() 在 main.c 里排在本模块**前面**，而注册回调是直接赋值，
 * 本模块会把 U3 的回调**覆盖**掉 —— 只是回环测试从此在 U3 上收不到东西了。 */

#include <stdint.h>

#include "bsp_uart.h"  /* BSP_UART_U3 —— 本模块固定用这一路 */
#include "robot_def.h" /* ARM_JOINT_NUM / Arm_mode */

/* 本模块用哪一路串口。写死：见上面"只管一路串口"。
 *
 * **USART3 = PD8(TX)/PD9(RX)**，板上接的是 **RS485 半双工收发器**：方向由外设硬件
 * 控制(PB14 = USART3_DE，见 Core/Src/usart.c 的 HAL_RS485Ex_Init)，软件不用管 DE。
 * 921600 8N1，收发各挂一条 DMA(DMA2_Stream0/1)。
 *
 * 上位机这侧是 **USB 转 RS485 适配器**，接这一路的 A/B 差分对。
 *
 * 之前是 USART1 = PA9/PA10（普通异步口，端对端接 USB 转 TTL：适配器 TX → PA10、
 * 适配器 RX → PA9、GND 互连）。2026-10-08 对端换成 RS485 适配器，所以挪到 U3。 */
#define ROBOT_UART_PORT BSP_UART_U3

/* 通信超时，ms。握手通过之后，这么久没收到任何有效帧就进通信离线模式。 */
#define ROBOT_COMM_TIMEOUT_MS 500U

/* 反馈帧周期，ms（4 ms = 250 Hz）。Robot_State_Loop() 是 500 Hz，用 HAL_GetTick 门控。
 * 4 ms 正好是那条 500 Hz 栅格(2 ms)的整数倍，所以每一帧都落在栅格上、间距精确。
 * (原来是 5 ms = 200 Hz，和 2 ms 栅格不对齐，实测只有 167 Hz，见 Feedback_Step 的注释。) */
#define ROBOT_FEEDBACK_PERIOD_MS 4U

/* 「预设初始化位置」容差，deg。
 *
 * 收到初始化命令帧(0x02)时，拿当前 6 个关节的反馈角和 ARM_RESET_ANGLE 逐个比，任何一个
 * 偏差超过这个数就**拒绝**这次初始化（保持待初始化模式、不回包）。
 *
 * 20° 是拍的初始值：够宽，能容忍"人手动摆得不太准"；够窄，能挡住"上位机在臂还歪着的
 * 时候就让它动"。TODO: 没在真机上量过，第一次跑初始化之前先确认这个数合不合适
 * (尤其 j2/j3 —— 它们接近限位，误差会放大)。 */
#define ROBOT_INIT_POSE_TOL_DEG 20.0f

/* 发起设零之后，等 MotorTask 认领 arm_zero_request 的最长时间，ms。
 * 正常情况下一拍(1 ms)内就被认领；这个超时是防"MotorTask 没在跑"导致永远卡在校准模式。 */
#define ROBOT_ZERO_START_TIMEOUT_MS 200U

/* 机器人总模式。**只在本模块里有意义**（gimbal_cmd 拿到的只是它推导出来的 Arm_mode）。
 *
 * 迁移图（→ 表示自动迁移，⇒ 表示要收到帧才动）：
 *
 *   上电 ──→ 等待握手 ──0x01 版本码相等──→ 离线等待 ──7 台电机全在线──→ 待初始化
 *              │                                                          │
 *              └─0x01 版本码不等─→ 原地锁死(不解析、不回复，直到复位)      ⇒0x02 且姿态合格
 *                                                                         ↓
 *                                                                      初始化 ──两步一键动作跑完──→ 正常运行
 *                                                                         ⇒0x10 设零        (默认 damping)
 *                                                                         ↓                      │
 *                                                                      校准零点 ⇒跑完回待初始化    │
 *                                                                                                ⇒0x12/0x13/0x14/0x15 改 Arm_mode
 *                                                                                                ⇒0x18 且遥控器离线 → 复位中
 *                                                                                                  复位中 ──一键复位跑完──→ 正常运行(默认 damping)
 *
 *   握手通过之后**任意状态**：500 ms 无有效帧 → 通信离线(damping)
 *   通信离线：收到任意有效帧 → 离线等待（回到上面那条链）
 */
typedef enum {
    robot_state_wait_handshake = 0, /* 等待握手：机械臂 stop，只认版本包 */
    robot_state_offline_wait,       /* 离线等待：stop，等 7 台电机全部上线 */
    robot_state_wait_init,          /* 待初始化：stop，等初始化命令帧 / 校准零点帧 */
    robot_state_initing,            /* 初始化：run，先一键复位、再一键夹爪找零 */
    robot_state_normal,             /* 正常运行：等上位机切 Arm_mode，默认 damping */
    robot_state_comm_lost,          /* 通信离线：damping，收到任意有效帧就回离线等待 */
    robot_state_calibrating,        /* 校准零点：stop，设零状态机跑完就回待初始化 */
    robot_state_resetting,          /* 一键复位：run，只跑 arm_auto_reset，跑完回正常运行(默认阻尼) */
    ROBOT_STATE_NUM,
} Robot_state;

/* 上位机最新一帧控制指令(0x16 / 0x17)。
 *
 * 命令层在 arm_run 且遥控器离线时读它 —— 位置走 joint_src（后面还要过软限位和 TD），
 * 速度在 Arm_Ref_Set() 里当速度前馈（只有 has_vel 才有）。
 *
 * **valid 在每次总模式/状态切换、以及 Arm_mode 切换时被清零**（robot_state.c 的
 * Ctrl_Invalidate）。不清的话 "run → 阻尼 → run" 会把切走之前那个旧目标重新灌进去，
 * 把臂猛拽一下。 */
typedef struct {
    uint8_t valid;   /* 1 = 下面这组值可用（收到过 0x16/0x17 且之后没切过模式/状态） */
    uint8_t has_vel; /* 1 = 收到的是 0x17（带角速度），joint_vel[] 有效 */
    float joint_angle[ARM_JOINT_NUM]; /* deg */
    float joint_vel[ARM_JOINT_NUM];   /* deg/s */
    float gripper_angle;              /* deg，口径同 Arm_control_data.gripper_oc_angle */
} Robot_ctrl_cmd;

/* ---- 初始化 / 调用 ---- */

/* 登记串口回调 + 清状态。**必须在 BSP_UART_Init() 之后**（那个函数会清空回调表）。
 * 放在 main.c 的 USER CODE 2 里（用户约定：libs 下的初始化都放 main.c，不放测试入口）。 */
void Robot_State_Init(void);

/* 每拍调一次（Cmd_Loop() 开头，500 Hz）。做三件事：排空接收环并逐帧分派、跑状态迁移、
 * 250 Hz 发反馈帧。非阻塞，绝不等待。 */
void Robot_State_Loop(void);

/* 当前总模式对应的机械臂出力模式 —— 给 gimbal_cmd.c 的模式判定取。
 * **遥控器在线时调用方不该用它**（那时模式归遥控器）。 */
Arm_mode Robot_State_Arm_Mode(void);

/* 当前总模式。 */
Robot_state Robot_State_Get(void);

/* 最新一帧上位机控制指令（永远非 NULL，是个静态对象；有效与否看 valid）。
 * 命令层的 Run_Mode_Update() 和 Arm_Ref_Set() 取它。 */
const Robot_ctrl_cmd* Robot_State_Ctrl(void);

/* ---- 观测量（JLink 读，不用串口打印）---- */

extern volatile uint32_t robot_state_transition_cnt; /* 状态迁移次数 */
extern volatile Robot_state robot_state_last;        /* 最近一次进入的总模式(和 Get() 冗余，
                                                      * 给 JLink 一个直接看得见的地址) */
extern volatile Arm_mode robot_arm_mode_last;        /* 最近一次给出的 Arm_mode */
extern volatile uint32_t robot_rx_frame_cnt;         /* 解析出的整帧数(含被闸门丢掉的) */
extern volatile uint32_t robot_rx_drop_cnt;          /* 环满丢帧数 */
extern volatile uint32_t robot_rx_over_cnt;          /* payload 超长丢帧数 */
extern volatile uint32_t robot_crc8_err_cnt;         /* crc8 错 */
extern volatile uint32_t robot_crc16_err_cnt;        /* crc16 错 */
extern volatile uint32_t robot_tx_fail_cnt;          /* 发帧失败(BSP 忙 / 编码失败) */
extern volatile uint32_t robot_fb_cnt;               /* 发出去的反馈帧数(期望 250/s) */
extern volatile uint32_t robot_handshake_cnt;        /* 握手成功次数 */
extern volatile uint32_t robot_handshake_reject_cnt; /* 版本码不符被锁的次数 */
extern volatile uint32_t robot_init_reject_cnt;      /* 初始化被拒次数(姿态不合格/模式不对) */
extern volatile uint32_t robot_init_abort_cnt;       /* 初始化被遥控器打断的次数 */
extern volatile uint32_t robot_reset_reject_cnt;     /* 一键复位被拒次数(状态不对/遥控器在线/有电机掉线) */
extern volatile uint32_t robot_reset_abort_cnt;      /* 一键复位被遥控器打断的次数 */
extern volatile uint32_t robot_ctrl_frame_cnt;       /* 接受的串口控制帧数 */
extern volatile uint8_t robot_handshake_ok;          /* 1 = 握手已通过 */
extern volatile uint8_t robot_locked;                /* 1 = 版本码不符，已锁死(要复位才恢复) */
