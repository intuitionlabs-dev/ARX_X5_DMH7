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
 *     上位机串口 ──→ Robot_state ──→(arm->arm_mode)──→ gimbal_cmd->mode
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
 *   - 模式：App/gimbal_cmd.c 的模式判定里，遥控器在线时**根本不读**
 *     arm->arm_mode，改用 rc.sw_a；
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
 *   Robot_State_Init();   // main.c 的 USER CODE 2，**必须在两个 BSP 的 Init 之后**
 *                         // (它们会清空自己的回调登记)，调度器启动之前
 *   Robot_State_Loop();   // Cmd_Loop() 开头，500 Hz
 *
 * 本模块**只管一条链路**：走串口还是走 USB，由下面的 ROBOT_LINK **编译期二选一**
 * (默认 USB)。这也是"共享状态不用临界区"的前提：收包回调只可能从这一条链路进来。
 *
 * 那个前提还有另一半：**只有被选中的那条链路会注册本模块的回调**。没选中的那条
 * 外设照样是活的(比如 USB 那边 MX_USB_DEVICE_Init() 是无条件调的，在 freertos.c 里)，
 * 但没人注册 Robot_State_OnRx()，所以它碰不到任何共享状态。
 *
 * 走串口 (ROBOT_LINK_UART) 时注意 U3 上原本挂着自机回环测试
 * App/test/uart_loopback_test.c，两者**抢同一路**的回调。那个测试现在是关着的
 * (TEST_UART_LOOPBACK_ENABLE = 0)；就算打开也不冲突，因为它的 UartLoopbackTest_Init()
 * 在 main.c 里排在本模块**前面**，而注册回调是直接赋值，本模块会把 U3 的回调**覆盖**掉
 * —— 只是回环测试从此在 U3 上收不到东西了。 */

#include <stdint.h>

#include "robot_def.h" /* ARM_JOINT_NUM / Arm_mode / Arm(前向声明) */
#include "frame.h"     /* FramePkt* —— 收包环的 payload union 要按这些结构体读 */

/* ==================== 走哪条链路（编译期二选一）====================
 *
 * 改下面 ROBOT_LINK 那一行就切换。两条链路的 BSP 接口是**同形**的
 * (对照表在 libs/usb/bsp_usb.h 开头)，所以 App/robot_state.c 的正文一句都不用动 ——
 * 差异全收在 .c 顶部那一小段别名宏里，见那里的注释。
 *
 * 为什么是编译期二选一，不是运行期切换、也不是双路并行：本模块的接收侧
 * (s_parser + 收包环)是单生产者单消费者的，靠"只有一个中断上下文会碰它"省掉了所有
 * 临界区。两条链路同时活着就有两个中断源，那条前提立刻不成立。 */
#define ROBOT_LINK_UART 0
#define ROBOT_LINK_USB  1

/* ← 改这一行切换链路。默认 USB：实测 250.4 Hz 反馈、crc 错 0，比走 RS485 的
 * ~244 Hz 干净(USB 是包传输，没有半双工那种收发碰撞)，而且不用额外适配器。
 * 换回串口就把它写成 ROBOT_LINK_UART —— 前提是那路 USART3 上接着
 * USB 转 RS485 适配器(见下面那段)；否则板子 USB 口插上不会有任何反应。 */
#define ROBOT_LINK ROBOT_LINK_USB

#if ROBOT_LINK == ROBOT_LINK_USB
#include "bsp_usb.h" /* BSP_USB_PORT —— 板载 USB CDC(OTG_HS, PA11/PA12) */
#else
#include "bsp_uart.h" /* BSP_UART_U3 —— 板载 RS485 半双工 */
#endif

/* ---- 串口那条 (ROBOT_LINK_UART，现在不是默认) ----
 *
 * **USART3 = PD8(TX)/PD9(RX)**，板上接的是 **RS485 半双工收发器**：方向由外设硬件
 * 控制(PB14 = USART3_DE，见 Core/Src/usart.c 的 HAL_RS485Ex_Init)，软件不用管 DE。
 * 921600 8N1，收发各挂一条 DMA(DMA2_Stream0/1)。
 *
 * 上位机这侧是 **USB 转 RS485 适配器**，接这一路的 A/B 差分对。
 *
 * ---- USB 那条 (ROBOT_LINK_USB，默认) ----
 *
 * 板子自己的 USB CDC 口，主机侧认出来是 VID:PID = **0483:5740**
 * ("STM32 Virtual ComPort")，挂在 /dev/ttyACM* 上。
 *
 * 两条链路**只走一条**，不用同时插。USB 这条上"波特率"是没有意义的(CDC 是包传输)，
 * 上位机设成多少都一样。 */


/* 通信超时，ms。握手通过之后，这么久没收到任何有效帧就进通信离线模式。 */
#define ROBOT_COMM_TIMEOUT_MS 500U

/* 反馈帧周期，ms（4 ms = 250 Hz）。Robot_State_Loop() 是 500 Hz，用 HAL_GetTick 门控。
 * 4 ms 正好是那条 500 Hz 栅格(2 ms)的整数倍，所以每一帧都落在栅格上、间距精确。 */
#define ROBOT_FEEDBACK_PERIOD_MS 4U

/* 没握手之前那帧 0x01 的周期，ms（100 ms = 10 Hz）。
 *
 * 未握手期间板子**主动**报版本码：上位机不用先问就知道板子在不在、是哪个版本 —— 版本码
 * 不符时当场就能看出是自己那边对不上，而不是对着静默猜。握手一过就停（那时 250 Hz 反馈
 * 帧已经在发了）。10 Hz 取的是"够快"和"占不了多少带宽"（12 B × 10 Hz）之间的值。 */
#define ROBOT_HANDSHAKE_PERIOD_MS 100U

/* 「预设初始化位置」容差，deg。
 *
 * 收到初始化命令帧(0x02)时，拿当前 6 个关节的反馈角和 ARM_RESET_ANGLE 逐个比，任何一个
 * 偏差超过这个数就**拒绝**这次初始化（保持待初始化模式、不回包）。
 *
 * 20° 是拍的初始值：够宽，能容忍"人手动摆得不太准"；够窄，能挡住"上位机在臂还歪着的
 * 时候就让它动"。TODO: 没在真机上量过，第一次跑初始化之前先确认这个数合不合适
 * (尤其 j2/j3 —— 它们接近限位，误差会放大)。 */
#define ROBOT_INIT_POSE_TOL_DEG 20.0f

/* 发起设零之后，等 MotorTask 认领 gimbal->zero.request 的最长时间，ms。
 * 正常情况下一拍(1 ms)内就被认领；这个超时是防"MotorTask 没在跑"导致永远卡在校准模式。 */
#define ROBOT_ZERO_START_TIMEOUT_MS 200U

/* 机器人总模式。**只在本模块里有意义**（gimbal_cmd 拿到的只是它推导出来的 Arm_mode）。
 *
 * 迁移图（→ 表示自动迁移，⇒ 表示要收到帧才动）：
 *
 *   上电 ──→ 等待握手 ──0x01 版本码相等──→ 离线等待 ──7 台电机全在线──→ 待初始化
 *              │                                                          │
 *              └─0x01 版本码不等─→ 丢掉这帧(不回包、状态不动，仍等待握手)  ⇒0x02 且姿态合格
 *                                                                         ↓
 *                                                                      初始化 ──两步一键动作跑完──→ 正常运行
 *                                                                         ⇒0x10 设零        (默认 damping)
 *                                                                         ↓                      │
 *                                                                      校准零点 ⇒跑完回待初始化    │
 *                                                                                                ⇒0x12/0x13/0x14/0x15 改 Arm_mode
 *                                                                                                ⇒0x18 且遥控器离线 → 复位中
 *                                                                                                  复位中 ──一键复位跑完──→ 正常运行(默认 damping)
 *                                                                                                ⇒0x02 且姿态合格、档位在 stop/damping → 初始化(★见下)
 *
 *   握手通过之后**任意状态**：500 ms 无有效帧 → 通信离线(damping)
 *   通信离线：收到任意有效帧 → 离线等待（回到上面那条链）
 *   没握手期间：每 100 ms 主动发一帧 0x01（Handshake_Step）—— "握手前一个字节都不发"
 *   的**唯一例外**，0x01 本来就是建立握手的那个包。
 *
 *   ★ 0x02 有**两个入口**（见 HandleArmInit）：
 *     ① 待初始化；② 正常运行 + Arm_mode 在 stop/damping。
 *     run/gravity 档故意不接：初始化期间状态机会把 Arm_mode 掰到 run（一键
 *     动作靠 auto_mode 驱动，stop 档下 Stop_Mode_Update 每拍清 auto_mode、复位撑不过
 *     一拍），能接手的必须是"没有位置环在跑"的档。
 *     **中止时退回发起时那个总模式**（s_init_entry），不是一律回待初始化 —— 否则从
 *     正常运行发起的那种被遥控器打断后会被降级进只受理 0x02/0x10 的死角。正常跑完
 *     两个入口都回正常运行。
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

/* ==================== 收包环的一个槽 ====================
 *
 * 环按**臂**分：中断解出一整帧之后看帧头里的 arm_id，往那一台臂的环里塞
 * (g_arm[arm_id].rx_slot[])。所以这两个类型定义在头里 —— `Arm` 要有它们当成员。
 * 环的读写纪律(单生产者单消费者、__DMB)在 robot_state.c 里，见那一段注释。 */

/* 环槽里 payload 的长度。**下行最长的包是 0x17 的 52 字节**，这里给 64 留了点余地。
 *
 * 为什么不是 255（协议上限）：8 个槽 × 255 就是 2 KB 的常驻 RAM，而超过
 * ROBOT_RX_DATA_LEN 的包是**直接丢掉并计数**的 —— 将来真要加一个长包，先看
 * robot_rx_over_cnt 涨不涨，再决定是改大这里还是换存储。 */
#define ROBOT_RX_DATA_LEN 64U
#define ROBOT_RX_SLOT_NUM 8U

/* 一个环槽。payload 用 union 而不是裸数组，为的是让任务侧能**按结构体读**：
 * 中断侧往 raw 里 memcpy，任务侧读 arm_ctrl_pos 这样的成员 —— 这是 C 标准 6.5.2.3
 * 认可的 type punning（和 libs/frame 的 FrameBuf 一个道理）。
 * 换成 `*(FramePktArmCtrlPos*)slot->payload.raw` 这种指针强转就会踩 strict aliasing。 */
typedef union {
    uint8_t raw[ROBOT_RX_DATA_LEN];
    FramePktVersion version;
    FramePktMotorSetZeroAck motor_set_zero_ack;
    FramePktArmCtrlPos arm_ctrl_pos;
    FramePktArmCtrlPosVel arm_ctrl_pos_vel;
} Robot_rx_payload;

typedef struct {
    uint8_t type_id;
    uint8_t len;
    Robot_rx_payload payload;
} Robot_rx_slot;

/* ---- 初始化 / 调用 ---- */

/* 登记链路回调 + 清状态。**必须在两个 BSP 的 Init 之后**（它们会清空自己的回调登记）。
 * 只登记 ROBOT_LINK 选中的那一条。
 * 放在 main.c 的 USER CODE 2 里（用户约定：libs 下的初始化都放 main.c，不放测试入口）。 */
void Robot_State_Init(void);

/* 每拍调一次（Cmd_Loop() 开头，500 Hz）。**两台臂一起**：排空共享接收环并按帧头里的
 * arm_id 分派、逐台跑状态迁移、逐台 250 Hz 发反馈帧、最后统一排空发送队列。
 * 非阻塞，绝不等待。
 *
 * 为什么不是"逐台调一次"：接收环和发送队列都是**共享链路**的资源（一条 USB/串口），
 * 每拍只能排空一次；而各台的状态机是独立推进的。**只给驱动建起来了的臂跑**(gimbal
 * 非 NULL)：没有驱动就没有反馈帧可发、也没有"电机在线"可判，让它白跑一个状态机只会
 * 在链路上多出一串没人要的帧。 */
void Robot_State_Loop(void);

/* 这台臂最新一帧上位机控制指令（永远非 NULL，在 Arm 里；有效与否看 valid）。
 * 命令层的 Run_Mode_Update() 和 Arm_Ref_Set() 取它。
 * **每台臂只认自己那一份** —— 帧是按 arm_id 分流的。 */
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm);

/* ---- 观测量（JLink 读，不用串口打印）----
 *
 * **链路级的**只有下面这几个：它们描述的是那一条 USB/串口本身(解析、收发)，和哪台臂
 * 无关。**按台的**观测量(状态迁移、握手、反馈帧率…)在 `Arm`(App/inc/arm.h)，
 * JLink 读 `g_arm[0].fb_cnt`。 */

extern volatile uint32_t robot_rx_frame_cnt; /* 解析出的整帧数(含被闸门丢掉的) */
extern volatile uint32_t robot_rx_drop_cnt;  /* 环满丢帧数 */
extern volatile uint32_t robot_rx_over_cnt;  /* payload 超长丢帧数 */
extern volatile uint32_t robot_crc8_err_cnt; /* crc8 错 */
extern volatile uint32_t robot_crc16_err_cnt; /* crc16 错 */
/* **不是丢帧计数**：发送走的是队列(见 robot_state.c)，BSP 报忙只是这一拍先不发、
 * 下一拍重试，帧还在队里。这里涨了说明"链路这段时间发不动"(没枚举出来 / 主机没开串口 /
 * 主机读得太慢)，配合下面的 rx 计数看是哪一头的问题。只有**队列满**和**编码失败**
 * 这两种情况才是真丢了帧。 */
extern volatile uint32_t robot_tx_fail_cnt;
/* arm_id 越界的帧数(帧头里那个字节 >= ARM_NUM)。**只可能来自上位机写错或抓到别的板子的
 * 包**，正常永远 0；非 0 说明有帧被静默丢掉了，先查这里。 */
extern volatile uint32_t robot_rx_bad_arm_cnt;
