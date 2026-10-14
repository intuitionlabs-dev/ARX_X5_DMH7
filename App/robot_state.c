#include "robot_state.h"

#include <string.h> /* memcpy / memset */

#include "main.h" /* HAL_GetTick() */

#include "arm.h"        /* g_arm[] / Arm —— 出力模式存在臂上 */
#include "arm_auto.h"   /* ARM_RESET_ANGLE / arm_auto_reset / arm_auto_gripper_zero */
#include "arm_zero.h"   /* gimbal->zero.request / Arm_Zero_Busy() / 状态与结果 */
#include "gimbal_arm.h" /* gimbal —— 反馈数据（关节角、掉线标志、温度） */
#include "gimbal_cmd.h" /* gimbal_cmd —— 初始化流程要写它的 auto_mode */
#include "rc.h"         /* rc / is_Offline() —— 遥控器优先的判据 */
#include "frame.h" /* FrameParser / FrameEncode() */

/* ==================== 链路选择（本文件唯一认识传输的地方）====================
 *
 * ROBOT_LINK 在 App/inc/robot_state.h 里定死（编译期二选一）。两条链路的 BSP 是**同形**的
 * (对照表在 libs/usb/bsp_usb.h 开头)，所以整个文件里只有这三个名字需要区分：
 *
 *     ROBOT_LINK_PORT         端口编号（USB 那边恒为 BSP_USB_PORT）
 *     Robot_Link_RegRx(...)   登记收包回调
 *     Robot_Link_Send(...)    发一段字节
 *
 * 下面正文里再没有第二个 #if —— 判 id、注册回调、发帧，三处写法完全一致。这正是
 * "让 bsp_usb 和 bsp_uart 同形"换来的东西；不然上层就得写两套代码，那 abstraction 白做了。 */
#if ROBOT_LINK == ROBOT_LINK_USB
#define ROBOT_LINK_PORT       BSP_USB_PORT
#define Robot_Link_RegRx(...) BSP_USB_RegisterRxCallback(__VA_ARGS__)
#define Robot_Link_Send(...)  BSP_USB_Send(__VA_ARGS__)
#else
#define ROBOT_LINK_PORT       BSP_UART_U3
#define Robot_Link_RegRx(...) BSP_UART_RegisterRxCallback(__VA_ARGS__)
#define Robot_Link_Send(...)  BSP_UART_Send(__VA_ARGS__)
#endif

/* 两个 BSP 的成功码都是 0，所以这个不用跟着 #if 分。 */
#define ROBOT_LINK_OK 0

/* ==================== 这一层和上下左右的关系 ====================
 *
 * 下面这几段是**并发**的，看代码时先记住谁跟谁不在一个上下文里：
 *
 *   Robot_State_OnRx()    中断上下文（BSP 的收包回调）。只做"喂解析器 + 拷贝进环"。
 *   Robot_State_Loop()    任务上下文（CmdTask，500 Hz）。排空环、跑状态机、发反馈。
 *   状态机自己            **和命令层同一个任务**（Cmd_Loop 里先调本模块再调
 *                          Gimbal_Cmd_Update），所以本模块写命令层对象的 auto_mode
 *                          和 Auto_Mode_Update() 读它之间**没有并发**，不需要保护。
 *
 * 和 libs 的分工：本模块**不做**切帧/校验（那是 libs/frame 的活），也**不碰**
 * 收发的缓冲（那是 libs/uart 或 libs/usb 的活）。BSP 只把字节推过来，本模块自己喂解析器。
 *
 * 一个"帧"要过两道闸门才起作用，顺序不能乱：
 *   ① !arm->handshake_ok（没握手前只认版本包）
 *   ② 各包自己的模式检查（比如 0x02 只在待初始化 / 正常运行-stop|damping 生效）
 *
 * 版本码不符只丢掉那一帧：状态一动不动，上位机把版本串改对后再发一帧就能握手，
 * 不用复位板子。
 *
 * 出帧侧只有一个例外：握手帧 0x01 在**没握手期间按 10 Hz 主动发**（Handshake_Step），
 * 其余所有包都只走 SendFrame()，握手通过之前一个字节都不发。
 */

/* ==================== 共享链路的状态 ====================
 *
 * 一条链路(USB 或 USART3，编译期二选一)服务**两台臂**：它们的帧在同一条字节流里交错。
 * 所以解析器只有一份；解出整帧之后按帧头里的 arm_id 分流到各台臂的收包环
 * (g_arm[arm_id].rx_slot[]，环本身在 App/inc/arm.h 的 Arm 里) —— 也就是说环是 per 台的，
 * 只有解析器是共享的。
 *
 * 收包环的读写纪律(每个环各自成立)：**单生产者(中断)单消费者(任务)**。head 只被中断写、
 * tail 只被任务写，两个都是单字节，读写天然原子；但 payload 的写可能被编译器排到 index
 * 写之后(-O2)，Cortex-M7 还有 store buffer，所以两边都要 __DMB()。丢帧是偶发的、
 * 难复现的，就在这儿。 */

/* 解析器。约 270 字节，文件级静态（别放栈上）。只在中断里被碰。 */
static FrameParser s_parser;

/* ==================== 发送队列 ====================
 *
 * 两台臂都在同一拍发 250 Hz 反馈，而两个 BSP 的发送都是**"忙就返回失败"**（USB 那边
 * CDC_Transmit_HS 要等传输完成回调清 TxState）。背靠背连发两帧，第二帧几乎必然被丢掉。
 * 所以：Feedback_Step() 只把编好的帧**入队**，Robot_State_Loop() 末尾统一
 * Link_DrainTx()(能发几帧发几帧，发不动就留到下一拍 2 ms 后)。
 *
 * 生产者(Feedback_Step，任务上下文)和消费者(Link_DrainTx，同一个任务)都在任务里，
 * **不需要临界区**。
 *
 * 缓冲为什么能立刻重用：走 USB 时 BSP 是零拷贝的(把这块内存直接交给 USB 栈)，理论上
 * 要等传输完成才能改写。这里靠的是**间隔**而不是握手：一帧 71 字节的 HS 批量传输是
 * 微秒级的，而同一个槽要绕 8 个位置、按 250 Hz 算至少 32 ms 才会被重新写到 —— 单缓冲
 * 只隔 4 ms 都够，这里更宽裕。真出现"发出去了但上位机收不到"的怪事，先看这里。 */
#define ROBOT_TX_Q_NUM 8U

typedef struct {
    /* len 是 uint16_t 而不是 uint8_t：帧长上限 FRAME_MAX_LEN 是 263（帧头 8 + datalen 255），
     * 一个字节装不下。实际发出去的帧最长 71 字节，但这儿不能按"现在最长"定类型。 */
    uint16_t len;
    uint8_t buf[FRAME_MAX_LEN];
} Robot_tx_slot;

/* 索引是 uint8_t，槽位数超过 255 就绕回去了。写死在编译期。 */
_Static_assert(ROBOT_TX_Q_NUM <= 255U, "TX 队列索引是 uint8_t，槽位不能超过 255");

static Robot_tx_slot s_tx_q[ROBOT_TX_Q_NUM];
static volatile uint8_t s_tx_head; /* 生产者(任务)写 */
static volatile uint8_t s_tx_tail; /* 消费者(任务)写 */

/* ---- 链路级观测量 ---- */
volatile uint32_t robot_rx_frame_cnt;   /* 解析出的整帧数(含被闸门丢掉的) */
volatile uint32_t robot_rx_drop_cnt;    /* 环满丢帧数 */
volatile uint32_t robot_rx_over_cnt;    /* payload 超长丢帧数 */
volatile uint32_t robot_crc8_err_cnt;   /* crc8 错 */
volatile uint32_t robot_crc16_err_cnt;  /* crc16 错 */
volatile uint32_t robot_tx_fail_cnt;    /* 发帧失败(BSP 忙 / 编码失败) */
volatile uint32_t robot_rx_bad_arm_cnt; /* arm_id >= ARM_NUM 的帧数(正常恒 0) */
volatile uint32_t robot_gated_cnt;      /* 被握手闸门挡下的帧数 */
volatile uint32_t robot_tid_cnt[FRAME_TID_SLOT_NUM]; /* 按 type_id 分类，下标见 Tid_Index */

/* 状态帧 rx_by_tid[] 的分类顺序（唯一出处是 type_id.h 的 FRAME_TID_DOWN_LIST）。 */
static const uint8_t s_tid_order[FRAME_TID_DOWN_NUM] = { FRAME_TID_DOWN_LIST };

_Static_assert(sizeof(s_tid_order) == FRAME_TID_DOWN_NUM, "分类表长度对不上");
_Static_assert(FRAME_TID_SLOT_NUM == FRAME_TID_DOWN_NUM + 1U, "槽位数该是分类数 + 1(其他)");

/* 把一个 type_id 映射到机器人 TID 统计数组的下标；表外的码一律进最后一格「其他」。
 * 表只有 11 项，线性查比建表更省事，也不占内存。 */
static uint8_t Tid_Index(uint8_t tid) {
    uint8_t i;

    for (i = 0U; i < (uint8_t)FRAME_TID_DOWN_NUM; i++) {
        if (s_tid_order[i] == tid) return i;
    }
    return (uint8_t)FRAME_TID_DOWN_NUM;
}

/* 本模块其余状态**全在 Arm 里**(App/inc/arm.h)：总模式、握手、控制帧、收包环、观测量…
 * 必须按台分开(每臂各自握手、各自 500 ms 看门狗、各自状态迁移)。所以本文件里每一处
 * 状态访问都长成 `arm->xxx`，**每个函数都得有那个 arm**。 */

/* ==================== 协议常量要和 App 对上 ====================
 *
 * libs/frame 那边线上有几个关节/电机是**协议定死的**（FRAME_PKT_*），不是
 * 由 App 的 ARM_JOINT_NUM/ARM_MOTOR_NUM 决定的。两个数现在正好相等，接上层时在这儿
 * 对一下 —— 哪天 App 加了第 7 个关节而协议没改，这里会红，比运行时静默错位好。 */
_Static_assert(FRAME_PKT_JOINT_NUM == ARM_JOINT_NUM, "协议的关节数和 App 对不上");
_Static_assert(FRAME_PKT_MOTOR_NUM == ARM_MOTOR_NUM, "协议的电机数和 App 对不上");
/* 反馈帧的结构体必须放得下 7 个 uint8_t 温度 + 6+6 个 float + 4 个头，正好 63 */
_Static_assert(sizeof(FramePktArmFeedback) == 63U, "反馈帧不是 63 字节");
/* 环槽要放得下最长的下行包，否则那个包会被静默丢掉（只在 robot_rx_over_cnt 上看得见） */
_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(FramePktArmCtrlPosVel), "环槽放不下 0x17 包");
_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(FramePktMotorSetZeroAck), "环槽放不下 0x11 包");

/* ==================== 发送 ==================== */

/* 入队一帧，**不查任何闸门**。只给下面两个 Send* 用：上层别直接调它。 */
static void TxEnqueue(Arm* arm, uint8_t type_id, const uint8_t* data, uint8_t len) {
    const uint8_t next = (uint8_t)((s_tx_head + 1U) % ROBOT_TX_Q_NUM);
    if (next == s_tx_tail) {
        /* 队列满：多半是上位机那边根本没在收（USB 还没枚举出来、或者口没打开），BSP 一直
         * 报忙。**丢这一帧 + 计数**，不排队等 —— 下一帧很快就到，积压起来只会越来越晚。 */
        robot_tx_fail_cnt++;
        return;
    }

    /* 帧头里的 arm_id 就是这台臂的 `id`（== CAN 索引）。**每帧都带**，上行下行都带。 */
    const int32_t n = FrameEncode(s_tx_q[s_tx_head].buf, (uint32_t)FRAME_MAX_LEN, type_id,
        arm->id, data, len);
    if (n <= 0) {
        robot_tx_fail_cnt++;
        return;
    }
    s_tx_q[s_tx_head].len = (uint16_t)n;
    s_tx_head = next;
}

static void SendFrame(Arm* arm, uint8_t type_id, const uint8_t* data, uint8_t len) {
    /* 握手之前**不发**（含反馈帧）。这是入队前的那道闸门 —— 唯一的例外是握手帧 0x01
     * 自己（走 TxEnqueue，见 SendVersionRaw），闸门挡住它谁也握不了手。 */
    if (!arm->handshake_ok) return;

    TxEnqueue(arm, type_id, data, len);
}

/* 排空发送队列：**能发几帧发几帧**，碰到 BSP 报忙就当场打住，剩下的留到下一拍(2 ms 后)
 * 再来。不重试、不忙等 —— 见队列定义上面那段。
 *
 * 两台臂的帧在这里**按入队先后**排成一队发出去，所以不需要"谁先谁后"的仲裁：两台都是
 * 250 Hz 各入各的队，先到先发。 */
static void Link_DrainTx(void) {
    while (s_tx_tail != s_tx_head) {
        const Robot_tx_slot* const slot = &s_tx_q[s_tx_tail];
        if (Robot_Link_Send(ROBOT_LINK_PORT, slot->buf, (uint16_t)slot->len) != ROBOT_LINK_OK) {
            robot_tx_fail_cnt++;
            return; /* 忙：这一帧留着，下一拍再发 */
        }
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % ROBOT_TX_Q_NUM);
    }
}

/* 版本包。**唯一走 TxEnqueue() 的包**：它要在 handshake_ok==0 的时候发（Handshake_Step
 * 那 10 Hz），被 SendFrame() 的闸门挡住就没意义了。 */
static void SendVersionRaw(Arm* arm) {
    FramePktVersion pkt;
    pkt.code = FrameVersionCode();
    TxEnqueue(arm, FRAME_TYPE_ID_VERSION, (const uint8_t*)&pkt, (uint8_t)sizeof(pkt));
}

/* 回过手机的那一帧：握手过了才发。 */
static void SendVersion(Arm* arm) {
    if (!arm->handshake_ok) return;
    SendVersionRaw(arm);
}

/* ==================== 接收 ==================== */

/* 中断上下文：只做"喂解析器 + 拷进环"，绝不阻塞、绝不打印、绝不调 FreeRTOS API。
 * data 指向 BSP 内部的接收缓冲（uart 那边是 DMA 收的，usb 那边是 USB 栈的），返回后随时
 * 会被下一次接收覆盖 —— 这里当场消费掉，不存指针。
 *
 * 函数体对两条链路**完全一样**，靠的是 bsp_usb 的收包回调类型和 bsp_uart 逐字一致。 */
static void Robot_State_OnRx(uint8_t id, const uint8_t* data, uint16_t len) {
    uint16_t i;

    /* 只挂了一条链路，正常恒成立；留着是为了万一以后有人给别的口也挂上这个回调。
     * 它同时是"共享状态不用临界区"的守门人：只有 ROBOT_LINK 选中的那条会注册进来。 */
    if (id != (uint8_t)ROBOT_LINK_PORT) return;

    for (i = 0U; i < len; i++) {
        const FrameStatus st = FrameParserFeed(&s_parser, data[i]);

        if (st == FRAME_OK) {
            robot_rx_frame_cnt++;
            /* 分类计数放在 arm_id 检查**之前**：统计口径是"解出来了多少"，
             * 越界 arm_id 的帧也算收进来了（它另有一格自己的计数）。 */
            robot_tid_cnt[Tid_Index(s_parser.type_id)]++;

            /* 帧头里的那个字节决定这一帧进哪台臂的环。
             * **越界的当场丢掉** —— g_arm[] 只有 ARM_NUM 个元素，拿越界 id 去索引就是在
             * 中断里踩别人的变量。正常只可能来自上位机写错、或者抓到了别的板子的包。 */
            if (s_parser.arm_id >= (uint8_t)ARM_NUM) {
                robot_rx_bad_arm_cnt++;
                continue;
            }

            {
                Arm* const arm = &g_arm[s_parser.arm_id];
                /* 整帧出来了：把 type_id + payload 拷进**这台臂**的环。
                 * 拷的是 s_parser.buf.raw 里 payload 那一段（从偏移 6 开始）—— 解析器马上
                 * 会开始解下一帧，帧头会覆盖 raw[0..5]，所以不能只存指针。 */
                const uint8_t next = (uint8_t)((arm->rx_head + 1U) % ROBOT_RX_SLOT_NUM);

                if (s_parser.data_len > ROBOT_RX_DATA_LEN) {
                    robot_rx_over_cnt++; /* 超长包：这个版本还处理不了，丢掉 */
                } else if (next == arm->rx_tail) {
                    /* 环满 → **丢新帧**（不是丢旧帧：丢旧会让上位机看到一个更陈旧的目标） */
                    robot_rx_drop_cnt++;
                } else {
                    arm->rx_slot[arm->rx_head].type_id = s_parser.type_id;
                    arm->rx_slot[arm->rx_head].len = s_parser.data_len;
                    memcpy(arm->rx_slot[arm->rx_head].payload.raw,
                        &s_parser.buf.raw[FRAME_HEADER_LEN], s_parser.data_len);
                    __DMB(); /* payload 写完再放 index，见环上面那段说明 */
                    arm->rx_head = next;
                }
            }

        } else if (st == FRAME_ERR_CRC8) {
            robot_crc8_err_cnt++;
        } else if (st == FRAME_ERR_CRC16) {
            robot_crc16_err_cnt++;
        } else {
            /* INCOMPLETE：绝大多数字节走这条，正常 */
        }
    }
}

/* ==================== 状态迁移 ==================== */

static void Ctrl_Invalidate(Arm* arm) {
    arm->ctrl.valid = 0U;
    arm->ctrl.has_vel = 0U;
}

/* 进一个总模式：清控制帧、定 Arm_mode、立刻安排一帧反馈。
 *
 * **所有迁移都必须走这里**（不要直接赋值 arm->state）—— 每个状态该配哪个 Arm_mode 就
 * 只有下面这一个 switch 说了算，散在别处迟早会出现"某条路径忘了设模式"。 */
static void Robot_State_Enter(Arm* arm, Robot_state next) {
    Ctrl_Invalidate(arm);

    /* 状态变了就让下一个反馈门控立刻到期：上位机看到状态迁移更及时（不用等 0~4 ms）。
     * 用减法而不是直接赋值，是为了不怕 HAL_GetTick() 刚上电还很小的时候下溢 —— 无符号
     * 减法回绕之后是个很大的数，"已经超时"的判断照样成立。 */
    arm->fb_last_ms = HAL_GetTick() - ROBOT_FEEDBACK_PERIOD_MS;

    switch (next) {
    case robot_state_wait_handshake:
        arm->arm_mode = arm_stop; /* 握手前机械臂保持 stop（需求原文） */
        break;
    case robot_state_offline_wait:
        arm->arm_mode = arm_stop;
        break;
    case robot_state_wait_init:
        arm->arm_mode = arm_stop;
        break;
    case robot_state_initing:
        /* 一键动作靠 auto_mode 驱动，而 arm_stop 档下 Stop_Mode_Update() 每拍都会把
         * auto_mode 清 0 —— 所以初始化期间**必须是 run**，否则复位刚起步就被清掉。 */
        arm->arm_mode = arm_run;
        break;
    case robot_state_normal:
        /* 默认 damping（用户指定）：初始化刚跑完，位置环关着、臂是"软"的，等上位机
         * 明确发 0x12 再交给它位置控制。 */
        arm->arm_mode = arm_damping;
        break;
    case robot_state_comm_lost:
        arm->arm_mode = arm_damping; /* 掉线兜底：被动力矩，人能推动、不会失控 */
        break;
    case robot_state_calibrating:
        arm->arm_mode = arm_stop; /* 设零要求所有参与电机失能，见 arm_zero.h */
        break;
    case robot_state_resetting:
        /* 和 initing 同款理由，而且这里还有第二层：arm_stop 档下 Stop_Mode_Update()
         * 每拍清 auto_mode（App/gimbal_cmd.c），复位撑不过一拍。而 **damping 也不行 ——
         * 它虽然不清 auto_mode，但被动分支把 ref_position 钉在反馈角上、kp=0
         * （gimbal_arm.c），规划喂进去电机也不动：复位会"跑完"但臂根本没动过**，
         * 比 stop 那种立刻看得出的失败更难查。所以必须是 run。 */
        arm->arm_mode = arm_run;
        break;
    default:
        break;
    }

    arm->state = next;
    arm->state_last = next;
    arm->arm_mode_last = arm->arm_mode;
    arm->transition_cnt++;
}

/* 7 台（含夹爪）全部在线。判据和 gimbal_arm.c 里 Arm_Auto_Init() 的循环同款。 */
static uint8_t All_Motors_Online(Arm* arm) {
    int i;
    if (arm->gimbal == NULL) return 0U;
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        if (arm->gimbal->arm_feedback_data.is_motor_lost[i]) return 0U;
    }
    return 1U;
}

/* 姿态是否合格：6 个关节（**不含夹爪**）的反馈角和预设初始化位置逐个比。 */
static uint8_t Init_Pose_Ok(Arm* arm) {
    int i;
    if (arm->gimbal == NULL) return 0U;

    for (i = 0; i < ARM_JOINT_NUM; i++) {
        float d = arm->gimbal->arm_feedback_data.joint_angle[i] - ARM_RESET_ANGLE[i];
        if (d < 0.0f) d = -d;
        if (d > ROBOT_INIT_POSE_TOL_DEG) return 0U;
    }
    return 1U;
}

/* 遥控器是否在线（100 ms 内收到过遥控帧）。
 * 判据和 App/gimbal_cmd.c 的 Rc_Online() 是同一个表达式 —— 两处必须一致，所以两边都写了
 * 注释指到对面。 */
static uint8_t Rc_Online(void) { return (uint8_t)(!is_Offline(rc.monitor)); }

/* ==================== 收帧分派 ==================== */

static void HandleVersion(Arm* arm, const Robot_rx_payload* pl, uint8_t len) {
    if (len != (uint8_t)sizeof(FramePktVersion)) return; /* 长度不对：当噪声丢掉 */

    if (pl->version.code != FrameVersionCode()) {
        /* 版本码不符 → **丢掉这一帧**，别的什么都不动。
         *
         * 仍然不回包：两端对包的理解本来就不一致，回什么都等于承认这种对话能继续。
         * 但也**不改任何状态** —— 不置 handshake_ok、不打回等待握手。于是握手闸门照旧
         * 不通过（继续等正确的版本包），而上位机把版本串改对后再发一帧就能握手，
         * **不用复位板子**。计数只增不减，给调试器和上位机一个"被拒了"的证据。 */
        arm->handshake_reject_cnt++;
        return;
    }

    /* 版本码相等 → 回一个版本包（自报家门），进离线等待。 */
    if (!arm->handshake_ok) {
        arm->handshake_ok = 1U;
        arm->handshake_ok_obs = 1U;
        arm->handshake_cnt++;
        /* 注意顺序：SendFrame() 里判 arm->handshake_ok，所以先置位再发。 */
        SendVersion(arm);
        Robot_State_Enter(arm, robot_state_offline_wait);
        return;
    }

    /* 已经握过手又收到一个版本包：**幂等地再回一个，状态不动**。
     * 上位机重连/重试时会这么做，回它一句不影响任何状态；但**不重置状态机** ——
     * 那会把正在跑的臂打回离线等待，代价比"多回一包"大得多。 */
    SendVersion(arm);
}

static void HandleArmInit(Arm* arm) {
    /* 两个入口：
     *   ① 「待初始化」—— 上电流程的常态入口；
     *   ② 「正常运行」+ 松劲侧档位（stop / damping）—— 用途是**不复位板子也能重走一遍
     *      上电流程**（Init_Step() 跑完回的是正常运行，所以只有这条口能回去）。
     *
     * 为什么只放 stop/damping、不放 run/gravity：初始化期间状态机会把 Arm_mode 掰到
     * **run**（见 Robot_State_Enter，一键动作靠 auto_mode 驱动，stop 档下
     * Stop_Mode_Update() 每拍清 auto_mode、复位撑不过一拍）。所以能接手的必须是"**没有
     * 位置环在跑**"的档 —— run 档下臂正被位置环攥着，突然拉去做复位会打架；stop 和
     * damping 都是"臂是软的、没人管"的档，接过去没有歧义。 */
    const uint8_t from_wait_init = (arm->state == robot_state_wait_init);
    const uint8_t from_normal_soft =
        (arm->state == robot_state_normal
         && (arm->arm_mode == arm_stop || arm->arm_mode == arm_damping));
    if (!from_wait_init && !from_normal_soft) return;

    if (Rc_Online()) {
        arm->init_reject_cnt++;
        return;
    }
    if (!All_Motors_Online(arm)) {
        arm->init_reject_cnt++;
        return;
    }
    if (!Init_Pose_Ok(arm)) {
        /* 偏差过大 → **拒绝，不回包**（需求原文）。两个入口**都查**这条：前提是"初始化"
         * 这件事本身要求臂在初始位附近，跟从哪个入口发起无关。停在原状态，上位机看到反馈
         * 帧里 robot_state 没变、加上 arm->init_reject_cnt 涨了就知道被拒了。
         * （对比 0x18 一键复位**故意不查** —— 那件事就是"把歪的臂收回原位"，见
         * HandleArmReset。两者不是一回事。） */
        arm->init_reject_cnt++;
        return;
    }

    /* 记下入口，中止时回它那儿去（见 arm->init_entry 的注释）。正常跑完两种情况都回
     * 「正常运行」，那条路不用这个变量。 */
    arm->init_entry = arm->state;
    arm->init_step = 0U;
    Robot_State_Enter(arm, robot_state_initing);
}

/* 一键复位（0x18）。和 HandleArmInit() 的区别有两条，都是刻意的：
 *   - **入口是「正常运行」**，不是「待初始化」—— 这是运行中主动收臂，不是上电流程；
 *   - **不查姿态**。0x02 的 ±20° 闸门是为了守住"初始化前提成不成立"；这里臂本来就该
 *     是歪的（正因为歪才要收），"歪"不是拒绝的理由。要不要先确认由**上位机**决定
 *     （协议 doc 里写明了：偏差大时它弹一次 y/n）。
 * 拒绝一律**静默不回包**（同上位机看反馈帧判）。 */
static void HandleArmReset(Arm* arm) {
    if (arm->state != robot_state_normal) {
        arm->reset_reject_cnt++;
        return;
    }
    if (Rc_Online()) {
        arm->reset_reject_cnt++;
        return;
    }
    if (!All_Motors_Online(arm)) {
        /* 有电机掉线：掉线会强制 arm_stop（见 Fsm_Step 的掉线处理），复位根本跑不动，
         * 与其进复位中再卡住，不如当场拒掉。 */
        arm->reset_reject_cnt++;
        return;
    }

    arm->reset_step = 0U;
    Robot_State_Enter(arm, robot_state_resetting);
}

static void HandleMotorSetZero(Arm* arm) {
    /* 校准零点模式**只能从待初始化模式进**（需求原文），且和初始化一样受遥控器闸门。 */
    if (arm->state != robot_state_wait_init) return;
    if (Rc_Online()) return;

    arm->zero_seen_busy = 0U;
    arm->zero_start_ms = HAL_GetTick();
    /* 写 1 触发一轮；arm_zero 的状态机在非忙时会消费它(清 0)并开跑。触发的就是**本台臂**
     * 那一份(gimbal->zero 在 Gimbal 上，一台臂一份)，所以两台臂各设各的零点、互不干扰。
     * 真正的"是否成功"要等它跑完读 arm->gimbal->zero.state / .result[]。 */
    arm->gimbal->zero.request = 1U;
    Robot_State_Enter(arm, robot_state_calibrating);
}

static void HandleModeSwitch(Arm* arm, uint8_t type_id) {
    /* 只在正常运行模式下接受模式切换帧（需求："正常运行 → 允许上位机通过帧切换
     * Arm_mode"）。 */
    if (arm->state != robot_state_normal) return;
    if (Rc_Online()) return;

    /* 切档要把控制帧作废：否则 "run → damping → run" 会把切走之前那个旧目标重新灌进
     * 位置环，把臂猛拽一下。 */
    Ctrl_Invalidate(arm);

    switch (type_id) {
    case FRAME_TYPE_ID_ARM_START:
        arm->arm_mode = arm_run;
        break;
    case FRAME_TYPE_ID_ARM_GRAVITY:
        arm->arm_mode = arm_only_gravity_comp;
        break;
    case FRAME_TYPE_ID_ARM_STOP:
        arm->arm_mode = arm_stop;
        break;
    case FRAME_TYPE_ID_ARM_DAMPING:
        arm->arm_mode = arm_damping;
        break;
    default:
        break;
    }
    arm->arm_mode_last = arm->arm_mode;
}

/* 控制帧（0x16 / 0x17）。**只在正常运行 + run 档 + 遥控器离线**时接受（需求：
 * "只在 run 状态接受控制帧"）。 */
static void HandleCtrl(Arm* arm, const Robot_rx_payload* pl, uint8_t len, uint8_t has_vel) {
    int i;

    if (arm->state != robot_state_normal) return;
    if (arm->arm_mode != arm_run) return;
    if (Rc_Online()) return;

    const uint8_t need = has_vel ? (uint8_t)sizeof(FramePktArmCtrlPosVel)
                                 : (uint8_t)sizeof(FramePktArmCtrlPos);
    if (len != need) return;

    if (has_vel) {
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            arm->ctrl.joint_angle[i] = pl->arm_ctrl_pos_vel.joint_angle[i];
            arm->ctrl.joint_vel[i] = pl->arm_ctrl_pos_vel.joint_vel[i];
        }
        arm->ctrl.gripper_angle = pl->arm_ctrl_pos_vel.gripper_angle;
    } else {
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            arm->ctrl.joint_angle[i] = pl->arm_ctrl_pos.joint_angle[i];
            arm->ctrl.joint_vel[i] = 0.0f;
        }
        arm->ctrl.gripper_angle = pl->arm_ctrl_pos.gripper_angle;
    }

    arm->ctrl.has_vel = has_vel;
    arm->ctrl.valid = 1U;
    arm->ctrl_frame_cnt++;
}

/* 一帧的分派口。**两道闸门**都在这儿，别的函数不管。 */
static void HandleFrame(Arm* arm, uint8_t type_id, const Robot_rx_payload* pl, uint8_t len) {
    /* 闸门 ①：握手通过之前只认版本包 —— 其他一律丢弃，也不回复 */
    if (!arm->handshake_ok && type_id != FRAME_TYPE_ID_VERSION) {
        robot_gated_cnt++; /* 上位机"发了但没回应"的那部分，从链路状态帧里能看见 */
        return;
    }

    /* 能走到这里的都算"上位机还在说话"（CRC 过了；码认不认识不论 —— 认识新包的旧版本
     * 固件不该被误判成掉线）。 */
    arm->last_rx_ms = HAL_GetTick();

    /* 通信离线模式：**收到任意有效帧就回离线等待**（需求原文），**不要求重新握手**
     * —— 握手是一次性的闸门，通过之后一直有效。
     * 回离线等待而不是直接回原状态，是因为掉线期间机械臂已经是 damping 了，得重新走
     * 一遍"电机全在线 → 待初始化"，上位机才知道现在能不能发初始化命令。
     *
     * 放在分派**之前**：这样本帧后面的处理看到的就是恢复后的状态（比如掉线期间发来的
     * 控制帧，会因为在 offline_wait 而不是 normal 被丢掉 —— 正是想要的）。 */
    if (arm->state == robot_state_comm_lost) {
        Robot_State_Enter(arm, robot_state_offline_wait);
    }

    switch (type_id) {
    case FRAME_TYPE_ID_VERSION:
        HandleVersion(arm, pl, len);
        break;
    case FRAME_TYPE_ID_ARM_INIT:
        HandleArmInit(arm);
        break;
    case FRAME_TYPE_ID_MOTOR_SET_ZERO:
        HandleMotorSetZero(arm);
        break;
    case FRAME_TYPE_ID_ARM_START:
    case FRAME_TYPE_ID_ARM_GRAVITY:
    case FRAME_TYPE_ID_ARM_STOP:
    case FRAME_TYPE_ID_ARM_DAMPING:
        HandleModeSwitch(arm, type_id);
        break;
    case FRAME_TYPE_ID_ARM_CTRL_POS:
        HandleCtrl(arm, pl, len, 0U);
        break;
    case FRAME_TYPE_ID_ARM_CTRL_POS_VEL:
        HandleCtrl(arm, pl, len, 1U);
        break;
    case FRAME_TYPE_ID_ARM_RESET:
        HandleArmReset(arm);
        break;
    default:
        /* 不认识的码（含 0x11 —— 那是**我们**发的，上位机不该发过来）：丢弃。
         * 不回复错误包、也不计数成错误 —— 协议没有 NACK，默默忽略是最省事也最安全的。 */
        break;
    }
}

/* 排空接收环。非阻塞，跑完整个环最多 8 帧。
 *
 * 每帧**先拷进局部再放行 tail**：拷贝只花几十个周期，但这样"处理一帧的这段时间里
 * 中断又写满一圈"就完全不可能影响正在处理的数据。 */
static void DrainRx(Arm* arm) {
    for (;;) {
        Robot_rx_payload pl;
        const uint8_t head = arm->rx_head;
        uint8_t type_id;
        uint8_t len;

        __DMB(); /* 先读 head，再读 payload —— 见环定义上面那段 */

        if (arm->rx_tail == head) return; /* 空 */

        type_id = arm->rx_slot[arm->rx_tail].type_id;
        len = arm->rx_slot[arm->rx_tail].len;
        memcpy(&pl, &arm->rx_slot[arm->rx_tail].payload, sizeof(pl));
        arm->rx_tail = (uint8_t)((arm->rx_tail + 1U) % ROBOT_RX_SLOT_NUM);

        HandleFrame(arm, type_id, &pl, len);
    }
}

/* ==================== 各状态的推进 ==================== */

/* 初始化流程。两步一键动作，靠命令层对象的 auto_mode 驱动（和遥控器点动、调试器手工
 * 触发走的是同一个入口）：
 *   ① arm_auto_reset        → 等它自己回 arm_auto_none
 *   ② arm_auto_gripper_zero → 同上（**目前是空壳**，立刻回 none，见 arm_auto.c）
 * 两步都完了 → 正常运行（默认 damping）。
 *
 * 本函数和 Auto_Mode_Update() 在**同一个任务**里（CmdTask），所以直接写 auto_mode
 * 不需要任何保护。
 *
 * ⚠ 一个看不出来的失败模式：临时规划池被占满时 Plan_Group_Temp_Calc_And_Run() 是
 * **静默返回**的（不报错，见 memory arm-auto-framework），于是 Arm_Reset() 的 case 0
 * 起了个空规划、case 1 立刻判定"规划跑完了"、两拍之内就退出。也就是说**复位什么都没做，
 * 但状态机会以为它做完了**。真要判，得在 case 2 结束时拿反馈角和 ARM_RESET_ANGLE 比
 * 一次（跟 Init_Pose_Ok() 同款）。现在没那么做，理由有两条：一是进 normal 之后默认是
 * **damping**、不是 run，臂不会因为"没复位就位控"而乱动；二是上位机本来就该看反馈帧
 * 确认姿态，那才是权威判据。 */
static void Init_Step(Arm* arm) {
    /* 遥控器一旦上线 → **判定复位中止，退回待初始化**。
     *
     * 为什么不接着跑：遥控器会覆盖 Arm_mode，一旦是 stop 档，命令层的
     * Stop_Mode_Update() 每拍清 auto_mode，正在跑的复位被打断；等遥控器再离线，
     * auto_mode 是 none —— 和"复位正常跑完"**长得一模一样**，会被误判成已完成、
     * 直接给电机使能。在这儿主动中止就没有这个歧义（计划评审时发现的洞）。 */
    if (Rc_Online()) {
        if (arm->gimbal_cmd != NULL) {
            arm->gimbal_cmd->auto_mode = arm_auto_none;
            arm->gimbal_cmd->auto_step = 0U;
        }
        arm->init_step = 0U;
        arm->init_abort_cnt++;
        /* 回**发起时**那个状态，不是死板的 wait_init：从「正常运行」发起的初始化被
         * 打断后掉进「待初始化」会把会话卡死（那个状态不受理模式切换帧，见
         * arm->init_entry 的注释）。两种入口进去时都是 arm->init_entry = 入口状态。 */
        Robot_State_Enter(arm, arm->init_entry);
        return;
    }

    /* 命令层没建起来（内存不够）：跑不了初始化。停在这儿等 —— 反馈帧里 robot_state
     * 会一直显示 initing，配合 arm->transition_cnt 不动就能看出卡住了。 */
    if (arm->gimbal_cmd == NULL) return;

    switch (arm->init_step) {
    case 0: /* 起：一键复位 */
        arm->gimbal_cmd->auto_step = 0U;
        arm->gimbal_cmd->auto_mode = arm_auto_reset;
        arm->init_step = 1U;
        break;

    case 1: /* 等复位跑完（Arm_Reset 的 default 分支会把 auto_mode 打回 none） */
        if (arm->gimbal_cmd->auto_mode == arm_auto_none) {
            arm->gimbal_cmd->auto_step = 0U;
            arm->gimbal_cmd->auto_mode = arm_auto_gripper_zero;
            arm->init_step = 2U;
        }
        break;

    case 2: /* 等夹爪找零跑完 */
        if (arm->gimbal_cmd->auto_mode == arm_auto_none) {
            arm->init_step = 0U;
            Robot_State_Enter(arm, robot_state_normal);
        }
        break;

    default:
        arm->init_step = 0U;
        break;
    }
}

/* 一键复位（robot_state_resetting）的推进。只有两拍 —— 这里**没有夹爪找零那一步**，
 * 那正是加 0x18 的理由：只想把臂收回原位时不用白等 4 s 找零、也不会把夹爪零点重设一遍。
 *
 * 跑完回**正常运行（默认 damping）**，不是回待初始化：复位是从 normal 发起的，回去还是
 * 一个可用的会话，上位机按 0x12 就能再进 run。 */
static void Reset_Step(Arm* arm) {
    /* 遥控器上线 → 中止，理由同 Init_Step()：遥控器会覆盖 Arm_mode，stop 档下每拍清
     * auto_mode，正在跑的复位被打断；等遥控器再离线时 auto_mode 也是 none —— 和"复位
     * 正常跑完"长得一模一样，会被误判成已完成。主动中止就没有这个歧义。 */
    if (Rc_Online()) {
        if (arm->gimbal_cmd != NULL) {
            arm->gimbal_cmd->auto_mode = arm_auto_none;
            arm->gimbal_cmd->auto_step = 0U;
        }
        arm->reset_step = 0U;
        arm->reset_abort_cnt++;
        Robot_State_Enter(arm, robot_state_normal);
        return;
    }

    if (arm->gimbal_cmd == NULL) return; /* 命令层没建起来：停在这儿，同 Init_Step() */

    switch (arm->reset_step) {
    case 0: /* 起：一键复位 */
        arm->gimbal_cmd->auto_step = 0U;
        arm->gimbal_cmd->auto_mode = arm_auto_reset;
        arm->reset_step = 1U;
        break;

    case 1: /* 等它跑完（Arm_Reset 的 default 分支把 auto_mode 打回 none） */
        if (arm->gimbal_cmd->auto_mode == arm_auto_none) {
            arm->reset_step = 0U;
            Robot_State_Enter(arm, robot_state_normal);
        }
        break;

    default:
        arm->reset_step = 0U;
        break;
    }
    /* ⚠ 和 Init_Step() 一样有那个静默失败：临时规划池满时 Plan_Group_Temp_Calc_And_Run()
     * 静默返回，Arm_Reset 起了个空规划、下一拍就"跑完"，复位什么都没做但状态机以为做完
     * 了。**这里不补姿态复核**（进去默认 damping，不会乱动），判据交给上位机 —— 它在
     * 7→4 的反馈帧里拿 joint_angle 和 ARM_RESET_ANGLE 比一次就知道。 */
}

static void SendZeroAck(Arm* arm) {
    FramePktMotorSetZeroAck ack;
    int i;

    ack.state = arm->gimbal->zero.state;
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        ack.result[i] = arm->gimbal->zero.result[i];
    }
    ack.elapsed_ms = arm->gimbal->zero.elapsed_ms;
    SendFrame(arm, FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK, (const uint8_t*)&ack, (uint8_t)sizeof(ack));
}

/* 校准零点：等 arm_zero 那台独立状态机跑完，回一个 0x11 ACK，退回待初始化。 */
static void Calib_Step(Arm* arm) {
    /* 遥控器上线同样中止：设零期间整条总线都被关掉(见 arm_zero.h)，遥控器根本控不动，
     * 早点退出比挂着好。**不改 gimbal->zero.request** —— 它是 arm_zero 自己的入口，
     * 已经写下去就让它跑完（半路撤不回来，撤了反而会把电机留在失能态）。 */
    if (Rc_Online()) {
        Robot_State_Enter(arm, robot_state_wait_init);
        return;
    }

    if (!arm->zero_seen_busy) {
        if (Arm_Zero_Busy(arm->gimbal)) {
            arm->zero_seen_busy = 1U;
            return;
        }
        /* 迟迟没人认领（MotorTask 没在跑？）：当失败，把请求撤掉别留下一个"以后会自己
         * 触发"的雷，然后回 ACK + 退回待初始化。 */
        if ((HAL_GetTick() - arm->zero_start_ms) >= ROBOT_ZERO_START_TIMEOUT_MS) {
            arm->gimbal->zero.request = 0U;
            SendZeroAck(arm);
            Robot_State_Enter(arm, robot_state_wait_init);
        }
        return;
    }

    if (Arm_Zero_Busy(arm->gimbal)) return; /* 还在跑 */

    /* 跑完了：DONE 还是 TIMEOUT 由 gimbal->zero.state 说明，原样回给上位机（它自己判）。 */
    SendZeroAck(arm);
    Robot_State_Enter(arm, robot_state_wait_init);
}

static void Fsm_Step(Arm* arm) {
    const uint32_t now = HAL_GetTick();

    /* 通信超时：握手通过之后，除了"还没开始说话"(等待握手)和"已经在离线里"(不再重复
     * 进)两种，**任何状态**都算。判据是"收到有效帧"的间隔 —— 反馈帧是我们自己发的，
     * 不算数。
     *
     * 副作用要知道：上位机只握手、然后干等不发包时，会看到
     *   wait_init(stop) ──500ms──→ comm_lost(damping) ──它发一帧──→ offline_wait
     *   ──电机在线──→ wait_init(stop) ──500ms──→ ...
     * 也就是 arm_stop 和 arm_damping 之间来回切。两个都是"安全态"（一个是失能、
     * 一个是使能但不给位置力），功能上没问题，但会让电机使能位反复动。
     * 要让上位机保持 wait_init 不动，它得**定期发点东西**（比如周期性重发版本包，
     * 已经握过手时是幂等的，不影响状态）。这一点写在 protocol_doc.md 里。 */
    if (arm->handshake_ok && arm->state != robot_state_wait_handshake
        && arm->state != robot_state_comm_lost
        && (now - arm->last_rx_ms) >= ROBOT_COMM_TIMEOUT_MS) {
        Robot_State_Enter(arm, robot_state_comm_lost);
    }

    switch (arm->state) {
    case robot_state_wait_handshake:
        /* 只有 HandleVersion() 能离开这里 */
        break;

    case robot_state_offline_wait:
        /* 7 台（含夹爪）全部上线 → 待初始化 */
        if (All_Motors_Online(arm)) Robot_State_Enter(arm, robot_state_wait_init);
        break;

    case robot_state_wait_init:
        /* 迁移由收帧触发（0x02 → 初始化、0x10 → 校准零点），这里没有自动迁移 */
        break;

    case robot_state_initing:
        Init_Step(arm);
        break;

    case robot_state_normal:
        /* 模式由收帧(0x12/0x13/0x14/0x15)改，这里没有自动迁移 */
        break;

    case robot_state_comm_lost:
        /* 收到任意有效帧就回离线等待 —— 在 HandleFrame() 里做（那里才知道"收到帧"）。
         * 这里什么也不做。 */
        break;

    case robot_state_calibrating:
        Calib_Step(arm);
        break;

    case robot_state_resetting:
        Reset_Step(arm);
        break;

    default:
        break;
    }
}

/* ==================== 250 Hz 反馈 ==================== */

static void Feedback_Step(Arm* arm) {
    /* 握手通过之后**无论什么状态**都发（需求原文）。 */
    if (!arm->handshake_ok) return;

    /* 4 ms 门控 = 250 Hz。
     *
     * ⚠ 这里**用相位累加(`+= 4`)而不是 `= now`**，别改回去。本函数跑在 CmdTask 的
     * 2 ms 栅格上：4 ms 正好是 2 的整数倍，两种写法现在都对；但**这个门控值不是永远
     * 是 4** —— 一改成和 2 ms 不对齐的数（比如 5 ms = 200 Hz），`= now` 就废了：
     * 相位会被每一次实际采样拖着走（0→6→12→18…，间隔恒为 6 ms），实测 1 秒只有 168 帧
     * (≈167 Hz)，而不是 200。
     * 累加的写法对任何周期都对：落点变成 6,4,6,4…，平均正好是设定值。
     *
     * 累加的代价是"落后会越积越多"：如果任务被长时间饿住，arm->fb_last_ms 会远远落在
     * now 后面，之后每一拍都会满足条件、连着补发一串。所以下面那句把落后超过一个周期
     * 的情况直接重新对齐到 now —— 丢掉的帧本来也已经晚了，补发没有意义。 */
    const uint32_t now = HAL_GetTick();
    if ((now - arm->fb_last_ms) < ROBOT_FEEDBACK_PERIOD_MS) return;
    arm->fb_last_ms += ROBOT_FEEDBACK_PERIOD_MS;
    if ((now - arm->fb_last_ms) >= ROBOT_FEEDBACK_PERIOD_MS) {
        arm->fb_last_ms = now;
    }

    FramePktArmFeedback fb;
    int i;

    /* mode 报的是**电机层真实在用的那一档**，不是我们"希望"的那一档：
     * gimbal_arm.c 的 Motor_Lost_Handle() 会在有电机掉线时把 arm_mode 打回 arm_stop，
     * 那时候上位机从这里的 mode 就能看出"我发的 run 没生效"，再对着 motor_lost_mask
     * 就知道为什么。遥控器在线时它也会反映遥控器选的档 —— 这也是有用信息。 */
    fb.mode = (arm->gimbal != NULL)
        ? (uint8_t)arm->gimbal->arm_feedback_data.mode
        : (uint8_t)arm->arm_mode;
    fb.robot_state = (uint8_t)arm->state;
    fb.motor_lost_mask = 0U;
    fb.motor_inited_mask = 0U;
    fb.gripper_angle = 0.0f;

    for (i = 0; i < ARM_JOINT_NUM; i++) {
        fb.joint_angle[i] = 0.0f;
        fb.joint_vel[i] = 0.0f;
    }
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        fb.motor_temp[i] = 0U;
    }

    if (arm->gimbal != NULL) {
        for (i = 0; i < ARM_MOTOR_NUM; i++) {
            if (arm->gimbal->arm_feedback_data.is_motor_lost[i]) {
                fb.motor_lost_mask |= (uint8_t)(1U << i);
            }
            if (arm->gimbal->arm_feedback_data.motor_inited[i]) {
                fb.motor_inited_mask |= (uint8_t)(1U << i);
            }
            fb.motor_temp[i] = arm->gimbal->arm_feedback_data.motor_temp[i];
        }
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            fb.joint_angle[i] = arm->gimbal->arm_feedback_data.joint_angle[i];
            fb.joint_vel[i] = arm->gimbal->arm_feedback_data.joint_vel[i];
        }
        fb.gripper_angle = arm->gimbal->arm_feedback_data.gripper_angle;
    }

    SendFrame(arm, FRAME_TYPE_ID_ARM_FEEDBACK, (const uint8_t*)&fb, (uint8_t)sizeof(fb));
    arm->fb_cnt++;
}

/* ==================== 未握手时的 10 Hz 版本帧 ====================
 *
 * 没握手之前，板子每 100 ms 主动发一帧版本包 0x01：上位机不用先问就知道板子在不在、
 * 是哪个版本码 —— 版本码不符时当场就能看出是自己那边对不上。握手一过就停（那时
 * 250 Hz 反馈帧已经在发）。
 *
 * 这是**唯一**绕过 SendFrame() 那道握手闸门的出帧路径（SendVersionRaw → TxEnqueue）：
 * 0x01 恰恰要在 handshake_ok==0 的时候发，闸门挡它等于谁也握不了手。 */
static void Handshake_Step(Arm* arm) {
    if (arm->handshake_ok) return;

    const uint32_t now = HAL_GetTick();
    /* `= now` 而不是像 Feedback_Step 那样 `+= 周期`：这里没有别的节拍要对齐，而累加的
     * 写法在任务被饿住之后会连发一串补帧 —— 补出去的握手帧没有意义。 */
    if ((now - arm->hs_last_ms) < ROBOT_HANDSHAKE_PERIOD_MS) return;
    arm->hs_last_ms = now;

    SendVersionRaw(arm);
}

/* ==================== 链路状态帧 ====================
 *
 * 10 Hz 把"这段时间里我收到了你多少帧"报给上位机：总数、按 type_id 分类、各类链路错误。
 * 全是**增量**，窗口 = 上一帧链路状态帧到这一帧之间。
 *
 * 链路级只有一份（统计量本身就是全链路一份），所以**不按台发**：这一帧从头到尾只发一帧，
 * 用哪台的 arm_id 取决于谁先握上手。这跟反馈帧那种"两台各发各的"不是一回事 —— 上位机
 * 那边也是按链路级接的（见 sdk/link.py）。
 *
 * 只在握手之后发：握手之前上位机还没证明自己是同一个协议，报统计没有意义；而且那时
 * 上行已经有 10 Hz 的版本帧了。 */

/* 上一帧发出那一刻的累计量快照（存进 st 的每个字段都是"现在 − 快照"）。
 * 和 s_parser 一样是链路级只有一份的文件级静态。 */
static uint32_t s_ls_rx_total;
static uint32_t s_ls_crc8;
static uint32_t s_ls_crc16;
static uint32_t s_ls_bad_arm;
static uint32_t s_ls_over;
static uint32_t s_ls_ring_drop;
static uint32_t s_ls_gated;
static uint32_t s_ls_tx_fail;
static uint32_t s_ls_bytes_drop; /* s_parser.bytes_drop 的快照 */
static uint32_t s_ls_tid[FRAME_TID_SLOT_NUM];
static uint32_t s_ls_last_ms;
static uint8_t s_ls_started; /* 0 = 还没起窗口（没握手过，或刚握上手） */

/* 把当前累计量全拍下来。起窗口和每发一帧之后都调，两处必须完全对称 ——
 * 少存一个字段，那个字段报出来的就是"开机至今"而不是一个窗口。 */
static void Link_Status_Take_Snapshot(void) {
    uint8_t i;

    s_ls_rx_total = robot_rx_frame_cnt;
    s_ls_crc8 = robot_crc8_err_cnt;
    s_ls_crc16 = robot_crc16_err_cnt;
    s_ls_bad_arm = robot_rx_bad_arm_cnt;
    s_ls_over = robot_rx_over_cnt;
    s_ls_ring_drop = robot_rx_drop_cnt;
    s_ls_gated = robot_gated_cnt;
    s_ls_tx_fail = robot_tx_fail_cnt;
    s_ls_bytes_drop = s_parser.bytes_drop;

    for (i = 0U; i < (uint8_t)FRAME_TID_SLOT_NUM; i++) {
        s_ls_tid[i] = robot_tid_cnt[i];
    }
}

static void Link_Status_Step(void) {
    /* 任一臂握手就发。两台都握了固定走 0 号；只有一个臂上了电（另一台的 gimbal 没建起来、
     * 所以根本没人喂它握手）也不会因此拿不到统计。 */
    Arm* arm = NULL;
    uint8_t i;

    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        if (g_arm[i].handshake_ok) {
            arm = &g_arm[i];
            break;
        }
    }
    if (arm == NULL) {
        s_ls_started = 0U; /* 没握手：窗口往后重起，握手那一刻的累计量不会被算进第一帧 */
        return;
    }

    const uint32_t now = HAL_GetTick();

    if (s_ls_started == 0U) {
        /* 起窗口：把当前累计量全拍下来，**这一拍不发**。于是第一帧报的是一个完整的
         * 100 ms 窗口，不会出现 dt_ms = 开机至今、或者把握手前那堆数算进来的假值。 */
        Link_Status_Take_Snapshot();
        s_ls_last_ms = now;
        s_ls_started = 1U;
        return;
    }

    /* `= now` 相位，同 Handshake_Step：累加会在任务被饿住之后连发一串补帧，
     * 而补出来的统计窗口是空的，没有意义。 */
    if ((now - s_ls_last_ms) < ROBOT_LINK_STATUS_PERIOD_MS) return;

    FramePktLinkStatus st;
    const uint8_t n = (uint8_t)FRAME_TID_SLOT_NUM;

    st.rx_total = robot_rx_frame_cnt - s_ls_rx_total;
    st.crc8_err = robot_crc8_err_cnt - s_ls_crc8;
    st.crc16_err = robot_crc16_err_cnt - s_ls_crc16;
    st.bad_arm = robot_rx_bad_arm_cnt - s_ls_bad_arm;
    st.over_len = robot_rx_over_cnt - s_ls_over;
    st.ring_drop = robot_rx_drop_cnt - s_ls_ring_drop;
    st.gated = robot_gated_cnt - s_ls_gated;
    st.tx_fail = robot_tx_fail_cnt - s_ls_tx_fail;
    /* 没能组成有效帧的字节。解析器只给一个数（它自己在中断里算好，见 frame.c），
     * 这里按窗口取差就是这一段的。 */
    st.drop_bytes = s_parser.bytes_drop - s_ls_bytes_drop;
    st.dt_ms = now - s_ls_last_ms;

    for (i = 0U; i < n; i++) {
        st.rx_by_tid[i] = robot_tid_cnt[i] - s_ls_tid[i];
    }

    Link_Status_Take_Snapshot();
    s_ls_last_ms = now;

    SendFrame(arm, FRAME_TYPE_ID_LINK_STATUS, (const uint8_t*)&st, (uint8_t)sizeof(st));
}

/* ==================== 入口 ==================== */

void Robot_State_Init(void) {
    uint8_t i;

    /* 两台臂的按台状态一起清。`id` 也在这儿定死：它就是帧头里的 arm_id、也是 CAN 索引
     * (见 Arm_Index)，从开机第一拍起就必须是对的 —— SendFrame() 拿它编帧，
     * 晚一步设上就会发出一串 arm_id 错的帧。 */
    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        Arm* const arm = &g_arm[i];

        arm->id = i;

        memset(arm->rx_slot, 0, sizeof(arm->rx_slot));
        arm->rx_head = 0U;
        arm->rx_tail = 0U;

        arm->handshake_ok = 0U;
        arm->state = robot_state_wait_handshake;
        arm->arm_mode = arm_stop;
        arm->init_step = 0U;
        /* 只是给 JLink 一个干净初值：真值在 HandleArmInit() 里赋。结构体本来是全零的
         * (= robot_state_wait_handshake)，写出来是为了读内存时不用猜这个 0 是什么意思。 */
        arm->init_entry = robot_state_wait_handshake;
        arm->reset_step = 0U;
        arm->zero_seen_busy = 0U;
        arm->zero_start_ms = 0U;
        Ctrl_Invalidate(arm);

        arm->last_rx_ms = HAL_GetTick();
        arm->fb_last_ms = HAL_GetTick();
        /* 退一个周期 = 上电第一拍就发第一帧（上位机开完口不用先发什么就能看见板子）。 */
        arm->hs_last_ms = HAL_GetTick() - ROBOT_HANDSHAKE_PERIOD_MS;

        arm->state_last = arm->state;
        arm->arm_mode_last = arm->arm_mode;
        arm->handshake_ok_obs = 0U;
    }

    /* ---- 共享链路这一份 ---- */
    FrameParserReset(&s_parser);
    s_tx_head = 0U;
    s_tx_tail = 0U;

    /* 链路状态帧的窗口从零起：累计量本身是静态零初始化，快照也跟着清零，
     * 这样两边的差从第一拍开始就是对的（不清的话快照是上一轮的值）。 */
    memset(&s_ls_tid, 0, sizeof(s_ls_tid));
    s_ls_rx_total = 0U;
    s_ls_crc8 = 0U;
    s_ls_crc16 = 0U;
    s_ls_bad_arm = 0U;
    s_ls_over = 0U;
    s_ls_ring_drop = 0U;
    s_ls_gated = 0U;
    s_ls_tx_fail = 0U;
    s_ls_bytes_drop = 0U;
    s_ls_last_ms = HAL_GetTick();
    s_ls_started = 0U;

    /* 必须在两个 BSP 的 Init 之后 —— 它们会清空自己的回调登记，注册早了会被抹掉。
     * **只登记 ROBOT_LINK 选中的那一条**：没选中的外设照样活着（USB 那边
     * MX_USB_DEVICE_Init() 是无条件调的），但没人注册 = 碰不到任何共享状态。 */
    Robot_Link_RegRx(ROBOT_LINK_PORT, Robot_State_OnRx);
}

void Robot_State_Loop(void) {
    uint8_t i;

    /* 逐台推进。**只给驱动建起来了的臂跑**（gimbal 非 NULL）：没有驱动就没有反馈帧可发、
     * 也没有"电机在线"可判，白跑一圈只会在链路上多出一串没人要的帧。 */
    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        Arm* const arm = &g_arm[i];
        if (arm->gimbal == NULL) continue;

        DrainRx(arm);       /* 收：中断按 arm_id 送进这台臂的环 */
        Fsm_Step(arm);      /* 状态迁移 */
        Handshake_Step(arm);/* 发：没握手时那 10 Hz 版本包（只入队；握手后空转） */
        Feedback_Step(arm); /* 发：250 Hz 反馈帧（只入队） */
    }

    /* 链路级的东西**不按台走**，所以在循环外面只调一次（放在排空之前，它就和这一拍的
     * 反馈帧一起发出去）。 */
    Link_Status_Step();     /* 发：10 Hz 链路状态帧（只入队） */

    /* 共享链路的出口：两台臂的帧在同一队里，**每拍只排空一次**（不在上面的循环里）。 */
    Link_DrainTx();
}

/* 命令层的 Run_Mode_Update() / Arm_Ref_Set() 取这台臂的那一份。永远非 NULL（在 Arm 里
 * 静态分配），有效与否看 valid。 */
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
