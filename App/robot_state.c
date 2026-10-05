#include "robot_state.h"

#include <string.h> /* memcpy / memset */

#include "main.h" /* HAL_GetTick() */

#include "arm_auto.h"   /* ARM_RESET_ANGLE / arm_auto_reset / arm_auto_gripper_zero */
#include "arm_zero.h"   /* arm_zero_request / Arm_Zero_Busy() / 状态与结果 */
#include "gimbal_arm.h" /* gimbal —— 反馈数据（关节角、掉线标志、温度） */
#include "gimbal_cmd.h" /* gimbal_cmd —— 初始化流程要写它的 auto_mode */
#include "rc.h"         /* rc / is_Offline() —— 遥控器优先的判据 */
#include "uart_frame.h" /* UartFrameParser / UartFrameEncode() */

/* ==================== 这一层和上下左右的关系 ====================
 *
 * 下面这几段是**并发**的，看代码时先记住谁跟谁不在一个上下文里：
 *
 *   Robot_State_OnRx()    中断上下文（BSP 的收包回调）。只做"喂解析器 + 拷贝进环"。
 *   Robot_State_Loop()    任务上下文（CmdTask，500 Hz）。排空环、跑状态机、发反馈。
 *   状态机自己            **和命令层同一个任务**（Cmd_Loop 里先调本模块再调
 *                          Gimbal_Cmd_Update），所以本模块写 gimbal_cmd->auto_mode
 *                          和 Auto_Mode_Update() 读它之间**没有并发**，不需要保护。
 *
 * 和 libs 的分工：本模块**不做**切帧/校验（那是 libs/uart_frame 的活），也**不碰**
 * DMA/缓冲（那是 libs/uart 的活）。BSP 只把字节推过来，本模块自己喂解析器。
 *
 * 一个"帧"要过三道闸门才起作用，顺序不能乱：
 *   ① s_locked（版本码不符过，永久静默）
 *   ② !s_handshake_ok（没握手前只认版本包）
 *   ③ 各包自己的模式检查（比如 0x02 只在待初始化模式生效）
 */

/* ==================== 接收环 ==================== */

/* 环槽里 payload 的长度。**下行最长的包是 0x17 的 52 字节**，这里给 64 留了点余地。
 *
 * 为什么不是 255（协议上限）：8 个槽 × 255 就是 2 KB 的常驻 RAM，而按上面的静态断言，
 * 超过 ROBOT_RX_DATA_LEN 的包是**直接丢掉并计数**的 —— 将来真要加一个长包，先看
 * robot_rx_over_cnt 涨不涨，再决定是改大这里还是换存储。 */
#define ROBOT_RX_DATA_LEN 64U
#define ROBOT_RX_SLOT_NUM 8U

/* 一个环槽。payload 用 union 而不是裸数组，为的是让任务侧能**按结构体读**：
 * 中断侧往 raw 里 memcpy，任务侧读 arm_ctrl_pos 这样的成员 —— 这是 C 标准 6.5.2.3
 * 认可的 type punning（和 libs/uart_frame 的 UartFrameBuf 一个道理）。
 * 换成 `*(UartPktArmCtrlPos*)slot->payload.raw` 这种指针强转就会踩 strict aliasing。 */
typedef union {
    uint8_t raw[ROBOT_RX_DATA_LEN];
    UartPktVersion version;
    UartPktMotorSetZeroAck motor_set_zero_ack;
    UartPktArmCtrlPos arm_ctrl_pos;
    UartPktArmCtrlPosVel arm_ctrl_pos_vel;
} Robot_rx_payload;

typedef struct {
    uint8_t type_id;
    uint8_t len;
    Robot_rx_payload payload;
} Robot_rx_slot;

/* 单生产者（中断）单消费者（任务）环。
 * head 只被中断写、tail 只被任务写，两个都是单字节，读写天然原子；但 **payload 的写
 * 可能被编译器排到 index 写之后**（-O2），Cortex-M7 还有 store buffer，所以两边都要
 * __DMB()。丢帧是偶发的、难复现的，就在这儿。 */
static Robot_rx_slot s_rx_slot[ROBOT_RX_SLOT_NUM];
static volatile uint8_t s_rx_head; /* 中断写 */
static volatile uint8_t s_rx_tail; /* 任务写 */

/* 解析器。约 270 字节，文件级静态（别放栈上）。只在中断里被碰。 */
static UartFrameParser s_parser;

/* 发送缓冲。UartFrameEncode() 组好一帧、BSP_UART_Send() 会**再拷一份**进它自己的
 * DMA 缓冲，所以这块只要在函数返回前有效即可 —— 用静态是为了不占任务栈。 */
static uint8_t s_tx_buf[UART_FRAME_MAX_LEN];

/* ==================== 状态 ==================== */

static Robot_state s_state = robot_state_wait_handshake;
static Arm_mode s_arm_mode = arm_stop; /* 总模式推导出来的出力模式 */
static Robot_ctrl_cmd s_ctrl;          /* 上位机最新控制帧 */

static uint8_t s_handshake_ok;         /* 握手通过 */
static uint8_t s_locked;               /* 版本码不符，永久静默（要复位才恢复） */
static uint32_t s_last_rx_ms;          /* 最后一次收到有效帧的时刻 */
static uint32_t s_fb_last_ms;          /* 上一次发反馈帧的时刻（4 ms 门控 = 250 Hz） */

/* 初始化流程(robot_state_initing)的子步骤：0 起复位 / 1 等复位跑完 / 2 等夹爪找零跑完 */
static uint8_t s_init_step;

/* 校准零点(robot_state_calibrating)：arm_zero_request 写下去到 MotorTask 认领之间有
 * 最多 1 ms 的窗口，**必须先等到"忙"再等"不忙"**，否则第一拍就读到"不忙"、误判成已完成。
 * s_zero_seen_busy 就是那个"已经见过忙"的锁存。 */
static uint8_t s_zero_seen_busy;
static uint32_t s_zero_start_ms; /* 写 request 的时刻，用来判"迟迟没人认领" */

/* ==================== 观测量（JLink 读） ==================== */

volatile uint32_t robot_state_transition_cnt;
volatile Robot_state robot_state_last;
volatile Arm_mode robot_arm_mode_last;
volatile uint32_t robot_rx_frame_cnt;
volatile uint32_t robot_rx_drop_cnt;
volatile uint32_t robot_rx_over_cnt;
volatile uint32_t robot_crc8_err_cnt;
volatile uint32_t robot_crc16_err_cnt;
volatile uint32_t robot_tx_fail_cnt;
volatile uint32_t robot_fb_cnt;
volatile uint32_t robot_handshake_cnt;
volatile uint32_t robot_handshake_reject_cnt;
volatile uint32_t robot_init_reject_cnt;
volatile uint32_t robot_init_abort_cnt;
volatile uint32_t robot_ctrl_frame_cnt;
volatile uint8_t robot_handshake_ok;
volatile uint8_t robot_locked;

/* ==================== 协议常量要和 App 对上 ====================
 *
 * libs/uart_frame 那边线上有几个关节/电机是**协议定死的**（UART_FRAME_PKT_*），不是
 * 由 App 的 ARM_JOINT_NUM/ARM_MOTOR_NUM 决定的。两个数现在正好相等，接上层时在这儿
 * 对一下 —— 哪天 App 加了第 7 个关节而协议没改，这里会红，比运行时静默错位好。 */
_Static_assert(UART_FRAME_PKT_JOINT_NUM == ARM_JOINT_NUM, "协议的关节数和 App 对不上");
_Static_assert(UART_FRAME_PKT_MOTOR_NUM == ARM_MOTOR_NUM, "协议的电机数和 App 对不上");
/* 反馈帧的结构体必须放得下 7 个 uint8_t 温度 + 6+6 个 float + 4 个头，正好 63 */
_Static_assert(sizeof(UartPktArmFeedback) == 63U, "反馈帧不是 63 字节");
/* 环槽要放得下最长的下行包，否则那个包会被静默丢掉（只在 robot_rx_over_cnt 上看得见） */
_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(UartPktArmCtrlPosVel), "环槽放不下 0x17 包");
_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(UartPktMotorSetZeroAck), "环槽放不下 0x11 包");

/* ==================== 发送 ==================== */

static void SendFrame(uint8_t type_id, const uint8_t* data, uint8_t len) {
    /* 握手之前**一个字节都不发**（含反馈帧）。版本码不符时更是一个字节都不发。 */
    if (!s_handshake_ok || s_locked) return;

    const int32_t n = UartFrameEncode(s_tx_buf, (uint32_t)sizeof(s_tx_buf), type_id, data, len);
    if (n <= 0) {
        robot_tx_fail_cnt++;
        return;
    }
    /* BSP_UART_Send() 是 DMA 非阻塞，上一笔还没发完返回 BUSY。
     * **丢这一帧 + 计数，不重试、不排队**：下一帧 4 ms 后就到，重试只会把积压越堆越长，
     * 而上位机看到的是"帧晚了"而不是"帧错了"。 */
    if (BSP_UART_Send(ROBOT_UART_PORT, s_tx_buf, (uint16_t)n) != BSP_UART_OK) {
        robot_tx_fail_cnt++;
    }
}

static void SendVersion(void) {
    UartPktVersion pkt;
    pkt.code = UartFrameVersionCode();
    SendFrame(UART_FRAME_TYPE_ID_VERSION, (const uint8_t*)&pkt, (uint8_t)sizeof(pkt));
}

/* ==================== 接收 ==================== */

/* 中断上下文：只做"喂解析器 + 拷进环"，绝不阻塞、绝不打印、绝不调 FreeRTOS API。
 * data 指向 BSP 内部的 DMA 缓冲，返回后随时会被下一次接收覆盖 —— 这里当场消费掉，
 * 不存指针。 */
static void Robot_State_OnRx(uint8_t id, const uint8_t* data, uint16_t len) {
    uint16_t i;

    /* 只挂了一路，正常恒成立；留着是为了万一以后有人给别的口也挂上这个回调。 */
    if (id != (uint8_t)ROBOT_UART_PORT) return;

    for (i = 0U; i < len; i++) {
        const UartFrameStatus st = UartFrameParserFeed(&s_parser, data[i]);

        if (st == UART_FRAME_OK) {
            /* 整帧出来了：把 type_id + payload 拷进环。
             * 拷的是 s_parser.buf.raw 里 payload 那一段（从偏移 5 开始）—— 解析器马上
             * 会开始解下一帧，帧头会覆盖 raw[0..4]，所以不能只存指针。 */
            const uint8_t next = (uint8_t)((s_rx_head + 1U) % ROBOT_RX_SLOT_NUM);

            if (s_parser.data_len > ROBOT_RX_DATA_LEN) {
                robot_rx_over_cnt++; /* 超长包：这个版本还处理不了，丢掉 */
            } else if (next == s_rx_tail) {
                /* 环满 → **丢新帧**（不是丢旧帧：丢旧会让上位机看到一个更陈旧的目标） */
                robot_rx_drop_cnt++;
            } else {
                s_rx_slot[s_rx_head].type_id = s_parser.type_id;
                s_rx_slot[s_rx_head].len = s_parser.data_len;
                memcpy(s_rx_slot[s_rx_head].payload.raw, &s_parser.buf.raw[UART_FRAME_HEADER_LEN],
                    s_parser.data_len);
                __DMB(); /* payload 写完再放 index，见环上面那段说明 */
                s_rx_head = next;
            }
            robot_rx_frame_cnt++;

        } else if (st == UART_FRAME_ERR_CRC8) {
            robot_crc8_err_cnt++;
        } else if (st == UART_FRAME_ERR_CRC16) {
            robot_crc16_err_cnt++;
        } else {
            /* INCOMPLETE：绝大多数字节走这条，正常 */
        }
    }
}

/* ==================== 状态迁移 ==================== */

static void Ctrl_Invalidate(void) {
    s_ctrl.valid = 0U;
    s_ctrl.has_vel = 0U;
}

/* 进一个总模式：清控制帧、定 Arm_mode、立刻安排一帧反馈。
 *
 * **所有迁移都必须走这里**（不要直接赋值 s_state）—— 每个状态该配哪个 Arm_mode 就
 * 只有下面这一个 switch 说了算，散在别处迟早会出现"某条路径忘了设模式"。 */
static void Robot_State_Enter(Robot_state next) {
    Ctrl_Invalidate();

    /* 状态变了就让下一个反馈门控立刻到期：上位机看到状态迁移更及时（不用等 0~4 ms）。
     * 用减法而不是直接赋值，是为了不怕 HAL_GetTick() 刚上电还很小的时候下溢 —— 无符号
     * 减法回绕之后是个很大的数，"已经超时"的判断照样成立。 */
    s_fb_last_ms = HAL_GetTick() - ROBOT_FEEDBACK_PERIOD_MS;

    switch (next) {
    case robot_state_wait_handshake:
        s_arm_mode = arm_stop; /* 握手前机械臂保持 stop（需求原文） */
        break;
    case robot_state_offline_wait:
        s_arm_mode = arm_stop;
        break;
    case robot_state_wait_init:
        s_arm_mode = arm_stop;
        break;
    case robot_state_initing:
        /* 一键动作靠 auto_mode 驱动，而 arm_stop 档下 Stop_Mode_Update() 每拍都会把
         * auto_mode 清 0 —— 所以初始化期间**必须是 run**，否则复位刚起步就被清掉。 */
        s_arm_mode = arm_run;
        break;
    case robot_state_normal:
        /* 默认 damping（用户指定）：初始化刚跑完，位置环关着、臂是"软"的，等上位机
         * 明确发 0x12 再交给它位置控制。 */
        s_arm_mode = arm_damping;
        break;
    case robot_state_comm_lost:
        s_arm_mode = arm_damping; /* 掉线兜底：被动力矩，人能推动、不会失控 */
        break;
    case robot_state_calibrating:
        s_arm_mode = arm_stop; /* 设零要求所有参与电机失能，见 arm_zero.h */
        break;
    default:
        break;
    }

    s_state = next;
    robot_state_last = next;
    robot_arm_mode_last = s_arm_mode;
    robot_state_transition_cnt++;
}

/* 7 台（含夹爪）全部在线。判据和 gimbal_arm.c 里 Arm_Auto_Init() 的循环同款。 */
static uint8_t All_Motors_Online(void) {
    int i;
    if (gimbal == NULL) return 0U;
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        if (gimbal->arm_feedback_data.is_motor_lost[i]) return 0U;
    }
    return 1U;
}

/* 姿态是否合格：6 个关节（**不含夹爪**）的反馈角和预设初始化位置逐个比。 */
static uint8_t Init_Pose_Ok(void) {
    int i;
    if (gimbal == NULL) return 0U;

    for (i = 0; i < ARM_JOINT_NUM; i++) {
        float d = gimbal->arm_feedback_data.joint_angle[i] - ARM_RESET_ANGLE[i];
        if (d < 0.0f) d = -d;
        if (d > ROBOT_INIT_POSE_TOL_DEG) return 0U;
    }
    return 1U;
}

/* 遥控器是否在线（100 ms 内收到过遥控帧）。
 * 判据和 gimbal_cmd.c 的 Rc_Online() 是同一个表达式 —— 两处必须一致，所以两边都写了
 * 注释指到对面。 */
static uint8_t Rc_Online(void) { return (uint8_t)(!is_Offline(rc.monitor)); }

/* ==================== 收帧分派 ==================== */

static void HandleVersion(const Robot_rx_payload* pl, uint8_t len) {
    if (len != (uint8_t)sizeof(UartPktVersion)) return; /* 长度不对：当噪声丢掉 */

    if (pl->version.code != UartFrameVersionCode()) {
        /* 版本码不符 → **锁死**：此后不解析、不回复任何帧（需求原文）。
         *
         * 注意后果：上位机侧**没有任何重试机会**，改对版本码也不行，要复位(或重新烧写)
         * 本板才恢复。这一点已经写进 protocol_doc.md 提醒对端先自检版本码再发。
         * 之所以做得这么绝，是因为"半信半疑地继续通信"更危险 —— 两端对包的理解不一致，
         * 解出来的东西没人能保证。 */
        s_locked = 1U;
        s_handshake_ok = 0U;
        robot_locked = 1U; 
        robot_handshake_ok = 0U;
        robot_handshake_reject_cnt++;
        Robot_State_Enter(robot_state_wait_handshake);
        return;
    }

    /* 版本码相等 → 回一个版本包（自报家门），进离线等待。 */
    if (!s_handshake_ok) {
        s_handshake_ok = 1U;
        robot_handshake_ok = 1U;
        robot_handshake_cnt++;
        /* 注意顺序：SendFrame() 里判 s_handshake_ok，所以先置位再发。 */
        SendVersion();
        Robot_State_Enter(robot_state_offline_wait);
        return;
    }

    /* 已经握过手又收到一个版本包：**幂等地再回一个，状态不动**。
     * 上位机重连/重试时会这么做，回它一句不影响任何状态；但**不重置状态机** ——
     * 那会把正在跑的臂打回离线等待，代价比"多回一包"大得多。 */
    SendVersion();
}

static void HandleArmInit(void) {
    /* 只能在待初始化模式发起（需求原文）。另外两条自检：
     *   - 电机必须全在线（wait_init 本来就是从那里来的，但中途掉过线要挡住）；
     *   - 遥控器在线则拒绝（遥控器优先）。 */
    if (s_state != robot_state_wait_init) return;
    if (Rc_Online()) {
        robot_init_reject_cnt++;
        return;
    }
    if (!All_Motors_Online()) {
        robot_init_reject_cnt++;
        return;
    }
    if (!Init_Pose_Ok()) {
        /* 偏差过大 → **拒绝，不回包**（需求原文）。停在待初始化模式，上位机看到反馈帧里
         * robot_state 还是 wait_init、加上 robot_init_reject_cnt 涨了就知道被拒了。 */
        robot_init_reject_cnt++;
        return;
    }

    s_init_step = 0U;
    Robot_State_Enter(robot_state_initing);
}

static void HandleMotorSetZero(void) {
    /* 校准零点模式**只能从待初始化模式进**（需求原文），且和初始化一样受遥控器闸门。 */
    if (s_state != robot_state_wait_init) return;
    if (Rc_Online()) return;

    s_zero_seen_busy = 0U;
    s_zero_start_ms = HAL_GetTick();
    /* 写 1 触发一轮；arm_zero 的状态机在非忙时会消费它(清 0)并开跑。
     * 真正的"是否成功"要等它跑完读 arm_zero_state / arm_zero_result[]。 */
    arm_zero_request = 1U;
    Robot_State_Enter(robot_state_calibrating);
}

static void HandleModeSwitch(uint8_t type_id) {
    /* 只在正常运行模式下接受模式切换帧（需求："正常运行 → 允许上位机通过帧切换
     * Arm_mode"）。 */
    if (s_state != robot_state_normal) return;
    if (Rc_Online()) return;

    /* 切档要把控制帧作废：否则 "run → damping → run" 会把切走之前那个旧目标重新灌进
     * 位置环，把臂猛拽一下。 */
    Ctrl_Invalidate();

    switch (type_id) {
    case UART_FRAME_TYPE_ID_ARM_START:
        s_arm_mode = arm_run;
        break;
    case UART_FRAME_TYPE_ID_ARM_GRAVITY:
        s_arm_mode = arm_only_gravity_comp;
        break;
    case UART_FRAME_TYPE_ID_ARM_STOP:
        s_arm_mode = arm_stop;
        break;
    case UART_FRAME_TYPE_ID_ARM_DAMPING:
        s_arm_mode = arm_damping;
        break;
    default:
        break;
    }
    robot_arm_mode_last = s_arm_mode;
}

/* 控制帧（0x16 / 0x17）。**只在正常运行 + run 档 + 遥控器离线**时接受（需求：
 * "只在 run 状态接受控制帧"）。 */
static void HandleCtrl(const Robot_rx_payload* pl, uint8_t len, uint8_t has_vel) {
    int i;

    if (s_state != robot_state_normal) return;
    if (s_arm_mode != arm_run) return;
    if (Rc_Online()) return;

    const uint8_t need = has_vel ? (uint8_t)sizeof(UartPktArmCtrlPosVel)
                                 : (uint8_t)sizeof(UartPktArmCtrlPos);
    if (len != need) return;

    if (has_vel) {
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            s_ctrl.joint_angle[i] = pl->arm_ctrl_pos_vel.joint_angle[i];
            s_ctrl.joint_vel[i] = pl->arm_ctrl_pos_vel.joint_vel[i];
        }
        s_ctrl.gripper_angle = pl->arm_ctrl_pos_vel.gripper_angle;
    } else {
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            s_ctrl.joint_angle[i] = pl->arm_ctrl_pos.joint_angle[i];
            s_ctrl.joint_vel[i] = 0.0f;
        }
        s_ctrl.gripper_angle = pl->arm_ctrl_pos.gripper_angle;
    }

    s_ctrl.has_vel = has_vel;
    s_ctrl.valid = 1U;
    robot_ctrl_frame_cnt++;
}

/* 一帧的分派口。**三道闸门**都在这儿，别的函数不管。 */
static void HandleFrame(uint8_t type_id, const Robot_rx_payload* pl, uint8_t len) {
    /* 闸门 ①：版本码不符之后，什么都不解析、什么都不回 */
    if (s_locked) return;

    /* 闸门 ②：握手通过之前只认版本包 —— 其他一律丢弃，也不回复 */
    if (!s_handshake_ok && type_id != UART_FRAME_TYPE_ID_VERSION) return;

    /* 能走到这里的都算"上位机还在说话"（CRC 过了；码认不认识不论 —— 认识新包的旧版本
     * 固件不该被误判成掉线）。 */
    s_last_rx_ms = HAL_GetTick();

    /* 通信离线模式：**收到任意有效帧就回离线等待**（需求原文），**不要求重新握手**
     * —— 握手是一次性的闸门，通过之后一直有效（版本码不符那种"永久锁"是另一回事）。
     * 回离线等待而不是直接回原状态，是因为掉线期间机械臂已经是 damping 了，得重新走
     * 一遍"电机全在线 → 待初始化"，上位机才知道现在能不能发初始化命令。
     *
     * 放在分派**之前**：这样本帧后面的处理看到的就是恢复后的状态（比如掉线期间发来的
     * 控制帧，会因为在 offline_wait 而不是 normal 被丢掉 —— 正是想要的）。 */
    if (s_state == robot_state_comm_lost) {
        Robot_State_Enter(robot_state_offline_wait);
    }

    switch (type_id) {
    case UART_FRAME_TYPE_ID_VERSION:
        HandleVersion(pl, len);
        break;
    case UART_FRAME_TYPE_ID_ARM_INIT:
        HandleArmInit();
        break;
    case UART_FRAME_TYPE_ID_MOTOR_SET_ZERO:
        HandleMotorSetZero();
        break;
    case UART_FRAME_TYPE_ID_ARM_START:
    case UART_FRAME_TYPE_ID_ARM_GRAVITY:
    case UART_FRAME_TYPE_ID_ARM_STOP:
    case UART_FRAME_TYPE_ID_ARM_DAMPING:
        HandleModeSwitch(type_id);
        break;
    case UART_FRAME_TYPE_ID_ARM_CTRL_POS:
        HandleCtrl(pl, len, 0U);
        break;
    case UART_FRAME_TYPE_ID_ARM_CTRL_POS_VEL:
        HandleCtrl(pl, len, 1U);
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
static void DrainRx(void) {
    for (;;) {
        Robot_rx_payload pl;
        const uint8_t head = s_rx_head;
        uint8_t type_id;
        uint8_t len;

        __DMB(); /* 先读 head，再读 payload —— 见环定义上面那段 */

        if (s_rx_tail == head) return; /* 空 */

        type_id = s_rx_slot[s_rx_tail].type_id;
        len = s_rx_slot[s_rx_tail].len;
        memcpy(&pl, &s_rx_slot[s_rx_tail].payload, sizeof(pl));
        s_rx_tail = (uint8_t)((s_rx_tail + 1U) % ROBOT_RX_SLOT_NUM);

        HandleFrame(type_id, &pl, len);
    }
}

/* ==================== 各状态的推进 ==================== */

/* 初始化流程。两步一键动作，靠 gimbal_cmd->auto_mode 驱动（和遥控器点动、调试器手工
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
static void Init_Step(void) {
    /* 遥控器一旦上线 → **判定复位中止，退回待初始化**。
     *
     * 为什么不接着跑：遥控器会覆盖 Arm_mode，一旦是 stop 档，命令层的
     * Stop_Mode_Update() 每拍清 auto_mode，正在跑的复位被打断；等遥控器再离线，
     * auto_mode 是 none —— 和"复位正常跑完"**长得一模一样**，会被误判成已完成、
     * 直接给电机使能。在这儿主动中止就没有这个歧义（计划评审时发现的洞）。 */
    if (Rc_Online()) {
        if (gimbal_cmd != NULL) {
            gimbal_cmd->auto_mode = arm_auto_none;
            gimbal_cmd->auto_step = 0U;
        }
        s_init_step = 0U;
        robot_init_abort_cnt++;
        Robot_State_Enter(robot_state_wait_init);
        return;
    }

    /* 命令层没建起来（内存不够）：跑不了初始化。停在这儿等 —— 反馈帧里 robot_state
     * 会一直显示 initing，配合 robot_state_transition_cnt 不动就能看出卡住了。 */
    if (gimbal_cmd == NULL) return;

    switch (s_init_step) {
    case 0: /* 起：一键复位 */
        gimbal_cmd->auto_step = 0U;
        gimbal_cmd->auto_mode = arm_auto_reset;
        s_init_step = 1U;
        break;

    case 1: /* 等复位跑完（Arm_Reset 的 default 分支会把 auto_mode 打回 none） */
        if (gimbal_cmd->auto_mode == arm_auto_none) {
            gimbal_cmd->auto_step = 0U;
            gimbal_cmd->auto_mode = arm_auto_gripper_zero;
            s_init_step = 2U;
        }
        break;

    case 2: /* 等夹爪找零跑完 */
        if (gimbal_cmd->auto_mode == arm_auto_none) {
            s_init_step = 0U;
            Robot_State_Enter(robot_state_normal);
        }
        break;

    default:
        s_init_step = 0U;
        break;
    }
}

static void SendZeroAck(void) {
    UartPktMotorSetZeroAck ack;
    int i;

    ack.state = arm_zero_state;
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        ack.result[i] = arm_zero_result[i];
    }
    ack.elapsed_ms = arm_zero_elapsed_ms;
    SendFrame(UART_FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK, (const uint8_t*)&ack, (uint8_t)sizeof(ack));
}

/* 校准零点：等 arm_zero 那台独立状态机跑完，回一个 0x11 ACK，退回待初始化。 */
static void Calib_Step(void) {
    /* 遥控器上线同样中止：设零期间整条总线都被关掉(见 arm_zero.h)，遥控器根本控不动，
     * 早点退出比挂着好。**不改 arm_zero_request** —— 它是 arm_zero 自己的入口，
     * 已经写下去就让它跑完（半路撤不回来，撤了反而会把电机留在失能态）。 */
    if (Rc_Online()) {
        Robot_State_Enter(robot_state_wait_init);
        return;
    }

    if (!s_zero_seen_busy) {
        if (Arm_Zero_Busy()) {
            s_zero_seen_busy = 1U;
            return;
        }
        /* 迟迟没人认领（MotorTask 没在跑？）：当失败，把请求撤掉别留下一个"以后会自己
         * 触发"的雷，然后回 ACK + 退回待初始化。 */
        if ((HAL_GetTick() - s_zero_start_ms) >= ROBOT_ZERO_START_TIMEOUT_MS) {
            arm_zero_request = 0U;
            SendZeroAck();
            Robot_State_Enter(robot_state_wait_init);
        }
        return;
    }

    if (Arm_Zero_Busy()) return; /* 还在跑 */

    /* 跑完了：DONE 还是 TIMEOUT 由 arm_zero_state 说明，原样回给上位机（它自己判）。 */
    SendZeroAck();
    Robot_State_Enter(robot_state_wait_init);
}

static void Fsm_Step(void) {
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
    if (s_handshake_ok && s_state != robot_state_wait_handshake
        && s_state != robot_state_comm_lost
        && (now - s_last_rx_ms) >= ROBOT_COMM_TIMEOUT_MS) {
        Robot_State_Enter(robot_state_comm_lost);
    }

    switch (s_state) {
    case robot_state_wait_handshake:
        /* 只有 HandleVersion() 能离开这里 */
        break;

    case robot_state_offline_wait:
        /* 7 台（含夹爪）全部上线 → 待初始化 */
        if (All_Motors_Online()) Robot_State_Enter(robot_state_wait_init);
        break;

    case robot_state_wait_init:
        /* 迁移由收帧触发（0x02 → 初始化、0x10 → 校准零点），这里没有自动迁移 */
        break;

    case robot_state_initing:
        Init_Step();
        break;

    case robot_state_normal:
        /* 模式由收帧(0x12/0x13/0x14/0x15)改，这里没有自动迁移 */
        break;

    case robot_state_comm_lost:
        /* 收到任意有效帧就回离线等待 —— 在 HandleFrame() 里做（那里才知道"收到帧"）。
         * 这里什么也不做。 */
        break;

    case robot_state_calibrating:
        Calib_Step();
        break;

    default:
        break;
    }
}

/* ==================== 250 Hz 反馈 ==================== */

static void Feedback_Step(void) {
    /* 握手通过之后**无论什么状态**都发（需求原文）。 */
    if (!s_handshake_ok || s_locked) return;

    /* 4 ms 门控 = 250 Hz。
     *
     * ⚠ 这里**用相位累加(`+= 4`)而不是 `= now`**，别改回去。本函数跑在 CmdTask 的
     * 2 ms 栅格上：4 ms 正好是 2 的整数倍，两种写法现在都对；但**这个门控值不是永远
     * 是 4** —— 一改成和 2 ms 不对齐的数（比如原来的 5 ms = 200 Hz），`= now` 就废了：
     * 相位会被每一次实际采样拖着走（0→6→12→18…，间隔恒为 6 ms），实测 1 秒只有 168 帧
     * (≈167 Hz)，而不是 200。(2026-10-06 在真机上用 JLink 注入帧 + 数 robot_fb_cnt 量的。)
     * 累加的写法对任何周期都对：落点变成 6,4,6,4…，平均正好是设定值。
     *
     * 累加的代价是"落后会越积越多"：如果任务被长时间饿住，s_fb_last_ms 会远远落在
     * now 后面，之后每一拍都会满足条件、连着补发一串。所以下面那句把落后超过一个周期
     * 的情况直接重新对齐到 now —— 丢掉的帧本来也已经晚了，补发没有意义。 */
    const uint32_t now = HAL_GetTick();
    if ((now - s_fb_last_ms) < ROBOT_FEEDBACK_PERIOD_MS) return;
    s_fb_last_ms += ROBOT_FEEDBACK_PERIOD_MS;
    if ((now - s_fb_last_ms) >= ROBOT_FEEDBACK_PERIOD_MS) {
        s_fb_last_ms = now;
    }

    UartPktArmFeedback fb;
    int i;

    /* mode 报的是**电机层真实在用的那一档**，不是我们"希望"的那一档：
     * gimbal_arm.c 的 Motor_Lost_Handle() 会在有电机掉线时把 arm_mode 打回 arm_stop，
     * 那时候上位机从这里的 mode 就能看出"我发的 run 没生效"，再对着 motor_lost_mask
     * 就知道为什么。遥控器在线时它也会反映遥控器选的档 —— 这也是有用信息。 */
    fb.mode = (gimbal != NULL) ? (uint8_t)gimbal->arm_feedback_data.mode : (uint8_t)s_arm_mode;
    fb.robot_state = (uint8_t)s_state;
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

    if (gimbal != NULL) {
        for (i = 0; i < ARM_MOTOR_NUM; i++) {
            if (gimbal->arm_feedback_data.is_motor_lost[i]) {
                fb.motor_lost_mask |= (uint8_t)(1U << i);
            }
            if (gimbal->arm_feedback_data.motor_inited[i]) {
                fb.motor_inited_mask |= (uint8_t)(1U << i);
            }
            fb.motor_temp[i] = gimbal->arm_feedback_data.motor_temp[i];
        }
        for (i = 0; i < ARM_JOINT_NUM; i++) {
            fb.joint_angle[i] = gimbal->arm_feedback_data.joint_angle[i];
            fb.joint_vel[i] = gimbal->arm_feedback_data.joint_vel[i];
        }
        fb.gripper_angle = gimbal->arm_feedback_data.gripper_angle;
    }

    SendFrame(UART_FRAME_TYPE_ID_ARM_FEEDBACK, (const uint8_t*)&fb, (uint8_t)sizeof(fb));
    robot_fb_cnt++;
}

/* ==================== 入口 ==================== */

void Robot_State_Init(void) {
    memset(s_rx_slot, 0, sizeof(s_rx_slot));
    s_rx_head = 0U;
    s_rx_tail = 0U;
    UartFrameParserReset(&s_parser);

    s_handshake_ok = 0U;
    s_locked = 0U;
    s_state = robot_state_wait_handshake;
    s_arm_mode = arm_stop;
    s_init_step = 0U;
    s_zero_seen_busy = 0U;
    s_zero_start_ms = 0U;
    Ctrl_Invalidate();

    s_last_rx_ms = HAL_GetTick();
    s_fb_last_ms = HAL_GetTick();

    robot_state_last = s_state;
    robot_arm_mode_last = s_arm_mode;
    robot_handshake_ok = 0U;
    robot_locked = 0U;

    /* 必须在 BSP_UART_Init() 之后 —— 那个函数会清空回调表，注册早了会被抹掉。 */
    BSP_UART_RegisterRxCallback(ROBOT_UART_PORT, Robot_State_OnRx);
}

void Robot_State_Loop(void) {
    DrainRx();      /* 收：中断送进来的整帧 */
    Fsm_Step();     /* 状态迁移 */
    Feedback_Step(); /* 发：250 Hz 反馈帧 */
}

Arm_mode Robot_State_Arm_Mode(void) { return s_arm_mode; }

Robot_state Robot_State_Get(void) { return s_state; }

const Robot_ctrl_cmd* Robot_State_Ctrl(void) { return &s_ctrl; }
