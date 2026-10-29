/* 收包回调写入每臂收包环，CmdTask 消费。发布槽位前执行内存屏障。 */
#include "robot_state.h"

#include <string.h>

#include "main.h" // IWYU pragma: keep

#include "arm.h"
#include "arm_protocol.h"
#include "arm_zero.h"
#include "gimbal_arm.h"
#include "gimbal_cmd.h"
#include "rc.h"
#include "frame.h"

/* 选择 USB 链路的端口及收发接口。 */
#if ROBOT_LINK == ROBOT_LINK_USB
#define ROBOT_LINK_PORT       BSP_USB_PORT
#define Robot_Link_RegRx(...) BSP_USB_RegisterRxCallback(__VA_ARGS__)
#define Robot_Link_Send(...)  BSP_USB_Send(__VA_ARGS__)
/* 选择 UART 链路的端口及收发接口。 */
#else
#define ROBOT_LINK_PORT       BSP_UART_U3
#define Robot_Link_RegRx(...) BSP_UART_RegisterRxCallback(__VA_ARGS__)
#define Robot_Link_Send(...)  BSP_UART_Send(__VA_ARGS__)
#endif

#define ROBOT_LINK_OK 0

static FrameParser s_parser;

#define ROBOT_TX_Q_NUM 8U
/* 保存一帧编码结果及其发送长度。 */

typedef struct {

    uint16_t len;
    uint8_t buf[FRAME_MAX_LEN];
} Robot_tx_slot;

_Static_assert(ROBOT_TX_Q_NUM <= 255U, "TX 队列索引是 uint8_t，槽位不能超过 255");

static Robot_tx_slot s_tx_q[ROBOT_TX_Q_NUM];
static volatile uint8_t s_tx_head;
/* 累计链路收包、解析和发送异常计数。 */
static volatile uint8_t s_tx_tail;

volatile uint32_t robot_rx_frame_cnt;
volatile uint32_t robot_rx_drop_cnt;
volatile uint32_t robot_rx_over_cnt;
volatile uint32_t robot_crc8_err_cnt;
volatile uint32_t robot_crc16_err_cnt;
volatile uint32_t robot_tx_fail_cnt;
volatile uint32_t robot_rx_bad_arm_cnt;
volatile uint32_t robot_gated_cnt;
volatile uint32_t robot_tid_cnt[FRAME_TID_SLOT_NUM];

/* 定义下行报文类型对应的统计槽位顺序。 */
static const uint8_t s_tid_order[FRAME_TID_DOWN_NUM] = { FRAME_TID_DOWN_LIST }; // NOLINT(readability-identifier-naming)

_Static_assert(sizeof(s_tid_order) == FRAME_TID_DOWN_NUM, "分类表长度对不上");
_Static_assert(FRAME_TID_SLOT_NUM == FRAME_TID_DOWN_NUM + 1U, "槽位数该是分类数 + 1(其他)");

/* 将报文类型映射到分类统计索引。 */
static uint8_t Tid_Index(uint8_t tid) { // NOLINT(readability-identifier-naming)
    uint8_t i;

    /* 在已登记的下行类型表中查找匹配项。 */
    for (i = 0U; i < (uint8_t)FRAME_TID_DOWN_NUM; i++) {
        /* 类型匹配时返回对应统计索引。 */
        if (s_tid_order[i] == tid) return i;
    }
    return (uint8_t)FRAME_TID_DOWN_NUM;
}

_Static_assert(sizeof(FramePktArmFeedback) == 63U, "反馈帧不是 63 字节");

_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(FramePktArmCtrlPosVel), "环槽放不下 0x17 包");
_Static_assert(ROBOT_RX_DATA_LEN >= sizeof(FramePktMotorSetZeroAck), "环槽放不下 0x11 包");

/* 编码报文并写入共享发送环形队列。 */
static void TxEnqueue(Arm* arm, uint8_t type_id, const uint8_t* data, uint8_t len) { // NOLINT(readability-identifier-naming)
    const uint8_t next = (uint8_t)((s_tx_head + 1U) % ROBOT_TX_Q_NUM);
    /* 发送队列已满时累计失败并结束入队。 */
    if (next == s_tx_tail) {

        robot_tx_fail_cnt++;
        return;
    }

    const int32_t n = FrameEncode(s_tx_q[s_tx_head].buf, (uint32_t)FRAME_MAX_LEN, type_id,
        arm->id, data, len);
    /* 帧编码失败时累计失败并结束入队。 */
    if (n <= 0) {
        robot_tx_fail_cnt++;
        return;
    }
    /* 保存编码长度并发布新的发送队列头索引。 */
    s_tx_q[s_tx_head].len = (uint16_t)n;
    s_tx_head = next;
}

/* 握手完成后将业务报文加入发送队列。 */
static void SendFrame(Arm* arm, uint8_t type_id, const uint8_t* data, uint8_t len) { // NOLINT(readability-identifier-naming)

    /* 握手尚未完成时保持发送门控。 */
    if (!arm->handshake_ok) return;

    TxEnqueue(arm, type_id, data, len);
}

/* 按入队顺序提交链路报文。 */
static void Link_DrainTx(void) { // NOLINT(readability-identifier-naming)
    /* 持续发送队列中的待发报文。 */
    while (s_tx_tail != s_tx_head) {
        const Robot_tx_slot* const slot = &s_tx_q[s_tx_tail];
        /* 链路暂时无法发送时保留当前槽位。 */
        if (Robot_Link_Send(ROBOT_LINK_PORT, slot->buf, (uint16_t)slot->len) != ROBOT_LINK_OK) {
            robot_tx_fail_cnt++;
            return;
        }
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % ROBOT_TX_Q_NUM);
    }
}

/* 将本机版本码加入发送队列。 */
static void SendVersionRaw(Arm* arm) { // NOLINT(readability-identifier-naming)
    FramePktVersion pkt;
    pkt.code = FrameVersionCode();
    TxEnqueue(arm, FRAME_TYPE_ID_VERSION, (const uint8_t*)&pkt, (uint8_t)sizeof(pkt));
}

/* 握手完成后发送版本应答。 */
static void SendVersion(Arm* arm) { // NOLINT(readability-identifier-naming)
    /* 握手尚未完成时保持版本应答门控。 */
    if (!arm->handshake_ok) return;
    SendVersionRaw(arm);
}

/* 逐字节解析链路数据并发布到对应机械臂收包环。 */
static void Robot_State_OnRx(uint8_t id, const uint8_t* data, uint16_t len) { // NOLINT(readability-identifier-naming)
    uint16_t i;

    /* 回调端口不匹配时结束处理。 */
    if (id != (uint8_t)ROBOT_LINK_PORT) return;

    /* 逐字节推进共享帧解析器。 */
    for (i = 0U; i < len; i++) {
        const FrameStatus st = FrameParserFeed(&s_parser, data[i]);

        /* 收到合法完整帧后更新统计并检查目标臂。 */
        if (st == FRAME_OK) {
            robot_rx_frame_cnt++;

            robot_tid_cnt[Tid_Index(s_parser.type_id)]++;

            /* 目标臂索引越界时累计异常并丢弃该帧。 */
            if (s_parser.arm_id >= (uint8_t)ARM_NUM) {
                robot_rx_bad_arm_cnt++;
                continue;
            }

            /* 根据目标臂选择收包环并检查可用槽位。 */
            {
                Arm* const arm = &g_arm[s_parser.arm_id];

                const uint8_t next = (uint8_t)((arm->rx_head + 1U) % ROBOT_RX_SLOT_NUM);

                /* 载荷超过槽位容量时累计超长计数。 */
                if (s_parser.data_len > ROBOT_RX_DATA_LEN) {
                    robot_rx_over_cnt++;
                } else if (next == arm->rx_tail) { /* 收包环已满时累计丢帧计数。 */

                    robot_rx_drop_cnt++;
                } else { /* 复制报文后用内存屏障发布队列头索引。 */
                    arm->rx_slot[arm->rx_head].type_id = s_parser.type_id;
                    arm->rx_slot[arm->rx_head].len = s_parser.data_len;
                    memcpy(arm->rx_slot[arm->rx_head].payload.raw,
                        &s_parser.buf.raw[FRAME_HEADER_LEN], s_parser.data_len);
                    __DMB();
                    arm->rx_head = next;
                }
            }

        } else if (st == FRAME_ERR_CRC8) { /* 帧头校验失败时累计 CRC8 错误。 */
            robot_crc8_err_cnt++;
        } else if (st == FRAME_ERR_CRC16) { /* 整帧校验失败时累计 CRC16 错误。 */
            robot_crc16_err_cnt++;
        } else { /* 帧尚未完整时等待后续字节。 */

        }
    }
}

/* 清除协议目标及速度目标的有效标志。 */
static void Ctrl_Invalidate(Arm* arm) { // NOLINT(readability-identifier-naming)
    arm->ctrl.valid = 0U;
    arm->ctrl.has_vel = 0U;
}

/* 执行状态退出处理并设置目标状态的控制模式。 */
static void Robot_State_Enter(Arm* arm, Robot_state next) { // NOLINT(readability-identifier-naming)
    /* 离开初始化状态时调用机型清理回调。 */
    if (arm->state == robot_state_initing && next != robot_state_initing
        && arm->gimbal_cmd && arm->gimbal->profile->cancel_initialize)
        arm->gimbal->profile->cancel_initialize(arm->gimbal_cmd);
    /* 清除夹爪初始化动作的输出接管标志。 */
    if (arm->gimbal_cmd) arm->gimbal_cmd->initialization.home_active = 0;
    Ctrl_Invalidate(arm);

    arm->fb_last_ms = HAL_GetTick() - ROBOT_FEEDBACK_PERIOD_MS;

    /* 按目标状态配置机械臂工作模式。 */
    switch (next) {
    /* 握手等待状态使用停止模式。 */
    case robot_state_wait_handshake: // NOLINT(bugprone-branch-clone)
        arm->arm_mode = arm_stop;
        break;
    /* 电机上线等待状态使用停止模式。 */
    case robot_state_offline_wait:
        arm->arm_mode = arm_stop;
        break;
    /* 初始化入口状态使用停止模式。 */
    case robot_state_wait_init:
        arm->arm_mode = arm_stop;
        break;
    /* 初始化动作使用运行模式。 */
    case robot_state_initing:

        arm->arm_mode = arm_run;
        break;
    /* 正常状态入口使用阻尼模式。 */
    case robot_state_normal: // NOLINT(bugprone-branch-clone)

        arm->arm_mode = arm_damping;
        break;
    /* 通信丢失状态使用阻尼模式。 */
    case robot_state_comm_lost:
        arm->arm_mode = arm_damping;
        break;
    /* 设零状态使用停止模式。 */
    case robot_state_calibrating:
        arm->arm_mode = arm_stop;
        break;
    /* 复位动作使用运行模式。 */
    case robot_state_resetting:

        arm->arm_mode = arm_run;
        break;
    /* 其余状态保持当前工作模式。 */
    default:
        break;
    }

    /* 发布状态和模式快照并累计迁移次数。 */
    arm->state = next;
    arm->state_last = next;
    arm->arm_mode_last = arm->arm_mode;
    arm->transition_cnt++;
}

/* 检查该臂全部电机是否在线。 */
static uint8_t All_Motors_Online(Arm* arm) { // NOLINT(readability-identifier-naming)
    int i;
    /* 控制对象尚未创建时报告未就绪。 */
    if (arm->gimbal == NULL) return 0U;
    /* 逐电机检查掉线标志。 */
    for (i = 0; i < ARM_MOTOR_NUM; i++) {
        /* 任一电机掉线时报告未就绪。 */
        if (arm->gimbal->arm_feedback_data.is_motor_lost[i]) return 0U;
    }
    return 1U;
}

/* 检查当前姿态是否满足初始化入口容差。 */
static uint8_t Init_Pose_Ok(Arm* arm) { // NOLINT(readability-identifier-naming)
    int i;
    /* 控制对象尚未创建时报告未就绪。 */
    if (arm->gimbal == NULL) return 0U;

    /* 逐关节比较反馈角与配置复位角。 */
    for (i = 0; i < ARM_JOINT_NUM; i++) {
        float d = arm->gimbal->arm_feedback_data.joint_angle[i] - arm->gimbal->profile->joints[i].reset_deg;
        /* 将关节角差换算为绝对误差。 */
        if (d < 0.0f) d = -d;
        /* 任一关节误差超限时拒绝初始化入口。 */
        if (d > arm->gimbal->profile->init.pose_tolerance_deg) return 0U;
    }
    return 1U;
}

/* 读取遥控器在线状态。 */
static uint8_t Rc_Online(void) { return (uint8_t)(!is_Offline(rc.monitor)); } // NOLINT(readability-identifier-naming)

/* 校验版本报文并完成对应机械臂的握手。 */
static void HandleVersion(Arm* arm, const Robot_rx_payload* pl, uint8_t len) { // NOLINT(readability-identifier-naming)
    /* 版本载荷长度不匹配时丢弃报文。 */
    if (len != (uint8_t)sizeof(FramePktVersion)) return;

    /* 版本码不匹配时累计握手拒绝次数。 */
    if (pl->version.code != FrameVersionCode()) {

        arm->handshake_reject_cnt++;
        return;
    }

    /* 首次握手成功后进入电机上线等待状态。 */
    if (!arm->handshake_ok) {
        arm->handshake_ok = 1U;
        arm->handshake_ok_obs = 1U;
        arm->handshake_cnt++;

        SendVersion(arm);
        Robot_State_Enter(arm, robot_state_offline_wait);
        return;
    }

    SendVersion(arm);
}

/* 检查初始化入口并保存夹爪反馈目标。 */
static void HandleArmInit(Arm* arm) { // NOLINT(readability-identifier-naming)

    const uint8_t from_wait_init = (arm->state == robot_state_wait_init);
    const uint8_t from_normal_soft =
        (arm->state == robot_state_normal
         && (arm->arm_mode == arm_stop || arm->arm_mode == arm_damping));
    /* 状态不满足初始化入口条件时结束处理。 */
    if (!from_wait_init && !from_normal_soft) return;

    /* 遥控器接管时拒绝协议初始化请求。 */
    if (Rc_Online()) {
        arm->init_reject_cnt++;
        return;
    }
    /* 电机未全部在线时拒绝初始化请求。 */
    if (!All_Motors_Online(arm)) {
        arm->init_reject_cnt++;
        return;
    }
    /* 姿态超出入口容差时拒绝初始化请求。 */
    if (!Init_Pose_Ok(arm)) {

        arm->init_reject_cnt++;
        return;
    }

    /* 保存初始化入口状态并锁定入口夹爪角。 */
    arm->init_entry = arm->state;
    arm->init_step = 0U;

    arm->gimbal->arm_control_data.gripper_angle = arm->gimbal->arm_feedback_data.gripper_angle;
    Robot_State_Enter(arm, robot_state_initing);
}

/* 检查复位入口并进入运行复位状态。 */
static void HandleArmReset(Arm* arm) { // NOLINT(readability-identifier-naming)
    /* 正常状态以外拒绝复位请求。 */
    if (arm->state != robot_state_normal) {
        arm->reset_reject_cnt++;
        return;
    }
    /* 遥控器接管时拒绝协议复位请求。 */
    if (Rc_Online()) {
        arm->reset_reject_cnt++;
        return;
    }
    /* 电机未全部在线时拒绝复位请求。 */
    if (!All_Motors_Online(arm)) {

        arm->reset_reject_cnt++;
        return;
    }

    arm->reset_step = 0U;
    Robot_State_Enter(arm, robot_state_resetting);
}

/* 在初始化入口状态发布电机设零请求。 */
static void HandleMotorSetZero(Arm* arm) { // NOLINT(readability-identifier-naming)

    /* 状态不满足设零入口条件时结束处理。 */
    if (arm->state != robot_state_wait_init) return;
    /* 遥控器接管时保持当前状态。 */
    if (Rc_Online()) return;

    arm->zero_seen_busy = 0U;
    arm->zero_start_ms = HAL_GetTick();

    arm->gimbal->zero.request = 1U;
    Robot_State_Enter(arm, robot_state_calibrating);
}

/* 根据协议报文切换正常状态下的控制模式。 */
static void HandleModeSwitch(Arm* arm, uint8_t type_id) { // NOLINT(readability-identifier-naming)

    /* 正常状态以外保持当前控制模式。 */
    if (arm->state != robot_state_normal) return;
    /* 遥控器接管时保持当前控制模式。 */
    if (Rc_Online()) return;

    Ctrl_Invalidate(arm);

    /* 将协议类型映射到运行模式。 */
    switch (type_id) {
    /* 启动报文选择运行模式。 */
    case FRAME_TYPE_ID_ARM_START:
        arm->arm_mode = arm_run;
        break;
    /* 重力补偿报文选择重力模式。 */
    case FRAME_TYPE_ID_ARM_GRAVITY:
        arm->arm_mode = arm_only_gravity_comp;
        break;
    /* 停止报文选择停止模式。 */
    case FRAME_TYPE_ID_ARM_STOP:
        arm->arm_mode = arm_stop;
        break;
    /* 阻尼报文选择阻尼模式。 */
    case FRAME_TYPE_ID_ARM_DAMPING:
        arm->arm_mode = arm_damping;
        break;
    /* 其余报文保持当前控制模式。 */
    default:
        break;
    }
    arm->arm_mode_last = arm->arm_mode;
}

/* 校验正常运行状态下的关节控制载荷。 */
static void HandleCtrl(Arm* arm, const Robot_rx_payload* pl, uint8_t len, uint8_t has_vel) { // NOLINT(readability-identifier-naming)

    /* 正常状态以外丢弃运动目标。 */
    if (arm->state != robot_state_normal) return;
    /* 运行模式以外丢弃运动目标。 */
    if (arm->arm_mode != arm_run) return;
    /* 遥控器接管时丢弃协议运动目标。 */
    if (Rc_Online()) return;

    const uint8_t need = has_vel ? (uint8_t)sizeof(FramePktArmCtrlPosVel)
                                 : (uint8_t)sizeof(FramePktArmCtrlPos);
    /* 载荷长度不匹配时丢弃运动目标。 */
    if (len != need) return;

    /* 控制对象或载荷无效时丢弃运动目标。 */
    if (!arm->gimbal || !Arm_Protocol_Control(arm, pl, has_vel)) return;
    arm->ctrl_frame_cnt++;
}

/* 检查握手门控并按报文类型分派业务处理。 */
static void HandleFrame(Arm* arm, uint8_t type_id, const Robot_rx_payload* pl, uint8_t len) { // NOLINT(readability-identifier-naming)

    /* 握手完成前仅受理版本报文。 */
    if (!arm->handshake_ok && type_id != FRAME_TYPE_ID_VERSION) {
        robot_gated_cnt++;
        return;
    }

    arm->last_rx_ms = HAL_GetTick();

    /* 通信恢复后重新检查电机在线状态。 */
    if (arm->state == robot_state_comm_lost) {
        Robot_State_Enter(arm, robot_state_offline_wait);
    }

    /* 按类型调用握手、模式或动作处理入口。 */
    switch (type_id) {
    /* 处理版本校验和握手应答。 */
    case FRAME_TYPE_ID_VERSION:
        HandleVersion(arm, pl, len);
        break;
    /* 处理机械臂初始化请求。 */
    case FRAME_TYPE_ID_ARM_INIT:
        HandleArmInit(arm);
        break;
    /* 处理电机设零请求。 */
    case FRAME_TYPE_ID_MOTOR_SET_ZERO:
        HandleMotorSetZero(arm);
        break;
    /* 将启动请求交给模式切换入口。 */
    case FRAME_TYPE_ID_ARM_START:
    /* 将重力请求交给模式切换入口。 */
    case FRAME_TYPE_ID_ARM_GRAVITY:
    /* 将停止请求交给模式切换入口。 */
    case FRAME_TYPE_ID_ARM_STOP:
    /* 将阻尼请求交给模式切换入口。 */
    case FRAME_TYPE_ID_ARM_DAMPING:
        HandleModeSwitch(arm, type_id);
        break;
    /* 校验并接收关节位置目标。 */
    case FRAME_TYPE_ID_ARM_CTRL_POS:
        HandleCtrl(arm, pl, len, 0U);
        break;
    /* 校验并接收关节位置和速度目标。 */
    case FRAME_TYPE_ID_ARM_CTRL_POS_VEL:
        HandleCtrl(arm, pl, len, 1U);
        break;
    /* 处理机械臂复位请求。 */
    case FRAME_TYPE_ID_ARM_RESET:
        HandleArmReset(arm);
        break;
    /* 其余报文结束本次业务处理。 */
    default:

        break;
    }
}

/* 消费单臂收包环中的全部待处理报文。 */
static void DrainRx(Arm* arm) { // NOLINT(readability-identifier-naming)
    /* 循环提取载荷并更新收包环尾索引。 */
    for (;;) {
        Robot_rx_payload pl;
        const uint8_t head = arm->rx_head;
        uint8_t type_id;
        uint8_t len;

        __DMB();

        /* 队列为空时结束本周期消费。 */
        if (arm->rx_tail == head) return;

        type_id = arm->rx_slot[arm->rx_tail].type_id;
        len = arm->rx_slot[arm->rx_tail].len;
        memcpy(&pl, &arm->rx_slot[arm->rx_tail].payload, sizeof(pl));
        arm->rx_tail = (uint8_t)((arm->rx_tail + 1U) % ROBOT_RX_SLOT_NUM);

        HandleFrame(arm, type_id, &pl, len);
    }
}

/* 推进机型初始化并处理接管、掉线和完成结果。 */
static void Init_Step(Arm* arm) { // NOLINT(readability-identifier-naming)

    /* 遥控器接管或电机掉线时中止初始化。 */
    if (Rc_Online() || (!Arm_Zero_Busy(arm->gimbal) && !All_Motors_Online(arm))) {
        /* 命令对象有效时清除自动动作状态。 */
        if (arm->gimbal_cmd != NULL) {
            arm->gimbal_cmd->auto_mode = arm_auto_none;
            arm->gimbal_cmd->auto_step = 0U;
        }
        arm->init_step = 0U;
        arm->init_abort_cnt++;

        Robot_State_Enter(arm, arm->init_entry);
        return;
    }

    /* 命令对象尚未创建时等待后续周期。 */
    if (arm->gimbal_cmd == NULL) return;

    /* 首周期启动机型策略，后续周期继续推进。 */
    const uint8_t start = arm->init_step == 0;
    arm->init_step = 1;
    /* 通过机型回调获取等待、完成或失败结果。 */
    const int result = arm->gimbal->profile->initialize(arm->gimbal_cmd, start);
    /* 初始化回调返回完成结果后执行状态迁移。 */
    if (result != 0) {
        arm->gimbal_cmd->initialization.home_active = 0;
        arm->init_step = 0;
        /* 初始化失败时累计中止次数并清零夹爪力矩。 */
        if (result < 0) {
            arm->init_abort_cnt++;
            arm->gimbal->arm_control_data.gripper_torque = 0;
        }
        Robot_State_Enter(arm, result > 0 ? robot_state_normal : arm->init_entry);
    }
}

/* 推进运行复位并处理遥控器接管。 */
static void Reset_Step(Arm* arm) { // NOLINT(readability-identifier-naming)

    /* 遥控器接管时中止复位并返回正常状态。 */
    if (Rc_Online()) {
        /* 命令对象有效时清除自动动作状态。 */
        if (arm->gimbal_cmd != NULL) {
            arm->gimbal_cmd->auto_mode = arm_auto_none;
            arm->gimbal_cmd->auto_step = 0U;
        }
        arm->reset_step = 0U;
        arm->reset_abort_cnt++;
        Robot_State_Enter(arm, robot_state_normal);
        return;
    }

    /* 命令对象尚未创建时等待后续周期。 */
    if (arm->gimbal_cmd == NULL) return;

    /* 按复位步骤启动动作或等待结束。 */
    switch (arm->reset_step) {
    /* 清除失败标志并启动关节自动复位。 */
    case 0:
        arm->gimbal_cmd->auto_step = 0U;
        arm->gimbal_cmd->auto_failed = 0U;
        arm->gimbal_cmd->auto_mode = arm_auto_reset;
        arm->reset_step = 1U;
        break;

    /* 等待自动动作退出复位模式。 */
    case 1:
        /* 自动动作结束后返回正常阻尼状态。 */
        if (arm->gimbal_cmd->auto_mode == arm_auto_none) {
            /* 规划失败时累计复位中止次数。 */
            if (arm->gimbal_cmd->auto_failed) arm->reset_abort_cnt++;
            arm->reset_step = 0U;
            Robot_State_Enter(arm, robot_state_normal);
        }
        break;

    /* 步骤索引异常时重置复位流程。 */
    default:
        arm->reset_step = 0U;
        break;
    }

}

/* 编码并发送电机设零结果应答。 */
static void SendZeroAck(Arm* arm) { // NOLINT(readability-identifier-naming)
    FramePktMotorSetZeroAck ack;

    Arm_Protocol_Zero_Result(arm->gimbal, &ack);
    SendFrame(arm, FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK, (const uint8_t*)&ack, (uint8_t)sizeof(ack));
}

/* 等待设零任务启动、结束或启动超时。 */
static void Calib_Step(Arm* arm) { // NOLINT(readability-identifier-naming)

    /* 遥控器接管时退出设零等待状态。 */
    if (Rc_Online()) {
        Robot_State_Enter(arm, robot_state_wait_init);
        return;
    }

    /* 设零尚未启动时监测忙标志和启动时限。 */
    if (!arm->zero_seen_busy) {
        /* 观察到忙标志后转入完成等待。 */
        if (Arm_Zero_Busy(arm->gimbal)) {
            arm->zero_seen_busy = 1U;
            return;
        }

        /* 启动等待超时后清除请求并反馈结果。 */
        if ((HAL_GetTick() - arm->zero_start_ms) >= ROBOT_ZERO_START_TIMEOUT_MS) {
            arm->gimbal->zero.request = 0U;
            SendZeroAck(arm);
            Robot_State_Enter(arm, robot_state_wait_init);
        }
        return;
    }

    /* 设零仍忙时继续等待下一周期。 */
    if (Arm_Zero_Busy(arm->gimbal)) return;

    /* 设零结束后发送结果并返回初始化入口。 */
    SendZeroAck(arm);
    Robot_State_Enter(arm, robot_state_wait_init);
}

/* 按当前业务状态执行通信超时处理。 */
static void Comm_Lost_Enter(Arm* arm) { // NOLINT(readability-identifier-naming)
    /* 正常状态以外进入通信丢失状态。 */
    if (arm->state != robot_state_normal) {
        Robot_State_Enter(arm, robot_state_comm_lost);
        return;
    }
    /* 正常状态已处于阻尼时保持当前状态。 */
    if (arm->arm_mode == arm_damping) return;
    Robot_State_Enter(arm, robot_state_normal);
}

/* 检查通信超时并推进当前状态的周期动作。 */
static void Fsm_Step(Arm* arm) { // NOLINT(readability-identifier-naming)
    const uint32_t now = HAL_GetTick();

    /* 已握手机械臂超时后执行通信丢失处理。 */
    if (arm->handshake_ok && arm->state != robot_state_wait_handshake
        && arm->state != robot_state_comm_lost
        && (now - arm->last_rx_ms) >= ROBOT_COMM_TIMEOUT_MS) {
        Comm_Lost_Enter(arm);
    }

    /* 按机械臂状态选择周期推进流程。 */
    switch (arm->state) {
    /* 握手等待状态由握手定时流程推进。 */
    case robot_state_wait_handshake:

        break;

    /* 电机上线等待状态检查全部在线条件。 */
    case robot_state_offline_wait:

        /* 全部电机在线后进入初始化入口状态。 */
        if (All_Motors_Online(arm)) Robot_State_Enter(arm, robot_state_wait_init);
        break;

    /* 初始化入口状态等待上位机请求。 */
    case robot_state_wait_init:

        break;

    /* 初始化状态推进机型初始化策略。 */
    case robot_state_initing:
        Init_Step(arm);
        break;

    /* 正常状态等待模式和控制报文。 */
    case robot_state_normal: // NOLINT(bugprone-branch-clone)

        break;

    /* 通信丢失状态等待有效报文恢复链路。 */
    case robot_state_comm_lost:

        break;

    /* 设零状态推进设零完成检查。 */
    case robot_state_calibrating:
        Calib_Step(arm);
        break;

    /* 复位状态推进自动动作完成检查。 */
    case robot_state_resetting:
        Reset_Step(arm);
        break;

    /* 其余状态保持当前状态。 */
    default:
        break;
    }
}

/* 按反馈周期编码并发送单臂状态。 */
static void Feedback_Step(Arm* arm) { // NOLINT(readability-identifier-naming)

    /* 握手尚未完成时等待反馈发送条件。 */
    if (!arm->handshake_ok) return;

    const uint32_t now = HAL_GetTick();
    /* 反馈周期未到时等待下一次调用。 */
    if ((now - arm->fb_last_ms) < ROBOT_FEEDBACK_PERIOD_MS) return;
    arm->fb_last_ms += ROBOT_FEEDBACK_PERIOD_MS;
    /* 反馈落后多个周期时重新对齐时间基准。 */
    if ((now - arm->fb_last_ms) >= ROBOT_FEEDBACK_PERIOD_MS) {
        arm->fb_last_ms = now;
    }

    FramePktArmFeedback fb;

    Arm_Protocol_Feedback(arm, &fb);

    SendFrame(arm, FRAME_TYPE_ID_ARM_FEEDBACK, (const uint8_t*)&fb, (uint8_t)sizeof(fb));
    arm->fb_cnt++;
}

/* 按握手周期主动发送版本报文。 */
static void Handshake_Step(Arm* arm) { // NOLINT(readability-identifier-naming)
    /* 握手已完成时结束握手重试。 */
    if (arm->handshake_ok) return;

    const uint32_t now = HAL_GetTick();

    /* 握手重试周期未到时继续等待。 */
    if ((now - arm->hs_last_ms) < ROBOT_HANDSHAKE_PERIOD_MS) return;
    arm->hs_last_ms = now;

    SendVersionRaw(arm);
}

/* 保存链路统计基线及上次上报时间。 */
static uint32_t s_ls_rx_total;
static uint32_t s_ls_crc8;
static uint32_t s_ls_crc16;
static uint32_t s_ls_bad_arm;
static uint32_t s_ls_over;
static uint32_t s_ls_ring_drop;
static uint32_t s_ls_gated;
static uint32_t s_ls_tx_fail;
static uint32_t s_ls_bytes_drop;
static uint32_t s_ls_tid[FRAME_TID_SLOT_NUM];
static uint32_t s_ls_last_ms;
static uint8_t s_ls_started;

/* 记录本周期链路计数器快照。 */
static void Link_Status_Take_Snapshot(void) { // NOLINT(readability-identifier-naming)
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

    /* 逐类型保存报文计数基线。 */
    for (i = 0U; i < (uint8_t)FRAME_TID_SLOT_NUM; i++) {
        s_ls_tid[i] = robot_tid_cnt[i];
    }
}

/* 汇总链路统计增量并经已握手机械臂上报。 */
static void Link_Status_Step(void) { // NOLINT(readability-identifier-naming)

    Arm* arm = NULL;
    uint8_t i;

    /* 查找可承载链路统计报文的机械臂。 */
    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        /* 选中首个已完成握手的机械臂。 */
        if (g_arm[i].handshake_ok) {
            arm = &g_arm[i];
            break;
        }
    }
    /* 没有已握手机械臂时复位上报起始标志。 */
    if (arm == NULL) {
        s_ls_started = 0U;
        return;
    }

    const uint32_t now = HAL_GetTick();

    /* 首次具备上报条件时记录计数基线。 */
    if (s_ls_started == 0U) {

        Link_Status_Take_Snapshot();
        s_ls_last_ms = now;
        s_ls_started = 1U;
        return;
    }

    /* 上报周期未到时继续累计计数。 */
    if ((now - s_ls_last_ms) < ROBOT_LINK_STATUS_PERIOD_MS) return;

    FramePktLinkStatus st;
    const uint8_t n = (uint8_t)FRAME_TID_SLOT_NUM;

    /* 计算接收、校验、门控及发送异常计数增量。 */
    st.rx_total = robot_rx_frame_cnt - s_ls_rx_total;
    st.crc8_err = robot_crc8_err_cnt - s_ls_crc8;
    st.crc16_err = robot_crc16_err_cnt - s_ls_crc16;
    st.bad_arm = robot_rx_bad_arm_cnt - s_ls_bad_arm;
    st.over_len = robot_rx_over_cnt - s_ls_over;
    st.ring_drop = robot_rx_drop_cnt - s_ls_ring_drop;
    st.gated = robot_gated_cnt - s_ls_gated;
    st.tx_fail = robot_tx_fail_cnt - s_ls_tx_fail;

    st.drop_bytes = s_parser.bytes_drop - s_ls_bytes_drop;
    st.dt_ms = now - s_ls_last_ms;

    /* 计算各报文类型在本周期内的接收增量。 */
    for (i = 0U; i < n; i++) {
        st.rx_by_tid[i] = robot_tid_cnt[i] - s_ls_tid[i];
    }

    Link_Status_Take_Snapshot();
    s_ls_last_ms = now;

    /* 通过选中的已握手机械臂发送共享链路状态。 */
    SendFrame(arm, FRAME_TYPE_ID_LINK_STATUS, (const uint8_t*)&st, (uint8_t)sizeof(st));
}

/* 初始化各臂状态、报文队列及链路统计。 */
void Robot_State_Init(void) {
    uint8_t i;

    /* 逐臂建立停止状态、握手时基和收包队列。 */
    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        Arm* const arm = &g_arm[i];

        arm->id = i;

        /* 清空该臂收包环和读写索引。 */
        memset(arm->rx_slot, 0, sizeof(arm->rx_slot));
        arm->rx_head = 0U;
        arm->rx_tail = 0U;

        /* 建立握手等待和停止模式的初始状态。 */
        arm->handshake_ok = 0U;
        arm->state = robot_state_wait_handshake;
        arm->arm_mode = arm_stop;
        arm->init_step = 0U;

        arm->init_entry = robot_state_wait_handshake;
        arm->reset_step = 0U;
        arm->zero_seen_busy = 0U;
        arm->zero_start_ms = 0U;
        Ctrl_Invalidate(arm);

        /* 初始化接收、反馈和握手的调度时基。 */
        arm->last_rx_ms = HAL_GetTick();
        arm->fb_last_ms = HAL_GetTick();

        arm->hs_last_ms = HAL_GetTick() - ROBOT_HANDSHAKE_PERIOD_MS;

        arm->state_last = arm->state;
        arm->arm_mode_last = arm->arm_mode;
        arm->handshake_ok_obs = 0U;
    }

    /* 复位共享解析器、发送队列和统计基线。 */
    FrameParserReset(&s_parser);
    s_tx_head = 0U;
    s_tx_tail = 0U;

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

    /* 注册选定链路的接收回调。 */
    Robot_Link_RegRx(ROBOT_LINK_PORT, Robot_State_OnRx);
}

/* 消费各臂报文并推进状态、握手和反馈。 */
void Robot_State_Loop(void) {
    uint8_t i;

    /* 逐臂执行本周期通信状态处理。 */
    for (i = 0U; i < (uint8_t)ARM_NUM; i++) {
        Arm* const arm = &g_arm[i];
        /* 跳过控制对象尚未创建的机械臂。 */
        if (arm->gimbal == NULL) continue;

        DrainRx(arm);
        Fsm_Step(arm);
        Handshake_Step(arm);
        Feedback_Step(arm);
    }

    /* 汇总并调度共享链路统计报文。 */
    Link_Status_Step();

    /* 依次提交当前发送队列中的报文。 */
    Link_DrainTx();
}

/* 返回指定机械臂的协议控制目标。 */
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
