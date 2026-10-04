#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "monitor.h"
#include "controller.h"
#include "bsp_supervise.h"
#include "encos_protocol.h"
#include <stdint.h>

/* 源工程的依赖，本工程这样补（不是搞 compat 头，是这两个根本不用引）：
 *   board_def.h —— 只为 DEVICE_CAN_CNT。那个宏在 libs/can/can_config.h 里就有，
 *                  而 bsp_can.h 已经 include 了 can_config.h，走上面那行就有了。
 *   bsp_time.h  —— 只为 BSP_sys_time_ms()，那函数体就一行 return HAL_GetTick()。
 *                  换成 HAL_GetTick()，见 ENCOS_motor.c 的 now_ms()。 */

/* ENCOS V1.20EAP driver.
 * Formerly published under MT4 names; API renamed to ENCOS_motor / ENCOSmotor_*.
 * All ranges MUST match the motor's actual stored protocol settings.
 * No automatic motor configuration, zeroing, brake release or ID changes.
 * APIs/Rx/Calc_Send must be serialized by the caller (see README_ENCOS.md).
 */
struct encos_motor_t;
typedef void (*control_encos_func)(struct encos_motor_t *motor);
typedef float (*calc_fdb_func_encos)(void *arg);
typedef enum { encos_stop, encos_enable } ENCOS_enable;
typedef enum { encos_output_normal, encos_output_reverse } ENCOS_output_mode;
typedef enum { encos_soft_stop, encos_damping_stop } ENCOS_stop_mode;

/* 2026-10-05：未使能态("停着")给电机发哪种帧 —— 实测结论是这一条决定 j3 认不认我们。
 *   1 = 发 8 字节零增益 mode-0 混控帧（厂商 SDK 空闲时的做法）；
 *   0 = 发 3 字节零电流 mode-3 停帧（原来的实现，`61 00 00`）。
 * 真机实测（2026-10-05 在 CAN1 上用一次性探针量的，探针已删）：
 *   - j3(MT4) 只认 8 字节 mode-0 混控帧的**帧流**；3 字节停帧、2 字节查询帧它当废帧，
 *     一帧都不回。厂商 SDK 对 MT4 从头到尾也只发 8 字节帧，同一个道理。
 *   - 只发混控帧（没有短帧掺和）时，帧流够密它就按查询帧持续回型号 5 应答。
 * Kp/Kd 恒为 0：手册 §3.1 `T_ref = Kp·e_p + Kd·e_v + T_des` ⇒ 扭矩恒 0，不出力。
 * 注意手册同一段提醒"全零 **不等于** 驱动器失能"——这不是机械抱闸，只是不出力。
 * damping_stop 仍然走原来的 mode-3 阻尼刹车，不受本宏影响。 */
#define ENCOS_IDLE_USE_MIXED_FRAME 1

typedef struct encos_motor_config_t {
    uint8_t bsp_can_index;
    uint16_t can_tx_id, can_rx_id; /* 1..0x7FE; must be equal */
    controller_config motor_controller_config;
    calc_fdb_func_encos pos_fdb_calc, speed_fdb_calc;
    void *arg;
    lost_callback lost_callback_;
    ENCOS_output_mode output_mode;
    /* 发送分组掩码(见 libs/can/can_config.h 的 CAN_TX_xxxHZ_Gn)：
     * 这一路控制帧属于哪些组。总线只在"当前组"匹配时才真的发出去，
     * 于是**控制频率由掩码决定**——每 1ms 调一次 Calc_Send、传 CAN_TX_500HZ_G0
     * 就是 500 Hz，不需要再在驱动里分频。0 会被兜底成 CAN_TX_GROUP_ALL
     * (不管控位掩码是"永远发不出去"，太难查)。 */
    uint8_t tx_group;
    ENCOS_Ranges ranges; /* REQUIRED: kp/kd/position/speed/torque/current min/max */
    float torque_constant; /* Nm/A; 0 = unknown, estimated torque is invalid */
    float position_offset; /* rad; host coordinate = motor position + offset */
    uint32_t feedback_timeout_ms; /* 0 means 100ms; independent of motor CAN timeout */
    ENCOS_stop_mode stop_mode; /* default zero-current, optional variable damping brake */
} ENCOS_motor_config;

typedef struct encos_motor_t {
    ENCOS_motor_config config;
    ENCOS_enable enable; /* 应用唯一要写的使能入口：encos_enable / encos_stop */
    uint8_t enable_cleared; /* 驱动私有：本次使能是否已清过环(见 Calc_Send 的 ⓪)。应用别写 */
    uint8_t update; /* set to 1 after accepted reply; caller may clear */
    uint8_t rx_data[8], rx_len, errcode;
    ENCOS_Feedback feedback; /* latest typed reply including raw query payload */
    float last_fdb_position, fdb_position, real_fdb_position; /* rad */
    float fdb_speed, fdb_current; /* rad/s, phase A */
    float fdb_torque; /* estimated current * configured Kt; NAN if unknown */
    float t_rotor, t_mos; /* deg C, half-degree resolution */
    uint8_t torque_valid, feedback_valid, seen_reply;
    uint32_t last_rx_ms, last_position_ms, last_speed_ms;
    float ref_position, ref_speed, ref_torque, kp, kd;
    monitor_item *monitor;
    FPS_t motor_fps;
    controller *motor_controller;
    control_encos_func control_calc;
    uint8_t (*check_motor_inplace)(struct encos_motor_t *, float range);
    uint8_t zero_pending, zero_ack_received, zero_ack_success;
    uint32_t zero_sent_ms;
    ENCOS_Result last_result;
} ENCOS_motor;

#define ENCOS_CAN_ID_MIN 0x001u
#define ENCOS_CAN_ID_MAX 0x7FEu
#define ENCOS_BROADCAST_ID ENCOS_SETTINGS_ID
#define ENCOS_OFFLINE_TIMEOUT_MS 100u
#define ENCOS_SETTING_GUARD_MS 501u /* manual section 3.5 requires >500ms */
#define ENCOS_ZERO_ACK_TIMEOUT_MS 1000u /* host policy, not a protocol guarantee */

void ENCOSmotor_Driver_Init(void);
ENCOS_motor *ENCOSmotor_Create(ENCOS_motor_config *config, control_encos_func func);
void ENCOSmotor_Calc_Send(void);
/* **使能没有接口给应用调**：应用只写 obj->enable = encos_enable / encos_stop。
 * 驱动在自己的发送拍上(ENCOSmotor_Calc_Send 的 ⓪)抓 stop->enable 的边沿，清一次
 * 控制器状态，再按反馈是否新鲜决定这一拍发控制帧还是停止帧 —— 所以应用层不用、
 * 也拿不到 ENCOSmotor_Enable()，别自己判"反馈新鲜了没"再决定要不要使能。
 * (原 ENCOSmotor_Enable() 于 2026-10-05 合并进 Calc_Send，不再对外。) */
/* Disable means software STOP, not physical power-stage disable. Sends stop frame now. */
ENCOS_Result ENCOSmotor_Disable(ENCOS_motor *obj);
/* Stop all motors on this CAN bus before zeroing. Bus TX pauses >500ms afterward. */
ENCOS_Result ENCOSmotor_Save_Zero(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Query(ENCOS_motor *obj, uint8_t decimal_code);
/* Pure builders in encos_protocol.h cover servo/current/torque/brake/ID commands.
 * Do not bypass this driver's bus setting guard when transmitting those frames. */
void ENCOSmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);
ENCOS_Result ENCOSmotor_FeedbackData_Update(ENCOS_motor *obj, const uint8_t *data, size_t len);
const char *ENCOSmotor_Err_String(uint8_t errcode);
uint8_t ENCOSmotor_Is_Online(ENCOS_motor *obj);
uint8_t ENCOSMotor_Check_InPlace(ENCOS_motor *obj, float range);
/* Legacy unsupported operations are explicit, never emit invented commands. */
ENCOS_Result ENCOSmotor_Clear_Err(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Reset_Round(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Broadcast_Cmd(ENCOS_motor *obj, uint8_t sub_cmd);
