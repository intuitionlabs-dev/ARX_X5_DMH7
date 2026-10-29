#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "monitor.h"
#include "controller.h"
#include "bsp_supervise.h"
#include "encos_protocol.h"
#include <stdint.h>

/* 编码量程须匹配驱动器设置；API、接收回调和发送步骤由调用方串行执行。 */
struct encos_motor_t;
typedef void (*control_encos_func)(struct encos_motor_t *motor);
typedef float (*calc_fdb_func_encos)(void *arg);
typedef enum { encos_stop, encos_enable } ENCOS_enable;
typedef enum { encos_output_normal, encos_output_reverse } ENCOS_output_mode;
typedef enum { encos_soft_stop, encos_damping_stop } ENCOS_stop_mode;

/* 1：软停止发送零增益 mode-0 帧；0：发送 mode-3 零电流帧，阻尼停止使用 mode-3。 */
#define ENCOS_IDLE_USE_MIXED_FRAME 1

typedef struct encos_motor_config_t {
    uint8_t bsp_can_index;
    uint16_t can_tx_id, can_rx_id;
    controller_config motor_controller_config;
    calc_fdb_func_encos pos_fdb_calc, speed_fdb_calc;
    void *arg;
    lost_callback lost_callback_;
    ENCOS_output_mode output_mode;
    /* 配置控制帧发送相位，0 使用 CAN_TX_GROUP_ALL。 */
    uint8_t tx_group;
    ENCOS_Ranges ranges;
    float torque_constant; /* Nm/A; 0 = unknown, estimated torque is invalid */
    float position_offset; /* rad; host coordinate = motor position + offset */
    uint32_t feedback_timeout_ms;
    ENCOS_stop_mode stop_mode;
} ENCOS_motor_config;

typedef struct encos_motor_t {
    ENCOS_motor_config config;
    ENCOS_enable enable; /* 应用唯一要写的使能入口：encos_enable / encos_stop */
    uint8_t enable_cleared; /* 驱动内部记录本次使能是否已清空控制器。 */
    uint8_t update;
    uint8_t rx_data[8], rx_len, errcode;
    ENCOS_Feedback feedback;
    float last_fdb_position, fdb_position, real_fdb_position; /* rad */
    float fdb_speed, fdb_current; /* rad/s, phase A */
    float fdb_torque;
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

    /* 周期开始清零，发送步骤处理或跳过电机后置位。 */
    uint8_t sent_this_tick;
} ENCOS_motor;

#define ENCOS_CAN_ID_MIN 0x001u
#define ENCOS_CAN_ID_MAX 0x7FEu
#define ENCOS_BROADCAST_ID ENCOS_SETTINGS_ID
#define ENCOS_OFFLINE_TIMEOUT_MS 100u
#define ENCOS_SETTING_GUARD_MS 501u
#define ENCOS_ZERO_ACK_TIMEOUT_MS 1000u

void ENCOSmotor_Driver_Init(void);
ENCOS_motor *ENCOSmotor_Create(ENCOS_motor_config *config, control_encos_func func);
/* 每次发送步骤最多提交一帧。 */
/* 每拍开跑前调一次，清掉所有电机的 sent_this_tick */
void ENCOSmotor_Send_Begin(void);
/* 在指定总线尝试发送一帧，已发送返回 1，本周期无待发帧返回 0。 */
uint8_t ENCOSmotor_Send_Step(uint8_t can_id);
/* 应用设置 enable，驱动在发送步骤检测使能边沿并重置控制器。 */

ENCOS_Result ENCOSmotor_Disable(ENCOS_motor *obj);

ENCOS_Result ENCOSmotor_Save_Zero(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Query(ENCOS_motor *obj, uint8_t decimal_code);
/* 直接构造设置类命令时仍须遵守驱动的总线设置保护间隔。 */
void ENCOSmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len);
ENCOS_Result ENCOSmotor_FeedbackData_Update(ENCOS_motor *obj, const uint8_t *data, size_t len);
const char *ENCOSmotor_Err_String(uint8_t errcode);
uint8_t ENCOSmotor_Is_Online(ENCOS_motor *obj);
uint8_t ENCOSMotor_Check_InPlace(ENCOS_motor *obj, float range);

ENCOS_Result ENCOSmotor_Clear_Err(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Reset_Round(ENCOS_motor *obj);
ENCOS_Result ENCOSmotor_Broadcast_Cmd(ENCOS_motor *obj, uint8_t sub_cmd);
