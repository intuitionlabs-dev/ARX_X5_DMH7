#pragma once
#include "cvector.h"
#include "bsp_can.h"
#include "monitor.h"
#include "math.h"
#include "controller.h"
#include "bsp_supervise.h"

#pragma pack(1)
struct dm_motor_t;
typedef void (*control_dm_func) (struct dm_motor_t* motor);
typedef float (*calc_fdb_func) (void* arg);

typedef enum { dm_mit = 0, dm_pos, dm_speed} DM_control_mode;
typedef enum { dm_stop, dm_enable} DM_enable;
typedef enum { dm_output_normal = 0, dm_output_reverse } DM_output_mode;
/* 错误状态定义，由errcode解析，详见达妙电机通信协议 */
typedef enum {
    dm_err_disable = 0x0,        /* 失能 */
    dm_err_enable = 0x1,         /* 使能 */
    dm_err_overvoltage = 0x8,    /* 超压 */
    dm_err_undervoltage = 0x9,   /* 欠压 */
    dm_err_overcurrent = 0xA,    /* 过流 */
    dm_err_mos_overtemp = 0xB,   /* MOS过温 */
    dm_err_coil_overtemp = 0xC,  /* 电机线圈过温 */
    dm_err_com_loss = 0xD,       /* 通讯丢失 */
    dm_err_overload = 0xE        /* 过载 */
} DM_err_status;

/* MIT 力矩为 Kp·位置误差+Kd·速度误差+t_ff，电流按 KT_out 换算。 */

typedef struct dm_motor_config_t {
    uint8_t bsp_can_index;
    uint8_t can_tx_id;  /* 发送给电机的id，需与上位机设置保持一致 */
    uint8_t can_rx_id;  /* 电机反馈的id，需与上位机设置保持一致 */
    controller_config motor_controller_config;
    calc_fdb_func pos_fdb_calc;
    calc_fdb_func speed_fdb_calc;
    void* arg;
    lost_callback lost_callback_;
    DM_control_mode control_mode;
    DM_output_mode output_mode;  /* 达妙电机输出方向，默认正转 */
    /* 组掩码决定控制计算与发送相位；维护命令使用 CAN_TX_GROUP_ALL。 */
    uint8_t tx_group;

    /* 各物理量(或其绝对值)最大值,用于反馈与MIT模式控制时线性映射,可在上位机查看 */
    float max_position;
    float max_speed;
    float max_torque;
    float max_kp;
    float max_kd;
} DM_motor_config;

typedef struct dm_motor_t {
    DM_motor_config config;
    DM_enable enable;

    uint8_t update;
    /* 反馈信息 */
    uint8_t rx_data[8];       /* 原始数据 */
    uint8_t errcode;  /* 错误码,对应状态类型为：0——失能；1——使能；8——超压；9——欠压；A——过电流；B——MOS 过温；C——电机线圈过温；D——通讯丢失；E——过载 */
    DM_err_status err_status;   /* 错误状态 */
    float last_fdb_position;  /* 上一次位置 */
    float fdb_position;       /* 位置（单位：rad） */
    float fdb_speed;          /* 速度 (单位：rad/s) */
    float fdb_torque;         /* 扭矩(解算结果待检验)（单位N·m牛米） */
    uint8_t t_mos;            /* 驱动MOS温度 */
    uint8_t t_rotor;          /* 电机内部线圈温度 */
    short round;              /* 电机正向超过max_position的次数 */
    float real_fdb_position;  /* 实际位置（单位：rad） */

    /* 控制信息 */
    float ref_position;       /* 位置(rad)      MIT,位置速度模式 */
    float ref_speed;          /* 速度(rad/s)    所有模式 */
    float ref_torque;         /* 转矩(N·m)      MIT模式 */
    float kp;                 /* Kp             MIT模式 */
    float kd;                 /* Kd             MIT模式 */

    uint8_t (*check_motor_inplace)(struct dm_motor_t* obj, float range);/* 电机到位了返回1 */

    monitor_item* monitor;
    FPS_t motor_fps;
    FPS_t motor_tx_fps;   /* 本机被排进发送的拍数，和 motor_fps(收到的反馈) 对拍 */
    controller* motor_controller;   /* 控制器 */
    control_dm_func control_calc;

    /* Send_Begin 每周期清零，Send_Step 处理或跳过电机后置位。 */
    uint8_t sent_this_tick;
} DM_motor;

#pragma pack()

void DMmotor_Driver_Init(void);
DM_motor* DMmotor_Create(DM_motor_config* config , control_dm_func func);
/* 每次发送步骤最多提交一帧。 */
/* 每拍开跑前调一次，清掉所有电机的 sent_this_tick */
void DMmotor_Send_Begin(void);
/* 在指定总线尝试发送一帧，已发送返回 1，本周期无待发帧返回 0。 */
uint8_t DMmotor_Send_Step(uint8_t can_id);
/* 发送 FF..FC 使能帧或 FF..FD 失能帧。 */
void DMmotor_Enable(DM_motor* obj);
void DMmotor_Disable(DM_motor* obj);
void DMmotor_Clear_Err(DM_motor* obj);
void DMmotor_Save_Zero(DM_motor *obj);
