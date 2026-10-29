#include "arm_motor.h"
#include "DM_motor.h"
#include "dm_mit_control.h"
#include "ENCOS_motor.h"
#include "encos_mixed_control.h"
#include "common.h"
#include <math.h>

/* 提供电机监控要求的掉线回调入口。 */
static void motor_lost(void* context) { (void)context; }

/* 将 DM 原始位置反馈换算为 deg。 */
static float dm_position(void* context) {
    const Arm_motor* motor = context;
    const DM_motor* dm = motor->context;
    const uint16_t raw = ((uint16_t)dm->rx_data[1] << 8) | dm->rx_data[2];
    return (raw / motor->config->position_divisor * 2.0f - 1.0f)
        * motor->config->position_max * RAD2DEG;
}

/* 将 DM 原始速度反馈换算为 deg/s。 */
static float dm_feedback_speed(void* context) {
    const Arm_motor* motor = context;
    const DM_motor* dm = motor->context;
    const uint16_t raw = ((uint16_t)dm->rx_data[3] << 4) | (dm->rx_data[4] >> 4);
    return (raw / motor->config->speed_divisor * 2.0f - 1.0f)
        * motor->config->speed_max * RAD2DEG;
}

/* 执行 DM MIT 控制并处理目标圈数配置。 */
static void dm_control(DM_motor* dm) {
    const Arm_motor* motor = dm->config.arg;
    DM_Mit_Control(dm);
    /* 单圈目标直接使用控制器参考角。 */
    if (!motor->config->cmd_multiturn)
        dm->ref_position = dm->motor_controller->ref_position * DEG2RAD;
}

/* 从机型参数生成位置、速度级联配置。 */
static controller_config control_config(const Arm_motor_config* cfg) {
    /* 将机型 PID 参数装配到通用控制器。 */
    controller_config control = {
        .control_type = PID_MODEL, .control_depth = POS_CONTROL_CASCADE,
        .position_pid_config = cfg->position_pid, .speed_pid_config = cfg->speed_pid,
    };
    return control;
}

/* 创建 DM 电机并绑定通用控制接口。 */
static int dm_create(Arm_motor* motor, const Arm_motor_config* cfg, uint8_t bus) {
    /* 装配 DM 总线标识、反馈换算和 MIT 量程。 */
    DM_motor_config config = {
        .bsp_can_index = bus, .can_tx_id = (uint8_t)cfg->tx_id,
        .can_rx_id = (uint8_t)cfg->rx_id, .motor_controller_config = control_config(cfg),
        .pos_fdb_calc = dm_position, .speed_fdb_calc = dm_feedback_speed, .arg = motor,
        .lost_callback_ = motor_lost, .control_mode = dm_mit,
        .tx_group = cfg->tx_group, .max_position = cfg->position_max,
        .max_speed = cfg->speed_max, .max_torque = cfg->torque_max,
        .max_kp = cfg->kp_max, .max_kd = cfg->kd_max,
    };
    DM_motor* dm = DMmotor_Create(&config, dm_control);
    /* 驱动实例创建失败时返回失败。 */
    if (!dm) return 0;
    motor->context = dm;
    motor->control = dm->motor_controller;
    motor->monitor = dm->monitor;
    return motor->control && motor->monitor;
}

/* 设置 DM 使能状态和 MIT 增益。 */
static void dm_mode(Arm_motor* motor, uint8_t enabled, float kp, float kd) {
    DM_motor* dm = motor->context;
    dm->enable = enabled ? dm_enable : dm_stop;
    dm->kp = kp;
    dm->kd = kd;
}

/* 提供 rad 单位的 DM 位置反馈。 */
static float dm_position_rad(const Arm_motor* motor) {
    return dm_position((void*)motor) * DEG2RAD;
}

/* 读取 DM 转子温度。 */
static uint8_t dm_temperature(const Arm_motor* motor) {
    return ((const DM_motor*)motor->context)->t_rotor;
}

/* 更新 DM 停止状态并发送失能命令。 */
static void dm_disable(Arm_motor* motor) {
    DM_motor* dm = motor->context;
    dm->enable = dm_stop;
    DMmotor_Disable(dm);
}

/* 依据失能反馈或反馈静默时间判断停止。 */
static uint8_t dm_stopped(const Arm_motor* motor, uint32_t now) {
    const DM_motor* dm = motor->context;
    return dm->errcode == dm_err_disable || now - (uint32_t)dm->motor_fps.now >= 300u;
}

/* 记录设零前位置并发送 DM 保存零点命令。 */
static int dm_zero_start(Arm_motor* motor, uint32_t now) {
    motor->zero_before = dm_position_rad(motor);
    motor->zero_stage = 0;
    /* 记录本次设零命令的时间基准。 */
    motor->zero_sent_ms = now;
    DMmotor_Save_Zero(motor->context);
    return 1;
}

/* 复核 DM 零点反馈并按需重试。 */
static int dm_zero_poll(Arm_motor* motor, uint32_t now) {
    DM_motor* dm = motor->context;
    const uint32_t delay = motor->zero_stage ? 100u : 20u;
    /* 等待保存零点或使能后的反馈间隔。 */
    if (now - motor->zero_sent_ms < delay) return 0;
    /* 收到设零命令之后的反馈时检查零点。 */
    if ((int32_t)((uint32_t)dm->motor_fps.now - motor->zero_sent_ms) >= 0) {
        /* 零点进入容差后清除累计圈数。 */
        if (fabsf(dm_position_rad(motor)) <= 0.01f) {
            /* 复核阶段结束后恢复失能状态。 */
            if (motor->zero_stage) dm_disable(motor);
            dm->round = 0;
            dm->real_fdb_position = dm->fdb_position;
            return fabsf(motor->zero_before) <= 0.01f ? 2 : 1;
        }
        dm_disable(motor);
        /* 零点仍超出容差时再次发送保存零点命令。 */
        DMmotor_Save_Zero(dm);
        motor->zero_stage = 0;
    } else { /* 反馈尚未更新时发送使能命令触发反馈。 */
        DMmotor_Enable(dm);
        motor->zero_stage = 1;
    }
    motor->zero_sent_ms = now;
    return 0;
}

/* 创建 ENCOS 电机并绑定通用控制接口。 */
static int encos_create(Arm_motor* motor, const Arm_motor_config* cfg, uint8_t bus) {
    /* 装配 ENCOS 总线标识、量程和反馈超时。 */
    ENCOS_motor_config config = {
        .bsp_can_index = bus, .can_tx_id = cfg->tx_id, .can_rx_id = cfg->rx_id,
        .motor_controller_config = control_config(cfg), .lost_callback_ = motor_lost,
        .tx_group = cfg->tx_group, .torque_constant = cfg->torque_constant,
        .feedback_timeout_ms = 100,
        /* 配置各 MIT 字段的编码上下界。 */
        .ranges = {.kp={0,cfg->kp_max}, .kd={0,cfg->kd_max},
            .position={-cfg->position_max,cfg->position_max},
            .speed={-cfg->speed_max,cfg->speed_max},
            .torque={-cfg->torque_max,cfg->torque_max},
            .current={-cfg->current_max,cfg->current_max}},
    };
    ENCOS_motor* encos = ENCOSmotor_Create(&config, ENCOS_Mixed_Control);
    /* 驱动实例创建失败时返回失败。 */
    if (!encos) return 0;
    motor->context = encos;
    motor->control = encos->motor_controller;
    motor->monitor = encos->monitor;
    return 1;
}

/* 同步 ENCOS 控制参数、使能状态和增益。 */
static void encos_mode(Arm_motor* motor, uint8_t enabled, float kp, float kd) {
    ENCOS_motor* encos = motor->context;
    encos->config.motor_controller_config.position_pid_config = motor->control->pid_pos_data.config;
    encos->config.motor_controller_config.speed_pid_config = motor->control->pid_speed_data.config;
    encos->enable = enabled ? encos_enable : encos_stop;
    encos->kp = kp;
    encos->kd = kd;
}

/* 读取 ENCOS 多圈位置反馈，单位 rad。 */
static float encos_position_rad(const Arm_motor* motor) {
    return ((const ENCOS_motor*)motor->context)->real_fdb_position;
}

/* 将有效 ENCOS 温度限制到字节范围。 */
static uint8_t encos_temperature(const Arm_motor* motor) {
    const float value = ((const ENCOS_motor*)motor->context)->t_rotor;
    return isfinite(value) ? (uint8_t)fclip(value, 0, 255) : 0;
}

/* 发送 ENCOS 失能命令。 */
static void encos_disable(Arm_motor* motor) {
    ENCOSmotor_Disable(motor->context);
}

/* 读取 ENCOS 驱动的停止状态。 */
static uint8_t encos_stopped(const Arm_motor* motor, uint32_t now) {
    (void)now;
    return ((const ENCOS_motor*)motor->context)->enable == encos_stop;
}

/* 发送 ENCOS 保存零点命令并记录发送时间。 */
static int encos_zero_start(Arm_motor* motor, uint32_t now) {
    const ENCOS_Result result = ENCOSmotor_Save_Zero(motor->context);
    /* 驱动繁忙时等待下一周期重试。 */
    if (result == ENCOS_BUSY) return 0;
    /* 保存命令发送异常时报告失败。 */
    if (result != ENCOS_OK) return -1;
    motor->zero_sent_ms = now;
    return 1;
}

/* 等待设置保护间隔并读取 ENCOS 设零确认。 */
static int encos_zero_poll(Arm_motor* motor, uint32_t now) {
    ENCOS_motor* encos = motor->context;
    /* 保护间隔内继续等待。 */
    if (now - motor->zero_sent_ms < ENCOS_SETTING_GUARD_MS) return 0;
    /* 根据设零确认结果返回成功或失败。 */
    if (encos->zero_ack_received) return encos->zero_ack_success ? 1 : -1;
    return 0;
}

/* 校验 DM 标识符和反馈编码分母。 */
static int dm_validate(const Arm_motor_config* cfg) {
    return cfg->tx_id > 0 && cfg->tx_id <= 15 && cfg->rx_id <= 255
        && isfinite(cfg->position_divisor) && cfg->position_divisor > 0
        && isfinite(cfg->speed_divisor) && cfg->speed_divisor > 0;
}

/* 校验 ENCOS 标识符、电流量程和力矩常数。 */
static int encos_validate(const Arm_motor_config* cfg) {
    return cfg->tx_id == cfg->rx_id && cfg->tx_id > 0 && cfg->tx_id < 0x7ff
        && isfinite(cfg->current_max) && cfg->current_max > 0
        && isfinite(cfg->torque_constant) && cfg->torque_constant >= 0;
}

/* 登记 DM 驱动操作及反馈 ID 共享能力。 */
const Arm_motor_driver ARM_DRIVER_DM = {
    dm_create, dm_mode, dm_position_rad, dm_temperature, dm_disable,
    dm_stopped, dm_zero_start, dm_zero_poll, dm_validate, 1,
};
/* 登记 ENCOS 驱动操作及独立反馈 ID 约束。 */
const Arm_motor_driver ARM_DRIVER_ENCOS = {
    encos_create, encos_mode, encos_position_rad, encos_temperature, encos_disable,
    encos_stopped, encos_zero_start, encos_zero_poll, encos_validate, 0,
};

/* 保存中断屏蔽状态并进入参考值发布临界区。 */
uint32_t Arm_Motors_Lock(void) {
/* ARM 目标使用中断屏蔽保护参考值发布。 */
#if defined(__arm__)
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
/* 主机环境使用空临界区实现。 */
#else
    return 0;
#endif
}

/* 恢复进入临界区前的中断屏蔽状态。 */
void Arm_Motors_Unlock(uint32_t interrupt_mask) {
/* ARM 目标使用中断屏蔽保护参考值发布。 */
#if defined(__arm__)
    __set_PRIMASK(interrupt_mask);
/* 主机环境使用空临界区实现。 */
#else
    (void)interrupt_mask;
#endif
}

/* 重置两类驱动的本周期发送调度状态。 */
void Arm_Motors_Send_Begin(void) {
    DMmotor_Send_Begin();
    ENCOSmotor_Send_Begin();
}

/* 在指定总线上发送一帧电机报文。 */
uint8_t Arm_Motors_Send_Step(uint8_t bus) {
    /* DM 驱动发送成功时结束本次调度。 */
    if (DMmotor_Send_Step(bus)) return 1;
    return ENCOSmotor_Send_Step(bus);
}
