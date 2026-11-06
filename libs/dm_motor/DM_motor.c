#include "DM_motor.h"
#include "common.h"
#include "main.h"
#include <stdlib.h>
/* CAN_ID == DM_motor_config.can_tx_id，即上位机中的 CAN ID */
/* MST_ID == DM_motor_config.can_rx_id，即上位机中的 Master ID */
/* 一般而言，CAN ID 和 Master ID 设置为同样值 */
/* 控制 ID：MIT 为 CAN_ID，位置速度为 0x100+CAN_ID，速度为 0x200+CAN_ID。 */
/* 反馈仲裁 ID 由电机 MST_ID 配置。 */

#define DM_CAN_ID_MIN 0x1  /* 电机 CAN_ID 范围为 0x1～0xF，反馈高 4 位保留给错误码。 */
#define DM_CAN_ID_MAX 0xF  /* 电机 CAN_ID 范围为 0x1～0xF，反馈高 4 位保留给错误码。 */
#define DM_MIT_OFFSET 0x000        /* MIT模式控制帧ID偏移 */
#define DM_POS_SPEED_OFFSET 0x100  /* 位置速度模式的控制帧 ID 偏移。 */
#define DM_SPEED_OFFSET 0x200      /* 速度模式的控制帧 ID 偏移。 */

cvector *dm_motor_list;

uint8_t DMMotor_Check_InPlace(DM_motor *obj , float range) {
    if(obj->monitor->count < 1) return 0;
    return fabs(obj->ref_position - obj->fdb_position) <= range;
}

void DMmotor_FeedbackData_Update(DM_motor *obj, uint8_t *data)
{
    obj->last_fdb_position = obj->fdb_position;
    obj->update = 0;
    obj->monitor->reset(obj->monitor);
    memcpy(obj->rx_data, data, 8);
    obj->errcode = data[0] >> 4;  /* 错误码在高4位 */
    obj->err_status = (DM_err_status)obj->errcode;
    obj->fdb_position = (((int16_t)data[1] << 8) + data[2]) / (float)0x10000 * obj->config.max_position * 2 - obj->config.max_position;

    obj->fdb_speed = (((int16_t)data[3] << 4) + (data[4] >> 4)) / (float)0x1000 * obj->config.max_speed * 2 - obj->config.max_speed;
    uint16_t fdb_torque_temp = ((data[4] & 0x0f) << 8) | data[5];
    obj->fdb_torque = (fdb_torque_temp - 2048) / 2048.0f * (obj->config.max_torque);
    obj->t_mos = data[6];
    obj->t_rotor = data[7];
    if (obj->fdb_position - obj->last_fdb_position > obj->config.max_position)  /* 位置跨过正向编码边界时减少累计圈数。 */
        obj->round--;
    else if (obj->fdb_position - obj->last_fdb_position < -obj->config.max_position)  /* 位置跨过反向编码边界时增加累计圈数。 */
        obj->round++;
    obj->real_fdb_position = (obj->fdb_position + obj->round * 2* obj->config.max_position);
    FrameRateStatistics(&obj->motor_fps);
}

void DMmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {
    (void)len;

    /* 同时匹配总线、反馈仲裁 ID 和载荷低 4 位电机 ID。 */
    if (!dm_motor_list || !data) return;
    for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
        if (obj->config.bsp_can_index == can_id &&
            obj->config.can_rx_id == identifier &&
            (data[0] & 0x0F) == obj->config.can_tx_id) {
            DMmotor_FeedbackData_Update(obj, data);
        }
    }
}

void DMmotor_Enable(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    /* 命令帧必须送达，不走分组，用 ALL 避免被当前组过滤掉 */
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Disable(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Clear_Err(DM_motor *obj)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Save_Zero(DM_motor *obj)
{
    uint8_t data[8] = {0xFF ,0xFF ,0xFF, 0xFF ,0xFF, 0xFF ,0xFF, 0xFE};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Driver_Init(void)
{
    dm_motor_list = cvector_create(sizeof(DM_motor *));
}

DM_motor* DMmotor_Create(DM_motor_config* config , control_dm_func func)
{
    static uint8_t registry[DEVICE_CAN_CNT];
    if (config->bsp_can_index >= DEVICE_CAN_CNT) {/* CAN索引超出范围 */

        return NULL;
    }
    /* CAN ID 超出范围则警告 */
    if(config->can_tx_id < DM_CAN_ID_MIN || config->can_tx_id > DM_CAN_ID_MAX) {
    }

    DM_motor *obj = (DM_motor *)malloc(sizeof(DM_motor));
    if (!registry[config->bsp_can_index]) {
        registry[config->bsp_can_index] = 1;
        BSP_CAN_RegisterRxCallback(config->bsp_can_index, DMmotor_RxCallBack);
    }
    memset(obj, 0, sizeof(DM_motor));
    obj->config = *config;
    obj->motor_controller = create_controller(&obj->config.motor_controller_config);
    obj->control_calc = func;
    BSP_CAN_AddFilter(obj->config.bsp_can_index, obj->config.can_rx_id);
    /* 监控计数以 Monitor_Loop 周期为单位，100 Hz 下计数 2 对应 20 ms。 */
    obj->monitor = Monitor_Register(obj->config.lost_callback_, 2, obj);
    obj->check_motor_inplace = DMMotor_Check_InPlace;
    /* 相位掩码为 0 时使用 CAN_TX_GROUP_ALL。 */
    if (obj->config.tx_group == 0) {
        obj->config.tx_group = CAN_TX_GROUP_ALL;
    }
    cvector_pushback(dm_motor_list, (void *)&obj);

    return obj;
}

/* 周期开始调用 Send_Begin 清除已处理标志，Send_Step 每次最多发送一帧。 */

#define DM_ENABLE_RESEND_MS 75u

/* 各总线独立轮转补发使能命令。 */
static uint32_t dm_enable_time[DEVICE_CAN_CNT];
static uint8_t dm_enable_next[DEVICE_CAN_CNT];

void DMmotor_Send_Begin(void) {
    if (!dm_motor_list) return;
    for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
        obj->sent_this_tick = 0;
    }
}

uint8_t DMmotor_Send_Step(uint8_t can_id) {
    if (!dm_motor_list || can_id >= DEVICE_CAN_CNT) return 0;

    /* 按总线维护周期轮转补发使能命令。 */
    if (HAL_GetTick() - dm_enable_time[can_id] >= DM_ENABLE_RESEND_MS) {
        size_t total = 0;
        for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
            if ((*(DM_motor **)cvector_val_at(dm_motor_list, i))->config.bsp_can_index
                == can_id) {
                total++;
            }
        }
        if (total) {
            size_t k = 0;
            if (dm_enable_next[can_id] >= total) dm_enable_next[can_id] = 0;
            for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
                DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
                if (obj->config.bsp_can_index != can_id) continue;
                if (k == dm_enable_next[can_id]) {
                    DMmotor_Enable(obj);
                    break;
                }
                k++;
            }
            dm_enable_time[can_id] = HAL_GetTick();
            dm_enable_next[can_id] = (uint8_t)((k + 1u) % total);
            return 1;
        }
    }

    /* 逐电机选择本周期待发控制帧，相位未命中时标记已处理并跳过。 */
    for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
        if (obj->config.bsp_can_index != can_id) continue;
        if (obj->sent_this_tick) continue;
        obj->sent_this_tick = 1;

        /* 按相同发送相位调度控制计算，保证 PID 调用频率与控制帧一致。 */
        if (!BSP_CAN_TxGroupMatch(obj->config.tx_group)) {
            continue;
        }
        FrameRateStatistics(&obj->motor_tx_fps);

		/* 控制大喵电机 */
		/* 软停止状态仍更新控制器。 */
		if(obj->config.arg != NULL && obj->config.pos_fdb_calc != NULL) {
                obj->motor_controller->fdb_position = (obj->config.pos_fdb_calc)(obj->config.arg);
            } else {
                obj->motor_controller->fdb_position = obj->real_fdb_position * RAD2DEG;
            }
            if(obj->config.arg != NULL && obj->config.speed_fdb_calc != NULL) {
                obj->motor_controller->fdb_speed = (obj->config.speed_fdb_calc)(obj->config.arg);
            } else {
                obj->motor_controller->fdb_speed = obj->fdb_speed * RAD2DEG;
            }

            if (obj->control_calc) {
                obj->control_calc(obj);
            } else {
                controller_calc(obj->motor_controller);
                if (obj->config.output_mode == dm_output_reverse) {
                    obj->ref_torque = (-1.0f) * obj->motor_controller->output;
                } else {
                    obj->ref_torque = obj->motor_controller->output;
                }
            }
        if (obj->enable == dm_enable)
        {
            if (obj->config.control_mode == dm_mit)
            {
                if (fabsf(obj->ref_position) > obj->config.max_position)
                    obj->ref_position = obj->config.max_position * (obj->ref_position > 0 ? 1 : -1);
                if (fabsf(obj->ref_speed) > obj->config.max_speed)
                    obj->ref_speed = obj->config.max_speed * (obj->ref_speed > 0 ? 1 : -1);
                if (fabsf(obj->ref_torque) > obj->config.max_torque)
                    obj->ref_torque = obj->config.max_torque * (obj->ref_torque > 0 ? 1 : -1);
                if (obj->kp > obj->config.max_kp)
                    obj->kp = obj->config.max_kp;
                if (obj->kd > obj->config.max_kd)
                    obj->kd = obj->config.max_kd;

                uint8_t send_data[8];
                uint16_t pos_send = float_to_uint(obj->ref_position, -obj->config.max_position, obj->config.max_position, 16);
                uint16_t speed_send = float_to_uint(obj->ref_speed, -obj->config.max_speed, obj->config.max_speed, 12);
                uint16_t kp_send = float_to_uint(obj->kp, 0, obj->config.max_kp, 12);
                uint16_t kd_send = float_to_uint(obj->kd, 0, obj->config.max_kd, 12);
                uint16_t torque_send = float_to_uint(obj->ref_torque, -obj->config.max_torque, obj->config.max_torque, 12);

                send_data[0] = (pos_send >> 8);
                send_data[1] = pos_send;
                send_data[2] = speed_send >> 4;
                send_data[3] = ((speed_send & 0xF) << 4) | (kp_send >> 8);
                send_data[4] = kp_send;
                send_data[5] = kd_send >> 4;
                send_data[6] = ((kd_send & 0xF) << 4) | (torque_send >> 8);
                send_data[7] = torque_send;
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_MIT_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_pos)
            {
                uint8_t send_data[8] = {0};
                uint8_t *ppos = (uint8_t *)&obj->ref_position;
                uint8_t *pspeed = (uint8_t *)&obj->ref_speed;
                memcpy(send_data, ppos, 4);
                memcpy(send_data + 4, pspeed, 4);
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_POS_SPEED_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_speed)
            {
                uint8_t send_data[4] = {0};
                uint8_t *pspeed = (uint8_t *)&obj->ref_speed;
                memcpy(send_data, pspeed, 4);
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_SPEED_OFFSET, send_data, 4);
            }

            //     cnt++;
        }
        /* 电机stop模式 */
        else
        {
            if (obj->config.control_mode == dm_mit)
            {
                uint8_t send_data[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
                BSP_CAN_Send(obj->config.bsp_can_index,
                    obj->config.can_tx_id + DM_MIT_OFFSET, send_data, 8);
            }
            else if (obj->config.control_mode == dm_pos)
            {
                DMmotor_Disable(obj);
            }
            else if (obj->config.control_mode == dm_speed)
            {
                DMmotor_Disable(obj);
            }

            //     cnt++;
        }
        /* 返回已发送状态，供上层按轮调度帧间隔。 */
        return 1;
    }
    return 0;
}
