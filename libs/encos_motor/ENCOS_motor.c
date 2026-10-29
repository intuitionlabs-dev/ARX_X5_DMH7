#include "ENCOS_motor.h"
#include "main.h" /* 使用 HAL 毫秒时基。 */
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 监控计数由超时毫秒数除以 Monitor_Loop 周期得到。 */
#define MONITOR_LOOP_PERIOD_MS 10

cvector *ENCOS_motor_list;
static uint8_t registry[DEVICE_CAN_CNT];
static uint8_t bus_guard[DEVICE_CAN_CNT];
static uint32_t bus_setting_ms[DEVICE_CAN_CNT];

static uint32_t now_ms(void) { return HAL_GetTick(); }
static int bus_busy(uint8_t bus) {
  if (!bus_guard[bus])
    return 0;
  if ((uint32_t)(now_ms() - bus_setting_ms[bus]) < ENCOS_SETTING_GUARD_MS)
    return 1;
  bus_guard[bus] = 0;
  return 0;
}
/* 控制帧使用电机相位，设零和查询命令使用全部相位。 */
static ENCOS_Result send_frame(ENCOS_motor *obj, const ENCOS_Frame *frame,
                               uint8_t tx_group) {
  if (bus_busy(obj->config.bsp_can_index))
    return ENCOS_BUSY;
  BSP_CAN_Send(obj->config.bsp_can_index, frame->id, (uint8_t *)frame->data,
               frame->len);
  return ENCOS_OK;
}
static void reset_controller(ENCOS_motor *obj) {
  controller *c = obj->motor_controller;
  PID_Init(&c->pid_pos_data,
           &obj->config.motor_controller_config.position_pid_config);
  PID_Init(&c->pid_speed_data,
           &obj->config.motor_controller_config.speed_pid_config);
  PID_Config p = c->pid_pos_data.config, s = c->pid_speed_data.config;
  memset(&c->pid_pos_data, 0, sizeof(c->pid_pos_data));
  memset(&c->pid_speed_data, 0, sizeof(c->pid_speed_data));
  c->pid_pos_data.config = p;
  c->pid_speed_data.config = s;
  c->output = 0;
}
static ENCOS_Result send_stop(ENCOS_motor *obj) {
  ENCOS_Frame frame;
#if ENCOS_IDLE_USE_MIXED_FRAME
  /* 软停止按配置发送零增益混控帧；阻尼停止发送 mode-3 命令。 */
  if (obj->config.stop_mode == encos_soft_stop) {
    ENCOS_Result mixed = ENCOS_PackMixed(&frame, obj->config.can_tx_id,
                                         &obj->config.ranges, 0, 0, 0, 0, 0);
    if (mixed == ENCOS_OK)
      return send_frame(obj, &frame, obj->config.tx_group);
  }
#endif
  ENCOS_CurrentMode mode = obj->config.stop_mode == encos_damping_stop
                               ? ENCOS_DAMPING_BRAKE
                               : ENCOS_CURRENT;
  ENCOS_Result result =
      ENCOS_PackCurrent(&frame, obj->config.can_tx_id, mode, 0, 1);
  return result == ENCOS_OK ? send_frame(obj, &frame, obj->config.tx_group)
                            : result;
}
static int fresh(ENCOS_motor *obj, uint8_t field, uint32_t at) {
  return (obj->feedback_valid & field) &&
         (uint32_t)(now_ms() - at) < obj->config.feedback_timeout_ms;
}
static int motion_ready(ENCOS_motor *obj) {
  return !obj->errcode && !obj->zero_pending &&
         fresh(obj, ENCOS_HAS_POSITION, obj->last_position_ms) &&
         fresh(obj, ENCOS_HAS_SPEED, obj->last_speed_ms);
}
uint8_t ENCOSmotor_Is_Online(ENCOS_motor *obj) {
  return obj && obj->seen_reply &&
         (uint32_t)(now_ms() - obj->last_rx_ms) <
             obj->config.feedback_timeout_ms;
}
uint8_t ENCOSMotor_Check_InPlace(ENCOS_motor *obj, float range) {
  return obj && isfinite(range) && range >= 0 && !obj->errcode &&
         fresh(obj, ENCOS_HAS_POSITION, obj->last_position_ms) &&
         fabsf(obj->ref_position - obj->real_fdb_position) <= range;
}
ENCOS_Result ENCOSmotor_FeedbackData_Update(ENCOS_motor *obj, const uint8_t *data,
                                          size_t len) {
  if (!obj)
    return ENCOS_INVALID;
  ENCOS_Feedback f;
  ENCOS_Result result = ENCOS_Decode(data, len, &obj->config.ranges, &f);
  if (result != ENCOS_OK)
    return result;
  obj->feedback = f;
  memset(obj->rx_data, 0, sizeof(obj->rx_data));
  memcpy(obj->rx_data, data, len);
  obj->rx_len = (uint8_t)len;
  obj->errcode = f.error;
  obj->last_rx_ms = now_ms();
  obj->seen_reply = 1;
  obj->update = 1;
  obj->monitor->reset(obj->monitor);
  if (f.valid & ENCOS_HAS_POSITION) {
    obj->last_fdb_position = obj->fdb_position;
    obj->fdb_position = f.position;
    obj->real_fdb_position = f.position + obj->config.position_offset;
    obj->last_position_ms = obj->last_rx_ms;
  }
  if (f.valid & ENCOS_HAS_SPEED) {
    obj->fdb_speed = f.speed;
    obj->last_speed_ms = obj->last_rx_ms;
  }
  if (f.valid & ENCOS_HAS_CURRENT) {
    obj->fdb_current = f.current;
    obj->torque_valid = obj->config.torque_constant > 0;
    obj->fdb_torque =
        obj->torque_valid ? f.current * obj->config.torque_constant : NAN;
  }
  if (f.valid & ENCOS_HAS_MOTOR_TEMP)
    obj->t_rotor = f.motor_temp;
  if (f.valid & ENCOS_HAS_MOS_TEMP)
    obj->t_mos = f.mos_temp;
  obj->feedback_valid |= f.valid;
  FrameRateStatistics(&obj->motor_fps);
  return ENCOS_OK;
}
void ENCOSmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data,
                         uint32_t len) {
  if (!ENCOS_motor_list || !data || len < 2 || len > 8)
    return;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *obj = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    if (obj->config.bsp_can_index != can_id)
      continue;
    if (identifier == obj->config.can_rx_id) {
      (void)ENCOSmotor_FeedbackData_Update(obj, data, (size_t)len);
    } else if (identifier == ENCOS_SETTINGS_ID && len == 4 && data[2] == 1 &&
               (((uint16_t)data[0] << 8) | data[1]) == obj->config.can_tx_id &&
               obj->zero_pending && (data[3] == 3 || data[3] == 0)) {
      obj->zero_ack_received = 1;
      obj->zero_ack_success = data[3] == 3;
      obj->zero_pending = 0;

      obj->feedback_valid &= (uint8_t)~(ENCOS_HAS_POSITION | ENCOS_HAS_SPEED);
      obj->last_rx_ms = now_ms();
      obj->seen_reply = 1;
      obj->monitor->reset(obj->monitor);
      obj->update = 1;
    }
  }
}
void ENCOSmotor_Driver_Init(void) {
  if (!ENCOS_motor_list)
    ENCOS_motor_list = cvector_create(sizeof(ENCOS_motor *));
}
ENCOS_motor *ENCOSmotor_Create(ENCOS_motor_config *config, control_encos_func func) {
  if (!config || config->bsp_can_index >= DEVICE_CAN_CNT ||
      !ENCOS_ValidId(config->can_tx_id) ||
      config->can_rx_id != config->can_tx_id ||
      !ENCOS_ValidRanges(&config->ranges) ||
      !isfinite(config->position_offset) ||
      !isfinite(config->torque_constant) || config->torque_constant < 0 ||
      config->feedback_timeout_ms > 0x7FFFFFFFu ||
      (config->stop_mode != encos_soft_stop &&
       config->stop_mode != encos_damping_stop) ||
      (config->output_mode != encos_output_normal &&
       config->output_mode != encos_output_reverse)) {
    return NULL;
  }
  ENCOSmotor_Driver_Init();
  if (!ENCOS_motor_list)
    return NULL;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *old = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    if (old->config.bsp_can_index == config->bsp_can_index &&
        old->config.can_rx_id == config->can_rx_id)
      return NULL;
  }
  ENCOS_motor *obj = malloc(sizeof(*obj));
  if (!obj)
    return NULL;
  memset(obj, 0, sizeof(*obj));
  obj->config = *config;
  /* 相位掩码为 0 时使用 CAN_TX_GROUP_ALL。 */
  if (!obj->config.tx_group)
    obj->config.tx_group = CAN_TX_GROUP_ALL;
  if (!obj->config.feedback_timeout_ms)
    obj->config.feedback_timeout_ms = ENCOS_OFFLINE_TIMEOUT_MS;

  obj->motor_controller =
      create_controller(&obj->config.motor_controller_config);
  if (!obj->motor_controller) {
    free(obj);
    return NULL;
  }
  /* 将反馈超时从毫秒换算为监控调用周期数。 */
  obj->monitor = Monitor_Register(obj->config.lost_callback_,
                                  (int)(obj->config.feedback_timeout_ms /
                                        MONITOR_LOOP_PERIOD_MS),
                                  obj);
  if (!obj->monitor) {
    free(obj->motor_controller);
    free(obj);
    return NULL;
  }
  obj->control_calc = func;
  obj->check_motor_inplace = ENCOSMotor_Check_InPlace;
  obj->fdb_torque = NAN;
  obj->t_rotor = NAN;
  obj->t_mos = NAN;
  cvector_pushback(ENCOS_motor_list, &obj);
  if (!registry[config->bsp_can_index]) {
    BSP_CAN_RegisterRxCallback(config->bsp_can_index, ENCOSmotor_RxCallBack);
    BSP_CAN_AddFilter(config->bsp_can_index, ENCOS_SETTINGS_ID);
    registry[config->bsp_can_index] = 1;
  }
  BSP_CAN_AddFilter(config->bsp_can_index, config->can_rx_id);
  return obj;
}
/* 使能边沿和显式失能时重置控制器状态。 */
ENCOS_Result ENCOSmotor_Disable(ENCOS_motor *obj) {
  if (!obj)
    return ENCOS_INVALID;
  obj->enable = encos_stop;
  reset_controller(obj);
  return obj->last_result = send_stop(obj);
}
ENCOS_Result ENCOSmotor_Save_Zero(ENCOS_motor *obj) {
  if (!obj)
    return ENCOS_INVALID;
  if (bus_busy(obj->config.bsp_can_index))
    return ENCOS_BUSY;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *other = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    if (other->config.bsp_can_index == obj->config.bsp_can_index &&
        other->enable != encos_stop)
      return ENCOS_BUSY;
  }
  ENCOS_Frame frame;
  ENCOS_Result r = ENCOS_PackZero(&frame, obj->config.can_tx_id);
  if (r != ENCOS_OK)
    return r;
  obj->zero_pending = 1;
  obj->zero_ack_received = 0;
  obj->zero_ack_success = 0;
  obj->zero_sent_ms = now_ms();
  obj->feedback_valid &= (uint8_t)~(ENCOS_HAS_POSITION | ENCOS_HAS_SPEED);
  r = send_frame(obj, &frame, CAN_TX_GROUP_ALL); /* 设零帧绝不能因分组被丢 */
  bus_setting_ms[obj->config.bsp_can_index] = now_ms();
  bus_guard[obj->config.bsp_can_index] = 1;
  return obj->last_result = r;
}
ENCOS_Result ENCOSmotor_Query(ENCOS_motor *obj, uint8_t code) {
  if (!obj)
    return ENCOS_INVALID;
  ENCOS_Frame frame;
  ENCOS_Result r = ENCOS_PackQuery(&frame, obj->config.can_tx_id, code);
  if (r != ENCOS_OK)
    return obj->last_result = r;
  r = send_frame(obj, &frame, CAN_TX_GROUP_ALL);
  if (r == ENCOS_OK) {
    /* 查询和设置操作共用总线设置保护间隔。 */
    bus_setting_ms[obj->config.bsp_can_index] = now_ms();
    bus_guard[obj->config.bsp_can_index] = 1;
  }
  return obj->last_result = r;
}
const char *ENCOSmotor_Err_String(uint8_t code) {
  return ENCOS_ErrorString(code);
}
ENCOS_Result ENCOSmotor_Clear_Err(ENCOS_motor *obj) {
  (void)obj;
  return ENCOS_UNSUPPORTED;
}
ENCOS_Result ENCOSmotor_Reset_Round(ENCOS_motor *obj) {
  (void)obj;
  return ENCOS_UNSUPPORTED;
}
ENCOS_Result ENCOSmotor_Broadcast_Cmd(ENCOS_motor *obj, uint8_t sub_cmd) {
  return sub_cmd == 3 ? ENCOSmotor_Save_Zero(obj) : ENCOS_UNSUPPORTED;
}
/* 周期开始调用 Send_Begin，Send_Step 每次最多发送一帧。 */

void ENCOSmotor_Send_Begin(void) {
  if (!ENCOS_motor_list)
    return;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *obj = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    obj->sent_this_tick = 0;
  }
}

uint8_t ENCOSmotor_Send_Step(uint8_t can_id) {
  if (!ENCOS_motor_list || can_id >= DEVICE_CAN_CNT)
    return 0;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *obj = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    /* 只处理本总线尚未调度的电机。 */
    if (obj->config.bsp_can_index != can_id)
      continue;
    if (obj->sent_this_tick)
      continue;
    obj->sent_this_tick = 1;
    if (bus_busy(can_id))
      continue;

    if (obj->zero_pending &&
        (uint32_t)(now_ms() - obj->zero_sent_ms) >= ENCOS_ZERO_ACK_TIMEOUT_MS)
      obj->zero_pending = 0;
    /* 按发送相位调度控制计算，设零超时检查独立执行。 */
    if (!BSP_CAN_TxGroupMatch(obj->config.tx_group))
      continue;

    /* 首次检测到使能请求时清空控制器状态。 */
    if (obj->enable == encos_enable) {
      if (!obj->enable_cleared) {
        reset_controller(obj);
        obj->enable_cleared = 1;
      }
    } else
      obj->enable_cleared = 0;

    /* 命中发送相位后更新控制器，包含软停止状态。 */
    controller *c = obj->motor_controller;
    c->fdb_position = obj->config.pos_fdb_calc
                          ? obj->config.pos_fdb_calc(obj->config.arg)
                          : obj->real_fdb_position * (180 / ENCOS_PI);
    c->fdb_speed = obj->config.speed_fdb_calc
                       ? obj->config.speed_fdb_calc(obj->config.arg)
                       : obj->fdb_speed * (180 / ENCOS_PI);
    if (!isfinite(c->fdb_position) || !isfinite(c->fdb_speed)) {
      (void)ENCOSmotor_Disable(obj);
      obj->last_result = ENCOS_INVALID;
      return 1; /* 失能帧已发出：一次调用最多一帧 */
    }
    if (obj->control_calc)
      obj->control_calc(obj);
    else {
      controller_calc(c);
      obj->ref_torque = obj->config.output_mode == encos_output_reverse
                            ? -c->output
                            : c->output;
    }

    /* 根据使能、反馈有效性、故障和设零状态选择停止帧或控制帧。 */
    if (obj->enable != encos_enable || !motion_ready(obj)) {
      obj->enable = encos_stop;
      obj->last_result = send_stop(obj);
      return 1; /* 停止帧已发出 */
    }
    if (bus_busy(obj->config.bsp_can_index))
      continue;
    /* 控制回调执行后再次检查使能状态。 */
    if (obj->enable != encos_enable) {
      obj->last_result = send_stop(obj);
      return 1; /* 停止帧已发出 */
    }
    ENCOS_Frame frame;
    ENCOS_Result r = ENCOS_PackMixed(
        &frame, obj->config.can_tx_id, &obj->config.ranges, obj->kp, obj->kd,
        obj->ref_position - obj->config.position_offset, obj->ref_speed,
        obj->ref_torque);
    if (r != ENCOS_OK) {
      (void)ENCOSmotor_Disable(obj);
      obj->last_result = r;
    } else
      obj->last_result = send_frame(obj, &frame, obj->config.tx_group);
    /* 返回已发送状态，供上层按轮调度帧间隔。 */
    return 1;
  }
  return 0;
}
