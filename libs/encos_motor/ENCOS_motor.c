#include "ENCOS_motor.h"
#include "main.h" /* HAL_GetTick()：源工程走 bsp_time.h 的 BSP_sys_time_ms()，那函数体就一行 return HAL_GetTick() */
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 源工程这里是 #include "bsp_log.h"（printf_log）。本工程 printf 是坏的
 * (__io_putchar 没定义)，而且全文件唯一一处 printf_log 上游已经注释掉了，
 * 所以这个 include 直接去掉。等接入本工程日志时再加回来。 */

/* Monitor_Loop() 的周期(ms)，见 Core/Src/freertos.c 的 StartMonitorTask()(tick += 10)。
 * monitor 的 count 单位是"Loop 被调了几次"不是毫秒，所以下面 Monitor_Register() 要除它。 */
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
/* tx_group: 见 libs/can/can_config.h。控制帧传 obj->config.tx_group(频率由掩码决定)；
 * 设零/查询这类必须送达的命令帧传 CAN_TX_GROUP_ALL，别被分组吃掉。 */
static ENCOS_Result send_frame(ENCOS_motor *obj, const ENCOS_Frame *frame,
                               uint8_t tx_group) {
  if (bus_busy(obj->config.bsp_can_index))
    return ENCOS_BUSY;
  BSP_CAN_Send(obj->config.bsp_can_index, frame->id, (uint8_t *)frame->data,
               frame->len, tx_group);
  return ENCOS_OK; /* submitted to BSP; NOT an acknowledgement from motor */
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
  /* 软停走"零增益混控帧"保活(见 ENCOS_motor.h 那个宏的注释)。damping_stop 是
   * 明确的阻尼刹车需求，仍走下面的 mode-3；PackMixed 失败(量程不合法)也不硬来。 */
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
      /* Prior coordinates are stale after zeroing; require new telemetry. */
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
    // printf_log("Invalid ENCOS config\n");
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
  /* 掩码 0 = 哪位都不属于 = 这一路永远发不出去，是最难查的那类故障，
   * 所以按旧语义兜底成"每拍都发"(以前 fre_rel 默认 1 就是这个意思)。 */
  if (!obj->config.tx_group)
    obj->config.tx_group = CAN_TX_GROUP_ALL;
  if (!obj->config.feedback_timeout_ms)
    obj->config.feedback_timeout_ms = ENCOS_OFFLINE_TIMEOUT_MS;
  /* Allocation failures are checked before publishing the motor to callbacks.
   */
  obj->motor_controller =
      create_controller(&obj->config.motor_controller_config);
  if (!obj->motor_controller) {
    free(obj);
    return NULL;
  }
  /* count 的单位是 Monitor_Loop() 的周期，**不是毫秒**（见 libs/monitor/monitor.h）。
   * 源工程那边 Monitor_Loop() 跑 1 kHz，传毫秒数正好；本工程是 100 Hz(10 ms)，
   * 照抄会让 100 ms 的超时变成 1 s。所以这里要除掉周期。
   * 改 StartMonitorTask() 的频率，这个换算跟着改。 */
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
/* 使能原来是这里的 ENCOSmotor_Enable()，2026-10-05 把它删了：应用层只写
 * obj->enable，跳变处理搬进 ENCOSmotor_Calc_Send() 的 ⓪（理由见那里）。
 * 注意 reset_controller() 现在是 Calc_Send 的 ⓪ 和 ENCOSmotor_Disable() 两处调。 */
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
  return obj->last_result = r == ENCOS_OK
                                ? send_frame(obj, &frame, CAN_TX_GROUP_ALL)
                                : r;
}
const char *ENCOSmotor_Err_String(uint8_t code) {
  return ENCOS_ErrorString(code);
}
ENCOS_Result ENCOSmotor_Clear_Err(ENCOS_motor *obj) {
  (void)obj;
  return ENCOS_UNSUPPORTED; /* No clear-fault command specified in manual. */
}
ENCOS_Result ENCOSmotor_Reset_Round(ENCOS_motor *obj) {
  (void)obj;
  return ENCOS_UNSUPPORTED; /* No invented 25rad / 2pi position wrapping. */
}
ENCOS_Result ENCOSmotor_Broadcast_Cmd(ENCOS_motor *obj, uint8_t sub_cmd) {
  return sub_cmd == 3 ? ENCOSmotor_Save_Zero(obj) : ENCOS_UNSUPPORTED;
}
void ENCOSmotor_Calc_Send(void) {
  if (!ENCOS_motor_list)
    return;
  for (size_t i = 0; i < ENCOS_motor_list->cv_len; ++i) {
    ENCOS_motor *obj = *(ENCOS_motor **)cvector_val_at(ENCOS_motor_list, i);
    if (bus_busy(obj->config.bsp_can_index))
      continue;
    /* Missing zero ACK is observable; do not retain an eternal pending state.
     */
    if (obj->zero_pending &&
        (uint32_t)(now_ms() - obj->zero_sent_ms) >= ENCOS_ZERO_ACK_TIMEOUT_MS)
      obj->zero_pending = 0;
    /* 控制频率由 obj->config.tx_group 的掩码决定：不在当前组的这一拍整拍跳过。
     * 不能"算了不发"：PID_Calc 没有 dt，增益按调用次数算，按 1 kHz 算而按
     * 500 Hz 发会让 KI 凭空翻倍。(上面的 zero_pending 超时清理是"每隔多久检查
     * 一次"，跟发送无关，所以留在门外面。) */
    if (!BSP_CAN_TxGroupMatch(obj->config.tx_group))
      continue;

    /* ---- ⓪ 使能边沿在驱动内部落地 ------------------------------------------
     * 应用只写 obj->enable，没有 ENCOSmotor_Enable() 可调，所以 stop -> enable
     * 这个跳变得由驱动自己抓：抓到就清一次控制器状态（原来在 Enable() 里做），
     * 清完紧接着 ① 就算，第一帧不会带着上面 stop 期间攒下的积分发出去。
     * 用"锁存"而不是"和上一拍比"：下面 ② 可能因为反馈不新鲜把 enable 打回
     * stop，比较法会在下一拍把环再清一次。清环**不能**挪进 ① 每拍都跑，
     * 否则 stop 期间算出来的积分/微分下一拍就被抹掉，"stop 时也算"就白算了。 */
    if (obj->enable == encos_enable) {
      if (!obj->enable_cleared) {
        reset_controller(obj);
        obj->enable_cleared = 1;
      }
    } else
      obj->enable_cleared = 0;

    /* ---- ① 只要轮到自己这一组，就无条件算一遍 controller --------------------
     * **和使能状态无关**：stop 时也照算，对齐 DMmotor_Calc_Send 里那句
     * "即使 stop 也应该更新 motor_controller"。发什么由下面的 ② 决定，
     * 所以停着的电机 controller 里的 fdb/ref/积分/微分一直跟着走，
     * 不会因为"这一拍整拍跳过了"而在切到 enable 的那一拍跳一下。 */
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
      continue;
    }
    if (obj->control_calc)
      obj->control_calc(obj);
    else {
      controller_calc(c);
      obj->ref_torque = obj->config.output_mode == encos_output_reverse
                            ? -c->output
                            : c->output;
    }

    /* ---- ② 算完之后的行为由状态决定 ----------------------------------------
     * 没使能(或反馈不新鲜/有错误码/正在设零) -> 这一拍发停止帧；否则发混控控制帧。
     * 注意这里**不再**每拍调 reset_controller()：那样 ① 刚算出来的积分/微分下一拍
     * 就被抹掉，"stop 时也算"就白算了。清环只发生在上面 ⓪ 的使能边沿里，
     * 以及 ENCOSmotor_Disable()(后者也是下面两条故障路径的收尾)。 */
    if (obj->enable != encos_enable || !motion_ready(obj)) {
      obj->enable = encos_stop;
      obj->last_result = send_stop(obj);
      continue;
    }
    if (bus_busy(obj->config.bsp_can_index))
      continue;
    /* control_calc 是外部钩子，理论上能在上面把 enable 改掉，所以再问一次 */
    if (obj->enable != encos_enable) {
      obj->last_result = send_stop(obj);
      continue;
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
  }
}
