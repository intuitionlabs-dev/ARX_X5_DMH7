#include "DM_motor.h"
#include "common.h"
#include "main.h"   // HAL_GetTick()
#include <stdlib.h> // malloc()
// CAN_ID == DM_motor_config.can_tx_id，即上位机中的 CAN ID
// MST_ID == DM_motor_config.can_rx_id，即上位机中的 Master ID
// 一般而言，CAN ID 和 Master ID 设置为同样值
/* 控制帧ID规则为 
    MIT模式：0x000+CAN_ID 
    位置速度模式：0x100+CAN_ID 
    速度模式：0x200+CAN_ID 
*/
/* 反馈帧ID为
    MST_ID
*/

#define DM_CAN_ID_MIN 0x1  // 为了不掩盖错误码，上位机设置的 CAN_ID 范围应该为 0x1~0xF
#define DM_CAN_ID_MAX 0xF  // 为了不掩盖错误码，上位机设置的 CAN_ID 范围应该为 0x1~0xF
#define DM_MIT_OFFSET 0x000        // MIT模式控制帧ID偏移
#define DM_POS_SPEED_OFFSET 0x100  // 位置速度模式控制帧ID偏移
#define DM_SPEED_OFFSET 0x200      // 速度模式控制帧ID偏移

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
    obj->errcode = data[0] >> 4;  // 错误码在高4位
    obj->err_status = (DM_err_status)obj->errcode;
    obj->fdb_position = (((int16_t)data[1] << 8) + data[2]) / (float)0x10000 * obj->config.max_position * 2 - obj->config.max_position;
    // uint16_t fdb_speed_temp = ((data[4] & 0xf) << 8) | data[5];

    obj->fdb_speed = (((int16_t)data[3] << 4) + (data[4] >> 4)) / (float)0x1000 * obj->config.max_speed * 2 - obj->config.max_speed;
    uint16_t fdb_torque_temp = ((data[4] & 0x0f) << 8) | data[5];
    obj->fdb_torque = (fdb_torque_temp - 2048) / 2048.0f * (obj->config.max_torque);
    obj->t_mos = data[6];
    obj->t_rotor = data[7];
    if (obj->fdb_position - obj->last_fdb_position > obj->config.max_position)  //这次比上次大很多，即为编码器从小过零导致本次收到反馈的机械角度很大而上次很小，故为往反方向转了一圈
        obj->round--;
    else if (obj->fdb_position - obj->last_fdb_position < -obj->config.max_position)  //这次比上次小很多，即为编码器从大过零导致本次收到的反馈的机械角度很小而上次很大，故为往正方向转了一圈
        obj->round++;
    obj->real_fdb_position = (obj->fdb_position + obj->round * 2* obj->config.max_position);
    FrameRateStatistics(&obj->motor_fps);
}

void DMmotor_RxCallBack(uint8_t can_id, uint32_t identifier, uint8_t *data, uint32_t len) {
    (void)len;
    // DEBUG_ASSERT(len == 8);

    /* 认电机要两个条件同时成立(与 ARX X5 SDK 的 MotorType2::CanAnalyze() 一致)：
     *  1. 仲裁ID == can_rx_id(= 上位机里的 MST_ID)。本工程 4 台达妙的出厂 MST_ID 都是 0，
     *     反馈帧的仲裁ID 因此全是 0，四台挤在同一条"通道"上；
     *  2. 载荷首字节低 4 位 == can_tx_id(= 上位机里的 CAN_ID)。达妙反馈帧 data[0] 的高 4 位
     *     是错误码、低 4 位才是电机自己的 ID，四台正是靠这个字段区分。
     * 少了第 2 条，MST_ID=0 时四台会互相抢帧；少了第 1 条，别处 ID 相同的数据也会被吃进来。
     * 若哪天把某台的 MST_ID 改成非 0，把它的 can_rx_id 跟着改即可，这个函数不用动。
     *
     * **再加一条：反馈必须来自这台电机自己的那条总线**（`can_id ==
     * obj->config.bsp_can_index`）。两台臂的达妙完全同构 —— 都是 MST_ID=0、
     * CAN_ID 5/6/7/8 —— 不按总线过滤的话，CAN1 的电机 5 会把 CAN2 电机 5 的反馈吃掉
     * (在全局 dm_motor_list 里按"仲裁ID + 载荷低 4 位"匹配，两条总线上这两台电机条件
     * 完全一样，先遍历到谁就是谁)。ENCOS 那边也按总线过滤(ENCOS_motor.c)。
     * 回调本身是按总线分别注册的(DMmotor_Create)。 */
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

// TODO: 测试DMmotor_Save_Zero函数
void DMmotor_Save_Zero(DM_motor *obj)
{
    uint8_t data[8] = {0xFF ,0xFF ,0xFF, 0xFF ,0xFF, 0xFF ,0xFF, 0xFE};
    BSP_CAN_Send(obj->config.bsp_can_index,
        obj->config.can_tx_id + obj->config.control_mode * 0x100, data, 8);
}

void DMmotor_Driver_Init(void)
{
    dm_motor_list = cvector_create(sizeof(DM_motor *));
    // BSP_CAN_RegisterRxCallback(0, DMmotor_RxCallBack);
    // BSP_CAN_RegisterRxCallback(1, DMmotor_RxCallBack);
}

DM_motor* DMmotor_Create(DM_motor_config* config , control_dm_func func)
{
    static uint8_t registry[DEVICE_CAN_CNT];
    if (config->bsp_can_index >= DEVICE_CAN_CNT) {// CAN索引超出范围
        /* TODO: 接入本工程日志后恢复
        printf_log("bsp_can_index out of range in %s", __func__); */
        return NULL;
    }
    // CAN ID 超出范围则警告
    if(config->can_tx_id < DM_CAN_ID_MIN || config->can_tx_id > DM_CAN_ID_MAX) {
        /* TODO: 接入本工程日志后恢复
        printf_log("can_tx_id out of range in %s\n", __func__); */
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
    /* count 的单位是 Monitor_Loop() 的周期(现 10 ms)：2 -> 连续 20 ms 收不到反馈
     * 就算掉线。Monitor_Loop() 的频率改了这里要跟着改。 */
    obj->monitor = Monitor_Register(obj->config.lost_callback_, 2, obj);
    obj->check_motor_inplace = DMMotor_Check_InPlace;
    /* 掩码 0 = 哪位都不属于 = 这一路永远发不出去，是最难查的那类故障，
     * 所以兜底成"每拍都发"。 */
    if (obj->config.tx_group == 0) {
        obj->config.tx_group = CAN_TX_GROUP_ALL;
    }
    cvector_pushback(dm_motor_list, (void *)&obj);
    // DMmotor_Enable(obj);
    return obj;
}

/* ==================== 发送：一拍一帧的步进 ====================
 *
 * **每次调用最多发一帧**，由 MotorTask 的轮转循环驱动(见 Core/Src/freertos.c 的
 * StartMotorTask)：**一轮里两条总线各发一帧，一轮只等一次**最小帧间隔
 * (BSP_CAN_TxGapWait)。一次把本总线该发的全发完会让两条总线各等一次，每路帧率砍半。
 *
 * 每个发送周期开始前先调 DMmotor_Send_Begin() 清 sent_this_tick —— 它保证"一台电机
 * 一拍只发一次"，不然轮转循环会把第一台反复发。
 */

#define DM_ENABLE_RESEND_MS 75u

/* 补发使能帧的轮转状态，**每条总线各一套**：一份全局的话 enable_next 会指到另一条
 * 总线的电机上，那台被补发的就永远轮不到。每条总线上 4 台达妙 → 75ms × 4 = 300ms/台。 */
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

    /* ---- ① 使能帧补发 ----
     * 每台电机每 300ms 补发一次使能信号，防止电机不成功启动。
     * 这帧是"载荷"：达妙在 MIT 下光收控制帧不会自己使能，所以它不能删。
     *
     * **但不能一次把 4 帧全发完。** 这 4 帧走 CAN_TX_GROUP_ALL，绕过分组，再叠上
     * 我们自己的最小帧间隔，那一拍要发 3(常规控制帧) + 4 = 7 帧 = 1.4ms，**必然**超过
     * MotorTask 的 1ms 预算。任务一超时，`tick += 1U` 就再也追不上真实时间，从此永久
     * 自由运行(实测 ~1312 Hz)，掩码分组给出的频率全错 —— 实测就是这么把 j5/j6 饿出
     * 20ms 看门狗、把遥控器 SWA→1 打回 arm_stop 的。
     *
     * 所以：每 75ms 只补一台、四台轮着来(每台还是 300ms 一次，但任何一拍最多只比别的
     * 拍多 1 帧)。**它占用这一轮的发送名额**：这一拍这一路就只发它，不能"使能帧 + 控制帧"
     * 连着发(那这一路一轮就出了两帧，"一包一等"就不成立了)。代价是那条总线上被轮到的
     * 电机这一拍少发一次控制帧，每 300ms 才一次。 */
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

    /* ---- ② 常规控制帧：本总线上"这一拍还没轮到"的下一台 ----
     * 相位不匹配的电机**整拍跳过(连 PID 都不算)**，但要标成"这一拍处理过了"，否则轮转
     * 循环会一直回头找它、永远走不到后面那些。 */
    for (size_t i = 0; i < dm_motor_list->cv_len; i++) {
        DM_motor *obj = *(DM_motor **)cvector_val_at(dm_motor_list, i);
        if (obj->config.bsp_can_index != can_id) continue;
        if (obj->sent_this_tick) continue;
        obj->sent_this_tick = 1;

        /* 控制频率由 obj->config.tx_group 的掩码决定：不在当前组的这一拍整拍跳过，
         * 连 PID 都不算 —— 不能"算了不发"，PID_Calc 没有 dt，增益按调用次数算，
         * 按 1 kHz 算而按 500 Hz 发会让 KI 凭空翻倍。发射那一步在 bsp_can 里还有
         * 一道同样的过滤(兜底)，这里是"连算都不算"。 */
        if (!BSP_CAN_TxGroupMatch(obj->config.tx_group)) {
            continue;
        }
        // if (obj->errcode) {
        //     DMmotor_Clear_Err(obj);
        //     continue;
        // }

		// 控制大喵电机
		//即使stop也应该更新motor_controller
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
            // obj->update = 0;
            // uint8_t cnt = 0;
            // while (obj->update != 1 && cnt < 50)
            // {
            //     BSP_Delay_us(5);
            //     cnt++;
            // }
        }
        // 电机stop模式
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
                // uint8_t send_data[8] = {0};
                // BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_tx_id + DMID_OFFSET + 0x100, send_data, 8);
                DMmotor_Disable(obj);
            }
            else if (obj->config.control_mode == dm_speed)
            {
                // uint8_t send_data[4] = {0};
                // BSP_CAN_Send(obj->config.bsp_can_index, obj->config.can_tx_id + DMID_OFFSET + 0x200, send_data, 4);
                DMmotor_Disable(obj);
            }
            // obj->update = 0;
            // uint8_t cnt = 0;
            // while (obj->update != 1 && cnt < 50)
            // {
            //     BSP_Delay_us(5);
            //     cnt++;
            // }
        }
        /* 走到这里就一定发出去了一帧 —— 两条控制路径(enable)和两条停/失能路径(stop)
         * 各自都只发一帧。这个不变式变了要同步改轮转循环的等待次数。 */
        return 1;
    }
    return 0;
}
