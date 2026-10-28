#include "gimbal_arm.h"
#include <stdlib.h>
#include <string.h>
#include "arm.h"
#include "common.h"
#include "dm_mit_control.h"
#include "gimbal_arm_def.h"

static void Motor_Lost(void* motor) { (void)motor; }
static float Metal_Pos_Fdb_Calc(void* arg);
static float Metal_Speed_Fdb_Calc(void* arg);
static void Metal_Mit_Control(DM_motor* motor);
static void Motor_Lost_Handle(Gimbal* obj);
static uint8_t Arm_Auto_Init(Gimbal* obj);
static void Get_Motor_Fdb(Gimbal* obj);
static void Joint_Dynamics_Comp_Calc(Gimbal* obj);
static void Set_Motor_ref(Gimbal* obj);
static void pid_init(Gimbal* obj);
static void set_pid_normal(Gimbal* obj);
static void set_pid_zero(Gimbal* obj);
static void set_pid_damping(Gimbal* obj);
static PID_Config zero_config;

/* 调试器观测臂 A 的 J3。 */
float g_fdb_pos, g_fdb_speed, g_ref_pos, g_ref_speed;
float g_speed_comp, g_output,g_speed_output, g_ouput_comp;

Gimbal* Gimbal_Create(uint8_t can_index) {
    if (can_index >= ARM_NUM) return NULL;
    Gimbal* obj = calloc(1, sizeof(Gimbal));
    if (obj == NULL) return NULL;
    obj->can_index = can_index;
    pid_init(obj);
    DM_motor** slots[ARM_MOTOR_NUM] = {&obj->joint1, &obj->joint2, &obj->joint3,
        &obj->joint4, &obj->joint5, &obj->joint6, &obj->gripper};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        DM_motor_config config = {
            .bsp_can_index = can_index,
            .can_tx_id = METAL_MOTOR_TX_ID[i],
            .can_rx_id = METAL_MOTOR_RX_ID[i],
            .motor_controller_config = {
                .control_type = PID_MODEL,
                .control_depth = POS_CONTROL_CASCADE,
                .position_pid_config = obj->pid_cfgs[i][0],
                .speed_pid_config = obj->pid_cfgs[i][1],
            },
            .pos_fdb_calc = Metal_Pos_Fdb_Calc,
            .speed_fdb_calc = Metal_Speed_Fdb_Calc,
            .lost_callback_ = Motor_Lost,
            .control_mode = dm_mit,
            .output_mode = dm_output_normal,
            .tx_group = METAL_MOTOR_TX_GROUP[i],
            .max_position = METAL_MOTOR_PMAX,
            .max_speed = METAL_MOTOR_VMAX[i],
            .max_torque = METAL_MOTOR_TMAX[i],
            .max_kp = 500.0f,
            .max_kd = 5.0f,
        };
        DM_motor* motor = DMmotor_Create(&config, Metal_Mit_Control);
        if (motor == NULL) return NULL;
        *slots[i] = motor;
        motor->config.arg = motor;
        motor->kp = obj->pid_mit_kpkd[i][0];
        motor->kd = obj->pid_mit_kpkd[i][1];
        obj->motor_controller_list[i] = motor->motor_controller;
        obj->motor_monitor_list[i] = motor->monitor;
        motor->monitor->count = 0;
    }
    PID_SetConfig_Pos(&zero_config, 0, 0, 0, 0, 0);
    return obj;
}

void Gimbal_Update(Gimbal* obj) {
    /* 取指令*/
    obj->arm_mode = obj->arm_control_data.mode;

    /* 电机掉线相关处理 */
    Motor_Lost_Handle(obj);  // 放在最前面

    // 首次上电自动初始化。
    /* 标志位在 obj 里(不是函数内 static)：两台臂各建一次，共用一个 static 的话
     * 后建的那台一进来就看到标志是 1，Arm_Auto_Init() 永远不跑。 */
    if (obj->first_init_flag == 0) {
        //成功后返回1，失败返回0
        obj->first_init_flag = Arm_Auto_Init(obj);
        if (!obj->first_init_flag) obj->arm_mode = arm_stop;  // 自动初始化未成功则停机
    }

    /* 获取电机反馈 */
    Get_Motor_Fdb(obj);

    /* 使用本周期反馈计算关节力矩。 */
    Joint_Dynamics_Comp_Calc(obj);

    /* 按模式挑 PID 配置 / kp / kd。**每拍都重设**，所以模式一换下一拍就生效，
     * 不需要在切换处做任何额外动作。
     *
     * 四种模式在这里分成三档：
     *   arm_run                          位置环正常，kp/kd 正常；
     *   arm_damping                      位置环全关，kp=0，kd=JOINT_DAMPING_MODE_KD；
     *   arm_only_gravity_comp            位置环全关，kp=kd=0；
     *   arm_stop / 其它                   走 normal —— 失能态下这些值用不上（电机不发
     *                                    力矩），但让它和 run 一致最省心：万一哪天从
     *                                    stop 直接使能，配置不会是"上一个模式的残留"。 */
    switch (obj->arm_mode) {
    case arm_damping:
        set_pid_damping(obj);
        break;
    case arm_only_gravity_comp:
        set_pid_zero(obj);
        break;
    case arm_run:
    case arm_stop:
    default:
        set_pid_normal(obj);
        break;
    }

    /* 电机启停标志由 MotorTask 中的 DM 驱动消费。 */
    DM_motor* motors[ARM_MOTOR_NUM] = {obj->joint1, obj->joint2, obj->joint3,
        obj->joint4, obj->joint5, obj->joint6, obj->gripper};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i)
        motors[i]->enable = obj->arm_mode == arm_stop ? dm_stop : dm_enable;

    /* 电机控制 */
    Set_Motor_ref(obj);

    // DEBUG:拷贝控制器数据
    if (obj == g_arm[0].gimbal) {
        g_fdb_pos = obj->joint2->motor_controller->fdb_position;
        g_fdb_speed = obj->joint2->motor_controller->fdb_speed;
        g_ref_pos = obj->joint2->motor_controller->ref_position;
        g_ref_speed = obj->joint2->motor_controller->ref_speed;
        g_speed_comp = obj->joint2->motor_controller->speed_comp;
        g_output = obj->joint2->motor_controller->output;
        g_ouput_comp = obj->joint2->motor_controller->output_comp;
        g_speed_output = obj->joint2->motor_controller->pid_speed_data.output;
    }

    /* 杂项 */
    obj->arm_feedback_data.mode = obj->arm_mode;
}

static uint8_t Arm_Auto_Init(Gimbal* obj) {
    // 判断有无电机掉线
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        if (obj->arm_feedback_data.is_motor_lost[i]) {
            return 0;  // 机械臂存在掉线电机则自动初始化失败
        }
    }

    // j1-j7电机具有绝对值编码器
    obj->motor_pos_init[JOINT1] = JOINT1_POS_INIT;
    obj->motor_pos_init[JOINT2] = JOINT2_POS_INIT;
    obj->motor_pos_init[JOINT3] = JOINT3_POS_INIT;
    obj->motor_pos_init[JOINT4] = JOINT4_POS_INIT;
    obj->motor_pos_init[JOINT5] = JOINT5_POS_INIT;
    obj->motor_pos_init[JOINT6] = JOINT6_POS_INIT;
    // 夹爪默认未初始化
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->arm_feedback_data.motor_inited[i] = 1;  // j1-j6电机初始化完成
    }

    return 1;  // 初始化完成
}

static void Motor_Lost_Handle(Gimbal* obj) {
    /* 检测 */
    uint8_t any_motor_lost = 0;  // 初始化为0
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        obj->arm_feedback_data.is_motor_lost[i] = is_Offline(obj->motor_monitor_list[i]);
        // 只要有一个电机掉线，就认为存在掉线
        any_motor_lost += obj->arm_feedback_data.is_motor_lost[i];
    }

    // 当j6掉线且未开启掉线保护，认为其未初始化
    if (ANY_MOTOR_LOST_STOP_ENABLE) {
        if (obj->arm_feedback_data.is_motor_lost[JOINT6])
            obj->arm_feedback_data.motor_inited[JOINT6] = 0;
    }
    // j1~j6始终认为已初始化，因为它们具有绝对值编码器，掉线后重启仍能获得正确的角度反馈，不会出现误判掉线的情况
    obj->arm_feedback_data.motor_inited[JOINT1] = 1;
    obj->arm_feedback_data.motor_inited[JOINT2] = 1;
    obj->arm_feedback_data.motor_inited[JOINT3] = 1;
    obj->arm_feedback_data.motor_inited[JOINT4] = 1;
    obj->arm_feedback_data.motor_inited[JOINT5] = 1;
    obj->arm_feedback_data.motor_inited[JOINT6] = 1;

    /* 处理 */
    if (any_motor_lost && ANY_MOTOR_LOST_STOP_ENABLE) {  // 若配置为掉线停机则停机
        obj->arm_mode = arm_stop;                        // 存在掉线电机，进入stop状态
    }
}

static void Set_Motor_ref(Gimbal* obj) {
    if (obj->arm_mode == arm_stop) {  // stop状态
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            // 电机位置跟随，防止跳变
            obj->motor_controller_list[i]->ref_position =
                obj->motor_controller_list[i]->fdb_position;
            // 电机速度前馈置零
            obj->motor_controller_list[i]->speed_comp = 0;
            // 电机输出补偿置零
            obj->motor_controller_list[i]->output_comp = 0;
        }
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            // 关节粘性摩擦前馈置零
            obj->damp_comp[i] = 0;
        }
        return;
    }

    /* 被动模式下目标位置跟随反馈；纯重力模式输出静态支撑力矩。 */
    if (obj->arm_mode == arm_damping || obj->arm_mode == arm_only_gravity_comp) {
        const uint8_t gravity_on = (obj->arm_mode == arm_only_gravity_comp);
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            // 电机位置跟随，防止跳变
            obj->motor_controller_list[i]->ref_position =
                obj->motor_controller_list[i]->fdb_position;
            // 电机速度前馈置零
            obj->motor_controller_list[i]->speed_comp = 0;
            // 电机输出补偿：先全清，要重力的话下面单独给 6 个关节补
            obj->motor_controller_list[i]->output_comp = 0;
        }
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            // 被动模式的粘性摩擦前馈为零。
            obj->damp_comp[i] = 0;
            if (gravity_on) {
                /* 关节力矩转换为电机输出。 */
                obj->motor_controller_list[i]->output_comp = MOTOR_DIR[i]
                    * obj->tau_comp[i] * JOINT_TORQUE_TRANS[i] * JOINT_TORQUE_TRANS_COEF[i];
            }
        }
        return;
    }

    /* 设置关节电机目标位置、速度和力矩前馈 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        // 位置
        obj->motor_controller_list[i]->ref_position =
            angle_to_pos(obj->arm_control_data.joint_angle[i], obj->motor_pos_init[i],
                MOTOR_DIR[i], MOTOR_REDUCTION[i]);
        // 速度前馈
        obj->motor_controller_list[i]->speed_comp =
            vel_to_speed(obj->arm_control_data.joint_vel[i], MOTOR_DIR[i], MOTOR_REDUCTION[i])
            * JOINT_SPEEDCOMP_COEF[i];
        // 粘性摩擦前馈
        obj->damp_comp[i] = obj->arm_control_data.joint_vel[i] * DEG2RAD
            * JOINT_DAMPING[i];
        // 总力矩前馈,带有修正系数
        obj->motor_controller_list[i]->output_comp = MOTOR_DIR[i]
            * (obj->tau_comp[i] + obj->damp_comp[i]) * JOINT_TORQUE_TRANS[i]
            * JOINT_TORQUE_TRANS_COEF[i];
    }

    /* 设置夹爪电机目标位置和力矩 */
    obj->motor_controller_list[GRIPPER]->ref_position = obj->arm_control_data.gripper_angle;
    obj->motor_controller_list[GRIPPER]->output_comp = obj->arm_control_data.gripper_torque;
}

static void Get_Motor_Fdb(Gimbal* obj) {
    /* 控制器回调已经按 Metal 协议转换为 deg / deg/s。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->arm_feedback_data.joint_angle[i] =
            pos_to_angle(obj->motor_controller_list[i]->fdb_position,
                obj->motor_pos_init[i], MOTOR_DIR[i], MOTOR_REDUCTION[i]);
        obj->arm_feedback_data.joint_vel[i] =
            speed_to_vel(obj->motor_controller_list[i]->fdb_speed,
                MOTOR_DIR[i], MOTOR_REDUCTION[i]);
    }
    obj->arm_feedback_data.gripper_angle = obj->gripper->motor_controller->fdb_position;
    DM_motor* motors[ARM_MOTOR_NUM] = {obj->joint1, obj->joint2, obj->joint3,
        obj->joint4, obj->joint5, obj->joint6, obj->gripper};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i)
        obj->arm_feedback_data.motor_temp[i] = motors[i]->t_rotor;
}

/* arm_run：M(q)*qdd_ref + sC*C(q,qd_ref)*qd_ref + sG*G(q) + F(qd_ref)。
 * q 使用关节反馈位置，qd_ref/qdd_ref 使用命令层的参考速度/加速度。
 * 普通位置目标及复位由同一条五次样条提供两阶导数；0x17 使用外部速度、零加速度。
 * 纯重力模式仅输出 sG*G(q)。
 */
static void Joint_Dynamics_Comp_Calc(Gimbal* obj) {
#if JOINT_DYNAMICS_COMP_ENABLE
    _Static_assert(ARM_JOINT_NUM == METAL_GRAVITY_DOF, "动力学模型关节数必须一致");
    if (obj->arm_mode != arm_run && obj->arm_mode != arm_only_gravity_comp) {
        memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
        obj->dynamics_status = METAL_DYNAMICS_OK;
        return;
    }

    metal_real q[ARM_JOINT_NUM], qd[ARM_JOINT_NUM], qdd[ARM_JOINT_NUM];
    metal_real torque[ARM_JOINT_NUM];
    int status;
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        q[i] = obj->arm_feedback_data.joint_angle[i] * DEG2RAD;
        qd[i] = obj->arm_control_data.joint_vel[i] * DEG2RAD;
        qdd[i] = obj->arm_control_data.joint_acc[i] * DEG2RAD;
    }
    if (obj->arm_mode == arm_only_gravity_comp) {
        status = metal_gravity_compute(JOINT_DYNAMICS_COMP_MODEL, q,
            JOINT_DYNAMICS_GRAVITY_BASE, JOINT_DYNAMICS_GRAVITY_MODE, torque);
    } else {
        status = metal_dynamics_compute(JOINT_DYNAMICS_COMP_MODEL, q, qd, qdd,
            JOINT_DYNAMICS_GRAVITY_BASE, JOINT_DYNAMICS_GRAVITY_MODE, torque);
    }
    if (status != METAL_DYNAMICS_OK) {
        memset(torque, 0, sizeof(torque));
        obj->arm_mode = arm_stop;
    }
    memcpy(obj->tau_comp, torque, sizeof(obj->tau_comp));
    obj->dynamics_status = status;
#else
    memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
    obj->dynamics_status = METAL_DYNAMICS_OK;
#endif
}

static void pid_init(Gimbal* obj) {
    memset(obj->pid_cfgs, 0, sizeof(obj->pid_cfgs));

    /* 下标 0：位置环；下标 1：速度环。
     * 参数顺序：Kp、Ki、Kd、积分累加限幅、输出限幅。
     */
    // joint1
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT1][0], 6, 0.00, 50, 200, 1000);
    obj->pid_cfgs[JOINT1][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT1][1], 0.000199999995, 0.00150000001, 9.99999975e-05, 500, 12);
    obj->pid_cfgs[JOINT1][1].Irange = 20;
    obj->pid_mit_kpkd[JOINT1][0] = 30;
    obj->pid_mit_kpkd[JOINT1][1] = 5;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT2][0], 4, 0, 50, 200, 1000);
    obj->pid_cfgs[JOINT2][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT2][1], 0.2, 0.0f, 0.0f, 0.0f, 12);
    obj->pid_mit_kpkd[JOINT2][0] = 30;
    obj->pid_mit_kpkd[JOINT2][1] = 5;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT3][0], 4, 0, 50, 200, 1000);
    obj->pid_cfgs[JOINT3][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT3][1], 0.1, 0.0f, 0.0f, 0.0f, 12);
    obj->pid_mit_kpkd[JOINT3][0] = 50;
    obj->pid_mit_kpkd[JOINT3][1] = 5;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT4][0], 10, 0.08, 50, 200, 1000);
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT4][1], 0, 0, 0, 0, 0);
    obj->pid_cfgs[JOINT4][0].Irange = 0;
    obj->pid_cfgs[JOINT4][1].Irange = 0;
    obj->pid_mit_kpkd[JOINT4][0] = 50;
    obj->pid_mit_kpkd[JOINT4][1] = 4;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT5][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[JOINT5][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT5][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[JOINT5][0] = 10;
    obj->pid_mit_kpkd[JOINT5][1] = 0.8;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT6][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[JOINT6][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT6][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[JOINT6][0] = 10;
    obj->pid_mit_kpkd[JOINT6][1] = 1;

    PID_SetConfig_Pos(&obj->pid_cfgs[GRIPPER][0], 0, 0, 0, 0, 0);
    PID_SetConfig_Pos(&obj->pid_cfgs[GRIPPER][1], 0, 0, 0, 0, 0);
    obj->pid_cfgs[GRIPPER][0].Irange = 0;
    obj->pid_cfgs[GRIPPER][1].Irange = 0;
    obj->pid_mit_kpkd[GRIPPER][0] = 20;
    obj->pid_mit_kpkd[GRIPPER][1] = 0.1f;
}

static void set_pid_normal(Gimbal* obj) {
    DM_motor* motors[ARM_MOTOR_NUM] = {obj->joint1, obj->joint2, obj->joint3,
        obj->joint4, obj->joint5, obj->joint6, obj->gripper};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        motors[i]->motor_controller->pid_pos_data.config = obj->pid_cfgs[i][0];
        motors[i]->motor_controller->pid_speed_data.config = obj->pid_cfgs[i][1];
        motors[i]->kp = obj->pid_mit_kpkd[i][0];
        motors[i]->kd = obj->pid_mit_kpkd[i][1];
    }
}

static void set_pid_damping(Gimbal* obj) {
    DM_motor* motors[ARM_MOTOR_NUM] = {obj->joint1, obj->joint2, obj->joint3,
        obj->joint4, obj->joint5, obj->joint6, obj->gripper};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        motors[i]->motor_controller->pid_pos_data.config = zero_config;
        motors[i]->motor_controller->pid_speed_data.config = zero_config;
        motors[i]->kp = 0;
        motors[i]->kd = JOINT_DAMPING_MODE_KD[i];
    }
}

static void set_pid_zero(Gimbal* obj) {
    set_pid_normal(obj);
    DM_motor* motors[ARM_JOINT_NUM] = {obj->joint1, obj->joint2, obj->joint3,
        obj->joint4, obj->joint5, obj->joint6};
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        motors[i]->motor_controller->pid_pos_data.config = zero_config;
        motors[i]->motor_controller->pid_speed_data.config = zero_config;
        motors[i]->kp = motors[i]->kd = 0;
    }
}

/* DM 驱动保留原始反馈及其内部刻度；应用层使用 Metal 的 65535/4095 刻度。
 * 回调只在 MotorTask 更新控制器反馈，设零复核使用同一位置换算。 */
float Gimbal_Motor_Position_Rad(const DM_motor* motor) {
    if ((motor->rx_data[0] & 0x0f) != motor->config.can_tx_id) return 0.0f;
    uint16_t raw = ((uint16_t)motor->rx_data[1] << 8) | motor->rx_data[2];
    return raw / 65535.0f * (2.0f * motor->config.max_position) - motor->config.max_position;
}

static float Metal_Pos_Fdb_Calc(void* arg) {
    return Gimbal_Motor_Position_Rad((DM_motor*)arg) * RAD2DEG;
}

static float Metal_Speed_Fdb_Calc(void* arg) {
    DM_motor* motor = arg;
    if ((motor->rx_data[0] & 0x0f) != motor->config.can_tx_id) return 0.0f;
    uint16_t raw = ((uint16_t)motor->rx_data[3] << 4) | (motor->rx_data[4] >> 4);
    return (raw / 4095.0f * (2.0f * motor->config.max_speed) - motor->config.max_speed) * RAD2DEG;
}

static void Metal_Mit_Control(DM_motor* motor) {
    DM_Mit_Control(motor);
    /* Metal 目标与反馈均为原始输出轴角，不使用驱动累计圈数修正目标。 */
    motor->ref_position = motor->motor_controller->ref_position * DEG2RAD;
}
