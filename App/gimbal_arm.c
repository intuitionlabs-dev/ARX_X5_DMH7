#include "gimbal_arm.h"
#include <stdlib.h> /* malloc()，源文件漏了这个 */
#include <string.h>
#include "common.h"
#include "dm_mit_control.h"      /* DM_Mit_Control() */
#include "encos_mixed_control.h" /* ENCOS_Mixed_Control() */
#include "gimbal_arm_def.h"
#include "temperature_protect.h"

/* 本工程没有的框架，先注释掉：
#include "arm_serial_link/arm_serial_link.h"  运动学/动力学(Newton-Euler)，见 Joint_Feedforward_Calc()
#include "print_help.h"                       printf_log()/Print_Float()，本工程 printf 是坏的
#include "pub_sub.h"                          指令来源，现由命令层直接写 arm_control_data
*/

// 使pid不输出，只使用前馈，便于调试
// #define JOINT_ONLY_COMP
// 打印关节角
// #define PRINT_ANGLE   /* 本工程 printf 是坏的(__io_putchar 没定义)，别开 */

/* 电机丢失回调函数。
 * 本来各自 printf_log 一句，本工程 printf 是坏的，所以只留空壳 —— 函数保留、传给了
 * lost_callback_，在调试器里给对应的那个挂断点就能知道是哪台电机掉线。
 * 接入日志后把 (void)motor; 换成 printf_log 即可。 */
void Joint1_Motor_Lost(void* motor) { (void)motor; }  /* joint1 ENCOS lost */
void Joint2_Motor_Lost(void* motor) { (void)motor; }  /* joint2 ENCOS lost */
void Joint3_Motor_Lost(void* motor) { (void)motor; }  /* joint3 ENCOS lost */
void Joint4_Motor_Lost(void* motor) { (void)motor; }  /* joint4 DM4310 lost */
void Joint5_Motor_Lost(void* motor) { (void)motor; }  /* joint5 DM4310 lost */
void Joint6_Motor_Lost(void* motor) { (void)motor; }  /* joint6 DM4310 lost */
void Gripper_Motor_Lost(void* motor) { (void)motor; } /* gripper DM4310 lost */
/* 反馈计算函数 */
//float Joint1_Pos_Fdb_Calc(void* arg);
// float Joint6_Pos_Fdb_Calc(void* arg);
// float Joint2_Speed_Fdb_Calc(void* arg);
// float Joint3_Speed_Fdb_Calc(void* arg);
// float Joint1_Speed_Fdb_Calc(void* lkmotor);
// float Joint23_Pos_Fdb_Calc(void* arg);
/* 自定义控制函数 DM_Mit_Control() 现在在 libs/dm_motor/dm_mit_control.h */

/* 主函数 */
// NOLINTBEGIN
static void Motor_Lost_Handle(Gimbal* obj);  // 电机掉线处理
static uint8_t Arm_Auto_Init(Gimbal* obj);   // 初次上电自动初始化
static void Get_Motor_Fdb(Gimbal* obj);      // 电机反馈
// static void Joint_Feedforward_Calc(Gimbal* obj);  // 动力学前馈计算(未接入，见下文)
static void Set_Motor_ref(Gimbal* obj);  // 电机控制
// NOLINTEND
/* 辅助函数 */
static void pid_init(Gimbal* obj);                            // 初始化pid参数
static void set_pid_normal(Gimbal* obj);                      // 给pid赋值
static void set_pid_zero(Gimbal* obj);                        // 给pid置0
static uint8_t encos_joint_init(Gimbal* obj, uint8_t joint);  // j1~j3 建完之后的公共收尾+判空
static float lowpass_filter(float current_speed, float a, uint8_t index);  // 低通滤波器
// 低通滤波后堵转检测
static uint8_t stalling_detect(controller* controller, float speed_diff, float pos_step,
    uint16_t stall_time, uint8_t index, float a);
/* 私有常量 */
static PID_Config zero_config;

/* j1~j3 的 ENCOS(V1.20EAP，就是 ARX X5 的 MotorType4)协议量程。数字取自
 * libs/gravity_compensation/sdk_parameters/parameters.json 里 motor_protocols 的
 * Type4 那组常量(驱动打 kd 用 9 bit，和 SDK 的 packed_bits[3]=9 对得上)。
 *
 * **这组数必须和电机里存的量程一致**，否则解码出来的位置/速度全是错的 ——
 * ENCOSmotor_Create() 只校验"格式合法"，不校验"和电机一致"。上电后拿
 * ENCOSmotor_Query(obj, 23..28) 把六项读回来对一遍。 */
static const ENCOS_Ranges ENCOS_JOINT_RANGES = {
    .kp = {0.0f, 500.0f},         // Type4 kp_min/kp_max
    .kd = {0.0f, 50.0f},          // Type4 kd_max
    .position = {-12.5f, 12.5f},  // Type4 position_min/max
    .speed = {-18.0f, 18.0f},     // Type4 velocity_min/max
    .torque = {-30.0f, 30.0f},    // Type4 effort_min/max
    .current = {-30.0f, 30.0f},   // 见上面「必须和电机一致」那段，实测核对
};

/* 私有变量 */
static controller* motor_controller_list[ARM_MOTOR_NUM];  // 电机控制器数组，便于缩减代码冗余
static monitor_item* motor_monitor_list[ARM_MOTOR_NUM];  // 电机监视器数组，便于缩减代码冗余

/* 机械臂对象，Gimbal_Create() 之后有效(Create 在 StartCmdTask 里调，见 gimbal_arm.h) */
Gimbal* gimbal;

// DEBUG:
float g_fdb_pos;
float g_fdb_speed;
float g_ref_pos;
float g_ref_speed;
float g_speed_comp;
float g_output;
float g_ouput_comp;

/* j1~j3 建好之后的公共收尾：回填 arg、把 kp/kd 从 encos_kpkd 抄进电机、登记
 * controller/monitor。返回 0 表示电机没建起来。
 *
 * 为什么要判空：ENCOSmotor_Create() 的入参校验比 DMmotor_Create() 严得多(量程必须
 * 合法、tx_id 必须等于 rx_id、同一总线同一个 rx_id 不能建两次……)，配置写错就是
 * 返回 NULL，而原来达妙那种 `obj->jointN->config.arg = ...` 的写法会直接空指针。
 * 达妙那几行是历史遗留，这里新写的就不跟着犯了。 */
static uint8_t encos_joint_init(Gimbal* obj, uint8_t joint) {
    ENCOS_motor* motor = joint == JOINT1 ? obj->joint1
        : joint == JOINT2                ? obj->joint2
                                         : obj->joint3;
    if (motor == NULL) return 0;
    motor->config.arg = motor;
    motor->kp = obj->encos_kpkd[joint][0];
    motor->kd = obj->encos_kpkd[joint][1];
    motor_controller_list[joint] = motor->motor_controller;
    motor_monitor_list[joint] = motor->monitor;
    return 1;
}

Gimbal* Gimbal_Create(void) {
    /* 创建实例 */
    Gimbal* obj = (Gimbal*)malloc(sizeof(Gimbal));
    memset(obj, 0, sizeof(Gimbal));

    /* 各关节电机创建 */
    pid_init(obj);
    controller_config joint_controller_config[ARM_MOTOR_NUM];
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        memset(&joint_controller_config[i], 0, sizeof(controller_config));
        joint_controller_config[i].control_type = PID_MODEL;
        joint_controller_config[i].control_depth = POS_CONTROL_CASCADE;
        joint_controller_config[i].position_pid_config.D_ahead = D_AHEAD_UNUSED;
        joint_controller_config[i].speed_pid_config.D_ahead = D_AHEAD_UNUSED;
        joint_controller_config[i].position_pid_config = obj->pid_cfgs[i][0];
        joint_controller_config[i].speed_pid_config = obj->pid_cfgs[i][1];
    }
    {
        // Joint1 ENCOS V1.20EAP(原 dm8006 的位置)。CAN id 用 SDK 那套：j1 = 1
        ENCOS_motor_config joint1_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT1],
            .bsp_can_index = 2, 
            .can_tx_id = 0x01,
            .can_rx_id = 0x01,  // ENCOS 要求收发同 id，否则 Create 直接返回 NULL
            .lost_callback_ = Joint1_Motor_Lost,
            .output_mode = encos_output_normal,
            .ranges = ENCOS_JOINT_RANGES,
            .torque_constant = 0.0f,  // SDK 没给 Kt -> fdb_torque 恒为 NAN(本工程没人用)
            .position_offset = 0.0f,  // SDK: motor_type4_position.offset = 0.0
            .feedback_timeout_ms = 100,  // SDK: Type4 timeout_us = 100000
            .stop_mode = encos_soft_stop,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = NULL,
            .arg = NULL,   // 速度或位置环的反馈计算函数的对象
            .fre_rel = 2,  //500Hz
        };
        obj->joint1 = ENCOSmotor_Create(&joint1_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT1)) return NULL;

        // Joint2 ENCOS V1.20EAP(原 dm8009 的位置)。CAN id 用 SDK 那套：j2 = 2
        ENCOS_motor_config joint2_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT2],
            .bsp_can_index = 2,
            .can_tx_id = 0x02,
            .can_rx_id = 0x02,  // ENCOS 要求收发同 id，否则 Create 直接返回 NULL
            .lost_callback_ = Joint2_Motor_Lost,
            .output_mode = encos_output_normal,
            .ranges = ENCOS_JOINT_RANGES,
            .torque_constant = 0.0f,  // SDK 没给 Kt -> fdb_torque 恒为 NAN(本工程没人用)
            .position_offset = 0.0f,  // SDK: motor_type4_position.offset = 0.0
            .feedback_timeout_ms = 100,  // SDK: Type4 timeout_us = 100000
            .stop_mode = encos_soft_stop,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = /* Joint2_Speed_Fdb_Calc */ NULL,
            .arg = NULL,   // 速度或位置环的反馈计算函数的对象
            .fre_rel = 2,  //500Hz
        };
        obj->joint2 = ENCOSmotor_Create(&joint2_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT2)) return NULL;

        // Joint3 ENCOS V1.20EAP(原 dm4340p 的位置)。CAN id 用 SDK 那套：j3 = 4
        ENCOS_motor_config joint3_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT3],
            .bsp_can_index = 2,
            .can_tx_id = 0x04,
            .can_rx_id = 0x04,  // ENCOS 要求收发同 id，否则 Create 直接返回 NULL
            .lost_callback_ = Joint3_Motor_Lost,
            .output_mode = encos_output_normal,
            .ranges = ENCOS_JOINT_RANGES,
            .torque_constant = 0.0f,  // SDK 没给 Kt -> fdb_torque 恒为 NAN(本工程没人用)
            .position_offset = 0.0f,  // SDK: motor_type4_position.offset = 0.0
            .feedback_timeout_ms = 100,  // SDK: Type4 timeout_us = 100000
            .stop_mode = encos_soft_stop,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = /* Joint3_Speed_Fdb_Calc */ NULL,
            .arg = NULL,   // 速度或位置环的反馈计算函数的对象
            .fre_rel = 2,  //500Hz
        };
        obj->joint3 = ENCOSmotor_Create(&joint3_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT3)) return NULL;

        // Joint4 DM4310
        DM_motor_config joint4_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT4],
            .bsp_can_index = 0,  
            .can_tx_id = 0x05,
            .can_rx_id = 0x05,
            .lost_callback_ = Joint4_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 3.14159f,
            .max_speed = 30,
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = NULL,
            .arg = NULL,
            .fre_rel = 2,  //500Hz
        };
        obj->joint4 = DMmotor_Create(&joint4_motor_config, DM_Mit_Control);
        obj->joint4->config.arg = obj->joint4;
        obj->joint4->kp = obj->pid_mit_kpkd[JOINT4][0];
        obj->joint4->kd = obj->pid_mit_kpkd[JOINT4][1];
        motor_controller_list[JOINT4] = obj->joint4->motor_controller;
        motor_monitor_list[JOINT4] = obj->joint4->monitor;

        // Joint5 DM4310
        DM_motor_config joint5_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT5],
            .bsp_can_index = 0,  
            .can_tx_id = 0x06,
            .can_rx_id = 0x06,
            .lost_callback_ = Joint5_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 3.14159f,
            .max_speed = 30,
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = NULL,
            .arg = NULL,
            .fre_rel = 2,  //500Hz
        };
        obj->joint5 = DMmotor_Create(&joint5_motor_config, DM_Mit_Control);
        obj->joint5->config.arg = obj->joint5;
        obj->joint5->kp = obj->pid_mit_kpkd[JOINT5][0];
        obj->joint5->kd = obj->pid_mit_kpkd[JOINT5][1];
        motor_controller_list[JOINT5] = obj->joint5->motor_controller;
        motor_monitor_list[JOINT5] = obj->joint5->monitor;

        // Joint6 DM2325
        DM_motor_config joint6_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT6],
            .bsp_can_index = 0,
            .can_tx_id = 0x07,
            .can_rx_id = 0x07,
            .lost_callback_ = Joint6_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 3.14159f,
            .max_speed = 30,
            .max_torque = 5,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc =
                /* Joint6_Pos_Fdb_Calc */ NULL,
            .speed_fdb_calc = NULL,
            .arg = NULL,
            .fre_rel = 2,  //500Hz
        };
        obj->joint6 = DMmotor_Create(&joint6_motor_config, DM_Mit_Control);
        obj->joint6->config.arg = obj->joint6;
        obj->joint6->kp = obj->pid_mit_kpkd[JOINT6][0];
        obj->joint6->kd = obj->pid_mit_kpkd[JOINT6][1];
        motor_controller_list[JOINT6] = obj->joint6->motor_controller;
        motor_monitor_list[JOINT6] = obj->joint6->monitor;
        obj->joint6->frequency_cnt =
            1;  // 使joint6与joint4、5频率错开，避免同时更新导致的总线拥堵

        // Gripper DM4310
        DM_motor_config gripper_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT4],
            .bsp_can_index = 0,
            .can_tx_id = 0x08,
            .can_rx_id = 0x08,
            .lost_callback_ = Gripper_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 3.14159f,
            .max_speed = 30,
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = NULL,
            .speed_fdb_calc = NULL,
            .arg = NULL,
            .fre_rel = 2,  //500Hz
        };
        obj->gripper = DMmotor_Create(&gripper_motor_config, DM_Mit_Control);
        obj->gripper->config.arg = obj->gripper;
        obj->gripper->kp = obj->pid_mit_kpkd[GRIPPER][0];
        obj->gripper->kd = obj->pid_mit_kpkd[GRIPPER][1];
        motor_controller_list[GRIPPER] = obj->gripper->motor_controller;
        motor_monitor_list[GRIPPER] = obj->gripper->monitor;
        obj->gripper->frequency_cnt =
            1;  // 使gripper与joint4、5频率错开，避免同时更新导致的总线拥堵
    }
    // 初始重置所有电机monitor掉线!!!防止误判在线状态
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        motor_monitor_list[i]->count = 0;
    }

    /* 杂项 */
    PID_SetConfig_Pos(&zero_config, 0, 0, 0, 0, 0);

#ifdef JOINT_ONLY_COMP
    set_pid_zero(obj);
#endif
    return obj;
}

void Gimbal_Update(Gimbal* obj) {
    /* 取指令。
     * 源工程是从 pub_sub 的订阅队列里取，取不到(len == -1)就把 arm_mode 打回 arm_stop。
     * 本工程没有 pub_sub：命令层(App/gimbal_cmd.c)在同一个任务里、上一拍已经把
     * obj->arm_control_data 写好了，这里直接读 mode —— 中间既没有队列也没有独立的
     * 指令源，所以不存在"没收到 cmd"这种情况，也就没有超时判定。
     * 真正会把 arm_mode 变成 arm_stop 的是下面的 Motor_Lost_Handle()(电机掉线)
     * 和命令层自己的模式判定(输入链路断了的时候)。 */
    obj->arm_mode = obj->arm_control_data.mode;

    /* 电机掉线相关处理 */
    Motor_Lost_Handle(obj);  // 放在最前面

    // 首次上电自动初始化
    {
        static uint8_t first_init_flag = 0;
        if (first_init_flag == 0) {
            //成功后返回1，失败返回0
            first_init_flag = Arm_Auto_Init(obj);
            if (!first_init_flag)
                obj->arm_mode = arm_stop;  // 自动初始化未成功则停机
        }
    }

    /* 获取电机反馈 */
    Get_Motor_Fdb(obj);

    /* 动力学前馈计算加载(arm_serial_link 未移植，整块注释掉，tau_comp[] 恒 0) */
    // Joint_Feedforward_Calc(obj);

#ifdef JOINT_ONLY_COMP
    set_pid_zero(obj);
#else
    set_pid_normal(obj);
#endif

    /* 电机状态。
     * j1~j3 是 ENCOS：**只改 enable 标志位，不调 ENCOSmotor_Enable()/Disable()**。
     *   - Enable() 会 reset_controller()(清 PID 积分/D 状态)，这个函数每拍都跑，
     *     每拍调一次等于把控制器废掉；
     *   - Disable() 会当场发一帧停止帧，和 MotorTask 里的 ENCOSmotor_Calc_Send()
     *     抢 CAN 总线(驱动要求所有 API 由调用者串行化)。
     * 真正的启停帧由 Calc_Send() 自己发：它看到 enable != encos_enable 或反馈不新鲜
     * 就发停止帧，和达妙那边的行为一致。 */
    if (obj->arm_mode == arm_stop) {
        // 各电机失能
        obj->joint1->enable = encos_stop;
        obj->joint2->enable = encos_stop;
        obj->joint3->enable = encos_stop;
        obj->joint4->enable = dm_stop;
        obj->joint5->enable = dm_stop;
        obj->joint6->enable = dm_stop;
        obj->gripper->enable = dm_stop;
    } else {
        // 各电机使能
        obj->joint1->enable = encos_enable;
        obj->joint2->enable = encos_enable;
        obj->joint3->enable = encos_enable;
        obj->joint4->enable = dm_enable;
        obj->joint5->enable = dm_enable;
        obj->joint6->enable = dm_enable;
        obj->gripper->enable = dm_enable;
    }

    /* 电机控制 */
    Set_Motor_ref(obj);

    // DEBUG:拷贝控制器数据
    g_fdb_pos = obj->gripper->motor_controller->fdb_position;
    g_fdb_speed = obj->gripper->motor_controller->fdb_speed;
    g_ref_pos = obj->gripper->motor_controller->ref_position;
    g_ref_speed = obj->gripper->motor_controller->ref_speed;
    g_speed_comp = obj->gripper->motor_controller->speed_comp;
    g_output = obj->gripper->motor_controller->output;
    g_ouput_comp = obj->gripper->motor_controller->output_comp;

    /* 杂项 */
    obj->arm_feedback_data.mode = obj->arm_mode;

    /* 反馈数据(pub_sub 未接入：数据就留在 obj->arm_feedback_data 里) */
    // publish_data arm_feedback;
    // arm_feedback.data = (uint8_t*)(&(obj->arm_feedback_data));
    // arm_feedback.len = sizeof(Arm_feedback_data);
    // obj->arm_feedback_pub->publish(obj->arm_feedback_pub, arm_feedback);
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
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->arm_feedback_data.motor_inited[i] = 1;  // 关节电机初始化完成
    }

    return 1;  // 初始化完成
}

static void Motor_Lost_Handle(Gimbal* obj) {
    /* 检测 */
    uint8_t any_motor_lost = 0;  // 初始化为0
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        obj->arm_feedback_data.is_motor_lost[i] = is_Offline(motor_monitor_list[i]);
        // 只要有一个电机掉线，就认为存在掉线
        any_motor_lost += obj->arm_feedback_data.is_motor_lost[i];
    }

    // 当j6掉线且未开启掉线保护，认为其未初始化
    if (ANY_MOTOR_LOST_STOP_ENABLE) {
        if (obj->arm_feedback_data.is_motor_lost[JOINT6])
            obj->arm_feedback_data.motor_inited[JOINT6] = 0;
    }
    // j1~j5与夹爪始终认为已初始化，因为它们具有绝对值编码器，掉线后重启仍能获得正确的角度反馈，不会出现误判掉线的情况
    obj->arm_feedback_data.motor_inited[JOINT1] = 1;
    obj->arm_feedback_data.motor_inited[JOINT2] = 1;
    obj->arm_feedback_data.motor_inited[JOINT3] = 1;
    obj->arm_feedback_data.motor_inited[JOINT4] = 1;
    obj->arm_feedback_data.motor_inited[JOINT5] = 1;
    obj->arm_feedback_data.motor_inited[GRIPPER] = 1;

    /* 处理 */
    if (any_motor_lost && ANY_MOTOR_LOST_STOP_ENABLE) {  // 若配置为掉线停机则停机
        obj->arm_mode = arm_stop;                        // 存在掉线电机，进入stop状态
    }
}

static void Set_Motor_ref(Gimbal* obj) {
    if (obj->arm_mode == arm_stop) {  // stop状态
        for (int i = 0; i < ARM_MOTOR_NUM; i++) {
            // 电机位置跟随，防止跳变
            motor_controller_list[i]->ref_position = motor_controller_list[i]->fdb_position;
            // 电机速度前馈置零
            motor_controller_list[i]->speed_comp = 0;
            // 电机输出补偿置零
            motor_controller_list[i]->output_comp = 0;
        }
        for (int i = 0; i < ARM_JOINT_NUM; i++) {
            // 关节粘性摩擦前馈置零
            obj->damp_comp[i] = 0;
        }
        return;
    }

    /* 设置关节电机目标位置、速度和力矩前馈 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        // 位置
        motor_controller_list[i]->ref_position =
            angle_to_pos(obj->arm_control_data.joint_angle[i], obj->motor_pos_init[i],
                MOTOR_DIR[i], MOTOR_REDUCTION[i]);
        // 速度前馈
        motor_controller_list[i]->speed_comp =
            vel_to_speed(obj->arm_control_data.joint_vel[i], MOTOR_DIR[i], MOTOR_REDUCTION[i])
            * JOINT_SPEEDCOMP_COEF[i];
        // 粘性摩擦前馈
        obj->damp_comp[i] = motor_controller_list[i]->ref_speed * DEG2RAD
            * JOINT_DAMPING[i] * MOTOR_DIR[i];
        // 总力矩前馈,带有修正系数
        motor_controller_list[i]->output_comp = MOTOR_DIR[i]
            * (obj->tau_comp[i] + obj->damp_comp[i]) * JOINT_TORQUE_TRANS[i]
            * JOINT_TORQUE_TRANS_COEF[i];
    }

    /* 设置夹爪电机目标位置和力矩 */
    // TODO:
}

/* 电机温度 float -> uint8_t(℃) 的转换，专门给 ENCOS 用。
 * arm_feedback_data.motor_temp[] 是 uint8_t，达妙的 t_rotor 本来就是 uint8_t；而 ENCOS
 * 的 t_rotor 是 **float，且收到第一帧反馈之前是 NAN**，NAN 转无符号是未定义行为。
 * 没数据(或解出来是负数)就当 0 —— "没读到"和"0℃"在这里没有区别，反正只是回传量。 */
static uint8_t Motor_Temp_U8(float t) {
    return (isfinite(t) && t > 0.0f) ? (uint8_t)t : 0;
}

static void Get_Motor_Fdb(Gimbal* obj) {
    // TODO: 添加关节速度反馈，并考虑对反馈进行滤波
    /* 关节角 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->arm_feedback_data.joint_angle[i] =
            pos_to_angle(motor_controller_list[i]->fdb_position, obj->motor_pos_init[i],
                MOTOR_DIR[i], MOTOR_REDUCTION[i]);
    }

    /* 电机温度 */
    obj->arm_feedback_data.motor_temp[JOINT1] = Motor_Temp_U8(obj->joint1->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT2] = Motor_Temp_U8(obj->joint2->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT3] = Motor_Temp_U8(obj->joint3->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT4] = obj->joint4->t_rotor;
    obj->arm_feedback_data.motor_temp[JOINT5] = obj->joint5->t_rotor;
    obj->arm_feedback_data.motor_temp[JOINT6] = obj->joint6->t_rotor;
    obj->arm_feedback_data.motor_temp[GRIPPER] = obj->gripper->t_rotor;
}

/* 动力学前馈(Newton-Euler)：依赖 arm_serial_link / robotics_port 那一套(DH 参数、
 * 连杆质量/惯量)，本工程没移植，整块先注释掉。所以 obj->tau_comp[] 从头到尾是 0，
 * Set_Motor_ref() 里那项力矩前馈实际只剩粘性摩擦 damp_comp。
 * 要接回来：把 arm_serial_link 那套搬进 libs/，再把这里和 Gimbal_Update() 里的调用点
 * 一起打开即可。 */
/*
// float cofs[6] = {1,1,1,1,1,1};
// uint8_t isload = 0;
static void Joint_Feedforward_Calc(Gimbal* obj) {
    Arm_q_c q;
    Arm_q_c qd;
    Arm_q_c qdd;

    memset(&q, 0, sizeof(Arm_q_c));
    memset(&qd, 0, sizeof(Arm_q_c));
    memset(&qdd, 0, sizeof(Arm_q_c));
    //位置使用当前值，速度、加速度使用目标值
    for (int i = 0; i < ARM_JOINT_NUM - 1; i++) {
        q.q[i] = obj->arm_feedback_data.joint_angle[i] * DEG2RAD;
        qd.q[i] = obj->arm_control_data.joint_vel[i] * DEG2RAD;
        qdd.q[i] = obj->arm_control_data.joint_acc[i] * DEG2RAD;
    }
    // 保证j6的运动学角度在-pi到pi之间
    q.q[JOINT6] =
        normalize(obj->arm_feedback_data.joint_angle[JOINT6] * DEG2RAD, -pi, pi, 2 * pi);
    qd.q[JOINT6] = obj->arm_control_data.joint_vel[JOINT6] * DEG2RAD;
    qdd.q[JOINT6] = obj->arm_control_data.joint_acc[JOINT6] * DEG2RAD;

    // 动力学，牛顿欧拉法
    Serial_Link_Rne_Alter(arm_serial_link, q, qd, qdd, obj->is_load, obj->tau_comp);
}
*/

static float lowpass_filter(float current_speed, float a, uint8_t index) {
    static float last_speed[ARM_MOTOR_NUM] = {0};
    float filtered_speed = (1 - a) * current_speed + a * last_speed[index];
    last_speed[index] = filtered_speed;
    return filtered_speed;
}

MAYBE_UNUSED static uint8_t stalling_detect(controller* con, float speed_diff,
    float pos_step, uint16_t stall_time, uint8_t index, float a) {
    static uint16_t cnt[3] = {0};
    // 判断输入是否合理
    if (index >= 3 || speed_diff <= 0) return 0;
    // 低通滤波避免噪声干扰判断
    float current_speed = lowpass_filter(con->fdb_speed, a, index);
    // ref和fdb的差值达到阈值则计数，cnt达到stall_time后判定为堵转
    if (fabsf(current_speed - con->ref_speed) < speed_diff) {
        con->ref_position += pos_step;
    } else if (cnt[index] < stall_time) {
        cnt[index]++;
    } else {
        cnt[index] = 0;
        con->ref_position = con->fdb_position - pos_step;
        return 1;
    }
    return 0;
}

// PID参数初始化
static void pid_init(Gimbal* obj) {
    /* 【调试开关】所有 PID 参数一律置零（按用户要求）。
     *   pid_cfgs[7][2]     位置环/速度环：KP/KI/KD/outputMax 全 0 -> 控制器输出恒 0
     *   encos_kpkd[3][2]   ENCOS 力位混合控制的 kp/kd 全 0
     *   pid_mit_kpkd[7][2] 达妙 MIT 的 kp/kd 全 0
     * 效果：电机彻底松掉 —— 使能后重力下会掉，位置力矩只剩 Set_Motor_ref() 的前馈，
     * 适合手动拖动、验反馈。
     *
     * 安全性：PID_POSITION == 0、D_AHEAD_UNUSED == 0，全 0 的 PID_Config 仍落在位置式
     * 分支，PID_Calc() 走 else 算出 0，不会除零/出 NaN。
     * Gimbal_Create() 在调本函数前已经 memset(obj, 0, sizeof(Gimbal))，本来也就是 0；
     * 显式再清一遍是为了不依赖那个调用顺序。 */
    memset(obj->pid_cfgs, 0, sizeof(obj->pid_cfgs));
    memset(obj->encos_kpkd, 0, sizeof(obj->encos_kpkd));
    memset(obj->pid_mit_kpkd, 0, sizeof(obj->pid_mit_kpkd));
}

MAYBE_UNUSED static void set_pid_normal(Gimbal* obj) {
    obj->joint1->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT1][0];
    obj->joint1->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT1][1];
    obj->joint1->kp = obj->encos_kpkd[JOINT1][0];
    obj->joint1->kd = obj->encos_kpkd[JOINT1][1];

    obj->joint2->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT2][0];
    obj->joint2->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT2][1];
    obj->joint2->kp = obj->encos_kpkd[JOINT2][0];
    obj->joint2->kd = obj->encos_kpkd[JOINT2][1];

    obj->joint3->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT3][0];
    obj->joint3->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT3][1];
    obj->joint3->kp = obj->encos_kpkd[JOINT3][0];
    obj->joint3->kd = obj->encos_kpkd[JOINT3][1];

    obj->joint4->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT4][0];
    obj->joint4->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT4][1];
    obj->joint4->kp = obj->pid_mit_kpkd[JOINT4][0];
    obj->joint4->kd = obj->pid_mit_kpkd[JOINT4][1];

    obj->joint5->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT5][0];
    obj->joint5->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT5][1];
    obj->joint5->kp = obj->pid_mit_kpkd[JOINT5][0];
    obj->joint5->kd = obj->pid_mit_kpkd[JOINT5][1];

    obj->joint6->motor_controller->pid_pos_data.config = obj->pid_cfgs[JOINT6][0];
    obj->joint6->motor_controller->pid_speed_data.config = obj->pid_cfgs[JOINT6][1];
    obj->joint6->kp = obj->pid_mit_kpkd[JOINT6][0];
    obj->joint6->kd = obj->pid_mit_kpkd[JOINT6][1];

    obj->gripper->motor_controller->pid_pos_data.config = obj->pid_cfgs[GRIPPER][0];
    obj->gripper->motor_controller->pid_speed_data.config = obj->pid_cfgs[GRIPPER][1];

    /* 自适应输出限幅,输出系数在0~1之间调整 */
    // TODO:
    // CONSTEXPR float j1_temp_low_th = 50;   // j1温度低阈值
    // CONSTEXPR float j1_temp_high_th = 75;  // j1温度高阈值
    // float j1_coeff_w =
    //     temprature_to_output_coeff(obj->joint1->t_rotor, j1_temp_low_th, j1_temp_high_th);
    // obj->joint1->motor_controller->pid_speed_data.config.outputMax *= j1_coeff_w;
    // obj->joint1->kp *= j1_coeff_w;
    // obj->joint1->kd *= j1_coeff_w;
    // CONSTEXPR float j5_temp_low_th = 40;   // j5温度低阈值
    // CONSTEXPR float j5_temp_high_th = 65;  // j5温度高阈值
    // float j5_coeff_w =
    //     temprature_to_output_coeff(obj->joint5->t_rotor, j5_temp_low_th, j5_temp_high_th);
    // obj->joint5->motor_controller->pid_speed_data.config.outputMax *= j5_coeff_w;
    // obj->joint5->kp *= j5_coeff_w;
    // obj->joint5->kd *= j5_coeff_w;
    // CONSTEXPR float j6_temp_low_th = 35;   // j6温度低阈值
    // CONSTEXPR float j6_temp_high_th = 55;  // j6温度高阈值
    // float j6_coeff_w =
    //     temprature_to_output_coeff(_MAX(obj->joint6->t_rotor, obj->joint6->t_mos),
    //         j6_temp_low_th, j6_temp_high_th);
    // obj->joint6->motor_controller->pid_speed_data.config.outputMax *= j6_coeff_w;
    // obj->joint6->kp *= j6_coeff_w;
    // obj->joint6->kd *= j6_coeff_w;
}

MAYBE_UNUSED static void set_pid_zero(Gimbal* obj) {
    obj->joint1->motor_controller->pid_pos_data.config = zero_config;
    obj->joint1->motor_controller->pid_speed_data.config = zero_config;
    obj->joint1->kp = 0;
    obj->joint1->kd = 0;

    obj->joint2->motor_controller->pid_pos_data.config = zero_config;
    obj->joint2->motor_controller->pid_speed_data.config = zero_config;
    obj->joint2->kp = 0;
    obj->joint2->kd = 0;

    obj->joint3->motor_controller->pid_pos_data.config = zero_config;
    obj->joint3->motor_controller->pid_speed_data.config = zero_config;
    obj->joint3->kd = 0;
    obj->joint3->kp = 0;

    obj->joint4->motor_controller->pid_pos_data.config = zero_config;
    obj->joint4->motor_controller->pid_speed_data.config = zero_config;
    obj->joint4->kp = 0;
    obj->joint4->kd = 0;

    obj->joint5->motor_controller->pid_pos_data.config = zero_config;
    obj->joint5->motor_controller->pid_speed_data.config = zero_config;
    obj->joint5->kp = 0;
    obj->joint5->kd = 0;

    obj->joint6->motor_controller->pid_pos_data.config = zero_config;
    obj->joint6->motor_controller->pid_speed_data.config = zero_config;
    obj->joint6->kp = 0;
    obj->joint6->kd = 0;

    // 夹爪正常使用
    obj->gripper->motor_controller->pid_pos_data.config = obj->pid_cfgs[GRIPPER][0];
    obj->gripper->motor_controller->pid_speed_data.config = obj->pid_cfgs[GRIPPER][1];
    // obj->gripper->motor_controller->pid_pos_data.config = zero_config;
    // obj->gripper->motor_controller->pid_speed_data.config = zero_config;
}


/* 【注意】Joint2_Speed_Fdb_Calc()/Joint3_Speed_Fdb_Calc() 是达妙时代留下的，里面把
 * void* 强转成 DM_motor*，而现在 j2/j3 已经是 ENCOS_motor 了 —— 这两个函数目前没有
 * 调用方(上面 config 里的 .speed_fdb_calc 是写死的 NULL，函数名被注释掉了)，所以
 * 先留着不删，但**要启用之前必须先改成 ENCOS_motor***，否则读到的是错位的字段。
 * ENCOS 的 fdb_speed 也是 rad/s，单位换算(RAD2DEG)那部分可以照抄。 */

float j2_a = 0.9;
float cur_speed_j2;
float filt_speed_j2;
float Joint2_Speed_Fdb_Calc(void* arg) {
    DM_motor* motor = (DM_motor*)arg;
    cur_speed_j2 = (motor->fdb_speed - (-0.0222945623))
        * RAD2DEG;  //dm电机静态时反馈速度有飘移，通过实测数据进行补偿
    filt_speed_j2 = lowpass_filter(cur_speed_j2, j2_a, JOINT2);  // 对速度反馈进行低通滤波
    return filt_speed_j2;
}

float j3_a = 0.95;
float cur_speed_j3;
float filt_speed_j3;
float Joint3_Speed_Fdb_Calc(void* arg) {
    DM_motor* motor = (DM_motor*)arg;
    cur_speed_j3 = motor->fdb_speed * RAD2DEG;
    filt_speed_j3 = lowpass_filter(cur_speed_j3, j3_a, JOINT3);  // 对速度反馈进行低通滤波
    return filt_speed_j3;
}