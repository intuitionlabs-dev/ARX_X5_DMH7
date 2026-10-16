#include "gimbal_arm.h"
#include <stdlib.h> /* malloc()，源文件漏了这个 */
#include <string.h>
#include "arm.h"
#include "common.h"
#include "dm_mit_control.h"      /* DM_Mit_Control() */
#include "encos_mixed_control.h" /* ENCOS_Mixed_Control() */
#include "gimbal_arm_def.h"
#include "temperature_protect.h"

/* 位置环关不关由**运行期的 arm_only_gravity_comp 模式**定（见 Gimbal_Update() 里的
 * set_pid_* 分派），调试时直接切模式就行，不用重编。
 * 别把它做成编译期开关 —— 那会和 arm_damping 打架（那个也要关位置环，但 kd 留着）。 */
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
float DM_Pos_Fdb_Calc(void* arg);
float DM_Speed_Fdb_Calc(void* arg);

/* 主函数 */
// NOLINTBEGIN
static void Motor_Lost_Handle(Gimbal* obj);        // 电机掉线处理
static uint8_t Arm_Auto_Init(Gimbal* obj);         // 初次上电自动初始化
static void Get_Motor_Fdb(Gimbal* obj);            // 电机反馈
static void Joint_Dynamics_Comp_Calc(Gimbal* obj);  // 关节逆动力学前馈
static void Set_Motor_ref(Gimbal* obj);            // 电机控制
// NOLINTEND
/* 辅助函数 */
static void pid_init(Gimbal* obj);                            // 初始化pid参数
static void set_pid_normal(Gimbal* obj);                      // 给pid赋值
static void set_pid_zero(Gimbal* obj);                        // 给pid置0
static void set_pid_damping(Gimbal* obj);                     // 阻尼：位置环关、只留 kd
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

/* 私有变量 —— 电机对象和 motor_controller_list[]/motor_monitor_list[] 都挂在 Gimbal 上
 * （见 gimbal_arm.h）：一份数组被两台臂共用的话，第二台 Create() 建完就把第一台的指针
 * 覆盖了，CmdTask 里第一台就会拿着第二台的电机在跑。 */

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
 * 返回 NULL，而 `obj->jointN->config.arg = ...` 这种不判空的写法会直接空指针。 */
static uint8_t encos_joint_init(Gimbal* obj, uint8_t joint) {
    ENCOS_motor* motor = joint == JOINT1 ? obj->joint1
        : joint == JOINT2                ? obj->joint2
                                         : obj->joint3;
    if (motor == NULL) return 0;
    motor->config.arg = motor;
    motor->kp = obj->encos_kpkd[joint][0];
    motor->kd = obj->encos_kpkd[joint][1];
    obj->motor_controller_list[joint] = motor->motor_controller;
    obj->motor_monitor_list[joint] = motor->monitor;
    return 1;
}

Gimbal* Gimbal_Create(uint8_t can_index) {
    /* 创建实例 */
    Gimbal* obj = (Gimbal*)malloc(sizeof(Gimbal));
    memset(obj, 0, sizeof(Gimbal));

    /* 这台臂走哪路 CAN(= 臂号)。下面 7 台电机的 bsp_can_index 全用它，达妙反馈回调
     * 也按它过滤（两台臂的达妙 ID 完全一样，不按总线过滤会互相吃反馈）。 */
    obj->can_index = can_index;

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
        // Joint1 ENCOS V1.20EAP。
        ENCOS_motor_config joint1_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT1],
            .bsp_can_index = can_index,  // 这台臂走哪路 CAN，见函数开头
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
            .arg = NULL,  // 速度或位置环的反馈计算函数的对象
            .tx_group = CAN_TX_500HZ_G0,
        };
        obj->joint1 = ENCOSmotor_Create(&joint1_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT1)) return NULL;

        // Joint2 ENCOS V1.20EAP(原 dm8009 的位置)。CAN id 用 SDK 那套：j2 = 2
        ENCOS_motor_config joint2_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT2],
            .bsp_can_index = can_index,  // 这台臂走哪路 CAN，见函数开头
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
            .arg = NULL,  // 速度或位置环的反馈计算函数的对象
            .tx_group = CAN_TX_250HZ_G1,
        };
        obj->joint2 = ENCOSmotor_Create(&joint2_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT2)) return NULL;

        // Joint3 ENCOS V1.20EAP(原 dm4340p 的位置)。CAN id 用 SDK 那套：j3 = 4
        ENCOS_motor_config joint3_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT3],
            .bsp_can_index = can_index,
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
            .arg = NULL,  // 速度或位置环的反馈计算函数的对象
            .tx_group = CAN_TX_250HZ_G1,
        };
        obj->joint3 = ENCOSmotor_Create(&joint3_motor_config, ENCOS_Mixed_Control);
        if (!encos_joint_init(obj, JOINT3)) return NULL;

        /* j4~j6 + 夹爪是达妙(ARX X5 的 MotorType2)，ID 取 SDK 的出厂值 5/6/7/8
         * (parameters.json 的 joint_parameters/gripper_motor 里写的 can_id)。
         *
         * **can_rx_id 一律写 0**：达妙反馈帧的仲裁ID 是电机里的 MST_ID，这 4 台出厂都是 0，
         * 所以四台反馈全挤在 ID=0 上；真正区分是哪一台的是载荷首字节低 4 位(= CAN_ID)，
         * 由 DMmotor_RxCallBack() 里和 can_tx_id 比对。这与 ARX X5 SDK 的
         * MotorType2::CanAnalyze()(要求 can_id==0 且 data[0]&0x0F==motor_id)完全一致。
         * 详见仓库根目录 ARX_X5_CAN_FEEDBACK_ROUTING.md。
         *
         * **量程必须和电机里存的 P_MAX/V_MAX/T_MAX 一致**：反馈帧的位置/速度/力矩都是按
         * 电机自己存的量程归一化后发出来的，本工程再按 config.max_* 还原回去，两边不等
         * 就差一个系数(比如 max_position 写 π 而电机里是 12.5，解出来的角度就小 3.98 倍，
         * 零位附近两种取值都读到 ≈0，看不出来)。下面取 SDK 的 Type2 量程——见
         * libs/gravity_compensation/sdk_parameters/parameters.json 的 motor_protocols
         * (type=2) 与 binary_constants.type2_*：位置 ±12.5 rad、速度 ±45 rad/s、
         * 力矩 ±10 N·m、kp 0~500、kd 0~5，与电机出厂配置一致。
         * 核对办法：在达妙上位机里读这 4 台的 P_MAX/V_MAX/T_MAX；或者手转一个已知角度
         * (90° = 1.571 rad)看 fdb_position 是否跟着变 1.571。 */
        // Joint4 DM4310
        DM_motor_config joint4_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT4],
            .bsp_can_index = can_index,
            .can_tx_id = 0x05,
            .can_rx_id = 0x00,
            .lost_callback_ = Joint4_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 12.5f,  // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_speed = 45.0f,     // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = DM_Pos_Fdb_Calc,
            .speed_fdb_calc = DM_Speed_Fdb_Calc,
            .arg = NULL,
            .tx_group = CAN_TX_250HZ_G2,
        };
        obj->joint4 = DMmotor_Create(&joint4_motor_config, DM_Mit_Control);
        obj->joint4->config.arg = obj->joint4;
        obj->joint4->kp = obj->pid_mit_kpkd[JOINT4][0];
        obj->joint4->kd = obj->pid_mit_kpkd[JOINT4][1];
        obj->motor_controller_list[JOINT4] = obj->joint4->motor_controller;
        obj->motor_monitor_list[JOINT4] = obj->joint4->monitor;

        // Joint5 DM4310
        DM_motor_config joint5_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT5],
            .bsp_can_index = can_index,
            .can_tx_id = 0x06,
            .can_rx_id = 0x00,  // MST_ID=0，见 JOINT4 处的说明
            .lost_callback_ = Joint5_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 12.5f,  // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_speed = 45.0f,     // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = DM_Pos_Fdb_Calc,
            .speed_fdb_calc = DM_Speed_Fdb_Calc,
            .arg = NULL,
            .tx_group = CAN_TX_250HZ_G2,
        };
        obj->joint5 = DMmotor_Create(&joint5_motor_config, DM_Mit_Control);
        obj->joint5->config.arg = obj->joint5;
        obj->joint5->kp = obj->pid_mit_kpkd[JOINT5][0];
        obj->joint5->kd = obj->pid_mit_kpkd[JOINT5][1];
        obj->motor_controller_list[JOINT5] = obj->joint5->motor_controller;
        obj->motor_monitor_list[JOINT5] = obj->joint5->monitor;

        // Joint6 DM2325
        DM_motor_config joint6_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT6],
            .bsp_can_index = can_index,
            .can_tx_id = 0x07,
            .can_rx_id = 0x00,  // MST_ID=0，见 JOINT4 处的说明
            .lost_callback_ = Joint6_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 12.5f,  // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_speed = 45.0f,     // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            /* SDK 对 Type2 这一组只有一套力气量程(binary_constants.type2_effort = ±10)，
             * J4/J5/J6/夹爪 都按 ±10 解码，所以这台跟其余三台取齐。真要确认得去达妙上位机
             * 读这台的 T_MAX。 */
            .max_torque = 10.0f,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = DM_Pos_Fdb_Calc,
            .speed_fdb_calc = DM_Speed_Fdb_Calc,
            .arg = NULL,
            /* 250Hz(组 1,5) —— 见 j5 处的相位说明 */
            .tx_group = CAN_TX_250HZ_G3,
        };
        obj->joint6 = DMmotor_Create(&joint6_motor_config, DM_Mit_Control);
        obj->joint6->config.arg = obj->joint6;
        obj->joint6->kp = obj->pid_mit_kpkd[JOINT6][0];
        obj->joint6->kd = obj->pid_mit_kpkd[JOINT6][1];
        obj->motor_controller_list[JOINT6] = obj->joint6->motor_controller;
        obj->motor_monitor_list[JOINT6] = obj->joint6->monitor;

        // Gripper DM4310
        DM_motor_config gripper_motor_config = {
            .motor_controller_config = joint_controller_config[JOINT4],
            .bsp_can_index = can_index,
            .can_tx_id = 0x08,
            .can_rx_id = 0x00,  // MST_ID=0，见 JOINT4 处的说明
            .lost_callback_ = Gripper_Motor_Lost,
            .control_mode = dm_mit,
            .max_position = 12.5f,  // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_speed = 45.0f,     // 达妙 Type2 协议量程，见上面 JOINT4 处的说明
            .max_torque = 10,
            .max_kp = 500,
            .max_kd = 5,
            .pos_fdb_calc = DM_Pos_Fdb_Calc,
            .speed_fdb_calc = DM_Speed_Fdb_Calc,
            .arg = NULL,
            /* 250Hz(组 0,4) —— 见 j5 处的相位说明 */
            .tx_group = CAN_TX_250HZ_G3,
        };
        obj->gripper = DMmotor_Create(&gripper_motor_config, DM_Mit_Control);
        obj->gripper->config.arg = obj->gripper;
        obj->gripper->kp = obj->pid_mit_kpkd[GRIPPER][0];
        obj->gripper->kd = obj->pid_mit_kpkd[GRIPPER][1];
        obj->motor_controller_list[GRIPPER] = obj->gripper->motor_controller;
        obj->motor_monitor_list[GRIPPER] = obj->gripper->monitor;
        /* 夹爪与 joint4/5 的相位错开由 config.tx_group 负责，别手工写频率计数。 */
    }
    // 初始重置所有电机monitor掉线!!!防止误判在线状态
    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        obj->motor_monitor_list[i]->count = 0;
    }

    /* 杂项 */
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

    /* 电机状态。
     * j1~j3 是 ENCOS：**只改 enable 标志位，不调 ENCOSmotor_Disable()**。
     *   - 使能没有单独接口（已并进 Send_Step 的 ⓪：由驱动自己抓 stop->enable 的边沿清环）；
     *   - Disable() 会 reset_controller() 并当场发一帧停止帧，和 MotorTask 里的
     *     ENCOSmotor_Send_Step() 抢 CAN 总线(驱动要求所有 API 由调用者串行化)。
     * 真正的启停帧由 Send_Step() 自己发：它看到 enable != encos_enable 或反馈不新鲜
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
    if (obj == g_arm[0].gimbal) {
        g_fdb_pos = obj->joint3->motor_controller->fdb_position;
        g_fdb_speed = obj->joint3->motor_controller->fdb_speed;
        g_ref_pos = obj->joint3->motor_controller->ref_position;
        g_ref_speed = obj->joint3->motor_controller->ref_speed;
        g_speed_comp = obj->joint3->motor_controller->speed_comp;
        g_output = obj->joint3->motor_controller->output;
        g_ouput_comp = obj->joint3->motor_controller->output_comp;
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

/* 电机温度 float -> uint8_t(℃) 的转换，专门给 ENCOS 用。
 * arm_feedback_data.motor_temp[] 是 uint8_t，达妙的 t_rotor 本来就是 uint8_t；而 ENCOS
 * 的 t_rotor 是 **float，且收到第一帧反馈之前是 NAN**，NAN 转无符号是未定义行为。
 * 没数据(或解出来是负数)就当 0 —— "没读到"和"0℃"在这里没有区别，反正只是回传量。 */
static uint8_t Motor_Temp_U8(float t) {
    return (isfinite(t) && t > 0.0f) ? (uint8_t)t : 0;
}

static void Get_Motor_Fdb(Gimbal* obj) {
    // TODO: 考虑对反馈进行滤波
    /* 关节角 + 关节角速度。
     * fdb_speed 的单位已经是 deg/s：达妙那边 DM_Speed_Fdb_Calc() 乘了 RAD2DEG，
     * ENCOS 那边 ENCOSmotor_Send_Step() 乘了 180/pi，两个驱动都归一化了，所以这里
     * 只用做 电机->关节 的减速比/方向换算，**不要再乘 RAD2DEG**。
     * 减速比和方向目前都是 1/{1,1,1}，speed_to_vel 出来就等于原值；写全是为了以后
     * 真加了减速比不用回来改这里。 */
    for (int i = 0; i < ARM_JOINT_NUM; i++) {
        obj->arm_feedback_data.joint_angle[i] =
            pos_to_angle(obj->motor_controller_list[i]->fdb_position,
                obj->motor_pos_init[i], MOTOR_DIR[i], MOTOR_REDUCTION[i]);
        obj->arm_feedback_data.joint_vel[i] =
            speed_to_vel(obj->motor_controller_list[i]->fdb_speed, MOTOR_DIR[i],
                MOTOR_REDUCTION[i]);
    }

    /* 夹爪实际角：直接用电机反馈角（已经是 deg，见 DM_motor.c 里 fdb_position 乘的
     * RAD2DEG），**不做 pos_to_angle 换算** —— MOTOR_DIR/MOTOR_REDUCTION 那两张表
     * 只有 6 个关节，夹爪不在里面（它没有减速比、也没有 JOINT_ANGLE_INIT 那种零点
     * 标定），硬套关节的公式只能拿别的关节的值凑，反而错。
     *
     * 现在这两个表全 1，所以数值上等同；等真给关节加了减速比，这一行**不应该**跟着变。
     * 注：Set_Motor_ref() 里夹爪的目标位置和力矩前馈**已经接上了**(见上面那两行 + 下面
     * Set_Motor_ref() 里"设置夹爪电机目标位置和力矩")，deg 由 dm_mit_control.c 的
     * DM_Mit_Control() 乘 DEG2RAD 进 MIT 帧的。但这条通道**还没在真机上验过**
     * (protocol_doc.md 里"夹爪不会动"是旧话)，找零(Arm_Gripper_Zero)是它的第一个用户。 */
    obj->arm_feedback_data.gripper_angle = obj->motor_controller_list[GRIPPER]->fdb_position;

    /* 电机温度 */
    obj->arm_feedback_data.motor_temp[JOINT1] = Motor_Temp_U8(obj->joint1->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT2] = Motor_Temp_U8(obj->joint2->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT3] = Motor_Temp_U8(obj->joint3->t_rotor);
    obj->arm_feedback_data.motor_temp[JOINT4] = obj->joint4->t_rotor;
    obj->arm_feedback_data.motor_temp[JOINT5] = obj->joint5->t_rotor;
    obj->arm_feedback_data.motor_temp[JOINT6] = obj->joint6->t_rotor;
    obj->arm_feedback_data.motor_temp[GRIPPER] = obj->gripper->t_rotor;
}

/* arm_run：M(q)*qdd_ref+C(q,qd)*qd+G(q)；纯重力模式：G(q)。
 * 反馈角度和反馈速度使用关节系，目标加速度由命令层提供。
 */
static void Joint_Dynamics_Comp_Calc(Gimbal* obj) {
    memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
    obj->dynamics_status = X5_DYNAMICS_OK;
#if JOINT_DYNAMICS_COMP_ENABLE
    _Static_assert(ARM_JOINT_NUM == X5_GRAVITY_DOF, "动力学模型关节数必须一致");
    if (obj->arm_mode != arm_run && obj->arm_mode != arm_only_gravity_comp) return;

    x5_real q[ARM_JOINT_NUM], qd[ARM_JOINT_NUM], qdd[ARM_JOINT_NUM];
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        q[i] = obj->arm_feedback_data.joint_angle[i] * DEG2RAD;
        qd[i] = obj->arm_feedback_data.joint_vel[i] * DEG2RAD;
        qdd[i] = obj->arm_control_data.joint_acc[i] * DEG2RAD;
    }
    if (obj->arm_mode == arm_only_gravity_comp) {
        obj->dynamics_status = x5_gravity_compute(JOINT_DYNAMICS_COMP_MODEL, q,
            JOINT_DYNAMICS_GRAVITY_BASE, JOINT_DYNAMICS_GRAVITY_MODE, obj->tau_comp);
    } else {
        obj->dynamics_status = x5_dynamics_compute(JOINT_DYNAMICS_COMP_MODEL, q, qd, qdd,
            JOINT_DYNAMICS_GRAVITY_BASE, JOINT_DYNAMICS_GRAVITY_MODE, obj->tau_comp);
    }
    if (obj->dynamics_status != X5_DYNAMICS_OK) {
        memset(obj->tau_comp, 0, sizeof(obj->tau_comp));
        obj->arm_mode = arm_stop;
    }
#endif
}

/* ⚠ 这两处函数内 static（last_speed[] / cnt[]）**是进程级的，不 per 臂**。
 * 它们唯一的使用者是下面达妙时代那两个速度反馈计算函数
 * (Joint2/3_Speed_Fdb_Calc)，而那两个函数**没有任何活的调用方**
 * (config 里的 .speed_fdb_calc 写死 NULL)，所以这份共享状态现在碰不到、也不会串台。
 *
 * 真要用它们的时候必须一起处理两件事：把状态挪进 Gimbal（两台臂不能共用一个滤波器），
 * 以及把函数体里的 (DM_motor*) 换成 (ENCOS_motor*) —— j2/j3 现在是 ENCOS。 */
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

    // joint1
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT1][0], 6, 0.00, 50, 200, 1000);
    obj->pid_cfgs[JOINT1][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT1][1], 0.0f, 0.0f, 0.0f, 0.0f, 12);
    obj->encos_kpkd[JOINT1][0] = 0;
    obj->encos_kpkd[JOINT1][1] = 35;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT2][0], 4, 0, 50, 200, 1000);
    obj->pid_cfgs[JOINT2][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT2][1], 0.0f, 0.0f, 0.0f, 0.0f, 12);
    obj->encos_kpkd[JOINT2][0] = 10;
    obj->encos_kpkd[JOINT2][1] = 50;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT3][0], 4, 0, 50, 200, 1000);
    obj->pid_cfgs[JOINT3][0].Irange = 1;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT3][1], 0.0f, 0.0f, 0.0f, 0.0f, 12);
    obj->encos_kpkd[JOINT3][0] = 10;
    obj->encos_kpkd[JOINT3][1] = 50;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT4][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[JOINT4][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT4][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[JOINT4][0] = 50;
    obj->pid_mit_kpkd[JOINT4][1] = 0.8;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT5][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[JOINT5][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT5][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[JOINT5][0] = 25;
    obj->pid_mit_kpkd[JOINT5][1] = 0.8;

    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT6][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[JOINT6][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[JOINT6][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[JOINT6][0] = 10;
    obj->pid_mit_kpkd[JOINT6][1] = 1;

    PID_SetConfig_Pos(&obj->pid_cfgs[GRIPPER][0], 6, 0.08, 50, 200, 1000);
    obj->pid_cfgs[GRIPPER][0].Irange = 0;
    PID_SetConfig_Pos(&obj->pid_cfgs[GRIPPER][1], 0.0f, 0.0f, 0.0f, 0.0f, 10);
    obj->pid_mit_kpkd[GRIPPER][0] = 8;
    obj->pid_mit_kpkd[GRIPPER][1] = 0.1;
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
    /* kp/kd 必须像 j4~j6 那样显式恢复 —— 达妙是 MIT 模式，力矩里 kp/kd 是独立的两项，
     * 光把 PID config 写对不够(理由同 set_pid_zero() 下面那两行)。
     *
     * **以前这里没有这两行，是个真 bug**：唯一会把夹爪 kp 打成 0 的是 set_pid_damping()
     * (arm_damping，整臂一致的被动)，而 set_pid_normal() 是 arm_run/arm_stop 共用的收尾，
     * 不补回来就再没人置回。后果链条：
     *   一次初始化跑完 → robot_state 进 normal，默认档是 arm_damping → 夹爪 kp 变 0
     *   → 同一次上电里**第二次**找零全程 kp=0。
     * 于是第二次找零又慢又回不到行程中点：kp=0 时位置误差一点力矩都不出，退回那一段
     * (case 2 本来就没有力矩前馈)只剩 0.1·(ref_speed-vel) ≈ 0.2 N·m，啃不动静摩擦
     * → 6 s 退回超时 → gripper_zero_finish() 不设零、目标角对齐反馈。
     * 顺带修掉的是"第一次找零跑完之后，夹爪在上位机正常操控下一直是软脚"这件事。 */
    obj->gripper->kp = obj->pid_mit_kpkd[GRIPPER][0];
    obj->gripper->kd = obj->pid_mit_kpkd[GRIPPER][1];

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

/* arm_damping 用：位置环全关，只留 kd。
 *
 * **不能拿 set_pid_zero() 来复用** —— 那个函数末尾会把夹爪恢复成正常 PID
 * （见下面 set_pid_zero 的注释，那是给 arm_only_gravity_comp 用的语义：整臂"松劲但不
 * 失能"，夹爪还能被上位机控制着开合）。阻尼模式要的是**整臂一致**的被动：位置环
 * 全关、kd 拉上、夹爪也一样。夹爪在阻尼模式里继续跟位置指令的话，通信掉线时它会自己
 * 夹下去或弹开，正是要避免的。
 *
 * kd 取 JOINT_DAMPING_MODE_KD[]（gimbal_arm_def.h，含夹爪那台）。 */
MAYBE_UNUSED static void set_pid_damping(Gimbal* obj) {
    obj->joint1->motor_controller->pid_pos_data.config = zero_config;
    obj->joint1->motor_controller->pid_speed_data.config = zero_config;
    obj->joint1->kp = 0;
    obj->joint1->kd = JOINT_DAMPING_MODE_KD[JOINT1];

    obj->joint2->motor_controller->pid_pos_data.config = zero_config;
    obj->joint2->motor_controller->pid_speed_data.config = zero_config;
    obj->joint2->kp = 0;
    obj->joint2->kd = JOINT_DAMPING_MODE_KD[JOINT2];

    obj->joint3->motor_controller->pid_pos_data.config = zero_config;
    obj->joint3->motor_controller->pid_speed_data.config = zero_config;
    obj->joint3->kp = 0;
    obj->joint3->kd = JOINT_DAMPING_MODE_KD[JOINT3];

    obj->joint4->motor_controller->pid_pos_data.config = zero_config;
    obj->joint4->motor_controller->pid_speed_data.config = zero_config;
    obj->joint4->kp = 0;
    obj->joint4->kd = JOINT_DAMPING_MODE_KD[JOINT4];

    obj->joint5->motor_controller->pid_pos_data.config = zero_config;
    obj->joint5->motor_controller->pid_speed_data.config = zero_config;
    obj->joint5->kp = 0;
    obj->joint5->kd = JOINT_DAMPING_MODE_KD[JOINT5];

    obj->joint6->motor_controller->pid_pos_data.config = zero_config;
    obj->joint6->motor_controller->pid_speed_data.config = zero_config;
    obj->joint6->kp = 0;
    obj->joint6->kd = JOINT_DAMPING_MODE_KD[JOINT6];

    // 夹爪同样处理（这里就是和 set_pid_zero 唯一的区别）
    obj->gripper->motor_controller->pid_pos_data.config = zero_config;
    obj->gripper->motor_controller->pid_speed_data.config = zero_config;
    obj->gripper->kp = 0;
    obj->gripper->kd = JOINT_DAMPING_MODE_KD[GRIPPER];
}

/* arm_only_gravity_comp 用：位置环、kp、kd 全关，力矩**只有**重力前馈。
 *
 * 夹爪**故意不关**（下面留的是正常 PID）：这个模式是"整臂松劲、靠重力补偿悬停"，
 * 夹爪跟着松掉的话会自己滑开、把夹着的东西丢了 —— 松劲的是臂，不是手。 */
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
    /* kp/kd 也要显式恢复 —— 达妙是 MIT 模式，力矩里有 kp/kd 这两项，光恢复 PID
     * config 不够。模式能在运行期互相切，从 arm_damping 切过来时夹爪 kp 已经被打成 0
     * 了，不补回来夹爪会松掉。 */
    obj->gripper->kp = obj->pid_mit_kpkd[GRIPPER][0];
    obj->gripper->kd = obj->pid_mit_kpkd[GRIPPER][1];
    // obj->gripper->motor_controller->pid_pos_data.config = zero_config;
    // obj->gripper->motor_controller->pid_speed_data.config = zero_config;
}

/* 【注意】Joint2_Speed_Fdb_Calc()/Joint3_Speed_Fdb_Calc() 里把 void* 强转成
 * DM_motor*，而 j2/j3 现在是 ENCOS_motor —— 这两个函数目前没有调用方(上面 config 里
 * 的 .speed_fdb_calc 是写死的 NULL，函数名被注释掉了)，所以先留着不删，但**要启用之前
 * 必须先改成 ENCOS_motor***，否则读到的是错位的字段。
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

float DM_Pos_Fdb_Calc(void* arg) {
    DM_motor* motor = (DM_motor*)arg;
    return motor->fdb_position * RAD2DEG;
}

float DM_Speed_Fdb_Calc(void* arg) {
    DM_motor* motor = (DM_motor*)arg;
    return motor->fdb_speed * RAD2DEG;
}
