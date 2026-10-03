#include "cmd.h"

#include "rc.h" /* Rc_Loop()：遥控器驱动的服务，Cmd_Loop() 里带跑一遍 */

#include <string.h> /* memset()，还有 NULL */

/* ================== 电机参数：先给默认，按实际电机/接线填 ==================
 *
 * 字段含义见 libs/dm_motor/DM_motor.h 的 DM_motor_config 注释，
 * 这几个是必须自己确认的：bsp_can_index / can_tx_id / can_rx_id /
 * control_mode / max_*。
 * ======================================================================== */
#define ARM_MOTOR_NUM 1
DM_motor *cmd_motor;
PID_Config pid_cfgs[ARM_MOTOR_NUM][2];  // [电机索引][环索引][PID参数]
float pid_mit_kpkd[ARM_MOTOR_NUM][2];   // [电机索引][环索引]

void Cmd_Init(void) {
    PID_SetConfig_Pos(&pid_cfgs[0][0], 0, 0, 0, 0, 0);
    pid_cfgs[0][0].Irange = 1;
    PID_SetConfig_Pos(&pid_cfgs[0][1], 0, 0, 0, 0, 0);
    pid_cfgs[0][1].Irange = 0;
    pid_mit_kpkd[0][0] = 0;
    pid_mit_kpkd[0][1] = 0;
    controller_config joint_controller_config[ARM_MOTOR_NUM];

    for (int i = 0; i < ARM_MOTOR_NUM; i++) {
        memset(&joint_controller_config[i], 0, sizeof(controller_config));
        joint_controller_config[i].control_type = PID_MODEL;
        joint_controller_config[i].control_depth = POS_CONTROL_CASCADE;
        joint_controller_config[i].position_pid_config.D_ahead = D_AHEAD_UNUSED;
        joint_controller_config[i].speed_pid_config.D_ahead = D_AHEAD_UNUSED;
        joint_controller_config[i].position_pid_config = pid_cfgs[i][0];
        joint_controller_config[i].speed_pid_config = pid_cfgs[i][1];
    }

    DM_motor_config cmd_motor_config = {
        .motor_controller_config = joint_controller_config[0],
        .bsp_can_index = 0,  // CAN3
        .can_tx_id = 0x01,
        .can_rx_id = 0x01,
        .lost_callback_ = NULL,
        .control_mode = dm_mit,
        // 以下三个参量需要和于上位机调参软件中给定的参数一致，否则会出现反馈数据与实际不符的情况
        // 设置为3.1415，可令电机转动一圈的编码器值为65535。若为6.28，则电机转动一圈的编码器值为32767，不利于机器人复活重启
        .max_position = 3.14159f,
        .max_speed = 30,
        .max_torque = 10,
        .max_kp = 500,
        .max_kd = 5,
        .pos_fdb_calc = NULL,
        .speed_fdb_calc = NULL,
        .arg = NULL,   // 速度或位置环的反馈计算函数的对象
        .fre_rel = 1,  //1000Hz
    };
    cmd_motor = DMmotor_Create(&cmd_motor_config, NULL);
    cmd_motor->config.arg = cmd_motor;
    cmd_motor->kp = pid_mit_kpkd[0][0];
    cmd_motor->kd = pid_mit_kpkd[0][1];
}

void Cmd_Loop(void) {
    /* 遥控器输入先在这里服务：换算成 µs/频率、判丢信号、推逻辑层、喂看门狗。
     * 驱动本体在 libs/rc/ 里。 */
    Rc_Loop();

    cmd_motor->enable = dm_stop;
    if (cmd_motor == NULL) {
        return;
    }
}
