#include "encos_mixed_control.h"

#include "common.h"     /* DEG2RAD */
#include "controller.h" /* controller_calc() */

void ENCOS_Mixed_Control(ENCOS_motor* motor) {
    /* 先跑原始 controller：位置环 -> 速度环 -> output / output_comp */
    controller_calc(motor->motor_controller);

    /* controller 里的位置、速度是"度"制 —— ENCOSmotor_Calc_Send() 在没给
     * pos_fdb_calc/speed_fdb_calc 时就是用 real_fdb_position*(180/pi) 和
     * fdb_speed*(180/pi) 填的，和达妙那边一样。电机的 ref_* 要的是弧度制。
     * config.position_offset 取 0(ARX X5 MotorType4 的 offset 就是 0.0)，
     * 所以这里不用再减偏置。 */
    motor->ref_position = motor->motor_controller->ref_position * DEG2RAD;
    motor->ref_speed = motor->motor_controller->ref_speed * DEG2RAD;

    /* 速度环输出 + 力矩前馈作为 ref_torque，和 DM_Mit_Control() 一致 */
    motor->ref_torque = motor->motor_controller->output_comp
        + motor->motor_controller->pid_speed_data.output;
}
