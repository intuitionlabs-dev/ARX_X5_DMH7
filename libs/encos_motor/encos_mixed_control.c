#include "encos_mixed_control.h"

#include "common.h"
#include "controller.h"

void ENCOS_Mixed_Control(ENCOS_motor* motor) {
    /* 计算位置、速度级联控制输出。 */
    controller_calc(motor->motor_controller);

    /* 控制器使用 deg 和 deg/s；协议位置、速度换算为 rad 和 rad/s。 */
    motor->ref_position = motor->motor_controller->ref_position * DEG2RAD;
    motor->ref_speed = motor->motor_controller->ref_speed * DEG2RAD;

    /* 速度环输出 + 力矩前馈作为 ref_torque，和 DM_Mit_Control() 一致 */
    motor->ref_torque = motor->motor_controller->output_comp
        + motor->motor_controller->pid_speed_data.output;
}
