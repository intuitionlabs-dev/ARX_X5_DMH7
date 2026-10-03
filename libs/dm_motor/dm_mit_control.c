#include "dm_mit_control.h"

#include "common.h"      /* DEG2RAD */
#include "controller.h"  /* controller_calc() */

// 自定义 DM 电机 MIT 控制函数
void DM_Mit_Control(struct dm_motor_t* motor) {
    // 先计算原始 controller
    controller_calc(motor->motor_controller);
    // 将 controller 中的目标位置转换为电机的 ref_position
    motor->ref_position = motor->motor_controller->ref_position * DEG2RAD
        - motor->round * 2 * motor->config.max_position;
    // 将 controller 中的目标速度转换为电机的 ref_speed
    motor->ref_speed = motor->motor_controller->ref_speed * DEG2RAD;
    // 将速度环的输出与力矩前馈相加作为电机的 ref_torque
    motor->ref_torque = motor->motor_controller->output_comp
        + motor->motor_controller->pid_speed_data.output;
}
