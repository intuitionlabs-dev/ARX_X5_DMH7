#pragma once

#include "DM_motor.h"

/* 达妙电机 MIT 模式的控制函数，给 DMmotor_Create() 当第二个参数用。
 *
 * 原来是 ec_code/engineer-wheel-leg-v-1/APP/utils/public_func.c 里的一个函数
 * (那边和底盘判断函数挤在一个文件里)，gimbal_arm.c 只能靠 extern 声明去引它。
 * 挪到这里是因为它只跟达妙电机有关，谁都能用。
 *
 * 和 DM_motor.c 里"不传 control_calc"的默认路径的区别：
 *   - 这里会用 motor->round 把 controller 出来的角度折回 -max_position~+max_position，
 *     默认路径不管多圈；
 *   - 这里把速度环输出和力矩前馈(output_comp)相加当 ref_torque，默认路径只用
 *     controller->output、且没有 output_comp。
 * 所以 MIT 模式下应该传这个而不是传 NULL。 */
void DM_Mit_Control(struct dm_motor_t* motor);
