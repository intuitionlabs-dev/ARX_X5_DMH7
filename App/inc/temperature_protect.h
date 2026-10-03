#pragma once

/* 电机过温降出力：把温度线性映射成 0~1 的输出系数。
 *
 * 原样搬自 ec_code/engineer-wheel-leg-v-1/APP/utils/temperature_protect.h，
 * 依赖只有 common.h 的 linear_mapping()/fclip()，本工程都有。
 *
 * 用法见 gimbal_arm.c 的 set_pid_normal()：拿关节电机的 t_rotor 算系数，
 * 乘到 pid_speed_data.config.outputMax 和 MIT 的 kp/kd 上，温度越高出力越小。 */

#include "common.h"
#include "robot_def.h"

// 将温度映射为输出限幅系数，温度在 low_th 和 high_th 之间线性映射为 0~1
MAYBE_UNUSED static float temprature_to_output_coeff(float temp, float low_th, float high_th) {
    return linear_mapping(low_th, high_th, 1, 0, fclip(temp, low_th, high_th));
}
