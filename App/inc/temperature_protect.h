#pragma once

#include "common.h"
#include "robot_def.h"

/* 温度从下阈值升到上阈值时，输出系数从 1 降到 0。 */
MAYBE_UNUSED static float temprature_to_output_coeff(float temp, float low_th, float high_th) {
    return linear_mapping(low_th, high_th, 1, 0, fclip(temp, low_th, high_th));
}
