#pragma once

/* 表查 sin/cos 的文件（fast_sin_f32.c / fast_cos_f32.c）靠这里拿到 uint16_t/int32_t
 * 和表长。源工程的 ROBOTICS_status / fast_matrix_instance_f32 是给 fast_matrix.c 用的，
 * 那个文件没有取过来（原因见 fast_math.h），这两个类型一并去掉。 */

#include <inttypes.h>

#define FAST_MATH_TABLE_SIZE 512
