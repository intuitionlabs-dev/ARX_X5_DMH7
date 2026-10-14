#pragma once

/* 这里只留表查 sin/cos。这套库的矩阵运算 fast_mat_init/add/sub/scale/mult/trans
 * 的唯一使用者是 libs/extlib/robotics 下那个 C++ 矩阵封装（matrix.h），本工程没有接
 * robotics 那一整套（运动学/动力学都没接），那些函数没有调用方、也连带 fast_def.h 里的
 * fast_matrix_instance_f32 / ROBOTICS_status 一起不取，相关声明去掉。
 * 以后要用，从 ec_code/engineer-wheel-leg-v-1/libs/extlib/robotics/fast_math/ 取回原件。 */

#ifdef __cplusplus
extern "C" {
#endif
float fast_sin_f32(float x);
float fast_cos_f32(float x);
#ifdef __cplusplus
}
#endif