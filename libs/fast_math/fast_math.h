#pragma once

/* 源工程这套库里还有一组矩阵运算 fast_mat_init/add/sub/scale/mult/trans。
 * 它们的唯一使用者是 libs/extlib/robotics 下那个 C++ 矩阵封装（matrix.h），
 * 而本工程没有移植 robotics 那一整套（运动学/动力学都没接），所以那些函数既没有
 * 调用方、也要连带 fast_def.h 里的 fast_matrix_instance_f32 / ROBOTICS_status。
 * 结论：fast_matrix.c 不取，相关声明一并去掉，这里只留表查 sin/cos。
 * 以后要用，从 ec_code/engineer-wheel-leg-v-1/libs/extlib/robotics/fast_math/ 取回原件。 */

#ifdef __cplusplus
extern "C" {
#endif
float fast_sin_f32(float x);
float fast_cos_f32(float x);
#ifdef __cplusplus
}
#endif