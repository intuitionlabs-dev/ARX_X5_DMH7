#pragma once

/* 使用查找表和插值计算单精度 sin、cos。 */

#ifdef __cplusplus
extern "C" {
#endif
float fast_sin_f32(float x);
float fast_cos_f32(float x);
#ifdef __cplusplus
}
#endif
