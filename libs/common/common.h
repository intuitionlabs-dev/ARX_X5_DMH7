#ifndef __COMMON_H
#define __COMMON_H
#include <math.h>
#include "stdint.h"

#define pi 3.1415926f
#define RAD2DEG (180.0f / pi)
#define DEG2RAD (pi / 180.0f)

// 取最大值
#define _MAX(a, b) ((a) > (b) ? (a) : (b))
// 取最小值
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
// 双端限幅 a<x<b
#define _MID(x, a, b) _MAX(_MIN((x), (b)), (a))

// NOLINTBEGIN
float invSqrt(float x);
int sgn(int x);
int fsgn(float x);
float sgn_like(float x, float d);
float fclip(float x, float low, float up);
int clip(int x, int low, int up);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float linear_mapping(const float x_min, const float x_max, const float tar_min,
    const float tar_max, float x);
float normalize(float x, float min, float max, float period);
// NOLINTEND
#endif