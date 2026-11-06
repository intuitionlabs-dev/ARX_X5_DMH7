#include <common.h>
#include <math.h>

/* 牛顿迭代法求解1/sqrt(x) */
float invSqrt(float x) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Warray-bounds"
	float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y; /* 按位重解释浮点编码，用于倒平方根初值估计。 */
    i = 0x5f3759df - (i >> 1); /* 倒平方根近似的位级初值常量。 */
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
#pragma GCC diagnostic pop
}

/* 传入参数为整型数的符号函数 */
int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}

/* 传入参数为浮点数的符号函数 */
int fsgn(float x) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0);
#pragma GCC diagnostic pop
}

/* fabs(x)≥d时，返回x的符号函数，否则返回x/d */
float sgn_like(float x, float d) {
    if (fabs(x) >= d)
        return fsgn(x);
    else
        return x / d;
}
int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}

/* 按指定位宽将无符号编码换算到浮点量程。 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/* 按给定量程将浮点值换算为指定位宽的无符号编码。 */
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/* 将输入按两个标定点线性映射到输出区间。 */
float linear_mapping(const float x_min, const float x_max, const float tar_min, const float tar_max, float x) {
    return (tar_max - tar_min) * (x - x_min) / (x_max - x_min) + tar_min;
}

/* 按 period 周期将输入归入指定区间，参数无效或无法归入时返回原值。 */
float normalize(float x, float min, float max, float period) {
    if (period <= 0) return x;  /* 防止死循环 */
    const float x_init = x;
    while (x < min) {
        x += period;
        if(x > max) return x_init;
    }
    while (x > max) {
        x -= period;
        if(x < min) return x_init;
    }
    return x;
}
