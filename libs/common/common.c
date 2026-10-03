#include <common.h>
#include <math.h>

// 牛顿迭代法求解1/sqrt(x)
float invSqrt(float x) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Warray-bounds"
	float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y; // 将float型数据转为long型，注意，通过指针转换只是将内存中的数据换了一种解读方式，与强制类型转换不同
    i = 0x5f3759df - (i >> 1); // 0x5f3759df是一个精度比较高的经验值
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
#pragma GCC diagnostic pop
}

// 传入参数为整型数的符号函数
int sgn(int x){
    return x == 0 ? 0 : x > 0 ? 1 : -1;
}

// 传入参数为浮点数的符号函数
int fsgn(float x) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (x != 0.0f ? (x < 0.0f ? -1 : 1) : 0); 
#pragma GCC diagnostic pop
}

// fabs(x)≥d时，返回x的符号函数，否则返回x/d
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

/**
 * @brief 对应位数的uint转成float
 * @param x_min 函数最终返回的float的最小可能值
 * @param x_max 函数最终返回的float的最大可能值
 * @param bits x_int的位数
 **/
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief float转成对应位数的uint
 * @param x_min x的最小可能值
 * @param x_max x的最大可能值
 * @param bits 函数最终返回的uint的位数
 **/
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 区间线性映射函数
 */
float linear_mapping(const float x_min, const float x_max, const float tar_min, const float tar_max, float x) {
    return (tar_max - tar_min) * (x - x_min) / (x_max - x_min) + tar_min;
}

/**
 * @brief 对输入值进行周期性归一化，使其保持在[min, max]区间内。归一化失败会返回原值
 * 
 * @param x 输入值
 * @param min 区间下界
 * @param max 区间上界
 * @param period 周期长度
 */
float normalize(float x, float min, float max, float period) {
    if (period <= 0) return x;  // 防止死循环
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
