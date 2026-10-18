/* 主机测试封装，算法由固件 C 源码编译。 */
#include <string.h>
#include "pid.h"
#include "adrc.h"
#include "linear_spline.h"
#include "cubic_spline.h"
#include "quintic_spline.h"

static PID pid;
static TD_t td;

void reference_pid_init(float kp, float ki, float kd, float integral_limit,
                        float output_limit, float integral_range) {
    memset(&pid, 0, sizeof(pid));
    PID_SetConfig_Pos(&pid.config, kp, ki, kd, integral_limit, output_limit);
    pid.config.Irange = integral_range;
}

float reference_pid_step(float ref, float fdb) {
    pid.ref = ref;
    pid.fdb = fdb;
    PID_Calc(&pid);
    return pid.output;
}

void reference_td_init(float r) {
    TD_Config config = {.r = r, .h = 0.002f, .h0 = 0.004f, .J = 0};
    TD_Init(&td, &config);
}

void reference_td_step(float ref, float *out) {
    td.ref = ref;
    TDFunction_independent(&td);
    out[0] = td.v1;
    out[1] = td.v2;
    out[2] = td.v3;
}

void reference_spline(int kind, const float *start, const float *end,
                      float duration, float t, float *out) {
    if (kind == 0) {
        Linear_spline_config config = {0, start[0], duration, end[0], 0, duration};
        Linear_spline *p = Linear_Spline_Create(&config);
        out[0] = Linear_Spline_Calc_y(p, t);
        out[1] = Linear_Spline_Calc_dy(p, t);
        out[2] = Linear_Spline_Calc_ddy(p, t);
        Linear_Spline_Destroy(p);
    } else if (kind == 1) {
        Cubic_spline_config config = {0, start[0], start[1], duration, end[0], end[1], 0, duration};
        Cubic_spline *p = Cubic_Spline_Create(&config);
        out[0] = Cubic_Spline_Calc_y(p, t);
        out[1] = Cubic_Spline_Calc_dy(p, t);
        out[2] = Cubic_Spline_Calc_ddy(p, t);
        Cubic_Spline_Destroy(p);
    } else {
        Quintic_spline_config config = {0, start[0], start[1], start[2],
                                       duration, end[0], end[1], end[2], 0, duration};
        Quintic_spline *p = Quintic_Spline_Create(&config);
        out[0] = Quintic_Spline_Calc_y(p, t);
        out[1] = Quintic_Spline_Calc_dy(p, t);
        out[2] = Quintic_Spline_Calc_ddy(p, t);
        Quintic_Spline_Destroy(p);
    }
}
