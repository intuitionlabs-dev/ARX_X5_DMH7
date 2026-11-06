#include "adrc.h"

#include <common.h>
#include <string.h>

#include "math.h"

float ConstrainFloat(float amt, float low, float high);
int fsg(float x, float d);
float fst(float x1_delta, float x2, float r, float h0);
float fal(float e, float a, float delta);
void TDFunction(ADRC_t* adrc_data);
void NLSEFFunction(ADRC_t* adrc_data);
void ESOFunction(ADRC_t* adrc_data);
void ADRCFunction(ADRC_t* adrc_data);

/* 将输入限制到给定上下界。 */
float ConstrainFloat(float amt, float low, float high) {
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/* 计算带阈值的二维符号函数。 */
int fsg(float x, float d) {
    return ((fsgn(x + d) - fsgn(x - d)) / 2);
}

/* fal 在误差阈值内采用线性增益，阈值外采用带符号幂函数。 */
float fal(float e, float a, float delta) {
    int s = 0;
    s = (fsgn(e + delta) - fsgn(e - delta)) / 2;
    return (e * s / (powf(delta, 1 - a)) + powf(fabs(e), a) * fsgn(e) * (1 - s));
}

/* 最速跟踪函数使用 r 限制跟踪加速度，h0 决定滤波时间尺度。 */
float fst(float x1_delta, float x2, float r, float h0) {
    float d, y, a0, a, a1, a2;
    d = r * h0 * h0;
    a0 = h0 * x2;
    y = x1_delta + a0;
    a1 = sqrt(d * (d + 8 * fabs(y)));
    a2 = a0 + fsgn(y) * (a1 - d) / 2;
    a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d));
    return (-r * (a / d) * fsg(a, d) - r * fsgn(a) * (1 - fsg(a, d)));
}

/* 更新参考信号及其导数的跟踪状态。 */
void TDFunction(ADRC_t* adrc_data) {
    float last_v1, last_v2;
    last_v1 = adrc_data->prog.v1;
    last_v2 = adrc_data->prog.v2;
    adrc_data->prog.v1 = last_v1 + adrc_data->adrc_config.td.h * last_v2;
    adrc_data->prog.v2 = last_v2 + adrc_data->adrc_config.td.h *
                                       fst(last_v1 - adrc_data->prog.ref, last_v2,
                                           adrc_data->adrc_config.td.r, adrc_data->adrc_config.td.h0);
}
/* 按跟踪误差计算非线性反馈。 */
void NLSEFFunction(ADRC_t* adrc_data) {
    adrc_data->prog.e2 = ConstrainFloat(adrc_data->prog.e2, -3000, 3000);
    adrc_data->prog.u0 = adrc_data->adrc_config.nlsef.Kp *
                             fal(adrc_data->prog.e1, adrc_data->adrc_config.nlsef.alpha1, adrc_data->adrc_config.nlsef.delta) +
                         adrc_data->adrc_config.nlsef.Kd *
                             fal(adrc_data->prog.e2, adrc_data->adrc_config.nlsef.alpha2, adrc_data->adrc_config.nlsef.delta);
}

/* 更新扩张状态观测器并限制扰动估计。 */
void ESOFunction(ADRC_t* adrc_data) {
    float error, last_z1, last_z2, last_z3;
    last_z1 = adrc_data->prog.z1;
    last_z2 = adrc_data->prog.z2;
    last_z3 = adrc_data->prog.z3;
    error = last_z1 - adrc_data->prog.fdb;
    adrc_data->prog.z1 = last_z1 + adrc_data->adrc_config.td.h *
                                       (last_z2 - adrc_data->adrc_config.eso.beta1 * error);
    adrc_data->prog.z2 = last_z2 + adrc_data->adrc_config.td.h *
                                       (last_z3 - adrc_data->adrc_config.eso.beta2 * fal(error, adrc_data->adrc_config.nlsef.alpha1, adrc_data->adrc_config.nlsef.delta) +
                                        adrc_data->adrc_config.eso.b * adrc_data->prog.output);
    adrc_data->prog.z3 = last_z3 + adrc_data->adrc_config.td.h *
                                       (-adrc_data->adrc_config.eso.beta3 * fal(error, adrc_data->adrc_config.nlsef.alpha2, adrc_data->adrc_config.nlsef.delta));
    adrc_data->prog.z3 = ConstrainFloat(adrc_data->prog.z3, -200, 200);
}

/* 推进跟踪、观测和扰动补偿控制。 */
void ADRCFunction(ADRC_t* adrc_data) {
    TDFunction(adrc_data);
    ESOFunction(adrc_data);
    adrc_data->prog.e1 = adrc_data->prog.v1 - adrc_data->prog.z1;
    adrc_data->prog.e2 = adrc_data->prog.v2 - adrc_data->prog.z2;
    NLSEFFunction(adrc_data);
    adrc_data->prog.output = adrc_data->prog.u0 - adrc_data->prog.z3 / adrc_data->adrc_config.eso.b;
    adrc_data->prog.output = ConstrainFloat(adrc_data->prog.output, -adrc_data->adrc_config.output_max, adrc_data->adrc_config.output_max);
}

/* 独立更新跟踪微分器状态。 */
void TDFunction_independent(TD_t* td) {
    float last_v1, last_v2;
    last_v1 = td->v1;
    last_v2 = td->v2;
    td->v3 = fst(last_v1 - td->ref, last_v2, td->config.r, td->config.h0);
    td->v1 = last_v1 + td->config.h * last_v2;
    td->v2 = last_v2 + td->config.h * td->v3;
}

/* 独立更新扩张状态观测器并限制估计值。 */
void ESOFunction_independent(ESO_t* eso) {
    float error, last_z1, last_z2, last_z3;
    float omega = eso->config.beta1 / 3;
    float beta1 = eso->config.beta1;
    float beta2 = 3 * omega * omega;
    float beta3 = omega * omega * omega;
    last_z1 = eso->z1;
    last_z2 = eso->z2;
    last_z3 = eso->z3;
    error = eso->fdb - last_z1;

    eso->z1 = last_z1 + eso->config.h *
                            (last_z2 + beta1 * error);
    eso->z2 = last_z2 + eso->config.h *
                            (last_z3 + eso->config.b * eso->output + beta2 * error);
    eso->z3 = last_z3 + eso->config.h * beta3 * error;
    eso->z3 = ConstrainFloat(eso->z3, -eso->config.z3_max, eso->config.z3_max);
}

/* 初始化 ADRC 配置参数。 */
void ADRC_SetConfig(ADRC_Config_t* adrc_config,
                    float r, float h, float h0,
                    float Kp, float Kd, float alpha1, float alpha2, float delta,
                    float beta1, float beta2, float beta3, float b, float output_max) {
    adrc_config->td.r = r;
    adrc_config->td.h = h;
    adrc_config->td.h0 = h0;
    adrc_config->nlsef.Kp = Kp;
    adrc_config->nlsef.Kd = Kd;
    adrc_config->nlsef.alpha1 = alpha1;
    adrc_config->nlsef.alpha2 = alpha2;
    adrc_config->nlsef.delta = delta;
    adrc_config->eso.beta1 = beta1;
    adrc_config->eso.beta2 = beta2;
    adrc_config->eso.beta3 = beta3;
    adrc_config->eso.b = b;
    adrc_config->output_max = output_max;
}

/* 初始化跟踪微分器配置。 */
void TD_SetConfig(TD_Config* td_config, float r, float h, float h0, float J) {
    td_config->r = r;
    td_config->h = h;
    td_config->h0 = h0;
    td_config->J = J;
}

/* 初始化扩张状态观测器配置。 */
 void ESO_SetConfig(ESO_Config* eso_config, float beta1, float beta2, float b, float h, float z3_max, int8_t output_mode) {
    eso_config->beta1 = beta1;
    eso_config->beta2 = beta2;
    eso_config->b = b;
    eso_config->h = h;
    eso_config->z3_max = z3_max;
    eso_config->output_mode = output_mode;
}

/* 清空 ADRC 状态并加载配置。 */
void ADRC_Init(ADRC_t* adrc_data, ADRC_Config_t* adrc_config) {
    adrc_data->adrc_config = *adrc_config;
    memset(&adrc_data->prog, 0, sizeof(adrc_data->prog));
}

/* 清空跟踪微分器状态并加载配置。 */
void TD_Init(TD_t* td, TD_Config* td_config) {
    memset(td, 0, sizeof(TD_t));
    td->config = *td_config;
}

/* 清空扩张状态观测器状态并加载配置。 */
void ESO_Init(ESO_t* eso, ESO_Config* eso_config) {
    memset(eso, 0, sizeof(ESO_t));
    eso->config.beta1 = eso_config->beta1;
    eso->config.beta2 = eso_config->beta2;
    eso->config.b = eso_config->b;
    eso->config.h = eso_config->h;
    eso->config.z3_max = eso_config->z3_max;
    eso->config.output_mode = eso_config->output_mode;
}
