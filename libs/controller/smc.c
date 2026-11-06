#include "smc.h"
#include "common.h"

/* sliding mode control 滑膜控制 */
static float sat(float error,float sat_para);
void SMC_Init(Smc* smc, Smc_config* config) { smc->config = *config;
                                                smc->sat_para=200; }

void SMC_SetConfig(Smc_config* config, float kp, float kd, float kmax, ReachingLaw reaching_law, float outputMax) {
    config->kp = kp;
    config->kd = kd;
    config->kmax = kmax;
    config->reaching_law = reaching_law;
    config->outputMax = outputMax;
}

void SMC_Calc(Smc* smc) {
    smc->error[1] = smc->error[0];
    smc->error[0] = smc->ref - smc->fdb;

    float u;
    u = smc->config.reaching_law(smc->error[0]);
    u*=sat(smc->error[0],smc->sat_para);
    smc->output = smc->config.kp * smc->error[0]                      /* 经典滑膜面选择 */
                  + smc->config.kd * (smc->error[0] - smc->error[1])
                  + smc->config.kmax * u;                             /* 趋近律 整项要能大于扰动的最大值 */
    if (smc->output < -smc->config.outputMax) smc->output = -smc->config.outputMax;
    if (smc->output > smc->config.outputMax) smc->output = smc->config.outputMax;
}

/* 最简单的趋近律 */
/* 易产生震动 */
float ReachingLaw_sgn(float error) { return fsgn(error); }

/* 根号趋近 */
/* 能有效减小稳态误差 */
float ReachingLaw_sqrt(float error) {
    float u;
    if (error > 0) {
        u = sqrtf(fabs(error));
    } else if (error < 0) {
        u = sqrtf(fabs(error));
        //u *= -1;
    } else
        u = 0;
    return u;
}

static float sat(float error,float sat_para){
    if (fabs(error/sat_para)<=1){
        return error/sat_para;
    } else
        return sgn(error/sat_para);
}
float ReachingLaw_square(float error) { return (error * error + 0.1) * fsgn(error); }
