#ifndef _ADRC_H_
#define _ADRC_H_

#include <stdint.h>
/* ADRC 由跟踪微分器、扩张状态观测器和非线性反馈组成。 */
#pragma pack(1)
/* ADRC运算的时候会出现的过程参数 */
typedef struct PROG_t {
    float ref;     /* 输入预期信号 */
    float v1;      /* 过渡过程之后的跟踪信号 */
    float v2;      /* 过渡过程之后的跟踪信号的微分(v1的微分) */
    float e1;      /* 误差信号1 */
    float e2;      /* 误差信号2 */
    float u0;      /* 非线性反馈输出的信号 */
    float output;  /* 结合ESO后的输出信号 */
    float z1;      /* ESO输出信号1 */
    float z2;      /* ESO输出信号2 */
    float z3;      /* ESO输出信号3 */
    float fdb;     /* 反馈信号 */
} PROG_t;

/* 跟踪微分器的参数 */
typedef struct TD_Config_t {
    float r;
    float h;
    float h0;
    float J;
} TD_Config;

typedef struct TD_t {
    TD_Config config;
    float ref;
    float v1;
    float v2;
    float v3;  /* 跟踪加速度，单位为输入单位/s² */
} TD_t;

/* 非线性反馈的参数 */
typedef struct NLSEF_Config_t {
    float Kp;
    float Kd;
    float alpha1;
    float alpha2;
    float delta;
} NLSEF_Config;

/* 扩张观测器的参数 */
typedef struct ESO_Config_t {
    float beta1;
    float beta2;
    float beta3;
    float b;
    float h;
    float z3_max;
    int8_t output_mode;
} ESO_Config;

typedef struct ESO_t {
    ESO_Config config;
    float fdb;
    float output;
    float z1;
    float z2;
    float z3;
} ESO_t;

typedef struct ADRC_Config_t {
    TD_Config td;
    NLSEF_Config nlsef;
    ESO_Config eso;
    float output_max;
} ADRC_Config_t;

/* ADRC总结构体 */
typedef struct ADRC_t {
    PROG_t prog;
    ADRC_Config_t adrc_config;
} ADRC_t;
#pragma pack()
void ADRC_SetConfig(ADRC_Config_t* adrc_config,
                    float r, float h, float h0,
                    float Kp, float Kd, float alpha1, float alpha2, float delta,
                    float beta1, float beta2, float beta3, float b, float output_max);
void ADRC_Init(ADRC_t* adrc_data, ADRC_Config_t* adrc_config);
void ADRCFunction(ADRC_t* adrc_data);

void TDFunction_independent(TD_t* td);
void TD_SetConfig(TD_Config* td_config, float r, float h, float h0, float J);
void TD_Init(TD_t* td, TD_Config* td_config);

void ESOFunction_independent(ESO_t* eso);
void ESO_SetConfig(ESO_Config* eso_config, float beta1, float beta2, float b, float h, float z3_max, int8_t output_mode);
void ESO_Init(ESO_t* eso, ESO_Config* eso_config);
#endif
