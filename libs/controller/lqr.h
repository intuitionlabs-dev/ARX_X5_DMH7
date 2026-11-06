#ifndef _LQR_H_
#define _LQR_H_

/* 以角度、角速度反馈和积分项生成控制输出。 */

enum Lqr_motor_e { MOTOR_6020 = 0,
                MOTOR_2006,
                MOTOR_3508 };

#pragma pack(1)
typedef struct Lqr_config_t {
    float k1;
    float k2;
    float ki;/* 积分项参数 */
    float ki_limit;/* 积分项限幅 */
    float errsummax;
    float outputmax;
    enum Lqr_motor_e motor;
    float feedforward;
} Lqr_config;

typedef struct Lqr_t {
    Lqr_config config;
    float x1;
    float x2;
    float x1_ref;
    float x1_err;
    double x1_errsum;
    float output;
    float output_full; /* 无输出限幅的计算输出 */
} Lqr;
#pragma pack()

void LQR_Init(Lqr* obj, Lqr_config* config);
void LQR_SetConfig(Lqr_config* obj, float k1, float k2, float ki, float errsummax, float outputmax,float ki_limit);
void LQR_Calc(Lqr* obj);

#endif
