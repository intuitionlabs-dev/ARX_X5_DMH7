#pragma once
#include "stdint.h"

#pragma pack(1)

typedef float (*Calc_feedforward_func_handle_t) (void* arg);

typedef struct Basic_controller_config_t {
    float kp;
    float kd;
    float limit;
    Calc_feedforward_func_handle_t calc_feedforward_func;
    void* arg;
    /* 不赋值output，方便调试 */
    uint8_t not_set_output;
} Basic_controller_config_t;

typedef struct Basic_controller_t {
    Basic_controller_config_t config;
    /* 用户设定值 */
    float ref_pos;
    float ref_vel;

    /* 状态量 */
    float fdb_pos;
    float fdb_vel;
    float feedforward;
    float output;
} Basic_controller_t;
#pragma pack()

Basic_controller_t* Create_Basic_Controller(const Basic_controller_config_t* config);
void Basic_Controller_Calc(Basic_controller_t* controller);
