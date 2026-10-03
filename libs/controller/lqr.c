/**
 * @file           : lqr.cpp
 * @brief          : LQR算法的代码实现
 * @Author         : 李鸣航
 * @Date           : 2022-04-09 14:42
 * @LastEditTime   : 2022-04-28 17:29
 * @Note           :
 * @Copyright(c)   : 哈尔滨工业大学（深圳）南工骁鹰机器人队版权所有 Critical HIT copyrighted
 */

#include "lqr.h"
#include "math.h"

void LQR_Init(Lqr* obj, Lqr_config* config) { obj->config = *config; }

void LQR_SetConfig(Lqr_config* obj, float k1, float k2, float ki, float errsummax, float outputmax,float ki_limit) {
    obj->k1 = k1;
    obj->k2 = k2;
    obj->ki = ki;
    obj->errsummax = errsummax;
    obj->outputmax = outputmax;
    obj->ki_limit = ki_limit;
}

void LQR_Calc(Lqr* obj){
    obj->x1_err = obj->x1 - obj->x1_ref; // x1"误差"定义为实际值-预期值
    if (fabs(obj->x1_err) < obj->config.ki_limit) {
        // obj->x1_errsum += obj->x1_err * 0.001; // 执行频率1kHz
        if (obj->output_full > obj->config.outputmax) {
            if (obj->x1_err < 0) {
                obj->x1_errsum += obj->x1_err * 0.001;
            }
        } else if (obj->output_full < -obj->config.outputmax) {
            if (obj->x1_err > 0) {
                obj->x1_errsum += obj->x1_err * 0.001;
            }
        } else {
            obj->x1_errsum += obj->x1_err * 0.001;
        }
    } else {
        obj->x1_errsum = 0;
    }

    if (obj->x1_errsum > obj->config.errsummax) obj->x1_errsum = obj->config.errsummax;
    else if (obj->x1_errsum < -obj->config.errsummax) obj->x1_errsum = -obj->config.errsummax;

    float i_ref = -(obj->x1_err * obj->config.k1 + obj->x2 * obj->config.k2 + obj->x1_errsum * obj->config.ki) / 0.741;
    i_ref += obj->config.feedforward;
    // 判断lqr运用电机的类型
    if (obj->config.motor == MOTOR_6020) {
        obj->output = 1111 * i_ref - 236;
    }
    if (obj->config.motor == MOTOR_2006) {
        obj->output = i_ref / 36.0f;//待测试
    }
    if (obj->config.motor == MOTOR_3508) {
        obj->output = i_ref;//待测试
    }
    obj->output_full = obj->output;
    if (obj->output > obj->config.outputmax) obj->output = obj->config.outputmax;
    else if (obj->output < -obj->config.outputmax) obj->output = -obj->config.outputmax;
}
