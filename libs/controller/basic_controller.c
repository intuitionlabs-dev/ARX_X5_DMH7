#include "basic_controller.h"

#include "common.h"
#include "stdlib.h"
#include "string.h"

Basic_controller_t* Create_Basic_Controller(const Basic_controller_config_t* config) {
    Basic_controller_t* obj = (Basic_controller_t*)malloc(sizeof(Basic_controller_t));
    memset(obj, 0, sizeof(Basic_controller_t));
    obj->config = *config;
    return obj;
}

void Basic_Controller_Calc(Basic_controller_t* controller) {
    if (controller->config.calc_feedforward_func != NULL) {
        controller->feedforward = controller->config.calc_feedforward_func(controller->config.arg);
    }
    if (controller->config.not_set_output) {
        return;
    }
    float output = controller->feedforward +
                   controller->config.kp * (controller->ref_pos - controller->fdb_pos) +
                   controller->config.kd * (controller->ref_vel - controller->fdb_vel);
    controller->output = _MID(output, -controller->config.limit, controller->config.limit);
    return;
}