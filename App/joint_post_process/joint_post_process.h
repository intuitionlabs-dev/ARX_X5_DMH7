#pragma once

#include <stdint.h>
#include "quintic_spline.h"

struct Gimbal_cmd_t;
struct ruckig_c_context;

typedef struct {
    float position, velocity, acceleration;
} Joint_motion_state;

/* 保存关节参考和当前算法的轨迹缓存。 */
typedef struct {
    Joint_motion_state state;
    int status;
    uint8_t initialized;
    union {
        struct ruckig_c_context* ruckig;
        struct {
            Quintic_spline curve;
            float target, elapsed;
            uint8_t active;
        } quintic;
    } trajectory;
} Joint_post_process_ref;

struct Joint_post_process_ops;

/* 从机型配置初始化轨迹算法，成功时返回 1。 */
int Joint_Post_Process_Init(struct Gimbal_cmd_t* obj);
