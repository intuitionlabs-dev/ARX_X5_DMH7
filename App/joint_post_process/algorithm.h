#pragma once

#include <math.h>
#include "arm_profile.h"
#include "joint_post_process.h"

/* 后处理模板调用的算法操作。 */
struct Joint_post_process_ops {
    int (*init)(Joint_post_process_ref* refs);
    void (*reset)(Joint_post_process_ref* ref);
    int (*step)(Joint_post_process_ref* ref, const Arm_joint_config* config, float target);
};

enum {
    JOINT_REFERENCE_WORKING = 0,
    JOINT_REFERENCE_FINISHED = 1,
    JOINT_REFERENCE_INVALID = -100,
};

static inline uint8_t joint_reference_is_finite(Joint_motion_state state) {
    return isfinite(state.position) && isfinite(state.velocity) && isfinite(state.acceleration);
}

extern const struct Joint_post_process_ops joint_ruckig_ops;
extern const struct Joint_post_process_ops joint_quintic_ops;
