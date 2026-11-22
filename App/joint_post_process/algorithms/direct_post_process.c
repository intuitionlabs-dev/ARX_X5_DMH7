#include "algorithm.h"

static void joint_direct_reset(Joint_post_process_ref* ref) {
    (void)ref;
}

static int joint_direct_step(Joint_post_process_ref* ref, const Arm_joint_config* config,
    Joint_motion_state target) {
    (void)config;
    if (!joint_reference_is_finite(target)) return JOINT_REFERENCE_INVALID;
    ref->state = target;
    return JOINT_REFERENCE_FINISHED;
}

const struct Joint_post_process_ops joint_direct_ops = {
    .reset = joint_direct_reset,
    .step = joint_direct_step,
};
