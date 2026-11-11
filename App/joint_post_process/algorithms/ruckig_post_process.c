#include "algorithm.h"
#include "ruckig_c.h"

static int joint_ruckig_init(Joint_post_process_ref* refs) {
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        refs[i].trajectory.ruckig = ruckig_c_create(ARM_CONTROL_PERIOD_S);
        if (refs[i].trajectory.ruckig == NULL) {
            for (int j = 0; j < i; ++j)
                ruckig_c_destroy(refs[j].trajectory.ruckig);
            return 0;
        }
    }
    return 1;
}

static void joint_ruckig_reset(Joint_post_process_ref* ref) {
    ruckig_c_reset(ref->trajectory.ruckig);
}

static int joint_ruckig_step(Joint_post_process_ref* ref, const Arm_joint_config* config, float target) {
    const ruckig_c_input input = {
        .current = {ref->state.position, ref->state.velocity, ref->state.acceleration},
        .target = {target, 0, 0},
        .max_velocity = config->max_velocity_deg_s,
        .max_acceleration = config->max_acceleration_deg_s2,
        .max_jerk = config->max_jerk_deg_s3,
    };
    ruckig_c_output output;
    const int status = ruckig_c_update(ref->trajectory.ruckig, &input, &output);
    if (status >= 0) {
        ref->state = (Joint_motion_state){output.state.position, output.state.velocity,
            output.state.acceleration};
    }
    return status;
}

const struct Joint_post_process_ops joint_ruckig_ops = {
    .init = joint_ruckig_init,
    .reset = joint_ruckig_reset,
    .step = joint_ruckig_step,
};
