#include "algorithm.h"

static void joint_quintic_reset(Joint_post_process_ref* ref) {
    ref->trajectory.quintic.active = 0;
}

static int joint_quintic_step(Joint_post_process_ref* ref, const Arm_joint_config* config, float target) {
    const float duration = config->quintic_duration_s;
    if (!isfinite(target) || !joint_reference_is_finite(ref->state)) return JOINT_REFERENCE_INVALID;

    if (!ref->trajectory.quintic.active || target != ref->trajectory.quintic.target) {
        Quintic_spline_config config = {
            .y0 = ref->state.position,
            .dy0 = ref->state.velocity,
            .ddy0 = ref->state.acceleration,
            .x1 = duration,
            .y1 = target,
            .xmax = duration,
        };
        Quintic_Spline_Reinit(&ref->trajectory.quintic.curve, &config);
        ref->trajectory.quintic.target = target;
        ref->trajectory.quintic.elapsed = 0;
        ref->trajectory.quintic.active = 1;
    }

    const float elapsed = ref->trajectory.quintic.elapsed + ARM_CONTROL_PERIOD_S;
    if (elapsed >= duration) {
        ref->trajectory.quintic.elapsed = duration;
        ref->state = (Joint_motion_state){target, 0, 0};
        return JOINT_REFERENCE_FINISHED;
    }
    ref->trajectory.quintic.elapsed = elapsed;
    Quintic_spline* curve = &ref->trajectory.quintic.curve;
    ref->state = (Joint_motion_state){Quintic_Spline_Calc_y(curve, elapsed),
        Quintic_Spline_Calc_dy(curve, elapsed), Quintic_Spline_Calc_ddy(curve, elapsed)};
    return JOINT_REFERENCE_WORKING;
}

const struct Joint_post_process_ops joint_quintic_ops = {
    .reset = joint_quintic_reset,
    .step = joint_quintic_step,
};
