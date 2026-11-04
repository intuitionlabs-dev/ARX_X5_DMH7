/* Ordinary position interpolation from rebuild 3317d9b, with private state.
 * Same 0.16 s quintic equations, derivative continuity and target-change rule.
 * The control-source selection and real robot objects are not part of this test. */
typedef struct {
    Quintic_spline curve;
    float target, elapsed, position, velocity, acc;
    int active;
} Joint_quintic_ref;

static void Joint_Quintic_Benchmark_Step(Joint_quintic_ref* ref, float target) {
    if (!ref->active || target != ref->target) {
        Quintic_spline_config config = {
            .x0 = 0, .y0 = ref->position, .dy0 = ref->velocity, .ddy0 = ref->acc,
            .x1 = .16f, .y1 = target, .dy1 = 0, .ddy1 = 0, .xmin = 0, .xmax = .16f,
        };
        Quintic_Spline_Reinit(&ref->curve, &config);
        ref->target = target;
        ref->elapsed = 0;
        ref->active = 1;
    }
    ref->elapsed += ARM_CONTROL_PERIOD_S;
    if (ref->elapsed >= .16f) {
        ref->elapsed = .16f;
        ref->position = ref->target;
        ref->velocity = ref->acc = 0;
    } else {
        ref->position = Quintic_Spline_Calc_y(&ref->curve, ref->elapsed);
        ref->velocity = Quintic_Spline_Calc_dy(&ref->curve, ref->elapsed);
        ref->acc = Quintic_Spline_Calc_ddy(&ref->curve, ref->elapsed);
    }
}
