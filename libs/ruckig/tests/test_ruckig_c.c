#include "ruckig_c.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const double dt = 0.002;

static ruckig_c_input input_for(double target) {
    const ruckig_c_input input = {
        .target = {target, 0, 0}, .max_velocity = 90,
        .max_acceleration = 360, .max_jerk = 3600, .minimum_duration = .16
    };
    return input;
}

static void check_step(const ruckig_c_input* input, const ruckig_c_output* output) {
    const ruckig_c_state a = input->current, b = output->state;
    assert(isfinite(b.position) && isfinite(b.velocity) && isfinite(b.acceleration));
    assert(fabs(b.velocity) <= input->max_velocity + 1e-7);
    assert(fabs(b.acceleration) <= input->max_acceleration + 1e-7);
    assert(fabs(b.acceleration - a.acceleration) <= input->max_jerk * dt + 1e-6);
    assert(fabs(b.velocity - a.velocity) <= input->max_acceleration * dt + 1e-7);
    assert(fabs(b.position - a.position) <= input->max_velocity * dt + 1e-7);
}

static void motion_and_replan(void) {
    ruckig_c_context* ctx = ruckig_c_create(dt);
    assert(ctx);
    ruckig_c_input in = input_for(30);
    ruckig_c_output out;
    for (int i = 0; i < 100; ++i) {
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_WORKING);
        assert(out.new_calculation == (i == 0));
        check_step(&in, &out);
        in.current = out.state;
    }
    assert(out.state.velocity > 0 && out.state.acceleration != 0);
    in.target.position = -10;
    assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_WORKING);
    assert(out.new_calculation && out.time == dt);
    check_step(&in, &out);
    in.current = out.state;
    int status = RUCKIG_C_WORKING;
    for (int i = 0; i < 5000 && status == RUCKIG_C_WORKING; ++i) {
        status = ruckig_c_update(ctx, &in, &out);
        assert(status >= 0 && !out.new_calculation);
        check_step(&in, &out);
        in.current = out.state;
    }
    assert(status == RUCKIG_C_FINISHED);
    assert(out.state.position == -10 && out.state.velocity == 0 && out.state.acceleration == 0);
    for (int i = 0; i < 20; ++i) {
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_FINISHED);
        assert(!out.new_calculation && out.time == out.duration);
    }
    ruckig_c_reset(ctx);
    in.current = (ruckig_c_state){5, 12, -20};
    assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_WORKING);
    assert(out.new_calculation);
    check_step(&in, &out);
    in.current = out.state;
    in.max_jerk *= .5;
    assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_WORKING && out.new_calculation);
    check_step(&in, &out);
    ruckig_c_destroy(ctx);
}

static void invalid_and_recovery(void) {
    assert(!ruckig_c_create(0) && !ruckig_c_create(-1));
    assert(!ruckig_c_create(NAN) && !ruckig_c_create(INFINITY));
    ruckig_c_destroy(NULL);
    ruckig_c_reset(NULL);
    ruckig_c_context* ctx = ruckig_c_create(dt);
    assert(ctx);
    ruckig_c_input in = input_for(1);
    ruckig_c_output out = {.state = {123, 456, 789}, .time = 12, .duration = 34};
    const ruckig_c_output sentinel = out;
    assert(ruckig_c_update(NULL, &in, &out) < 0);
    assert(ruckig_c_update(ctx, NULL, &out) < 0);
    assert(ruckig_c_update(ctx, &in, NULL) < 0);
    double* values[] = {&in.current.position, &in.current.velocity, &in.current.acceleration,
        &in.target.position, &in.target.velocity, &in.target.acceleration,
        &in.max_velocity, &in.max_acceleration, &in.max_jerk, &in.minimum_duration};
    for (unsigned i = 0; i < sizeof(values)/sizeof(values[0]); ++i) {
        double saved = *values[i];
        *values[i] = NAN;
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_INVALID_INPUT);
        *values[i] = INFINITY;
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_INVALID_INPUT);
        assert(memcmp(&out, &sentinel, sizeof(out)) == 0);
        *values[i] = saved;
    }
    double* limits[] = {&in.max_velocity, &in.max_acceleration, &in.max_jerk};
    for (unsigned i = 0; i < 3; ++i) {
        double saved = *limits[i];
        *limits[i] = 0;
        assert(ruckig_c_update(ctx, &in, &out) < 0);
        *limits[i] = -1;
        assert(ruckig_c_update(ctx, &in, &out) < 0);
        *limits[i] = saved;
    }
    in.minimum_duration = -1;
    assert(ruckig_c_update(ctx, &in, &out) < 0);
    in.minimum_duration = .16;
    in.target.velocity = in.max_velocity * 2;
    assert(ruckig_c_update(ctx, &in, &out) < 0);
    in.target.velocity = 0;
    assert(ruckig_c_update(ctx, &in, &out) >= 0 && out.new_calculation);
    in.current = out.state;
    assert(ruckig_c_update(ctx, &in, &out) >= 0 && !out.new_calculation);
    ruckig_c_destroy(ctx);
}

static void durations_and_isolation(void) {
    ruckig_c_context* a = ruckig_c_create(dt);
    ruckig_c_context* b = ruckig_c_create(dt);
    assert(a && b);
    ruckig_c_input ia = input_for(.01), ib = input_for(-100);
    ia.minimum_duration = 1;
    ruckig_c_output oa, ob;
    for (int i = 0; i < 5000; ++i) {
        assert(ruckig_c_update(a, &ia, &oa) >= 0);
        assert(ruckig_c_update(b, &ib, &ob) >= 0);
        assert(oa.new_calculation == (i == 0) && ob.new_calculation == (i == 0));
        check_step(&ia, &oa);
        check_step(&ib, &ob);
        ia.current = oa.state;
        ib.current = ob.state;
    }
    assert(oa.duration >= 1 && oa.state.position == .01);
    assert(ob.duration > .16 && ob.state.position == -100);
    ruckig_c_reset(a);
    ia = input_for(0);
    ia.minimum_duration = 0;
    assert(ruckig_c_update(a, &ia, &oa) == RUCKIG_C_FINISHED);
    assert(oa.duration == 0 && oa.state.position == 0);
    /* Initially excessive speed can be recovered without discarding continuity. */
    ruckig_c_reset(a);
    ia.current.velocity = 100;
    assert(ruckig_c_update(a, &ia, &oa) == RUCKIG_C_WORKING);
    assert(fabs(oa.state.velocity - 100) <= ia.max_acceleration*dt);
    ruckig_c_destroy(a);
    ruckig_c_destroy(b);
}

int main(void) {
    motion_and_replan();
    invalid_and_recovery();
    durations_and_isolation();
    puts("Ruckig C API: constraints, replanning, endpoints, errors and isolation passed");
    return 0;
}
