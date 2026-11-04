#include "ruckig_c.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static const float dt = 0.002f;

static ruckig_c_input input_for(float target) {
    const ruckig_c_input input = {
        .target = {target, 0, 0}, .max_velocity = 90,
        .max_acceleration = 360, .max_jerk = 3600, .minimum_duration = .16f
    };
    return input;
}

static void check_step(const ruckig_c_input* input, const ruckig_c_output* output) {
    const ruckig_c_state a = input->current, b = output->state;
    /* Limits permit a few FP32 ulps; differencing also rounds sample time. */
    const float v_roundoff = 16.0f*FLT_EPSILON*fmaxf(1.0f, input->max_velocity);
    const float a_roundoff = 16.0f*FLT_EPSILON*fmaxf(1.0f, input->max_acceleration);
    const float time_roundoff = 4.0f*FLT_EPSILON*fmaxf(dt, output->time);
    assert(isfinite(b.position) && isfinite(b.velocity) && isfinite(b.acceleration));
    assert(fabsf(b.velocity) <= input->max_velocity + v_roundoff);
    assert(fabsf(b.acceleration) <= input->max_acceleration + a_roundoff);
    assert(fabsf(b.acceleration - a.acceleration) <= input->max_jerk * (dt + time_roundoff) + a_roundoff);
    assert(fabsf(b.velocity - a.velocity) <= input->max_acceleration * (dt + time_roundoff) + v_roundoff);
    assert(fabsf(b.position - a.position) <= input->max_velocity * (dt + time_roundoff)
        + 8.0f*FLT_EPSILON*fmaxf(1.0f, fabsf(a.position)));
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
    in.max_jerk *= .5f;
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
    float* values[] = {&in.current.position, &in.current.velocity, &in.current.acceleration,
        &in.target.position, &in.target.velocity, &in.target.acceleration,
        &in.max_velocity, &in.max_acceleration, &in.max_jerk, &in.minimum_duration};
    for (unsigned i = 0; i < sizeof(values)/sizeof(values[0]); ++i) {
        float saved = *values[i];
        *values[i] = NAN;
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_INVALID_INPUT);
        *values[i] = INFINITY;
        assert(ruckig_c_update(ctx, &in, &out) == RUCKIG_C_INVALID_INPUT);
        assert(memcmp(&out, &sentinel, sizeof(out)) == 0);
        *values[i] = saved;
    }
    float* limits[] = {&in.max_velocity, &in.max_acceleration, &in.max_jerk};
    for (unsigned i = 0; i < 3; ++i) {
        float saved = *limits[i];
        *limits[i] = 0;
        assert(ruckig_c_update(ctx, &in, &out) < 0);
        *limits[i] = -1;
        assert(ruckig_c_update(ctx, &in, &out) < 0);
        *limits[i] = saved;
    }
    in.minimum_duration = -1;
    assert(ruckig_c_update(ctx, &in, &out) < 0);
    in.minimum_duration = .16f;
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
    ruckig_c_input ia = input_for(.01f), ib = input_for(-100);
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
    assert(oa.duration >= 1 && oa.state.position == .01f);
    assert(ob.duration > .16f && ob.state.position == -100);
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

/* Regression cases for small quartic coefficients, cancellation at reversal,
 * and a positive acceleration plateau shorter than FLT_EPSILON seconds. */
static void float_regressions(void) {
    const float cases[][4] = {
        {0x1.b94346p+3f, 0x1.0d3f98p-2f, -0x1.3c11e2p+4f, 0x1.b8fc66p+3f},
        {0x1.b841bp+4f, -0x1.860acp-3f, -0x1.68p+8f, 0x1.a5d4dp+4f},
        {0x1.73465ap+4f, 0x1.2df05ap+5f, -0x1.68p+8f, 0x1.4370dp+5f},
    };
    for (unsigned c=0; c<sizeof(cases)/sizeof(cases[0]); ++c) {
        ruckig_c_context* ctx = ruckig_c_create(dt);
        assert(ctx);
        ruckig_c_input in = input_for(cases[c][3]);
        in.current = (ruckig_c_state){cases[c][0], cases[c][1], cases[c][2]};
        ruckig_c_output out;
        int status = 0;
        for (int k=0; k<5000 && status==0; ++k) {
            status = ruckig_c_update(ctx, &in, &out);
            assert(status >= 0);
            check_step(&in, &out);
            in.current = out.state;
        }
        assert(status == RUCKIG_C_FINISHED);
        assert(out.state.position == in.target.position);
        ruckig_c_destroy(ctx);
    }
}

static void streaming_float(void) {
    ruckig_c_context* contexts[12];
    ruckig_c_input inputs[12];
    uint32_t random_state = 0x12345678;
    for (unsigned scenario=0; scenario<3; ++scenario) {
        for (unsigned i=0; i<12; ++i) {
            contexts[i] = ruckig_c_create(dt);
            assert(contexts[i]);
            inputs[i] = input_for(0);
        }
        for (unsigned k=0; k<205000; ++k) {
            for (unsigned i=0; i<12; ++i) {
                float target;
                if (scenario == 2) {
                    random_state ^= random_state << 13;
                    random_state ^= random_state >> 17;
                    random_state ^= random_state << 5;
                    target = 120.0f*(random_state/4294967295.0f)-60.0f;
                } else {
                    const unsigned tick = scenario == 0 ? k/5*5 : k;
                    target = (10.0f+2.0f*(i%6))*sinf(6.283185307179586f
                        *(.2f+.03f*(i%6))*(tick*dt)+.4f*(i/6));
                }
                inputs[i].target.position = target;
                ruckig_c_output out;
                int status = ruckig_c_update(contexts[i], &inputs[i], &out);
                if (status < 0) fprintf(stderr, "stream %u tick %u axis %u status %d\n",
                    scenario, k, i, status);
                assert(status >= 0);
                check_step(&inputs[i], &out);
                inputs[i].current = out.state;
            }
        }
        for (unsigned i=0; i<12; ++i) ruckig_c_destroy(contexts[i]);
    }
}

int main(void) {
    _Static_assert(sizeof(float) == 4, "32-bit float required");
    _Static_assert(sizeof(ruckig_c_state) == 12, "C state must contain only FP32");
    motion_and_replan();
    invalid_and_recovery();
    durations_and_isolation();
    float_regressions();
    streaming_float();
    puts("Ruckig C API: constraints, replanning, endpoints, errors and isolation passed");
    return 0;
}
