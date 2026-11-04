/* Included by h7_joint_benchmark.inc; no project build changes required. */
#include "joint_benchmark.h"
#include "robot_def.h"
#include "ruckig_c.h"
#include "../profiles/profile_constants.h"
#include "quintic_spline.h"
#include <math.h>
#include <string.h>

#ifndef JOINT_BENCHMARK_STORAGE
#define JOINT_BENCHMARK_STORAGE
#endif
#include "joint_quintic_baseline.inc"

typedef struct {
    ruckig_c_context* planner;
    ruckig_c_input input;
    ruckig_c_state output;
    Joint_quintic_ref quintic;
    int status;
} JointBenchmarkAxis;

static JointBenchmarkAxis axes[ARM_NUM][ARM_JOINT_NUM] JOINT_BENCHMARK_STORAGE;
static float targets[ARM_NUM][ARM_JOINT_NUM] JOINT_BENCHMARK_STORAGE;
static unsigned active_arms, active_algorithm;
static uint32_t random_state;
static int contexts_created;

/* Caller serializes these startup/cleanup allocations against other tasks. */
void JointBenchmark_Destroy(void) {
    if (!contexts_created) return;
    for (unsigned a=0; a<ARM_NUM; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j)
            ruckig_c_destroy(axes[a][j].planner);
    memset(axes, 0, sizeof(axes));
    contexts_created = 0;
    active_arms = 0;
}

int JointBenchmark_Create(void) {
    if (contexts_created) return 1;
    /* RAM_D1 is NOLOAD: initialize explicitly before examining any pointers. */
    memset(axes, 0, sizeof(axes));
    contexts_created = 1;
    for (unsigned a=0; a<ARM_NUM; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            axes[a][j].planner = ruckig_c_create(ARM_CONTROL_PERIOD_S);
            if (!axes[a][j].planner) { JointBenchmark_Destroy(); return 0; }
        }
    return 1;
}

int JointBenchmark_Init(unsigned arms, unsigned algorithm) {
    if (!contexts_created || arms < 1 || arms > ARM_NUM || algorithm >= JOINT_BENCH_ALGORITHMS) return 0;
    memset(targets, 0, sizeof(targets));
    random_state = 0x12345678;
    active_algorithm = algorithm;
    active_arms = arms;
    for (unsigned a=0; a<arms; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            JointBenchmarkAxis* axis = &axes[a][j];
            axis->input = (ruckig_c_input){
                .max_velocity = ARM_PROFILE_MAX_VELOCITY_DEG_S,
                .max_acceleration = ARM_PROFILE_MAX_ACCELERATION_DEG_S2,
                .max_jerk = ARM_PROFILE_MAX_JERK_DEG_S3,
                .minimum_duration = ARM_PROFILE_POSITION_MIN_DURATION_S,
            };
            axis->output = (ruckig_c_state){0};
            memset(&axis->quintic, 0, sizeof(axis->quintic));
            axis->status = 0;
            ruckig_c_reset(axis->planner);
        }
    return 1;
}

void JointBenchmark_Target(unsigned scenario, unsigned tick) {
    for (unsigned a=0; a<active_arms; ++a) {
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            float target = 0;
            if (scenario == JOINT_BENCH_STEP) {
                target = ((tick/500+j+a)%2 ? 1.0f : -1.0f)*(10.0f+2.0f*j);
            } else if (scenario == JOINT_BENCH_RANDOM) {
                random_state ^= random_state << 13;
                random_state ^= random_state >> 17;
                random_state ^= random_state << 5;
                target = 120.0f*(random_state/4294967295.0f)-60.0f;
            } else if (scenario == JOINT_BENCH_SMOOTH_100 || scenario == JOINT_BENCH_SMOOTH_500) {
                const unsigned input_tick = scenario == JOINT_BENCH_SMOOTH_100 ? tick/5*5 : tick;
                target = (10.0f+2.0f*j)*sinf(6.283185307179586f*(.2f+.03f*j)
                    *(input_tick*ARM_CONTROL_PERIOD_S)+.4f*a);
            }
            targets[a][j] = target;
        }
    }
}

/* Timed scope: private joint interpolation and state update only.
 * No Arm_Ref_Set, live Arm/Gimbal/RC objects, motor commands or IO. */
void JointBenchmark_Step(void) {
    for (unsigned a=0; a<active_arms; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            JointBenchmarkAxis* axis = &axes[a][j];
            if (active_algorithm == JOINT_BENCH_RUCKIG) {
                axis->input.target.position = targets[a][j];
                ruckig_c_output result;
                axis->status = ruckig_c_update(axis->planner, &axis->input, &result);
                if (axis->status >= 0) axis->input.current = axis->output = result.state;
            } else {
                Joint_Quintic_Benchmark_Step(&axis->quintic, targets[a][j]);
                axis->output = (ruckig_c_state){axis->quintic.position,
                    axis->quintic.velocity, axis->quintic.acc};
            }
        }
}

int JointBenchmark_Check(void) {
    if (!active_arms) return -1;
    for (unsigned a=0; a<active_arms; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            const JointBenchmarkAxis* axis = &axes[a][j];
            if (axis->status < 0) return axis->status;
            if (!isfinite(axis->output.position) || !isfinite(axis->output.velocity)
                || !isfinite(axis->output.acceleration)) return -1;
        }
    return 0;
}

float JointBenchmark_Checksum(void) {
    float sum = 0;
    for (unsigned a=0; a<active_arms; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j)
            sum += axes[a][j].output.position + axes[a][j].output.velocity + axes[a][j].output.acceleration;
    return sum;
}

void JointBenchmark_AddSample(JointBenchmarkStats* s, uint32_t start, uint32_t end, uint32_t hz) {
    const uint32_t cycles = end-start; /* Unsigned subtraction handles one wrap. */
    if (!s->samples || cycles < s->min_cycles) s->min_cycles = cycles;
    if (cycles > s->max_cycles) s->max_cycles = cycles;
    s->total_cycles += cycles;
    ++s->samples;
    if ((uint64_t)cycles*1000000u > (uint64_t)hz*JOINT_BENCH_PERIOD_US) ++s->over_budget;
}

int JointBenchmark_Summarize(JointBenchmarkStats* s, uint32_t hz) {
    if (!s->samples || !hz) return 0;
    s->mean_cycles = (float)s->total_cycles/s->samples;
    s->mean_us = s->mean_cycles/hz*1000000.0f;
    s->max_us = (float)s->max_cycles/hz*1000000.0f;
    s->cpu_percent = s->mean_us/JOINT_BENCH_PERIOD_US*100.0f;
    s->peak_cpu_percent = s->max_us/JOINT_BENCH_PERIOD_US*100.0f;
    return 1;
}
