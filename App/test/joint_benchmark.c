#include "joint_benchmark.h"
#include "gimbal_cmd.h"
#include "arm.h"
#include "robot_state.h"
#include "rc.h"
#include "profile_constants.h"
#include "quintic_spline.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GIMBAL_CMD_PERIOD_S ARM_CONTROL_PERIOD_S
#include "joint_quintic_baseline.inc"
#undef GIMBAL_CMD_PERIOD_S

typedef struct {
    Arm arm;
    Gimbal gimbal;
    Arm_profile profile;
    Arm_joint_config joints[ARM_JOINT_NUM];
    Gimbal_cmd* cmd;
    Joint_quintic_ref quintic[ARM_JOINT_NUM];
} Fixture;

static Fixture fixtures[ARM_NUM];
static float targets[ARM_NUM][ARM_JOINT_NUM];
static monitor_item offline_monitor;
static monitor_item* saved_monitor;
static unsigned active_arms, active_algorithm;
static uint32_t random_state;
static int owns_monitor;

void JointBenchmark_Destroy(void) {
    for (unsigned a=0; a<ARM_NUM; ++a) {
        Gimbal_cmd* cmd = fixtures[a].cmd;
        if (!cmd) continue;
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j)
            ruckig_c_destroy(cmd->joint_ruckig_ref[j].planner);
        free(cmd->temp_group);
        free(cmd);
        fixtures[a].cmd = NULL;
    }
    if (owns_monitor) rc.monitor = saved_monitor;
    owns_monitor = 0;
    active_arms = 0;
}

int JointBenchmark_Init(unsigned arms, unsigned algorithm) {
    JointBenchmark_Destroy();
    if (arms < 1 || arms > ARM_NUM || algorithm >= JOINT_BENCH_ALGORITHMS) return 0;
    memset(fixtures, 0, sizeof(fixtures));
    memset(targets, 0, sizeof(targets));
    saved_monitor = rc.monitor;
    rc.monitor = &offline_monitor;
    owns_monitor = 1;
    random_state = 0x12345678;
    active_algorithm = algorithm;
    for (unsigned a=0; a<arms; ++a) {
        Fixture* f = &fixtures[a];
        f->arm.gimbal = &f->gimbal;
        f->arm.ctrl.valid = 1; /* Ordinary position command, has_vel remains 0. */
        f->gimbal.profile = &f->profile;
        f->profile.joints = f->joints;
        f->profile.position_min_duration_s = ARM_PROFILE_POSITION_MIN_DURATION_S;
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            f->joints[j].td_r = 1000;
            f->joints[j].plan_velocity_scale = 1;
            f->joints[j].max_velocity_deg_s = ARM_PROFILE_MAX_VELOCITY_DEG_S;
            f->joints[j].max_acceleration_deg_s2 = ARM_PROFILE_MAX_ACCELERATION_DEG_S2;
            f->joints[j].max_jerk_deg_s3 = ARM_PROFILE_MAX_JERK_DEG_S3;
        }
        f->cmd = Gimbal_Cmd_Create(&f->arm);
        if (!f->cmd) { JointBenchmark_Destroy(); return 0; }
        f->cmd->mode = arm_run;
    }
    active_arms = arms;
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

/* Timed scope: target copy, reference selection, interpolation, publication. */
void JointBenchmark_Step(void) {
    for (unsigned a=0; a<active_arms; ++a) {
        memcpy(fixtures[a].cmd->ctrl_src.joint_angle_solved, targets[a], sizeof(targets[a]));
        if (active_algorithm == JOINT_BENCH_RUCKIG)
            Gimbal_Cmd_Benchmark_Reference(fixtures[a].cmd);
        else
            Quintic_Reference(fixtures[a].cmd, fixtures[a].quintic);
    }
}

int JointBenchmark_Check(void) {
    for (unsigned a=0; a<active_arms; ++a) {
        const Fixture* f = &fixtures[a];
        if (f->cmd->mode != arm_run) {
            for (unsigned j=0; j<ARM_JOINT_NUM; ++j)
                if (active_algorithm == JOINT_BENCH_RUCKIG && f->cmd->joint_ruckig_ref[j].status < 0)
                    return f->cmd->joint_ruckig_ref[j].status;
            return -1;
        }
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            const Arm_control_data* c = &f->gimbal.arm_control_data;
            if (!isfinite(c->joint_angle[j]) || !isfinite(c->joint_vel[j]) || !isfinite(c->joint_acc[j]))
                return -1;
        }
    }
    return active_arms ? 0 : -1;
}

float JointBenchmark_Checksum(void) {
    float sum = 0;
    for (unsigned a=0; a<active_arms; ++a)
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            const Arm_control_data* c = &fixtures[a].gimbal.arm_control_data;
            sum += c->joint_angle[j] + c->joint_vel[j] + c->joint_acc[j];
        }
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
