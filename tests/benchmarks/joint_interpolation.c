/* Compile unchanged against either the rebuild or Ruckig command-layer source.
 * This program never opens hardware and measures reference generation only. */
#define _POSIX_C_SOURCE 200809L
#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include "gimbal_cmd.c"
#include "profile_constants.h"

rc_t rc;
uint8_t is_Offline(monitor_item* item) { (void)item; return 1; }
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
Plan_group_temp Plan_Group_Temp_Create(uint8_t count) {
    return calloc(count, sizeof(Move_plan*));
}

typedef struct {
    Arm arm;
    Gimbal gimbal;
    Arm_profile profile;
    Arm_joint_config joints[ARM_JOINT_NUM];
    Gimbal_cmd* cmd;
} Fixture;

static uint64_t now_ns(clockid_t clock) {
    struct timespec ts;
    if (clock_gettime(clock, &ts)) { perror("clock_gettime"); exit(2); }
    return (uint64_t)ts.tv_sec*1000000000u + (uint64_t)ts.tv_nsec;
}

static void init(Fixture* f) {
    f->arm.gimbal = &f->gimbal;
    f->gimbal.profile = &f->profile;
    f->profile.joints = f->joints;
#if BENCH_RUCKIG
    f->profile.position_min_duration_s = ARM_PROFILE_POSITION_MIN_DURATION_S;
#else
    f->profile.quintic_duration_s = ARM_PROFILE_QUINTIC_DURATION_S;
#endif
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        f->joints[i].td_r = 1000;
        f->joints[i].plan_velocity_scale = 1;
#if BENCH_RUCKIG
        f->joints[i].max_velocity_deg_s = ARM_PROFILE_MAX_VELOCITY_DEG_S;
        f->joints[i].max_acceleration_deg_s2 = ARM_PROFILE_MAX_ACCELERATION_DEG_S2;
        f->joints[i].max_jerk_deg_s3 = ARM_PROFILE_MAX_JERK_DEG_S3;
#endif
    }
    f->cmd = Gimbal_Cmd_Create(&f->arm);
    if (!f->cmd) { fputs("Fixture allocation failed\n", stderr); exit(2); }
    f->cmd->mode = arm_run;
}

static void verify(const Fixture* f) {
    if (f->cmd->mode != arm_run) {
        fputs("Planner entered failure fallback; timing is invalid\n", stderr);
        exit(3);
    }
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        const Arm_control_data* c = &f->gimbal.arm_control_data;
        if (!isfinite(c->joint_angle[i]) || !isfinite(c->joint_vel[i])
            || !isfinite(c->joint_acc[i])) {
            fputs("Nonfinite reference; timing is invalid\n", stderr); exit(3);
        }
    }
}

static void destroy(Fixture* f) {
#if BENCH_RUCKIG
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        ruckig_c_destroy(f->cmd->joint_ruckig_ref[i].planner);
#endif
    free(f->cmd->temp_group);
    free(f->cmd);
}

static uint32_t random_state = 0x12345678;
static float random_target(void) {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return 120.0f*(random_state / 4294967295.0f) - 60.0f;
}

static float target_at(unsigned scenario, unsigned tick, unsigned arm, unsigned axis) {
    if (scenario == 0) return 0; /* Settled target: no replanning. */
    if (scenario == 1) /* Alternating steps at 1 Hz, all six axes change. */
        return ((tick/500 + axis + arm)%2 ? 1 : -1)*(10.0f + 2.0f*axis);
    if (scenario == 4) return random_target(); /* Abrupt 500 Hz stress input. */
    const unsigned input_tick = scenario == 2 ? tick/5*5 : tick;
    const double time = input_tick*0.002;
    return (float)((10.0 + 2*axis)*sin(6.283185307179586*(.2 + .03*axis)*time + .4*arm));
}

int main(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "--info") == 0) {
#if BENCH_RUCKIG
        printf("{\"context_bytes\":%zu,\"scalar_bytes\":%zu}\n",
               ruckig_c_context_size(), sizeof(((ruckig_c_state*)0)->position));
#else
        puts("{}");
#endif
        return 0;
    }
    if (argc != 5) {
        fputs("usage: joint-benchmark ARMS SCENARIO CYCLES WARMUP\n", stderr); return 2;
    }
    const unsigned arms = (unsigned)strtoul(argv[1], NULL, 10);
    const unsigned scenario = (unsigned)strtoul(argv[2], NULL, 10);
    const unsigned cycles = (unsigned)strtoul(argv[3], NULL, 10);
    const unsigned warmup = (unsigned)strtoul(argv[4], NULL, 10);
    if (arms < 1 || arms > ARM_NUM || scenario > 4 || !cycles || cycles > 5000000
        || warmup < 500 || warmup > 1000000) return 2;
    Fixture fixtures[ARM_NUM] = {0};
    for (unsigned a=0; a<arms; ++a) init(&fixtures[a]);
    /* All trigonometry, random input, allocation and warmup are outside timing. */
    float (*targets)[ARM_NUM][ARM_JOINT_NUM] = calloc(cycles + warmup, sizeof(*targets));
    if (!targets) return 2;
    for (unsigned k=0; k<cycles+warmup; ++k)
        for (unsigned a=0; a<arms; ++a)
            for (unsigned j=0; j<ARM_JOINT_NUM; ++j)
                targets[k][a][j] = target_at(scenario, k, a, j);
    for (unsigned k=0; k<warmup; ++k)
        for (unsigned a=0; a<arms; ++a) {
            memcpy(fixtures[a].cmd->ctrl_src.joint_angle_solved, targets[k][a],
                   sizeof(targets[k][a]));
            Arm_Ref_Set(fixtures[a].cmd);
        }
    for (unsigned a=0; a<arms; ++a) verify(&fixtures[a]);
    const uint64_t wall_start = now_ns(CLOCK_MONOTONIC_RAW);
    const uint64_t cpu_start = now_ns(CLOCK_THREAD_CPUTIME_ID);
    for (unsigned k=warmup; k<cycles+warmup; ++k)
        for (unsigned a=0; a<arms; ++a) {
            memcpy(fixtures[a].cmd->ctrl_src.joint_angle_solved, targets[k][a],
                   sizeof(targets[k][a]));
            Arm_Ref_Set(fixtures[a].cmd);
        }
    const uint64_t cpu_ns = now_ns(CLOCK_THREAD_CPUTIME_ID) - cpu_start;
    const uint64_t wall_ns = now_ns(CLOCK_MONOTONIC_RAW) - wall_start;
    double checksum = 0;
    for (unsigned a=0; a<arms; ++a) {
        verify(&fixtures[a]);
        for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
            checksum += fixtures[a].gimbal.arm_control_data.joint_angle[j];
            checksum += fixtures[a].gimbal.arm_control_data.joint_vel[j];
            checksum += fixtures[a].gimbal.arm_control_data.joint_acc[j];
        }
        destroy(&fixtures[a]);
    }
    free(targets);
    printf("%u,%u,%u,%" PRIu64 ",%" PRIu64 ",%.17g\n",
           arms, scenario, cycles, cpu_ns, wall_ns, checksum);
    return 0;
}
