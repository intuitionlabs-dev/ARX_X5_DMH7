#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../App/test/joint_benchmark.c"

rc_t rc;
uint8_t is_Offline(monitor_item* item) { return item->count <= 0; }
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
Plan_group_temp Plan_Group_Temp_Create(uint8_t count) { return calloc(count, sizeof(Move_plan*)); }

static void statistics_checks(void) {
    JointBenchmarkStats stats = {0};
    assert(!JointBenchmark_Summarize(&stats, 400000000));
    JointBenchmark_AddSample(&stats, UINT32_MAX-99, 100, 400000000);
    assert(stats.total_cycles == 200 && stats.min_cycles == 200);
    for (unsigned i=0; i<8; ++i)
        JointBenchmark_AddSample(&stats, 0, 0x40000000u, 400000000);
    assert(stats.total_cycles == UINT64_C(8589934792));
    assert(stats.samples == 9 && stats.max_cycles == 0x40000000u);
    memset(&stats, 0, sizeof(stats));
    JointBenchmark_AddSample(&stats, 100, 400100, 400000000);
    JointBenchmark_AddSample(&stats, 100, 1200100, 400000000);
    JointBenchmark_AddSample(&stats, 100, 800100, 400000000); /* Exactly 2 ms. */
    assert(JointBenchmark_Summarize(&stats, 400000000));
    assert(fabsf(stats.mean_us-2000) < .001f && fabsf(stats.max_us-3000) < .001f);
    assert(fabsf(stats.cpu_percent-100) < .0001f && fabsf(stats.peak_cpu_percent-150) < .0001f);
    assert(stats.over_budget == 1);
    assert(!JointBenchmark_Summarize(&stats, 0));
}

static void workload_checks(void) {
    monitor_item original = {.count=5};
    rc.monitor = &original;
    assert(!JointBenchmark_Init(0, JOINT_BENCH_RUCKIG));
    assert(!JointBenchmark_Init(ARM_NUM+1, JOINT_BENCH_RUCKIG));
    assert(!JointBenchmark_Init(1, JOINT_BENCH_ALGORITHMS));
    assert(rc.monitor == &original);
    for (unsigned arms=1; arms<=ARM_NUM; ++arms) {
        for (unsigned scenario=0; scenario<JOINT_BENCH_SCENARIOS; ++scenario) {
            float checksums[JOINT_BENCH_ALGORITHMS] = {0};
            uint32_t expected_input_hash = 0;
            for (unsigned repeat=0; repeat<2; ++repeat) {
                for (unsigned algorithm=0; algorithm<JOINT_BENCH_ALGORITHMS; ++algorithm) {
                    assert(JointBenchmark_Init(arms, algorithm));
                    assert(rc.monitor != &original && is_Offline(rc.monitor));
                    uint32_t input_hash = 2166136261u;
                    for (unsigned tick=0; tick<11000; ++tick) {
                        JointBenchmark_Target(scenario, tick);
                        for (unsigned a=0; a<arms; ++a)
                            for (unsigned j=0; j<ARM_JOINT_NUM; ++j) {
                                uint32_t bits;
                                memcpy(&bits, &targets[a][j], sizeof(bits));
                                input_hash = (input_hash ^ bits)*16777619u;
                            }
                        JointBenchmark_Step();
                        assert(JointBenchmark_Check() == 0);
                    }
                    float checksum = JointBenchmark_Checksum();
                    if (!repeat && !algorithm) expected_input_hash = input_hash;
                    else assert(input_hash == expected_input_hash);
                    assert(isfinite(checksum));
                    if (!repeat) checksums[algorithm] = checksum;
                    else assert(checksums[algorithm] == checksum);
                    if (scenario == JOINT_BENCH_SETTLED) assert(checksum == 0);
                    /* A planner failure must invalidate the timing result. */
                    fixtures[0].cmd->mode = arm_damping;
                    assert(JointBenchmark_Check() < 0);
                    JointBenchmark_Destroy();
                    JointBenchmark_Destroy();
                    assert(rc.monitor == &original);
                }
            }
        }
    }
}

int main(void) {
    statistics_checks();
    workload_checks();
    puts("H7 workload, repeatability, failure detection, wrap and CPU formulas passed");
}
