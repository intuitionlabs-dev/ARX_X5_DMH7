#pragma once
#include <stdint.h>

enum { JOINT_BENCH_QUINTIC, JOINT_BENCH_RUCKIG, JOINT_BENCH_ALGORITHMS };
enum { JOINT_BENCH_SETTLED, JOINT_BENCH_STEP, JOINT_BENCH_SMOOTH_100,
       JOINT_BENCH_SMOOTH_500, JOINT_BENCH_RANDOM, JOINT_BENCH_SCENARIOS };
enum { JOINT_BENCH_PERIOD_US = 2000 };

typedef struct {
    uint64_t total_cycles;
    uint32_t samples, min_cycles, max_cycles, over_budget;
    float mean_cycles, mean_us, max_us, cpu_percent, peak_cpu_percent;
} JointBenchmarkStats;

/* Private contexts, no shared robot/RC state. Allocate once before the cases;
 * Init resets the private state without allocating. */
int JointBenchmark_Create(void);
int JointBenchmark_Init(unsigned arms, unsigned algorithm);
void JointBenchmark_Destroy(void);
void JointBenchmark_Target(unsigned scenario, unsigned tick);
void JointBenchmark_Step(void);
int JointBenchmark_Check(void);
float JointBenchmark_Checksum(void);
void JointBenchmark_AddSample(JointBenchmarkStats* stats, uint32_t start,
                              uint32_t end, uint32_t core_hz);
int JointBenchmark_Summarize(JointBenchmarkStats* stats, uint32_t core_hz);
