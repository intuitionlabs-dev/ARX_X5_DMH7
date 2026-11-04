#pragma once
#include "joint_benchmark.h"
#include "robot_def.h"

#ifndef H7_JOINT_BENCHMARK_ROUNDS
#define H7_JOINT_BENCHMARK_ROUNDS 3
#endif
#define H7_JOINT_BENCHMARK_RESULTS (ARM_NUM*JOINT_BENCH_SCENARIOS*JOINT_BENCH_ALGORITHMS*H7_JOINT_BENCHMARK_ROUNDS)

typedef struct {
    uint32_t arms, scenario, algorithm, round;
    int32_t error;
    uint32_t error_tick;
    float checksum;
    JointBenchmarkStats stats;
} H7JointBenchmarkResult;

typedef struct {
    /* 0=not started, 1=running, 2=done; negative=startup failure. */
    int32_t state;
    uint32_t core_hz, period_us, irq_masked, icache_enabled, dcache_enabled;
    uint32_t samples_per_case, warmup, rounds, counter_overhead_cycles;
    uint32_t completed, failed_cases, active_result, active_tick;
    uint32_t uart_errors, stack_free_bytes;
    H7JointBenchmarkResult results[H7_JOINT_BENCHMARK_RESULTS];
} H7JointBenchmarkReport;

extern volatile H7JointBenchmarkReport h7_joint_benchmark;
/* Called immediately after SystemClock_Config; starts only the benchmark task. */
__attribute__((noreturn)) void H7_Joint_Benchmark_Start(void);
