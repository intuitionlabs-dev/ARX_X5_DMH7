#pragma once
#include "test_config.h"
#include "joint_benchmark.h"
#include "robot_def.h"

/* 使用方法：在 test_config.h 将 TEST_JOINT_BENCHMARK_ENABLE 设为 1，
 * 按原项目方式构建、下载；无需 CMake 选项。正常启动和控制任务照常运行。
 * freertos.c 创建低优先级测试线程，等待 2 秒后执行，结束后退出该线程。
 * 不占用串口；调试器查看 h7_joint_benchmark，state=2 且 failed_cases=0 为完成。
 * Release 如需结构类型可用现有 RelWithDebInfo 或给调试器加载带符号的构建。
 * 测量时让 MCU 连续运行，完成后再暂停查看。
 *
 * 测量范围是独立关节插值（Ruckig C 接口 / 原五次曲线），不含 Arm_Ref_Set、
 * 动力学、通信及真实控制对象。计数保留中断和其他任务抢占，因此 cpu_percent
 * 是“含抢占的平均经过时间 / 2 ms * 100”的预算占比，并非精确线程 CPU 使用率。
 * 不能直接与此前主机 Arm_Ref_Set 基准数值比较；counter_overhead_cycles 不扣除。
 * 启用测试会增加系统负载；线程不改动控制源、关节目标或电机状态。
 *
 * 默认 1/2 臂、5 场景、2 算法、3 轮，共 60 条 results。场景枚举见 joint_benchmark.h。
 * stats.total_cycles 是 64 位累计；mean_us/max_us 为微秒；over_budget 计数超过 2 ms。
 * mean_us = total_cycles / samples / core_hz * 1e6；cpu_percent = mean_us / 2000 * 100。
 * state: -10 时钟无效，-11 DWT 不可用，-12 创建线程失败，-20 测试上下文分配失败。
 * 每条 error: 0 成功，-21 计数为零或超过一秒，其余为求解/校验错误；失败条目无效。
 * error_tick 包含预热，成功为 UINT32_MAX。峰值含抢占，不是算法最坏执行时间保证。
 */
#define H7_JOINT_BENCHMARK_RESULTS (ARM_NUM*JOINT_BENCH_SCENARIOS*JOINT_BENCH_ALGORITHMS*TEST_JOINT_BENCHMARK_ROUNDS)

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
    uint32_t core_hz, period_us, includes_preemption, icache_enabled, dcache_enabled;
    uint32_t samples_per_case, warmup, rounds, counter_overhead_cycles;
    uint32_t completed, failed_cases, active_result, active_tick;
    uint32_t stack_free_bytes;
    H7JointBenchmarkResult results[H7_JOINT_BENCHMARK_RESULTS];
} H7JointBenchmarkReport;

extern volatile H7JointBenchmarkReport h7_joint_benchmark;
void H7_Joint_Benchmark_Run(void);
