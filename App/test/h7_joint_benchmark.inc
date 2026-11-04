/* Included only by the optional thread in Core/Src/freertos.c. */
#include "h7_joint_benchmark.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

#define JOINT_BENCHMARK_STORAGE __attribute__((section(".RAM_D1"), aligned(8)))
#include "joint_benchmark.inc"
#undef JOINT_BENCHMARK_STORAGE

_Static_assert(TEST_JOINT_BENCHMARK_SAMPLES > 0 && TEST_JOINT_BENCHMARK_SAMPLES <= 1000000, "invalid sample count");
_Static_assert(TEST_JOINT_BENCHMARK_WARMUP > 0 && TEST_JOINT_BENCHMARK_WARMUP <= 1000000, "invalid warmup");
_Static_assert(TEST_JOINT_BENCHMARK_ROUNDS > 0 && TEST_JOINT_BENCHMARK_ROUNDS <= 10, "invalid rounds");

volatile H7JointBenchmarkReport h7_joint_benchmark __attribute__((section(".RAM_D1"), aligned(8)));

static uint32_t joint_benchmark_counter(void) {
    __DSB();
    __ISB();
    return DWT->CYCCNT;
}

static int joint_benchmark_counter_init(void) {
    /* CAN also uses CYCCNT. Never reset or stop the shared counter. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) return 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = joint_benchmark_counter();
    for (unsigned i=0; i<64; ++i) __NOP();
    return joint_benchmark_counter() != before;
}

static uint32_t joint_benchmark_measure(int execute) {
    /* Do not mask interrupts or suspend scheduling around the measurement. */
    const uint32_t start = joint_benchmark_counter();
    if (execute) JointBenchmark_Step();
    return joint_benchmark_counter()-start;
}

static void joint_benchmark_case(H7JointBenchmarkResult* r) {
    r->error_tick = UINT32_MAX;
    if (!JointBenchmark_Init(r->arms, r->algorithm)) { r->error = -20; return; }
    for (unsigned tick=0; tick<TEST_JOINT_BENCHMARK_WARMUP+TEST_JOINT_BENCHMARK_SAMPLES; ++tick) {
        h7_joint_benchmark.active_tick = tick;
        JointBenchmark_Target(r->scenario, tick);
        uint32_t elapsed = 0;
        if (tick < TEST_JOINT_BENCHMARK_WARMUP) JointBenchmark_Step();
        else elapsed = joint_benchmark_measure(1);
        r->error = JointBenchmark_Check();
        if (!r->error && tick >= TEST_JOINT_BENCHMARK_WARMUP
            && (!elapsed || elapsed > h7_joint_benchmark.core_hz)) r->error = -21;
        if (r->error) { r->error_tick = tick; break; }
        if (tick >= TEST_JOINT_BENCHMARK_WARMUP)
            JointBenchmark_AddSample(&r->stats, 0, elapsed, h7_joint_benchmark.core_hz);
        if (tick%16 == 15) osDelay(1);
    }
    r->checksum = JointBenchmark_Checksum();
    JointBenchmark_Summarize(&r->stats, h7_joint_benchmark.core_hz);
}

void H7_Joint_Benchmark_Run(void) {
    /* Report and fixture storage use NOLOAD RAM_D1; initialize explicitly. */
    memset((void*)&h7_joint_benchmark, 0, sizeof(h7_joint_benchmark));
    h7_joint_benchmark.core_hz = SystemCoreClock;
    h7_joint_benchmark.period_us = JOINT_BENCH_PERIOD_US;
    h7_joint_benchmark.includes_preemption = 1;
    h7_joint_benchmark.icache_enabled = !!(SCB->CCR & SCB_CCR_IC_Msk);
    h7_joint_benchmark.dcache_enabled = !!(SCB->CCR & SCB_CCR_DC_Msk);
    h7_joint_benchmark.samples_per_case = TEST_JOINT_BENCHMARK_SAMPLES;
    h7_joint_benchmark.warmup = TEST_JOINT_BENCHMARK_WARMUP;
    h7_joint_benchmark.rounds = TEST_JOINT_BENCHMARK_ROUNDS;
    if (!SystemCoreClock) { h7_joint_benchmark.state = -10; return; }
    if (!joint_benchmark_counter_init()) { h7_joint_benchmark.state = -11; return; }

    /* Newlib malloc is shared with control tasks. Serialize allocation/free
     * only, outside all measurements; interrupts stay enabled. Reuse contexts
     * across every case so the timed loop never allocates or takes this lock. */
    vTaskSuspendAll();
    int created = JointBenchmark_Create();
    xTaskResumeAll();
    if (!created) { h7_joint_benchmark.state = -20; return; }
    h7_joint_benchmark.state = 1;
    uint32_t overhead = UINT32_MAX;
    for (unsigned i=0; i<64; ++i) {
        uint32_t elapsed = joint_benchmark_measure(0);
        if (elapsed < overhead) overhead = elapsed;
    }
    h7_joint_benchmark.counter_overhead_cycles = overhead;
    for (unsigned arms=1; arms<=ARM_NUM; ++arms)
        for (unsigned scenario=0; scenario<JOINT_BENCH_SCENARIOS; ++scenario)
            for (unsigned round=0; round<TEST_JOINT_BENCHMARK_ROUNDS; ++round)
                for (unsigned order=0; order<JOINT_BENCH_ALGORITHMS; ++order) {
                    H7JointBenchmarkResult result = {
                        .arms = arms, .scenario = scenario, .round = round+1,
                        .algorithm = (order+round)%JOINT_BENCH_ALGORITHMS,
                    };
                    unsigned index = h7_joint_benchmark.completed;
                    h7_joint_benchmark.active_result = index;
                    joint_benchmark_case(&result);
                    h7_joint_benchmark.results[index] = result;
                    if (result.error) ++h7_joint_benchmark.failed_cases;
                    h7_joint_benchmark.stack_free_bytes = osThreadGetStackSpace(osThreadGetId());
                    __DMB();
                    h7_joint_benchmark.completed = index+1;
                }
    vTaskSuspendAll();
    JointBenchmark_Destroy();
    xTaskResumeAll();
    h7_joint_benchmark.state = 2;
}
