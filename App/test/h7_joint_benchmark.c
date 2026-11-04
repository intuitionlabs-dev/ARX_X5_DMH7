#include "h7_joint_benchmark.h"
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

#ifndef H7_JOINT_BENCHMARK_SAMPLES
#define H7_JOINT_BENCHMARK_SAMPLES 10000
#endif
#ifndef H7_JOINT_BENCHMARK_WARMUP
#define H7_JOINT_BENCHMARK_WARMUP 1000
#endif
#ifndef H7_JOINT_BENCHMARK_MASK_IRQ
#define H7_JOINT_BENCHMARK_MASK_IRQ 1
#endif

volatile H7JointBenchmarkReport h7_joint_benchmark;
static uint64_t benchmark_stack[2048]; /* 16 KiB, 8-byte aligned. */
static StaticTask_t benchmark_task;
static const osThreadAttr_t task_attributes = {
    .name = "JointBenchmark",
    .cb_mem = &benchmark_task, .cb_size = sizeof(benchmark_task),
    .stack_mem = benchmark_stack, .stack_size = sizeof(benchmark_stack),
    .priority = osPriorityNormal,
};

static void serial_write(const char* text) {
    /* Blocking UART is used only outside measured intervals; no printf retarget. */
    if (HAL_UART_Transmit(&huart1, (uint8_t*)text, (uint16_t)strlen(text), 1000) != HAL_OK)
        ++h7_joint_benchmark.uart_errors;
}

static uint32_t counter_read(void) {
    __DSB();
    __ISB();
    return DWT->CYCCNT;
}

static int counter_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u;
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) return 0;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before = counter_read();
    for (unsigned i=0; i<64; ++i) __NOP();
    return counter_read() != before;
}

static uint32_t measure(int execute) {
    uint32_t primask = __get_PRIMASK();
#if H7_JOINT_BENCHMARK_MASK_IRQ
    __disable_irq();
#endif
    const uint32_t start = counter_read();
    if (execute) JointBenchmark_Step();
    const uint32_t elapsed = counter_read()-start;
    __set_PRIMASK(primask);
    return elapsed;
}

static void print_result(const H7JointBenchmarkResult* r) {
    static const char* const scenarios[] = {"settled", "step_1hz", "smooth_100hz", "smooth_500hz", "random_500hz"};
    const JointBenchmarkStats* s = &r->stats;
    /* Fixed-point text avoids nano printf float/long-long support. A sample
     * over one second is rejected before accumulation, so these casts fit. */
    const uint32_t mean_ns = (uint32_t)(s->mean_us*1000.0f + .5f);
    const uint32_t peak_ns = (uint32_t)(s->max_us*1000.0f + .5f);
    const uint32_t cpu_1e4 = (uint32_t)(s->cpu_percent*10000.0f + .5f);
    uint32_t checksum_bits;
    memcpy(&checksum_bits, &r->checksum, sizeof(checksum_bits));
    char line[384];
    snprintf(line, sizeof(line),
        "case,%lu,%s,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu.%03lu,%lu.%04lu,%lu.%03lu,%lu,%ld,%lu,%08lx\r\n",
        (unsigned long)r->arms, scenarios[r->scenario],
        r->algorithm == JOINT_BENCH_RUCKIG ? "ruckig_float" : "quintic",
        (unsigned long)r->round, (unsigned long)s->samples,
        (unsigned long)(s->total_cycles >> 32), (unsigned long)(uint32_t)s->total_cycles,
        (unsigned long)s->min_cycles, (unsigned long)s->max_cycles,
        (unsigned long)(mean_ns/1000), (unsigned long)(mean_ns%1000),
        (unsigned long)(cpu_1e4/10000), (unsigned long)(cpu_1e4%10000),
        (unsigned long)(peak_ns/1000), (unsigned long)(peak_ns%1000),
        (unsigned long)s->over_budget, (long)r->error, (unsigned long)r->error_tick,
        (unsigned long)checksum_bits);
    serial_write(line);
}

static void run_case(H7JointBenchmarkResult* r) {
    r->error_tick = UINT32_MAX;
    if (!JointBenchmark_Init(r->arms, r->algorithm)) { r->error = -20; return; }
    for (unsigned tick=0; tick<H7_JOINT_BENCHMARK_WARMUP+H7_JOINT_BENCHMARK_SAMPLES; ++tick) {
        h7_joint_benchmark.active_tick = tick;
        JointBenchmark_Target(r->scenario, tick);
        uint32_t elapsed = 0;
        if (tick < H7_JOINT_BENCHMARK_WARMUP) JointBenchmark_Step();
        else elapsed = measure(1);
        r->error = JointBenchmark_Check();
        if (!r->error && tick >= H7_JOINT_BENCHMARK_WARMUP
            && (!elapsed || elapsed > h7_joint_benchmark.core_hz)) r->error = -21;
        if (r->error) { r->error_tick = tick; break; }
        if (tick >= H7_JOINT_BENCHMARK_WARMUP)
            JointBenchmark_AddSample(&r->stats, 0, elapsed, h7_joint_benchmark.core_hz);
        /* Run synthetic time as fast as possible; yield outside measurement. */
        if (tick%128 == 127) osDelay(1);
    }
    r->checksum = JointBenchmark_Checksum();
    JointBenchmark_Destroy();
    JointBenchmark_Summarize(&r->stats, h7_joint_benchmark.core_hz);
}

static void benchmark_run(void* argument) {
    (void)argument;
    SystemCoreClockUpdate();
    h7_joint_benchmark.core_hz = SystemCoreClock;
    h7_joint_benchmark.period_us = JOINT_BENCH_PERIOD_US;
    h7_joint_benchmark.irq_masked = H7_JOINT_BENCHMARK_MASK_IRQ;
    h7_joint_benchmark.icache_enabled = !!(SCB->CCR & SCB_CCR_IC_Msk);
    h7_joint_benchmark.dcache_enabled = !!(SCB->CCR & SCB_CCR_DC_Msk);
    h7_joint_benchmark.samples_per_case = H7_JOINT_BENCHMARK_SAMPLES;
    h7_joint_benchmark.warmup = H7_JOINT_BENCHMARK_WARMUP;
    h7_joint_benchmark.rounds = H7_JOINT_BENCHMARK_ROUNDS;
    if (!SystemCoreClock) h7_joint_benchmark.state = -10;
    else if (!counter_init()) h7_joint_benchmark.state = -11;
    else h7_joint_benchmark.state = 1;
    if (h7_joint_benchmark.state != 1) {
        serial_write("error,clock_or_dwt_unavailable\r\n");
        osThreadExit();
    }
    uint32_t overhead = UINT32_MAX;
    for (unsigned i=0; i<64; ++i) {
        uint32_t elapsed = measure(0);
        if (elapsed < overhead) overhead = elapsed;
    }
    h7_joint_benchmark.counter_overhead_cycles = overhead;
    char line[256];
    snprintf(line, sizeof(line),
        "meta,core_hz,%lu,period_us,%lu,irq_masked,%lu,icache,%lu,dcache,%lu,samples,%lu,warmup,%lu,rounds,%lu,overhead_cycles,%lu\r\n",
        (unsigned long)SystemCoreClock, (unsigned long)JOINT_BENCH_PERIOD_US,
        (unsigned long)h7_joint_benchmark.irq_masked,
        (unsigned long)h7_joint_benchmark.icache_enabled, (unsigned long)h7_joint_benchmark.dcache_enabled,
        (unsigned long)H7_JOINT_BENCHMARK_SAMPLES, (unsigned long)H7_JOINT_BENCHMARK_WARMUP,
        (unsigned long)H7_JOINT_BENCHMARK_ROUNDS, (unsigned long)overhead);
    serial_write(line);
    serial_write("type,arms,scenario,algorithm,round,samples,total_hi,total_lo,min_cycles,max_cycles,mean_us,cpu_percent,max_us,over_budget,error,error_tick,checksum_bits\r\n");
    for (unsigned arms=1; arms<=ARM_NUM; ++arms)
        for (unsigned scenario=0; scenario<JOINT_BENCH_SCENARIOS; ++scenario)
            for (unsigned round=0; round<H7_JOINT_BENCHMARK_ROUNDS; ++round)
                for (unsigned order=0; order<JOINT_BENCH_ALGORITHMS; ++order) {
                    H7JointBenchmarkResult result = {
                        .arms = arms, .scenario = scenario, .round = round+1,
                        .algorithm = (order+round)%JOINT_BENCH_ALGORITHMS,
                    };
                    unsigned index = h7_joint_benchmark.completed;
                    h7_joint_benchmark.active_result = index;
                    run_case(&result);
                    h7_joint_benchmark.results[index] = result;
                    if (result.error) ++h7_joint_benchmark.failed_cases;
                    h7_joint_benchmark.stack_free_bytes = osThreadGetStackSpace(osThreadGetId());
                    __DMB();
                    h7_joint_benchmark.completed = index+1;
                    print_result(&result);
                }
    snprintf(line, sizeof(line), "done,cases,%lu,failed,%lu,uart_errors,%lu,stack_free_bytes,%lu\r\n",
        (unsigned long)h7_joint_benchmark.completed, (unsigned long)h7_joint_benchmark.failed_cases,
        (unsigned long)h7_joint_benchmark.uart_errors, (unsigned long)h7_joint_benchmark.stack_free_bytes);
    serial_write(line);
    h7_joint_benchmark.state = 2;
    osThreadExit();
}

void H7_Joint_Benchmark_Start(void) {
    /* Normal BSP, CAN/motor drivers, protocol and control tasks are not started. */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    if (osKernelInitialize() != osOK) h7_joint_benchmark.state = -13;
    else if (!osThreadNew(benchmark_run, NULL, &task_attributes)) h7_joint_benchmark.state = -12;
    else {
        osKernelStart();
        h7_joint_benchmark.state = -13; /* Scheduler must not return. */
    }
    serial_write("error,benchmark_task_or_scheduler_start\r\n");
    for (;;) __WFI();
}
