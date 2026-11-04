#pragma once

/* CAN1 到 CAN2 的回环测试开关：1 启用，0 停用。 */
#define TEST_CAN_LOOPBACK_ENABLE 0

/* USART2 与 USART3 的双向回环测试开关。 */
#define TEST_UART_LOOPBACK_ENABLE 0

/* USART1 原样回显测试开关。 */
#define TEST_UART_ECHO_ENABLE 0

/* 独立低优先级关节插值测试线程；1 启用，0 停用。使用方法见 h7_joint_benchmark.h。
 * 直接按原项目构建，不添加 CMake 选项，不替换正常控制任务。 */
#ifndef TEST_JOINT_BENCHMARK_ENABLE
#define TEST_JOINT_BENCHMARK_ENABLE 0
#endif
#ifndef TEST_JOINT_BENCHMARK_SAMPLES
#define TEST_JOINT_BENCHMARK_SAMPLES 10000
#endif
#ifndef TEST_JOINT_BENCHMARK_WARMUP
#define TEST_JOINT_BENCHMARK_WARMUP 1000
#endif
#ifndef TEST_JOINT_BENCHMARK_ROUNDS
#define TEST_JOINT_BENCHMARK_ROUNDS 3
#endif
