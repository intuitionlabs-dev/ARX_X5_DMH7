#pragma once

/* ===================== 测试例程开关 =====================
 *
 * 上机验证用的例程都放在 App/test/ 下，每个例程一个开关宏：
 *
 *   1 = 正常编译，并在 main.c / freertos.c 里挂上运行
 *   0 = 该例程的 .c 整份编成空文件(不占 Flash)，它申请的任务、静态栈和
 *       全局变量也都不会生成(不占 RAM)；头文件里的接口退化成空函数，
 *       所以 main.c / freertos.c 的调用点不用加 #if
 *
 * 所以关掉一个测试 = 把对应宏改成 0 重新编译，别的文件一行都不用动。
 *
 * 两点注意：
 *  1. 例程的全局观测变量(can_lb_* 之类)只在该测试打开时才存在，
 *     关掉后用调试器读不到，读之前先回来确认这里开着。
 *  2. 关掉宏以后，main.c 里那句 Init() 依然会编译通过，只是什么都不做，
 *     别把它当成"还在跑"。
 * ======================================================= */

/* CAN1 -> CAN2 自机通信(回环)测试，见 App/test/can_loopback_test.c */
#define TEST_CAN_LOOPBACK_ENABLE 0

/* USART2 <-> USART3 自机通信(回环)测试，见 App/test/uart_loopback_test.c
 * 跑之前要先把两路的 RS485 挂到同一条总线上(A2↔A3、B2↔B3)，接线表在头文件里。 */
#define TEST_UART_LOOPBACK_ENABLE 0

/* USART1 <-> PC 连通性(回显)测试，见 App/test/uart_echo_test.c
 * 单片机在 USART1 上把 PC 发来的字节原样发回去，一个回环同时验收发两个方向。
 * 跑之前要把 PC 的 USB 转 TTL 接到 USART1(下面是适配器侧看的方向)：
 *   适配器 TX -> 单片机 PA10(USART1_RX)，适配器 RX -> 单片机 PA9(USART1_TX)，
 *   GND <-> GND。端对端(不是同名对接)。
 * 用它验证过：921600 8N1，4 组不同长度的包 4/4 原样回显。 */
#define TEST_UART_ECHO_ENABLE 0

/* 以后新增测试，照抄下面这两行(宏名和文件名自己改)：
 * #define TEST_XXX_ENABLE 1
 */
