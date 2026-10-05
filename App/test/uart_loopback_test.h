#pragma once

#include <stdint.h>

#include "test_config.h"

/* USART2 <-> USART3 自机通信(回环)测试
 *
 * 拓扑:本板 USART2 发、USART3 收，下一轮反过来。两路都是 **RS485 半双工**。
 *
 *   需要板外短接(把两路的 RS485 收发器挂到同一条总线上):
 *       USART2 的 A  <->  USART3 的 A
 *       USART2 的 B  <->  USART3 的 B
 *   （能用的话两头各加一个 120Ω 终端电阻；短线飞线一般不加也能跑。）
 *
 *   DE 不用管:CubeMX 生成的是 HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0)
 *   (见 Core/Src/usart.c)，PD4/PB14 两个 DE 由外设硬件在发送期间自动拉高、发完最后
 *   一个停止位自动放开，所以同一时刻只有一个在驱动总线。软件不用拨。
 *
 * 前提:两路波特率必须一致。当前 CubeMX 配置都是 921600 8N1(16 倍过采样)，
 * 满足该前提。
 *
 * 收发都走 DMA:CubeMX 里 USART2 = DMA1_Stream1(RX)/Stream2(TX)、
 * USART3 = DMA2_Stream0(RX)/Stream1(TX)，都是 NORMAL 模式、逐字节。接收用
 * HAL_UARTEx_ReceiveToIdle_DMA(靠总线空闲判一帧收完)，发送用 HAL_UART_Transmit_DMA。
 * 这不是为了快，是为了**不丢字节** —— 原因见下面"实测过的坑"。
 *
 * **但收发底座不在这个文件里了**:DMA 缓冲、武装/重装、出错恢复、.RAM_D1 段属性
 * 全部封在 libs/uart 的 BSP 层(bsp_uart.c)里，本测试只跟"字节进 / 字节出"打交道。
 * 所以**本测试的 .c/.h 里没有一处内存段属性**，两路的接收也是常驻武装的 ——
 * 那套容易错的东西只写一遍，别的串口用户不用再背一遍。见 bsp_uart.h 开头的说明。
 *
 * 「谁做帧」这条也在代码里划清楚了:帧协议(libs/uart_frame)和解析器属于**上层**
 * (就是本测试)，BSP 不认识帧 —— 它只管把收到的原始字节推给注册的回调，
 * 本测试当场喂进 UartFrameParserFeed()。
 *
 * 用法:
 *   BSP_UART_Init();           // 必须先调(main.c 的 USER CODE BEGIN 2)。串口的 DMA 收发、
 *                              // 缓冲、回调表都在 BSP 里，本测试只是它的一个用户。
 *   UartLoopbackTest_Init();   // 调度器启动前调用,在 MX_USART2_UART_Init() /
 *                              // MX_USART3_UART_Init() 之后、BSP_UART_Init() 之后
 *                              // (BSP 初始化会清空回调表,所以注册必须在它之后)
 *   循环体里: UartLoopbackTest_Loop();  tick += 1;  osDelayUntil(tick);
 *             (uint32_t tick = osKernelGetTickCount(); 在循环外初始化一次)
 *
 * 结果不走串口，全部记在下面的全局变量里，直接用调试器(JLink/gdb)读地址。
 * 接线正确时:uart_lb_ok_cnt 稳定增长，bad/lost/crc8/crc16/tx_fail 恒为 0，
 * dir_ok[0] 和 dir_ok[1] 同步增长(两个方向都通)，case_ok[] 每一项都非 0
 * (每个包类型都走通了)。另外 uart_lb_dma_mem_bad 必须是 0，
 * byte_dir[0] 和 byte_dir[1] 应该**完全相等**(每个用例被两个方向各收一次)。
 *
 * ---- 实测过的坑，先看这条再对着计数器猜 ----
 *
 * 【已解决，2026-10-05】第一次跑时 ok 只有 ~70%，lost 一直涨，而且**丢的全是帧尾**:
 * 780 条丢帧里 542 条解析器停在 ST_CRC16(帧身全收完、只差最后一两个字节)，
 * 516 条"只差 1 个字节"。帧头几乎从不丢。crc16_err 恒为 0。
 *
 * 根因不是接线、不是 RS485/DE、不是协议:是**接收中断被挡住了**。
 * USART2/3 的 NVIC 优先级是 5(见 usart.c 的 HAL_NVIC_SetPriority)，正好等于
 * FreeRTOSConfig.h 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY —— 也就是
 * 任何一处 taskENTER_CRITICAL()(电机任务 500 Hz、CAN 发送那一摊)都能把它挡住。
 * 而 921600 下**一个字节只有 1.086 µs**(480 MHz 下 521 个周期)。被挡超过一个
 * 字节时间，下一个字节就把只有 1 字节深的 RDR 冲掉 —— CubeMX 还关了两路的 RX
 * FIFO(usart.c 的 HAL_UARTEx_DisableFifoMode)—— 于是置 ORE；而 ORE 在 HAL 里是
 * **阻塞型**错误，HAL_UART_IRQHandler() 会直接 UART_EndRxTransfer() 把接收停掉，
 * 这一帧就从那个字节处截断。中断里重装也救不回来。
 * 所以现象是"帧头好好的、尾巴没了"，而且帧越长越容易中(每字节一次风险)。
 *
 * 当时先把这两路的中断优先级提到屏蔽线以上验证过 —— 1985 轮零丢帧、零 bad、零 ORE，
 * 根因确认。那个补丁**已经从 Init() 里拿掉了**:现在 CubeMX 把 USART2/3 改成 DMA 收发，
 * 字节是 DMA 搬进内存的，USART 中断只管 IDLE，晚几微秒不丢数据，提优先级没必要了
 * (反而会去抢 CAN/定时器的中断)。当前这一版就是 DMA 版。
 *
 * **这条对全工程都成立**:libs/uart_frame 接到别的串口上做真实收包时，只要那一路是
 * 逐字节中断收、而且波特率高，一样会中。三条路:改 NVIC 优先级(提到
 * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 以上)、打开 RX FIFO、或者改 DMA。
 * 别在协议层找原因 —— 协议层一点问题都没有。
 *
 * 【已解决，2026-10-05】改成 DMA 之后第一次跑，全军覆没:tx_cnt=2 / tx_fail=1997 /
 * rx_cnt=0 / lost=1998，两路的 uart_lb_hal_err[] 都是 **0x10 = HAL_UART_ERROR_DMA**，
 * 而 byte_dir 两路各是 **293706 = 294(缓冲区长度) × 999**。当时第一反应是"DMA 冲突"，
 * 换过流号(USART3 从 DMA1_Stream3/4 挪到 DMA2_Stream0/1)—— **没用**。
 *
 * 根因是 **DTCM 陷阱，和 DMA 流号一点关系都没有**:
 * STM32H7 的 DMA1/DMA2 在 D2 域，是总线主设备，**访问不到 DTCM/ITCM**
 * (TCM 只挂在 Cortex-M7 核上，不在 AXI/AHB 总线矩阵上)。而 STM32H723xG_flash.ld 里
 * .data/.tdata/.tbss/.bss/栈/堆 **全部** `>DTCMRAM`(0x20000000)，RAM_D1(0x24000000)
 * 一个字节都没用 —— 于是 s_rx_dma(0x2000d128) 和 s_tx_frame(0x2000d47c) 都落在 DTCM。
 * HAL_UART_Transmit_DMA() 会正常返回 HAL_OK(它只负责把流支起来)，但流一使能，第一次
 * 访存就报传输错 → HAL_UART_ERROR_DMA(0x10) → UART_DMAError() 收尾。发送侧如此；
 * 接收侧 DMA 写不进 DTCM，同样报 TE，HAL 的收尾路径会调一次 RxEvent 回调，于是
 * "收到 Size = 整个缓冲区长度"这种假事件每轮出现一次，把 byte_dir 刷成 294 × 轮数。
 *
 * 对策:在链接脚本里加一个 `.RAM_D1 (NOLOAD)` 段(>RAM_D1)，把两个 DMA 缓冲区用
 * `__attribute__((section(".RAM_D1")))` 挂进去。**任何走 DMA 的缓冲区都得这么放** ——
 * 这条对全工程成立(Core/Src/adc.c 的 hdma_adc1 也是 DMA1，哪天 ADC 真去 Start_DMA 了，
 * 它的缓冲区一样会中)。本工程没开 D-Cache，所以放 RAM_D1 不需要 cache 维护;哪天
 * main.c 里加了 SCB_EnableDCache()，这些缓冲区就得配 MPU 非缓存区或前后加 Clean/Invalidate。
 *
 * 【收尾，2026-10-05】后来这件事整个挪进了 libs/uart 的 BSP 层:**段属性只写一遍**，
 * 写在 bsp_uart.c 的收发缓冲上，上层(包括本文件)传进来的数组在哪都行 —— BSP 会拷进
 * 自己那块 .RAM_D1 缓冲再收发。所以本文件里现在一个 `section` 属性都没有。代价要记住:
 * **BSP 只保证了"它收发用的缓冲"可 DMA**，调用方自己拿数组去调 HAL 的 DMA API 照样中招。
 *
 * uart_lb_dma_mem_bad 是兜底:值来自 BSP_UART_LayoutOk() —— 链接脚本的 .RAM_D1 段要是被
 * CubeMX 重新生成时冲掉了，BSP 的缓冲会变孤儿段、静默回到 DTCM，这个变量就变 1 ——
 * 免得再从头查一遍。
 *
 * 【已解决，2026-10-05】上面那条修完，TX 通了(ok_cnt 稳定涨、bad/lost/hal_err 全 0)，
 * 但 offdir_cnt 也跟着涨到 ≈ 轮数:每轮都有一路"非接收方"喂进解析器一整帧。
 *
 * 现场(把那一刻的状态全存下来看的):RxEventType=**TC**、Size=**整个缓冲区长度**、
 * 而 huart->RxState 已经是 READY、CR3.DMAR=0、hdmarx->State 已经是 READY、
 * NDTR 还停在 294(一个字节都没搬过);进来的那一路是**上一轮的接收方**，
 * 此刻它刚在 Arm() 里被 HAL_UART_AbortReceive() 中止掉。
 * 也就是说:这是 HAL 在"刚被 abort 掉的流"上报的一次**假 RxEvent**，
 * 而缓冲区里还留着它上一轮收到的那帧，所以解析器能拿这些残渣又解出一整帧 ——
 * 无 CRC 错、无丢帧，只是把 byte_dir / rx_dir / offdir_cnt 全刷了一遍。
 *
 * 根因在我们的**闸门选错了**:回调里是用 s_rx_armed[] 判的，而那一刻这个标志还是 1
 * (中止之后才轮到清它)，所以拦不住。判据换成 s_rx_idx(本轮指定谁收)就与标志位时序
 * 无关了 —— RunOneRound() 里 s_rx_idx 在 Arm() 之前就翻好了，压在这个窗口里的事件
 * 一律当"上一轮的残渣"丢掉。
 *
 * 【收尾，2026-10-05】搬到 BSP 之后这个坑**从根上没有了**:假事件本来就是"换方向时
 * abort 掉另一路"这套动作引出来的，而 BSP 是**常驻武装 + 收完就重装**，收发之间从不
 * abort，也就没有"刚被中止的流"可以报残影。所以回调里那个闸门 s_rx_idx 语义变干净了:
 * 它现在只挡**发送方那一路自己听到的字节**(它自己的收发器 RE̅ 跟不跟 DE 走，取决于板子),
 * 这些字节 uart_lb_offdir_cnt 按**字节**累计(不再是"事件次数")。
 * 它 ≈ 轮数 × 帧长才是正常的;要盯的还是它涨而 ok_cnt 不涨。
 *
 * 教训:用 `ReceiveToIdle_DMA` 收包时，"这是不是我要的那一路"必须由**调用方自己的
 * 轮次状态**决定，不能靠 HAL 的句柄状态 —— HAL 的 RxEvent 回调会带上一次已经结束的
 * 收发的残影。BSP 那套"常驻武装、永不 abort"的写法是绕开它的根本办法
 * (libs/uart_frame 以后接真实收包，直接照 BSP 用就行)。
 *
 * 【还没验的】HAL_RS485Ex_Init(&huart2, ...) 最后两个参数是 DE 的**提前**和**延后**
 * 时间，CubeMX 生成的是 0 和 0，即 DE 在最后一个停止位结束的同一刻就放开。上面那次
 * 实测里帧头从不丢、hal_err 里也没有 FE(帧错)，所以 DE 时序**目前看是够的**;
 * 但真接上长线/带隔离的收发器如果出现"每帧就差最后那一两个字节 + crc16_err 涨"，
 * 还是要回到这里，去 CubeMX 的 RS485 配置里把 Deassertion Time 加到几个位时间。
 */

/* ---- 可调参数 ---------------------------------------------------------- */
#define UART_LB_U2 0u           /* USART2 在内部索引里的编号 */
#define UART_LB_U3 1u           /* USART3 在内部索引里的编号 */
#define UART_LB_PERIOD_MS 10u   /* 一轮的周期, ms */

/* 用例数:**每两轮**换一个(不是每轮),循环走。第 5 个是 255 字节的边界帧。
 * 为什么是两轮:方向每轮换一次,用例要是也每轮加一,方向就和用例锁死了 ——
 * 偶数轮永远 U2 收、奇数轮永远 U3 收,于是 U2 一辈子只收到偶数码的包,
 * 262 字节的边界帧永远轮不到 U2 收。两轮一换才能让每个用例被两个方向各收一次
 * (判据很好认:uart_lb_byte_dir[0] 和 [1] 应该**完全相等**)。 */
#define UART_LB_CASE_CNT 6u

#if TEST_UART_LOOPBACK_ENABLE

/* ---- 收发统计 ---------------------------------------------------------- */
extern volatile uint32_t uart_lb_tx_cnt;      /* 成功发出的帧数 */
extern volatile uint32_t uart_lb_tx_fail_cnt; /* 发送没发出去:上轮还没发完(gState 非 READY)或 HAL 返回非 OK */
extern volatile uint32_t uart_lb_rx_cnt;      /* 解析器解出完整合法帧的次数 */
extern volatile uint32_t uart_lb_ok_cnt;      /* 类型+长度+内容全对 */
extern volatile uint32_t uart_lb_bad_cnt;     /* 解出来了但对不上 */
extern volatile uint32_t uart_lb_lost_cnt;    /* 发出去了但没解出来 */
extern volatile uint32_t uart_lb_crc8_err;    /* 帧头 crc8 校验失败 */
extern volatile uint32_t uart_lb_crc16_err;   /* 整帧 crc16 校验失败(见上面 DE 时序那条) */
extern volatile uint32_t uart_lb_encode_err;  /* UartFrameEncode 返回负值,不该发生 */

/* 下面这三个是**搬运值**:BSP 的统计每轮被搬进这几个变量，好让 JLink 读的地址习惯不变
 * (真正的源头是 BSP_UART_GetStat(0/1)，也可以直接读那边)。 */
extern volatile uint32_t uart_lb_rearm_fail;  /* 两路合计的接收重装失败次数 */

/* 非 0 = BSP 的 DMA 缓冲区没落在 DMA 够得着的 RAM 里(链接脚本的 .RAM_D1 段丢了)。
 * 这个值非 0 时 tx_fail 满天飞、hal_err 出现 0x10 都是必然的，**别去查接线和 DMA 流号**,
 * 先把 .ld 里的 .RAM_D1 段补回来。见下面"DTCM 陷阱"那条。 */
extern volatile uint32_t uart_lb_dma_mem_bad;

/* ---- 分类统计 ---------------------------------------------------------- */
extern volatile uint32_t uart_lb_case_ok[UART_LB_CASE_CNT]; /* 每个用例各自的成功数 */
extern volatile uint32_t uart_lb_dir_ok[2];                 /* 每个接收方向各自的成功数 */
extern volatile uint32_t uart_lb_hal_err[2];  /* 两路 UART 见过的 HAL 错误码(按位 OR 累积) */

/* ---- 按路分开的诊断量(索引同 UART_LB_U2/U3) --------------------------- */
extern volatile uint32_t uart_lb_byte_dir[2]; /* 每一路各收到多少字节：某一路恒 0 就是硬/配置问题 */
extern volatile uint32_t uart_lb_rx_dir[2];   /* 每一路各自解出完整合法帧的次数 */
extern volatile uint32_t uart_lb_ok_dir[2];   /* 每一路各自"解出且内容全对"的次数 */
extern volatile uint32_t uart_lb_ore_cnt[2];  /* 每一路各自的 ORE 次数(不是 OR 累积，是计数) */

/* 非本轮接收方那一路收到的**字节数** —— 全部被丢弃，不计进 byte_dir、不喂解析器。
 *
 * 两路现在是常驻武装的(见 .c 里的说明)，所以发送方那一路也会听到总线上的字节
 * (取决于板上收发器的 RE̅ 跟不跟 DE 走)。**它 ≈ 轮数 × 帧长才是正常的，不是故障。**
 * uart_lb_offdir_size 是最近一次被丢弃的那一段有多大(正常就是一整帧)。
 * 真正该关心的是:它涨而 ok_cnt 不涨 / lost_cnt 开始涨。 */
extern volatile uint32_t uart_lb_offdir_cnt;
extern volatile uint32_t uart_lb_offdir_size;

/* 最近一次"丢帧"的现场(只在 lost_cnt 涨的时候有意义)。
 *   lost_got  那一轮接收方实际收到多少字节
 *   lost_need 那个用例本来该有多少字节(整帧)
 *   lost_state/lost_len  判丢的那一刻解析器停在哪、攒了多少字节
 * 用法：got == need 就是字节到齐了却没凑成帧(问题在解析器/标志)；
 * got < need 就是真少了字节，差值告诉你断在哪儿。 */
extern volatile uint32_t uart_lb_lost_state;
extern volatile uint32_t uart_lb_lost_len;
extern volatile uint32_t uart_lb_lost_got;
extern volatile uint32_t uart_lb_lost_need;
extern volatile uint32_t uart_lb_lost_case;

/* 丢帧时"还差几个字节"的直方图，分桶: 0 / 1 / 2 / 3..7 / 8..31 / >=32。
 * 全挤在 1 号桶 = 帧尾被截；铺满各桶 = 结算早于收完。 */
#define UART_LB_SHORT_BINS 6U
extern volatile uint32_t uart_lb_lost_short[UART_LB_SHORT_BINS];

/* 丢帧时解析器停在哪个状态，索引 = UartFrameState。
 * 全落在 ST_SYNC0(0) = 帧头根本没看见(丢的是开头)；
 * 落在 ST_CRC16(4) = 帧身收全了只差尾巴(丢的是结尾)。 */
#define UART_LB_STATE_BINS 5U
extern volatile uint32_t uart_lb_lost_state_hist[UART_LB_STATE_BINS];

/* ---- 吞吐与最近一帧 ---------------------------------------------------- */
extern volatile uint32_t uart_lb_tx_bytes;
extern volatile uint32_t uart_lb_rx_bytes;
extern volatile uint8_t uart_lb_last_type_id;
extern volatile uint8_t uart_lb_last_datalen;
extern volatile uint32_t uart_lb_last_seq;   /* 最近一次收到的帧是第几轮发的 */
extern volatile uint32_t uart_lb_round;      /* 已经跑过的轮数 */

void UartLoopbackTest_Init(void);
void UartLoopbackTest_Loop(void);
#else
/* 该测试已关闭：接口退化成空函数，main.c / freertos.c 的调用点不用加 #if。
 * 观测变量 uart_lb_* 此时不存在，调试器里读不到。 */
static inline void UartLoopbackTest_Init(void) {}
static inline void UartLoopbackTest_Loop(void) {}

#endif /* TEST_UART_LOOPBACK_ENABLE */
