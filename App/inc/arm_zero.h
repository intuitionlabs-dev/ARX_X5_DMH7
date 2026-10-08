#pragma once

/* 机械臂「设零点」维护路径 —— 一个**独立的全局状态机**，不属于常规控制管线。
 *
 * 为什么单独一个模块，而不是往 Arm_mode(robot_def.h) 里加一个状态：
 *   设零点指令(`DMmotor_Save_Zero()` 的 FF..FE / `ENCOSmotor_Save_Zero()` 的 0x7FF ..03)
 *   和常规控制帧是**互斥**的，设零期间必须一条控制帧都不发。而常规控制帧有两处：
 *     - `DMmotor_Send_Step()`(MotorTask 1kHz)：stop 模式下 500Hz 发零输出 MIT 帧，
 *       而且**每 300ms 无条件给所有达妙发一次使能帧** —— 它会直接把"全部失能"抵消掉；
 *     - `ENCOSmotor_Send_Step()`：stop 模式下 500Hz 发停止帧。
 *   这两处都在 MotorTask 里按拍跑，跟 arm_mode 无关；把它们关掉只能在调用它们的地方关。
 *   所以本模块跑在 **MotorTask**、和两个发送循环同一个任务，用 Arm_Zero_Busy()
 *   在设零期间把它那条总线的发送一起跳过(见 Core/Src/freertos.c 的 StartMotorTask)。
 *   一条总线被关掉之后**每一条总线**都不发帧（不做"只关某条总线"）。
 *
 * 流程（用户指定）：
 *   得到设零请求 → ①所有参与电机失能 → ②等失能确认 → ③依次发设零指令 →
 *   ④检查是否成功，不成功就反复(重发) → 直到成功或单台/整体超时 →
 *   成功和超时两种情况都通过下面的全局变量给出反馈。
 *
 * 实测两个坑(细节见 arm_zero.c 文件头的记录)：
 *   - 达妙**失能之后不一定还发反馈**：j4 会继续发(errcode 报 0)，j5 收到 0xFD 之后就
 *     彻底不发了。所以"失能确认"是 errcode==0 或"安静够久"二者之一；"检查设零是否
 *     成功"对后者要先用使能帧(0xFC)把它唤醒才读得到位置，读完再关回去。
 *   - 设零期间总线故意安静，会把 20ms 掉线看门狗骗到，进而清掉 motor_inited[JOINT6]，
 *     所以设零期间主动喂狗(keep_monitors_alive)。
 *   - ENCOS 有明确 ACK，但**ACK 只说明命令收到了、不等于零点写进去了**(库自己的 README
 *     也要求"核对反馈后由应用决定是否重试")。所以 ACK 报成功之后还要核一帧**设零后的新
 *     位置**；结果判据和达妙一致(前后都在容差内 → UNCONFIRMED)。
 *     ⚠ ACK 的迟到量实测 **0.4~5s 不等、反复触发还会越来越慢**(它写完才回，跟主动发不发帧
 *     无关，位置查询和零增益停止帧都顶不动它)，所以等 ACK 要有上限：ACK 在
 *     ARM_ZERO_ENCOS_ACK_WAIT_MS 内来了就按上面这条口径判；等够还不来就**退让到"只看设零
 *     后的新位置"**(和达妙那四台同一套判据、同一容差)，并把这几台记进
 *     Arm_zero::ack_missing_mask —— 死等会把"零点其实写好了"的电机报成假超时。
 *     ACK 之后(没等 ACK 则是退让之后)那帧**新位置**得自己去要 —— ENCOS 是问答式的 ——
 *     发的是零增益停止帧(ENCOSmotor_Disable()，和空闲期扫描同一形状)，见 check_current。
 *
 * 参与集：默认是**六个关节**。夹爪**不在默认集里** —— 它的零点是行程中点(∓142)，只能由
 * "推到限位→退回半个行程"那个找零流程给出，把当前开口设成零是错的；要动夹爪就显式写
 * participate_override 点名(一键夹爪找零就是这么用的，见 arm_auto.c)。
 * j1~j3(ENCOS) 由 arm_zero_encos_enable(2026-10-08 起为 1)控制：为 0 时它们只占位、
 * 不参与失能也不发帧，结果记 ARM_ZERO_RES_SKIP。
 * 于是"默认触发"= j1~j6，两臂各 3 台 ENCOS + 3 台达妙。
 * 另外 Arm_zero::participate_override 能把一轮**缩到指定的几台**(一键夹爪找零就这么用)：
 * 那一轮只失能/设零被点名的电机，其余电机虽然同样收不到帧(见上面"整条总线都不发帧")，
 * 但不会被失能 —— 对"只想设夹爪"这种用法，这就是避免整条臂松掉的唯一办法。
 * 注意**一轮设零的价值全在参与集**：finish() 只按参与集判成败，没点名的记 SKIP。
 *
 * 用法（JLink 直接读写内存，本工程就是这么验的）：
 *   写 g_arm[N].gimbal->zero.request = 1 → 目标跑几百 ms → 读 g_arm[N].gimbal->zero 里的
 *   state / result[] / retry[] / elapsed_ms / disable_confirmed。
 *   结束后 request 会被自动清 0；再写 1 可以再来一轮。
 *
 * 警告：设零把"当前位姿"定义成零点，会**覆盖电机里的原零点**。每次触发之前必须先把
 * 机械臂摆到基准位姿；摆的姿态若和 gimbal_arm_def.h 里 JOINTn_POS_INIT 标定的姿态不同，
 * 设零后要跟着改那些宏，否则关节角整体偏一个常量。
 */

#include <stdint.h>

#include "robot_def.h" /* ARM_MOTOR_NUM / JOINT1..6 / GRIPPER */

/* Gimbal 的完整定义在 App/inc/gimbal_arm.h，但那个头**反过来要 include 本头**(它要把
 * 下面的 Arm_zero 当成员嵌进去)，互相包含只会让其中一个拿到不完整的类型。本头里 Gimbal
 * 只出现在函数参数和指针里，前向声明足够。 */
typedef struct Gimbal_t Gimbal;

/* 状态机状态。JLink 读的就是这个值(声明成 uint8_t，一个字节)。 */
typedef enum {
    ARM_ZERO_IDLE = 0,     /* 空闲 */
    ARM_ZERO_DISABLE,      /* 瞬态：失能帧已发，正等确认(本模块里没有停留点) */
    ARM_ZERO_WAIT_DISABLE, /* ① 等所有参与电机的失能确认 */
    ARM_ZERO_SEND,         /* ② 给第 s_index 台发设零帧 */
    ARM_ZERO_CHECK,        /* ③ 等新反馈(达妙)/等 ACK(ENCOS)，判成功，不成功则重发 */
    ARM_ZERO_DONE,         /* 全部参与电机都没报错，结束 */
    ARM_ZERO_TIMEOUT,      /* 有台没搞定(或失能没确认)，结束 */
} Arm_zero_state;

/* 每台的结果码，写进 Arm_zero::result[] */
typedef enum {
    ARM_ZERO_RES_NONE = 0,        /* 没走到 */
    ARM_ZERO_RES_UNCONFIRMED = 1, /* 达妙：帧发了、位置本来就在容差内，无法证明写进去了 */
    ARM_ZERO_RES_OK = 2,          /* 成功(达妙：位置被拉到 0；ENCOS：ACK 报成功) */
    ARM_ZERO_RES_FAIL = 3,        /* ENCOS：ACK 报失败 */
    ARM_ZERO_RES_TIMEOUT = 4,     /* 反复重发到单台超时 */
    ARM_ZERO_RES_SKIP = 5,        /* 占位跳过(当前是 j1~j3) */
} Arm_zero_result;

/* j1~j3 是否真的参与(0=占位，只标记 SKIP)。CAN3 通了、零点标定完之后再翻这个。
 * **在触发那一轮开始时锁定**：运行途中翻它不影响本轮参与集，只影响下一轮。
 * 这是**策略开关、不是状态**，所以留在全局：它描述的是"这块板子上 ENCOS 那条路能不能
 * 用"，和具体哪台臂无关。 */
extern uint8_t arm_zero_encos_enable;

/* ==================== 一轮设零的全部状态 ====================
 *
 * 原来这些是一组**文件级全局**(arm_zero_request / arm_zero_state / arm_zero_result[] / …)，
 * JLink 直接按符号读写。两台臂之后它们必须按台分开 —— 两台臂各有各的设零进度，共用一份的
 * 话 A 台设到一半、B 台一触发就把它的进度整个覆盖。所以整块收进这个结构体、挂在 `Gimbal`
 * 上(见 App/inc/gimbal_arm.h)，**一台臂一个**。
 *
 * JLink 的读法跟着变了(原来那个符号没有了)：
 *     g_arm[0].gimbal->zero.state       // Arm_zero_state
 *     g_arm[0].gimbal->zero.result[6]   // 夹爪的结果码
 * 触发一轮：写 `g_arm[0].gimbal->zero.request = 1`。
 *
 * 字段分两段：前一半是"外面写/外面读"的入口与反馈，后一半是本模块自己的跨拍状态。 */
typedef struct {
    /* ---- 入口 ---- */
    /* 写 1 触发一轮；状态机在**非忙**状态下会消费它(清 0)并开跑。 */
    volatile uint8_t request;

    /* ---- 反馈(JLink 读这几个) ---- */
    volatile uint8_t state;                 /* Arm_zero_state */
    volatile uint8_t result[ARM_MOTOR_NUM]; /* Arm_zero_result，每台一个 */
    volatile uint8_t retry[ARM_MOTOR_NUM];  /* 每台重发了几次 */
    volatile uint32_t elapsed_ms;           /* 本轮耗时 */
    /* 失能确认是否拿到过(达妙反馈 errcode==0)。超时时看它就知道是卡在失能还是卡在设零。 */
    volatile uint8_t disable_confirmed;
    /* 哪些电机在"检查设零结果"时是靠先发使能帧(0xFC)唤醒才读到位位置的，bit = 电机索引。
     * 实测 j5 失能后不再发反馈，会落到这一位；全 0 说明 4 台失能后都还在汇报。 */
    volatile uint8_t wake_mask;

    /* 参与集覆盖：非 0 = 这一轮**只让这些位参与**(bit = Arm_Motor_Index)，0 = 照旧(整臂)。
     * 一键夹爪找零用它把一轮设零缩到夹爪一台(见 App/arm_auto.c 的 case 3)，用完整的设零
     * 机制(失能确认 / 唤醒读回 / 重发 / 喂狗)但不动 j1~j6。
     * **一次性**：触发那一轮一开始(start())就锁进参与集并把它清回 0，和 request 被消费
     * 一个路子 —— 所以不会留下"以后每次设零都只设夹爪"的雷。 */
    uint8_t participate_override;

    /* ---- 内部跨拍状态(静态零初始化即可) ---- */
    uint32_t start_ms; /* 本轮开始时刻 */
    uint32_t phase_ms; /* 当前阶段(等失能确认 / 当前这一台)的起点 */
    uint32_t send_ms;  /* 当前这一台最近一次"发出的那一帧"的时刻 */
    uint32_t retry_ms; /* 最近一次重发失能帧的时刻 */
    uint8_t index;     /* 正在处理的电机，ARM_MOTOR_NUM 表示跑完了 */
    uint8_t dm_woken;  /* 当前这台达妙是不是已经被使能帧唤醒过(读完要关回去) */
    float before_pos;  /* 发设零帧之前的位置：分辨"真被拉到 0"还是"本来就在 0" */
    /* 本轮的参与集，bit = Arm_Motor_Index，**在 start() 里按当时的 arm_zero_encos_enable
     * 锁定**。不能每次现问那个全局：判定成功/超时是在 finish() 里做的，若运行途中有人把
     * 开关翻回 0，现问就会把已经记成"超时"的 ENCOS 当成"本来就不参与"，于是本该报 TIMEOUT
     * 的一轮被判成 DONE —— 实测就是这么撞出来的。 */
    uint8_t participate_mask;

    /* 哪些 ENCOS 是在**没有 ACK** 的情况下判的(bit = 电机索引)。实测 0x7FF 的应答迟到量
     * 0.4~5s 不等、反复触发还会更慢(见 arm_zero.c 实测记录第 5 条)，等到 1.5s 还不来就
     * 退让到"只看设零后的新位置"(和达妙同一套判据)。这一位上的人 = 走了退让路径，
     * 让"ACK 确认过的成功"和"只看位置的成功"在结果里分得开，不会混成一个 OK。 */
    uint8_t ack_missing_mask;
} Arm_zero;

/* ---- API ---- */
/* 状态机步进，在 MotorTask 里每拍调一次(1 kHz)，**每台臂各调一次**。
 * g 为 NULL(驱动没建起来)时直接返回：请求留着不消费，下一拍再来。 */
void Arm_Zero_Loop(Gimbal* g);

/* 设零进行中返回 1：此时**不许发任何常规控制帧**(它那条总线的发送循环要跳过)。
 * **按台判**：一台臂在设零，另一台的帧照发(两台各走一路 CAN，互不影响)。
 * g 为 NULL 时返回 0(没有驱动就没有帧要发)。 */
uint8_t Arm_Zero_Busy(Gimbal* g);
