#pragma once

/*
 * 包类型码 type_id + 各包的 payload 结构体 —— 本工程的串口协议目录。
 *
 * uart_frame 帧头的第 3 个字节（偏移 2）就是 type_id，标记这一包是干什么的；payload
 * 怎么解释完全由它决定：
 *
 *     AA 55 | type_id | datalen | crc8 | data[datalen] | crc16
 *      0 1  |    2    |    3    |  4   |  5..5+N-1     | 5+N 5+N+1
 *
 * 加新包：下面码表加一个 #define，再照着现有结构体加一个 packed struct，最后在
 * uart_frame.h 的 UartFramePayload union 里挂上成员（那一处漏了就不会被解包）。
 *
 * 三条约定，收发两端必须一致：
 *   1. **小端**。本工程跑在 Cortex-M7 上，本身就是小端，所以结构体直接按内存序搬到
 *      线上就是对的（packed 保证了不会有编译器插进来的填充字节）。
 *   2. **角度/速度分别用 deg / deg·s⁻¹ 的比值口径**，和 Arm_control_data 一致
 *      （那边的注释就是这么写的），不是 rad。和 libs/gravity_compensation 那套
 *      URDF 的 rad 差一个 DEG2RAD，接的时候别混。
 *   3. 全部 float32 单精度，没有 double、没有定点。
 *
 * 关于方向：码表里标了「下发」（上位机 -> 板子）还是「上报」（板子 -> 上位机）。
 * 现在都挤在 0x1x 这一段，按包的功能排；以后多了可以按方向重新分段（比如上位机
 * 0x1x、板子 0x8x），那属于协议改版，不是这里的排版问题。
 *
 * 这一层**不 include App 的任何头** —— 线上有几个关节、几台电机是协议定死的，不是
 * 由 ARM_JOINT_NUM / ARM_MOTOR_NUM 决定的。两个数现在正好都是 6 / 7，接上层的时候
 * 在 App 那边加个 _Static_assert 对一下就行（见 uart_frame.h 末尾的说明）。
 */

#include <stdint.h>

/* 线上的包类型码就是一个字节。用 typedef 而不是 enum：解析器收到的是**任意**字节，
 * 包括本工程还不认识的码，用 enum 接会有"取值不在枚举里"的坑。 */
typedef uint8_t UartFrameTypeId;

/* 保留：0 表示"没有类型"。解析器复位后 type_id 就是 0，收到真帧才会被覆盖，
 * 所以调试时看它是不是 0 就知道"这帧解出来没有"。编码方不要主动用它。 */
#define UART_FRAME_TYPE_ID_NONE 0x00U

/* ---- 线上的固定尺寸（协议定的，不是 App 的 ARM_JOINT_NUM/ARM_MOTOR_NUM） ---- */
#define UART_FRAME_PKT_JOINT_NUM 6U /* 六个关节 */
#define UART_FRAME_PKT_MOTOR_NUM 7U /* 六个关节 + 夹爪 */

/* ==================== 码表 ==================== */
/* 0x01 单独放低段：握手必须最先发生，而且它最短。0x02..0x0F 预留给以后的高优先级/
 * 短包（心跳、ACK 之类）。 */
#define UART_FRAME_TYPE_ID_VERSION 0x01U /* 版本匹配包 · 双向 */
#define UART_FRAME_TYPE_ID_MOTOR_SET_ZERO 0x10U     /* 电机设置零点包 · 下发 */
#define UART_FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK 0x11U /* 电机设置零点回复包 · 上报 */
#define UART_FRAME_TYPE_ID_ARM_START 0x12U          /* 机械臂启动包 · 下发 */
#define UART_FRAME_TYPE_ID_ARM_GRAVITY 0x13U        /* 机械臂重力补偿包 · 下发 */
#define UART_FRAME_TYPE_ID_ARM_STOP 0x14U           /* 机械臂停止包 · 下发 */
#define UART_FRAME_TYPE_ID_ARM_DAMPING 0x15U        /* 机械臂阻尼包 · 下发 */
#define UART_FRAME_TYPE_ID_ARM_CTRL_POS 0x16U       /* 机械臂控制包(角度+夹爪) · 下发 */
#define UART_FRAME_TYPE_ID_ARM_CTRL_POS_VEL 0x17U   /* 机械臂控制包(角度+角速度+夹爪) · 下发 */

/* ==================== 各包的 payload ====================
 *
 * 下面**所有结构体**都在一对 #pragma pack(push, 1) / #pragma pack(pop) 里面。两个理由：
 *
 *   - 线格式不允许编译器插填充字节，结构体得跟线上字节一一对应；
 *   - payload 在帧里从**偏移 5** 开始，不是 4 的倍数，包里的 float 成员天生是非对齐的。
 *     压到 1 字节对齐，编译器才知道这件事，会生成逐字节访问的代码。Cortex-M7 上
 *     LDR/STR 是按对齐访问的，不告诉它就会在非对齐地址上出错。
 *
 * 用 #pragma pack 而不是 __attribute__((packed))：前者 ARMCC / IAR / MSVC 都认，
 * 后者是 GCC / Clang 专有的。
 *
 * 代价：pack 属于**编译器的全局状态**，不是某个类型的属性 —— 这是它比
 * __attribute__((packed)) 笨的地方。两种错法方向相反：
 *
 *   - **漏了 pop**：后面所有类型都被压成 1 对齐。对正确性无害（本来就是想压），
 *     但会静默影响本文件之后、甚至别的翻译单元里的无关类型。没有任何断言能拦这个，
 *     只能靠 push/pop 成对且紧贴定义这条纪律 —— 所以下面这一对是连着的。
 *   - **漏了 push / 结构体定义在 pack 区外面**：该压的没压，对齐悄悄变成 4，
 *     payload 在偏移 5 上读 float 就出事。这个 uart_frame.c 里的 _Alignof 断言能拦。
 *
 * 加新包时注意：要挂进 UartFramePayload 的结构体必须在这个区内，否则断言立刻红。 */
#pragma pack(push, 1)

/* 「版本匹配包」：握手用，**双向同码同布局** —— 上位机发过来是"我是什么版本"，
 * 板子发出去是自报家门。一个包一个码就够了，不做请求/回复两个码（那是设零包才需要的，
 * 因为它的回复里带了状态机结果，语义不同）。
 *
 *   code  版本码 = FNV-1a 32(版本串)。两边相等才认为协议对得上，可以继续通信。
 *         版本串本身和哈希的算法说明都在 libs/uart_frame/uart_frame_config.h，
 *         板上取当前值用 UartFrameVersionCode()，别在这里硬写数字。
 *
 * 4 字节小端。 */
typedef struct {
  uint32_t code;
} UartPktVersion;

/* 空 payload 的包：作用完全在 type_id 上，线上 datalen = 0。
 *
 * 结构体是空的（GCC 的零长度结构体扩展，`-std=gnu11` 下合法，sizeof 为 0）——
 * 放进来只是为了让 union 里的成员一一对上码表，`p->buf.f.payload.arm_start` 这种
 * 写法能编过。**它们没有任何字段可读**，用的时候只判 p->type_id。 */
typedef struct {
} UartPktEmpty;

/* 「电机设置零点包」：触发一轮设零，参与集固定是全部 7 台（由 libs 那边的
 * arm_zero_encos_enable 决定 j1~j3 是真参与还是占位跳过）。整包的语义就是
 * "开始设零"，没有参数 —— 所以不带 payload，码表见上。 */
typedef UartPktEmpty UartPktMotorSetZero;

/* 「电机设置零点回复包」：一轮设零结束后上报。
 *
 *   state      本轮结束时的状态机状态，取值是 arm_zero.h 的 Arm_zero_state
 *              （DONE / TIMEOUT 是终态；跑到一半来问会看到中间态）
 *   result[i]  第 i 台的结果码，取值是 arm_zero.h 的 Arm_zero_result
 *              （索引同 Arm_Motor_Index：0..5 = j1..j6，6 = 夹爪）
 *   elapsed_ms 本轮从触发到结束的毫秒数（arm_zero_elapsed_ms）
 *
 * 字段特意声明成 uint8_t 而不是直接引 arm_zero.h 的枚举 —— 这个头不 include App 的
 * 东西。取值含义对上就行，接上层时按枚举用。
 *
 * 没放进来的两个诊断量：arm_zero_retry[]（每台重发几次）、arm_zero_wake_mask
 * （哪几台失能后要靠使能帧唤醒才读得到位置）。上位机拿这两个也没法做什么，需要时
 * 再扩包。 */
typedef struct {
  uint8_t state;
  uint8_t result[UART_FRAME_PKT_MOTOR_NUM];
  uint32_t elapsed_ms;
} UartPktMotorSetZeroAck;

/* 「机械臂启动包」：使能电机、进常规控制。空 payload。 */
typedef UartPktEmpty UartPktArmStart;

/* 「机械臂重力补偿包」：进重力补偿模式 —— 就是 libs/gravity_compensation 算出来的
 * 静态重力力矩顶在前馈上，位置环的 Kp 通常要放松。空 payload。 */
typedef UartPktEmpty UartPktArmGravity;

/* 「机械臂停止包」：失能电机。空 payload，**不是**"停在这不动" —— 是松劲。 */
typedef UartPktEmpty UartPktArmStop;

/* 「机械臂阻尼包」：进阻尼/保护模式。空 payload。 */
typedef UartPktEmpty UartPktArmDamping;

/* 「机械臂控制包(角度+夹爪)」：纯位置指令。
 * joint_angle[] 单位 deg，和 Arm_control_data.joint_angle 一个口径；
 * gripper_angle 是夹爪开合（对应 Arm_control_data.gripper_oc_angle）。
 * 一共 7 个 float = 28 字节。 */
typedef struct {
  float joint_angle[UART_FRAME_PKT_JOINT_NUM];
  float gripper_angle;
} UartPktArmCtrlPos;

/* 「机械臂控制包(角度+角速度+夹爪)」：位置 + 速度前馈。
 * joint_vel[] 单位 deg/s，喂 Arm_control_data.joint_vel（那一项经
 * JOINT_SPEEDCOMP_COEF 变成 controller.speed_comp）。
 * 一共 13 个 float = 52 字节。 */
typedef struct {
  float joint_angle[UART_FRAME_PKT_JOINT_NUM];
  float joint_vel[UART_FRAME_PKT_JOINT_NUM];
  float gripper_angle;
} UartPktArmCtrlPosVel;

#pragma pack(pop)
