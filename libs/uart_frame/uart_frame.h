#pragma once
#include <stdint.h>

#include "type_id.h" /* UartFrameTypeId：帧头的包类型码 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 串口帧格式（小端）：
 *
 *   偏移  长度  字段
 *   0     2     帧头 0xAA 0x55
 *   2     1     包类型码 type_id  标记这一包是干什么的，见 type_id.h
 *   3     1     datalen        0..255
 *   4     1     crc8           覆盖偏移 0..3 这 4 个字节
 *   5     N     data           datalen 字节
 *   5+N   2     crc16          覆盖偏移 0..(4+N)，低字节在前
 *
 * 整帧长度 = 7 + datalen，最大 262 字节。
 *
 * crc8 只盖帧头，收完头就能把长度字段被干扰的帧挡掉，不用等数据收完；
 * crc16 盖帧头（含 crc8）+ data。
 *
 * 这一层只管"切出一帧、校验、把 type_id 和 payload 交出去"，**不认识任何 type_id
 * 的具体含义** —— 码值和 payload 各字段的含义都在 type_id.h。
 *
 * 校验通过的那一刻，UartFrameParser 里的 UartFrameBuf 已经能**直接当结构体读**了
 * （见下面 UartFrameBuf 的说明），不用 memcpy、不用逐字段拼。
 */

/* 帧头按字节定义，不用 uint16_t 再 memcpy，避免大小端歧义 */
#define UART_FRAME_SYNC0 0xAAU
#define UART_FRAME_SYNC1 0x55U

#define UART_FRAME_HEADER_LEN 5U  /* 帧头 4B + crc8 */
#define UART_FRAME_TRAILER_LEN 2U /* crc16 */
#define UART_FRAME_OVERHEAD 7U    /* 帧头 + 帧尾 */
#define UART_FRAME_MAX_DATA 255U  /* datalen 就是一个字节 */
#define UART_FRAME_MAX_LEN \
  (UART_FRAME_OVERHEAD + UART_FRAME_MAX_DATA) /* 262 */

/* 整帧长度，datalen 已知时用来准备缓冲区 */
#define UART_FRAME_LEN(data_len) (UART_FRAME_OVERHEAD + (uint32_t)(data_len))

/* UartFrameEncode 的负返回值 */
#define UART_FRAME_ENC_ERR_NULL (-1) /* out 为空，或 data 为空但 datalen 不为 0 */
#define UART_FRAME_ENC_ERR_SIZE (-2) /* out_size 不足 */

/* UartFrameParserFeed 的返回值，可直接 switch */
typedef enum {
  UART_FRAME_INCOMPLETE = 0, /* 还没凑齐一帧，继续喂字节 */
  UART_FRAME_OK,             /* 解出完整合法帧，payload 已在 buf.f.payload 原地就位 */
  UART_FRAME_ERR_CRC8,       /* 帧头校验失败，已复位 */
  UART_FRAME_ERR_CRC16,      /* 整帧校验失败，已复位 */
} UartFrameStatus;

typedef enum {
  UART_FRAME_ST_WAIT_SYNC0 = 0, /* 等 0xAA */
  UART_FRAME_ST_WAIT_SYNC1,     /* 已收到 0xAA，等 0x55 */
  UART_FRAME_ST_HEADER,         /* 收 type_id/datalen/crc8，即 raw[2..4] */
  UART_FRAME_ST_DATA,           /* 收 datalen 个数据字节 */
  UART_FRAME_ST_CRC,            /* 收 crc16 两个字节 */
} UartFrameState;

/*
 * ==================== 自动解包用的两层 union ====================
 *
 * 思路：帧在缓冲区里本来就是**连续**的（头 5 字节、payload、尾 2 字节挨着放），
 * 那就没必要解析完再拷一遍 —— 让同一个缓冲区既是"一串字节"又是"一个结构体"，
 * 怎么解读由你选。于是：
 *
 *   raw[0] ....................... 当字节数组用（CRC、状态机索引）
 *   f.header / f.payload / f.trailer   当结构体用（直接取字段）
 *
 * 两者是同一块内存，所以**零开销、零拷贝**：喂完最后一个字节返回 UART_FRAME_OK
 * 时，结构体已经在原地等着了。
 *
 * 为什么必须用 union 而不是"把 raw 强转成结构体指针"：C 标准 6.5.2.3 明确规定
 * 读 union 的非当前成员是**按对象表示重新解释**（type punning），这是定义良好的；
 * 而 `*(UartPktArmCtrlPos *)buf.raw` 这种指针强转会踩 strict aliasing，优化开起来
 * 就可能被编歪。（C++ 里 union type punning 反而是 UB，这个头是给 C 用的。）
 *
 * 代价：结构体天然是**非对齐**的 —— payload 从偏移 5 开始，float 成员会落在奇数
 * 地址上。所以下面两个 union 和 type_id.h 里的包结构体一样，都压到 1 字节对齐
 * （#pragma pack(1)），编译器收到这个信号才会生成逐字节访问的代码。别去掉。
 *
 * 两个 union 也显式 pack，而不是靠"成员恰好都是 1 对齐"推出来 —— 推导出来的东西
 * 没有守卫：哪天有人往 UartFramePayload 里挂一个 pack 区外面的、带 uint32_t 的结构体，
 * 对齐就悄悄变成 4 了。对齐断言在 uart_frame.c 里。
 */
#pragma pack(push, 1)

/* payload 视图：从帧的偏移 5 开始，最大 255 字节。
 * 各成员和 type_id.h 的码表一一对应，按 p->type_id 选一个读。
 * 空包的成员都是同一个空结构体（sizeof 为 0），写 `payload.arm_start` 这种名字
 * 纯粹是为了读代码时和码表对得上，取不出任何值。 */
typedef union {
  uint8_t raw[UART_FRAME_MAX_DATA]; /* 当普通字节用 */
  UartPktEmpty empty;               /* 对照用，同下 */
  UartPktVersion version;
  UartPktMotorSetZero motor_set_zero;
  UartPktMotorSetZeroAck motor_set_zero_ack;
  UartPktArmStart arm_start;
  UartPktArmGravity arm_gravity;
  UartPktArmStop arm_stop;
  UartPktArmDamping arm_damping;
  UartPktArmCtrlPos arm_ctrl_pos;
  UartPktArmCtrlPosVel arm_ctrl_pos_vel;
} UartFramePayload;

/* 整帧视图：头和尾留在结构体里，payload 夹在中间。
 * 这样 raw 依然是"从 0xAA 开始的连续 262 字节"，CRC16 可以一次算完整帧，
 * 状态机也能照旧按 raw[i] 逐字节收 —— 加了 union 不改变任何一个字节的语义。 */
typedef union {
  uint8_t raw[UART_FRAME_MAX_LEN];

  struct {
    uint8_t header[UART_FRAME_HEADER_LEN]; /* raw[0..4]：AA 55 / type_id / datalen / crc8 */
    UartFramePayload payload;              /* raw[5..5+datalen-1] */
    uint8_t trailer[UART_FRAME_TRAILER_LEN]; /* raw[5+N..6+N]：crc16，低字节在前 */
  } f;
} UartFrameBuf;

#pragma pack(pop)

/*
 * 流式解析器。约 270 字节，放文件作用域或全局，别放栈上。
 * 零初始化等价于 Reset()：INCOMPLETE 和 WAIT_SYNC0 都是 0。
 *
 * 线上出现一个"假的" 0xAA 0x55（噪声，或丢字节导致数据错位）时，后面的
 * 3 个字节会被当成本帧帧头吃掉，于是真帧的 0xAA 0x55 也一起被吞——表现
 * 是丢一帧，下一帧恢复正常。这是所有"匹配到帧头就开始消费"的协议的固有
 * 代价，这一层没有帧编号，**发现不了**丢帧 —— 要判丢帧得让上层在 payload 里
 * 自己放递增计数或时间戳（type_id 不递增，同一个值会连续出现，帮不上忙）。
 * 数据区里不做帧头搜索，因为二进制 payload 本来就可能包含 0xAA 0x55。
 */
typedef struct {
  UartFrameState state;
  uint16_t len;            /* buf.raw[] 中已收字节数 */
  uint8_t data_len;        /* 当前帧的 datalen */
  UartFrameTypeId type_id; /* 当前帧的包类型码，见 type_id.h */
  UartFrameBuf buf;        /* 原始帧 + 解包视图，同一块内存 */
} UartFrameParser;

/**
 * @brief 把一帧组好写进调用方缓冲区
 * @param out      输出缓冲区，失败时一个字节都不会被写
 * @param out_size out 的字节数
 * @param type_id  包类型码，标记这一包是干什么的，见 type_id.h
 * @param data     payload，data_len 为 0 时可以是 NULL
 * @param data_len payload 长度，0..UART_FRAME_MAX_DATA
 * @return 写入的字节数（7 + data_len）；失败返回负的 UART_FRAME_ENC_ERR_*
 */
int32_t UartFrameEncode(uint8_t *out, uint32_t out_size, UartFrameTypeId type_id,
                        const uint8_t *data, uint8_t data_len);

/**
 * @brief 复位解析器，丢弃半帧
 */
void UartFrameParserReset(UartFrameParser *p);

/**
 * @brief 往解析器里喂一个字节
 * @return 返回 UART_FRAME_OK 时：包类型码是 p->type_id，长度是 p->data_len，
 *         payload 已经能当结构体读了 —— 按 p->type_id 选一个成员，例如
 *
 *             if (p->type_id == UART_FRAME_TYPE_ID_ARM_CTRL_POS) {
 *                 const UartPktArmCtrlPos *c = &p->buf.f.payload.arm_ctrl_pos;
 *                 // c->joint_angle[0] ... c->gripper_angle
 *             }
 *
 *         原始字节仍在 p->buf.raw[0..6+data_len-1]（从 0xAA 起）。
 *         连续收帧时请在这一刻把要用的值取走：下一帧的头部字节会覆盖
 *         buf.raw[0..4]，那个值就没了。
 */
UartFrameStatus UartFrameParserFeed(UartFrameParser *p, uint8_t byte);

/**
 * @brief FNV-1a 32 位哈希，算到字符串结尾的 '\0' 为止（按字节，和 char 有无符号无关）
 * @param s 版本串，为 NULL 时返回 0（调用方的编程错误，0 不当版本码用）
 * @return 哈希值
 *
 * 选 FNV-1a 而不是 CRC32，就为两条：不用 1 KB 查表（CRC32 的表 256×4 字节，
 * 为一个握手包不值），以及任何语言三行就能复刻 —— 上位机不必移植我们这份实现。
 */
uint32_t UartFrameVersionHash(const char *s);

/**
 * @brief 本机版本码，即 UartFrameVersionHash(UART_FRAME_VERSION_STRING)
 *
 * 不缓存：一次 33 字节的循环，只在握手那一包里调，不值得为它引入可变全局
 * （这个库目前一个可变全局都没有）。
 */
uint32_t UartFrameVersionCode(void);

#ifdef __cplusplus
}
#endif
