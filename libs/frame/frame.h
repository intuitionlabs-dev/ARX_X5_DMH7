#pragma once
#include <stdint.h>

#include "type_id.h" /* FrameTypeId：帧头的包类型码 */

#ifdef __cplusplus
extern "C" {
#endif

/* 小端帧：AA 55、type_id、datalen、arm_id、CRC8、载荷、CRC16；总长为载荷长度加 8。 */

/* 帧头按字节存储，避免主机端序影响。 */
#define FRAME_SYNC0 0xAAU
#define FRAME_SYNC1 0x55U

/* arm_id 位于原始帧偏移 4。 */
#define FRAME_ARM_ID_OFFSET 4U

#define FRAME_HEADER_LEN 6U  /* 帧头 5B + crc8 */
#define FRAME_TRAILER_LEN 2U
#define FRAME_OVERHEAD 8U    /* 帧头 + 帧尾 */
#define FRAME_MAX_DATA 255U  /* datalen 为单字节长度。 */
#define FRAME_MAX_LEN \
  (FRAME_OVERHEAD + FRAME_MAX_DATA)

/* 整帧长度，datalen 已知时用来准备缓冲区 */
#define FRAME_LEN(data_len) (FRAME_OVERHEAD + (uint32_t)(data_len))

/* FrameEncode 的负返回值 */
#define FRAME_ENC_ERR_NULL (-1) /* out 为空，或 data 为空但 datalen 不为 0 */
#define FRAME_ENC_ERR_SIZE (-2) /* out_size 不足 */

/* FrameParserFeed 的返回值，可直接 switch */
typedef enum {
  FRAME_INCOMPLETE = 0, /* 等待后续字节完成帧解析。 */
  FRAME_OK,             /* 解出完整合法帧，payload 已在 buf.f.payload 原地就位 */
  FRAME_ERR_CRC8,       /* 帧头校验失败，已复位 */
  FRAME_ERR_CRC16,      /* 整帧校验失败，已复位 */
} FrameStatus;

typedef enum {
  FRAME_ST_WAIT_SYNC0 = 0, /* 等 0xAA */
  FRAME_ST_WAIT_SYNC1,     /* 已收到 0xAA，等 0x55 */
  FRAME_ST_HEADER,         /* 收 type_id/datalen/arm_id/crc8，即 raw[2..5] */
  FRAME_ST_DATA,           /* 收 datalen 个数据字节 */
  FRAME_ST_CRC,            /* 收 crc16 两个字节 */
} FrameState;

/* 原始字节与结构化视图共享缓冲，载荷类型保持 1 字节对齐。 */
#pragma pack(push, 1)

/* 载荷从帧偏移 6 开始，最大 255 字节；按 type_id 选择结构成员。 */
typedef union {
  uint8_t raw[FRAME_MAX_DATA]; /* 当普通字节用 */
  FramePktEmpty empty;               /* 对照用，同下 */
  FramePktVersion version;
  FramePktArmInit arm_init;
  FramePktMotorSetZero motor_set_zero;
  FramePktMotorSetZeroAck motor_set_zero_ack;
  FramePktArmStart arm_start;
  FramePktArmGravity arm_gravity;
  FramePktArmStop arm_stop;
  FramePktArmDamping arm_damping;
  FramePktArmCtrlPos arm_ctrl_pos;
  FramePktArmCtrlPosVel arm_ctrl_pos_vel;
  FramePktArmReset arm_reset;
  FramePktArmFeedback arm_feedback;
  FramePktLinkStatus link_status;
} FramePayload;

/* 整帧缓冲提供原始字节和结构化视图。 */
typedef union {
  uint8_t raw[FRAME_MAX_LEN];

  struct {
    /* 帧头依次为 AA 55、type_id、datalen、arm_id 和 CRC8。 */
    uint8_t header[FRAME_HEADER_LEN];
    FramePayload payload;
    uint8_t trailer[FRAME_TRAILER_LEN]; /* raw[6+N..7+N]：crc16，低字节在前 */
  } f;
} FrameBuf;

#pragma pack(pop)

/* 流式解析器逐字节同步帧头并校验 CRC，零初始化等价于解析状态复位。 */
typedef struct {
  FrameState state;
  uint16_t len;            /* buf.raw[] 中已收字节数 */
  uint8_t data_len;        /* 当前帧的 datalen */
  FrameTypeId type_id; /* 当前帧的包类型码，见 type_id.h */
  uint8_t arm_id;      /* 当前帧的 arm_id（原样，不校验范围 —— 那是上层的事） */
  FrameBuf buf;        /* 原始帧和解包视图共享存储。 */

  /* 字节累计量在解析状态复位时保留；bytes_drop 为 bytes_in-bytes_ok 的快照。 */
  uint32_t bytes_in;   /* 喂进来的总字节数 */
  uint32_t bytes_ok;   /* 成功解出的那些帧的线上字节数之和 */
  uint32_t bytes_drop; /* = bytes_in - bytes_ok，每喂一个字节刷新一次 */
} FrameParser;

/* 编码一帧并返回总长度；参数或容量无效时返回负值且保持输出缓冲不变。 */
int32_t FrameEncode(uint8_t *out, uint32_t out_size, FrameTypeId type_id,
                        uint8_t arm_id, const uint8_t *data, uint8_t data_len);

/* 复位解析状态并丢弃未完成帧。 */
void FrameParserReset(FrameParser *p);

/* 输入一个字节；返回 FRAME_OK 后可按 type_id 读取载荷，下一次输入会复用缓冲。 */
FrameStatus FrameParserFeed(FrameParser *p, uint8_t byte);

/* 对版本串计算 FNV-1a 32 位哈希；空指针返回 0。 */
uint32_t FrameVersionHash(const char *s);

/* 返回 FRAME_VERSION_STRING 的 FNV-1a 版本码。 */
uint32_t FrameVersionCode(void);

#ifdef __cplusplus
}
#endif
