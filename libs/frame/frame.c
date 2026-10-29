#include "frame.h"

#include <stddef.h>

#include "soft_crc.h"
#include "frame_config.h" /* 协议版本串，只有这个文件看得到 */

_Static_assert(FRAME_MAX_LEN == 263U, "帧长常量算错了");

/* 编译期校验帧缓冲尺寸及载荷偏移，保证原始字节与结构化视图一致。 */
_Static_assert(sizeof(FrameBuf) == FRAME_MAX_LEN, "union 两个视图尺寸不一致");
_Static_assert(offsetof(FrameBuf, f.header) == 0U, "帧头没在偏移 0");
_Static_assert(offsetof(FrameBuf, f.payload) == FRAME_HEADER_LEN,
               "payload 没在偏移 6，解包视图整体错位");
_Static_assert(offsetof(FrameBuf, f.trailer) == FRAME_MAX_LEN - FRAME_TRAILER_LEN,
               "帧尾没贴在 263 字节的最末尾");
/* arm_id 必须落在帧头数组里面。这层不认识它的语义，但偏移错了就成了"改写 datalen
 * 或 crc8"这种静默错帧，比报错难查得多。 */
_Static_assert(FRAME_ARM_ID_OFFSET < FRAME_HEADER_LEN,
               "arm_id 的偏移跑到帧头外面去了");

/* 各包的线上长度。这几个数写进注释里给上位机看了，编译器再核一遍。 */
_Static_assert(sizeof(FramePktMotorSetZeroAck) == 12U, "电机设置零点回复包不是 12 字节");
_Static_assert(sizeof(FramePktArmCtrlPos) == 28U, "控制包(角度+夹爪)不是 28 字节");
_Static_assert(sizeof(FramePktArmCtrlPosVel) == 52U, "控制包(角度+角速度+夹爪)不是 52 字节");
/* 反馈帧是 250 Hz 常流里最长的一包，63 是协议承诺给上位机的数（两端按这个预留
 * 缓冲），改结构体就得同步改协议文档，不能让它在编译期悄悄漂移。 */
_Static_assert(sizeof(FramePktArmFeedback) == 63U, "机械臂反馈帧不是 63 字节");
_Static_assert(sizeof(FramePktLinkStatus) == 88U, "链路状态包不是 88 字节");
/* 空包必须是 0 字节：线上 datalen 就靠它。sizeof 万一变成 1，空包会多带一个字节。 */
_Static_assert(sizeof(FramePktEmpty) == 0U, "空包结构体不是 0 字节");

_Static_assert(sizeof(FramePktVersion) == 4U, "版本匹配包不是 4 字节");

/* 校验各载荷类型的字节对齐，允许从偏移 6 读取多字节字段。 */
_Static_assert(_Alignof(FramePktEmpty) == 1U, "空包结构体不是 1 字节对齐");
_Static_assert(_Alignof(FramePktVersion) == 1U, "版本匹配包不是 1 字节对齐");
_Static_assert(_Alignof(FramePktMotorSetZeroAck) == 1U, "设零回复包不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmCtrlPos) == 1U, "控制包(角度+夹爪)不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmCtrlPosVel) == 1U, "控制包(角度+角速度+夹爪)不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmReset) == 1U, "一键复位包不是 1 字节对齐");
/* 反馈帧里有 4 个 uint8_t 开头，4 字节之后紧跟 float[6] —— 要是没压成 1 对齐，
 * 编译器会在那 4 个字节后面插 0 字节填充看着没事，但整个包会变成 64 字节。 */
_Static_assert(_Alignof(FramePktArmFeedback) == 1U, "机械臂反馈帧不是 1 字节对齐");
_Static_assert(_Alignof(FramePktLinkStatus) == 1U, "链路状态包不是 1 字节对齐");
_Static_assert(_Alignof(FramePayload) == 1U, "payload union 不是 1 字节对齐");
_Static_assert(_Alignof(FrameBuf) == 1U, "帧 union 不是 1 字节对齐");

int32_t FrameEncode(uint8_t *out, uint32_t out_size, FrameTypeId type_id,
                        uint8_t arm_id, const uint8_t *data, uint8_t data_len) {
  const uint32_t frame_len = FRAME_LEN(data_len);
  uint16_t crc16;
  uint32_t i;

  /* 先全部校验完再动手写，失败时 out 保持原样 */
  if (out == NULL) {
    return FRAME_ENC_ERR_NULL;
  }
  if ((data == NULL) && (data_len > 0U)) {
    return FRAME_ENC_ERR_NULL;
  }
  if (out_size < frame_len) {
    return FRAME_ENC_ERR_SIZE;
  }

  out[0] = FRAME_SYNC0;
  out[1] = FRAME_SYNC1;
  out[2] = type_id;
  out[3] = data_len;
  out[FRAME_ARM_ID_OFFSET] = arm_id;
  /* CRC8 覆盖帧头前 5 字节，包含 arm_id。 */
  out[FRAME_HEADER_LEN - 1U] = CRC8_Modbus_calc(out, FRAME_HEADER_LEN - 1U);

  for (i = 0U; i < (uint32_t)data_len; i++) {
    out[FRAME_HEADER_LEN + i] = data[i];
  }

  crc16 = CRC16_Modbus_calc(out, FRAME_HEADER_LEN + (uint32_t)data_len);
  out[FRAME_HEADER_LEN + (uint32_t)data_len] = (uint8_t)(crc16 & 0xFFU);
  out[FRAME_HEADER_LEN + (uint32_t)data_len + 1U] = (uint8_t)(crc16 >> 8);

  return (int32_t)frame_len;
}

uint32_t FrameVersionHash(const char *s) {
  /* FNV-1a 32。offset basis 和 prime 是规范里定死的两个魔数。 */
  uint32_t h = 2166136261U;

  if (s == NULL) {
    return 0U;
  }

  while (*s != '\0') {
    h ^= (uint8_t)(*s++); /* 按无符号字节计算哈希，避免 char 符号扩展。 */
    h *= 16777619U;
  }

  return h;
}

uint32_t FrameVersionCode(void) {
  return FrameVersionHash(FRAME_VERSION_STRING);
}

void FrameParserReset(FrameParser *p) {
  p->state = FRAME_ST_WAIT_SYNC0;
  p->len = 0U;
  p->data_len = 0U;
  p->type_id = FRAME_TYPE_ID_NONE;
  p->arm_id = 0U;
}

static FrameStatus FrameParserStep(FrameParser *p, uint8_t byte) {
  FrameStatus result = FRAME_INCOMPLETE;

  /* 计在循环外面：crc 出错时要靠 continue 把同一个字节重喂一遍，写在里面会重复计数。 */
  p->bytes_in++;

  for (;;) {
    switch (p->state) {
    case FRAME_ST_WAIT_SYNC0:
      if (byte == FRAME_SYNC0) {
        p->buf.raw[0] = byte;
        p->len = 1U;
        p->state = FRAME_ST_WAIT_SYNC1;
      }
      return result;

    case FRAME_ST_WAIT_SYNC1:
      if (byte == FRAME_SYNC1) {
        p->buf.raw[1] = byte;
        p->len = 2U;
        p->state = FRAME_ST_HEADER;
      } else if (byte == FRAME_SYNC0) {
        /* AA AA 55：把这个字节当成新的帧头首字节，重新锁存 */
        p->buf.raw[0] = byte;
        p->len = 1U;
      } else {
        p->state = FRAME_ST_WAIT_SYNC0;
      }
      return result;

    case FRAME_ST_HEADER:
      p->buf.raw[p->len] = byte;
      p->len++;
      if (p->len < FRAME_HEADER_LEN) {
        return result;
      }
      if (CRC8_Modbus_calc(p->buf.raw, FRAME_HEADER_LEN - 1U) !=
          p->buf.raw[FRAME_HEADER_LEN - 1U]) {
        FrameParserReset(p);
        result = FRAME_ERR_CRC8;
        /* 出错这一字节本身可能是下一帧的帧头，重新喂一遍 */
        continue;
      }
      p->type_id = p->buf.raw[2];
      p->data_len = p->buf.raw[3];
      p->arm_id = p->buf.raw[FRAME_ARM_ID_OFFSET];
      p->state = (p->data_len == 0U) ? FRAME_ST_CRC : FRAME_ST_DATA;
      return result;

    case FRAME_ST_DATA:
      p->buf.raw[p->len] = byte;
      p->len++;
      if (p->len >= (uint16_t)(FRAME_HEADER_LEN + p->data_len)) {
        p->state = FRAME_ST_CRC;
      }
      return result;

    case FRAME_ST_CRC:
      p->buf.raw[p->len] = byte;
      p->len++;
      if (p->len < (uint16_t)FRAME_LEN(p->data_len)) {
        return result;
      }
      if ((uint16_t)(p->buf.raw[p->len - 2U] |
                     ((uint16_t)p->buf.raw[p->len - 1U] << 8)) !=
          CRC16_Modbus_calc(p->buf.raw, (uint32_t)(p->len - 2U))) {
        FrameParserReset(p);
        result = FRAME_ERR_CRC16;
        continue;
      }
      /* 完整帧校验通过后复位解析状态，并保留载荷供调用方读取。 */
      p->state = FRAME_ST_WAIT_SYNC0;
      p->len = 0U;
      p->bytes_ok += (uint32_t)FRAME_LEN(p->data_len);
      return FRAME_OK;

    default:
      FrameParserReset(p);
      return result;
    }
  }
}

FrameStatus FrameParserFeed(FrameParser *p, uint8_t byte) {
  const FrameStatus st = FrameParserStep(p, byte);

  /* 保存 bytes_in-bytes_ok 的快照，供任务读取中断更新的统计。 */
  p->bytes_drop = p->bytes_in - p->bytes_ok;

  return st;
}
