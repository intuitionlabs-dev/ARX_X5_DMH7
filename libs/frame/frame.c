#include "frame.h"

#include <stddef.h>

#include "soft_crc.h"
#include "frame_config.h" /* 协议版本串，只有这个文件看得到 */

_Static_assert(FRAME_MAX_LEN == 262U, "帧长常量算错了");

/*
 * FrameBuf 的布局是"自动解包"能不能成立的全部依据：raw 和 f 必须是同尺寸、
 * 且 payload 恰好落在偏移 5。错位了不会报错，只会**静默解出垃圾**，所以在这守住。
 * （真正跑的时候这套布局已经用小工具在 arm-none-eabi-gcc 上实测过一遍。）
 */
_Static_assert(sizeof(FrameBuf) == FRAME_MAX_LEN, "union 两个视图尺寸不一致");
_Static_assert(offsetof(FrameBuf, f.header) == 0U, "帧头没在偏移 0");
_Static_assert(offsetof(FrameBuf, f.payload) == FRAME_HEADER_LEN,
               "payload 没在偏移 5，解包视图整体错位");
_Static_assert(offsetof(FrameBuf, f.trailer) == FRAME_MAX_LEN - FRAME_TRAILER_LEN,
               "帧尾没贴在 262 字节的最末尾");

/* 各包的线上长度。这几个数写进注释里给上位机看了，编译器再核一遍。 */
_Static_assert(sizeof(FramePktMotorSetZeroAck) == 12U, "电机设置零点回复包不是 12 字节");
_Static_assert(sizeof(FramePktArmCtrlPos) == 28U, "控制包(角度+夹爪)不是 28 字节");
_Static_assert(sizeof(FramePktArmCtrlPosVel) == 52U, "控制包(角度+角速度+夹爪)不是 52 字节");
/* 反馈帧是 250 Hz 常流里最长的一包，63 是**协议承诺给上位机**的数（两端按这个预留
 * 缓冲），改结构体就得同步改协议文档，不能让它在编译期悄悄漂移。 */
_Static_assert(sizeof(FramePktArmFeedback) == 63U, "机械臂反馈帧不是 63 字节");
/* 空包必须是 0 字节：线上 datalen 就靠它。sizeof 万一变成 1，空包会多带一个字节。 */
_Static_assert(sizeof(FramePktEmpty) == 0U, "空包结构体不是 0 字节");

/* 版本码是 4 字节。这里**没法**断言"哈希等于某个具体值" —— C 的静态期算不出哈希
 * （没有 constexpr），那个值钉在主机测试 uf_unpack_test.c 里，字符串一改测试就红。 */
_Static_assert(sizeof(FramePktVersion) == 4U, "版本匹配包不是 4 字节");

/*
 * ==================== 对齐断言 ====================
 *
 * 上面那组 sizeof 断言**证明不了对齐**：`{uint8_t; uint8_t[7]; uint32_t;}` 在对齐 1
 * 和对齐 4 下 sizeof 都是 12。而"payload 落在帧偏移 5 上还能安全读 float"恰恰取决于
 * 对齐是不是 1，所以得单独断言。两组互补，不是重复。
 *
 * 换成 #pragma pack 之后这组断言更有必要：pack 是**编译器的全局状态**，靠
 * #pragma pack(push,1)/(pop) 一对括起来，一旦有人在 pack 区外面新加一个要挂进
 * FramePayload 的结构体（或者把 push 挪错了位置），它不会自动被压，编译期
 * 一声不响 —— 只有这里能拦住。
 *
 * 反过来说：**漏 pop 导致的"压过头"这组断言拦不住**（压过头也是 1，断言照样过）。
 * 那个只能靠 push/pop 成对紧贴定义的纪律，见 type_id.h 的说明。
 */
_Static_assert(_Alignof(FramePktEmpty) == 1U, "空包结构体不是 1 字节对齐");
_Static_assert(_Alignof(FramePktVersion) == 1U, "版本匹配包不是 1 字节对齐");
_Static_assert(_Alignof(FramePktMotorSetZeroAck) == 1U, "设零回复包不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmCtrlPos) == 1U, "控制包(角度+夹爪)不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmCtrlPosVel) == 1U, "控制包(角度+角速度+夹爪)不是 1 字节对齐");
_Static_assert(_Alignof(FramePktArmReset) == 1U, "一键复位包不是 1 字节对齐");
/* 反馈帧里有 4 个 uint8_t 开头，4 字节之后紧跟 float[6] —— 要是没压成 1 对齐，
 * 编译器会在那 4 个字节后面插 0 字节填充看着没事，但整个包会变成 64 字节。 */
_Static_assert(_Alignof(FramePktArmFeedback) == 1U, "机械臂反馈帧不是 1 字节对齐");
_Static_assert(_Alignof(FramePayload) == 1U, "payload union 不是 1 字节对齐");
_Static_assert(_Alignof(FrameBuf) == 1U, "帧 union 不是 1 字节对齐");

int32_t FrameEncode(uint8_t *out, uint32_t out_size, FrameTypeId type_id,
                        const uint8_t *data, uint8_t data_len) {
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
  out[4] = CRC8_Modbus_calc(out, FRAME_HEADER_LEN - 1U);

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
    h ^= (uint8_t)(*s++); /* 按字节异或：转 uint8_t 是为了不依赖 char 有没有符号 */
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
}

FrameStatus FrameParserFeed(FrameParser *p, uint8_t byte) {
  FrameStatus result = FRAME_INCOMPLETE;

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
      /*
       * 校验通过。复位状态以便接着解下一帧，但保留 type_id/data_len/frame，
       * 调用方在这一刻取 payload。
       */
      p->state = FRAME_ST_WAIT_SYNC0;
      p->len = 0U;
      return FRAME_OK;

    default:
      FrameParserReset(p);
      return result;
    }
  }
}
