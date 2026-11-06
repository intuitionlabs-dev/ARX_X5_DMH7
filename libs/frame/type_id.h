#pragma once

/* 定义报文类型和小端载荷；线格式固定为六个关节及一个夹爪电机槽位。 */

#include <stdint.h>

/* 类型码以单字节存储，允许解析表外类型。 */
typedef uint8_t FrameTypeId;

/* 类型码 0 表示解析器当前没有有效类型。 */
#define FRAME_TYPE_ID_NONE 0x00U

/* 线上的固定尺寸（协议定的，不是 App 的 ARM_JOINT_NUM/ARM_MOTOR_NUM） */
#define FRAME_PKT_JOINT_NUM 6U /* 六个关节 */
#define FRAME_PKT_MOTOR_NUM 7U /* 六个关节 + 夹爪 */

/* 码表 */
/* 0x0x 为链路控制，0x1x 为机械臂命令，0x8x 为反馈上报。 */
#define FRAME_TYPE_ID_VERSION 0x01U /* 版本匹配包 · 双向 */
#define FRAME_TYPE_ID_ARM_INIT 0x02U /* 初始化命令包 · 下发 */
#define FRAME_TYPE_ID_MOTOR_SET_ZERO 0x10U     /* 电机设置零点包 · 下发 */
#define FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK 0x11U /* 电机设置零点回复包 · 上报 */
#define FRAME_TYPE_ID_ARM_START 0x12U          /* 机械臂启动包 · 下发 */
#define FRAME_TYPE_ID_ARM_GRAVITY 0x13U        /* 机械臂重力补偿包 · 下发 */
#define FRAME_TYPE_ID_ARM_STOP 0x14U           /* 机械臂停止包 · 下发 */
#define FRAME_TYPE_ID_ARM_DAMPING 0x15U        /* 机械臂阻尼包 · 下发 */
#define FRAME_TYPE_ID_ARM_CTRL_POS 0x16U       /* 机械臂控制包(角度+夹爪) · 下发 */
#define FRAME_TYPE_ID_ARM_CTRL_POS_VEL 0x17U   /* 机械臂控制包(角度+角速度+夹爪) · 下发 */
#define FRAME_TYPE_ID_ARM_RESET 0x18U          /* 一键复位命令包 · 下发 */
#define FRAME_TYPE_ID_ARM_FEEDBACK 0x80U       /* 机械臂反馈帧 · 上报 */
#define FRAME_TYPE_ID_LINK_STATUS 0x81U        /* 链路状态包 · 上报 */

/* rx_by_tid 按此表顺序统计，最后一槽累计表外类型；两端须采用相同顺序。 */
#define FRAME_TID_DOWN_LIST                                                    \
  FRAME_TYPE_ID_VERSION, FRAME_TYPE_ID_ARM_INIT, FRAME_TYPE_ID_MOTOR_SET_ZERO, \
      FRAME_TYPE_ID_MOTOR_SET_ZERO_ACK, FRAME_TYPE_ID_ARM_START,               \
      FRAME_TYPE_ID_ARM_GRAVITY, FRAME_TYPE_ID_ARM_STOP, FRAME_TYPE_ID_ARM_DAMPING, \
      FRAME_TYPE_ID_ARM_CTRL_POS, FRAME_TYPE_ID_ARM_CTRL_POS_VEL,              \
      FRAME_TYPE_ID_ARM_RESET
#define FRAME_TID_DOWN_NUM 11U
#define FRAME_TID_SLOT_NUM 12U

/* 载荷使用 1 字节对齐，确保偏移 6 的多字节字段可安全访问。 */
#pragma pack(push, 1)

/* 双向版本报文：code 为版本串的 FNV-1a 32 位哈希，共 4 字节。 */
typedef struct {
  uint32_t code;
} FramePktVersion;

/* 空载荷使用 GCC 零长度结构体，datalen 为 0，行为由 type_id 指定。 */
typedef struct {
} FramePktEmpty;

/* 空载荷初始化请求；入口姿态、在线状态及机型初始化策略由应用层检查。 */
typedef FramePktEmpty FramePktArmInit;

/* 空载荷设零请求；参与电机由应用层选择。 */
typedef FramePktEmpty FramePktMotorSetZero;

/* 设零应答包含状态、各电机结果和耗时；结果索引 0～5 为关节，6 为夹爪。 */
typedef struct {
  uint8_t state;
  uint8_t result[FRAME_PKT_MOTOR_NUM];
  uint32_t elapsed_ms;
} FramePktMotorSetZeroAck;

/* 空载荷请求：进入运行模式。 */
typedef FramePktEmpty FramePktArmStart;

/* 空载荷请求：进入重力补偿模式。 */
typedef FramePktEmpty FramePktArmGravity;

/* 空载荷请求：进入失能停止模式。 */
typedef FramePktEmpty FramePktArmStop;

/* 空载荷请求：进入阻尼模式。 */
typedef FramePktEmpty FramePktArmDamping;

/* 位置载荷共 28 字节，关节及夹爪角单位为 deg。 */
typedef struct {
  float joint_angle[FRAME_PKT_JOINT_NUM];
  float gripper_angle;
} FramePktArmCtrlPos;

/* 位置速度载荷共 52 字节，角度单位为 deg，关节速度单位为 deg/s。 */
typedef struct {
  float joint_angle[FRAME_PKT_JOINT_NUM];
  float joint_vel[FRAME_PKT_JOINT_NUM];
  float gripper_angle;
} FramePktArmCtrlPosVel;

/* 空载荷关节复位请求；应用层检查入口条件并执行机型配置的复位动作。 */
typedef FramePktEmpty FramePktArmReset;

/* 反馈载荷共 63 字节：模式、状态、电机掩码、角度 deg、速度 deg/s 和温度 ℃；bit 6 为夹爪。 */
typedef struct {
  uint8_t mode;
  uint8_t robot_state;
  uint8_t motor_lost_mask;
  uint8_t motor_inited_mask;
  float joint_angle[FRAME_PKT_JOINT_NUM];
  float joint_vel[FRAME_PKT_JOINT_NUM];
  float gripper_angle;
  uint8_t motor_temp[FRAME_PKT_MOTOR_NUM];
} FramePktArmFeedback;

/* 共享链路统计载荷记录上次上报以来的计数增量，dt_ms 为统计窗口长度。 */
typedef struct {
  uint32_t rx_total;
  uint32_t rx_by_tid[FRAME_TID_SLOT_NUM];
  uint32_t crc8_err;
  uint32_t crc16_err;
  uint32_t drop_bytes;
  uint32_t bad_arm;
  uint32_t over_len;
  uint32_t ring_drop;
  uint32_t gated;
  uint32_t tx_fail;
  uint32_t dt_ms;
} FramePktLinkStatus;

#pragma pack(pop)
