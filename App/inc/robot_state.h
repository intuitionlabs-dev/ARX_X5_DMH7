#pragma once

#include <stdint.h>

#include "robot_def.h"
#include "type_id.h"

/* 定义上位机链路类型和板级选择。 */
#define ROBOT_LINK_UART 0
#define ROBOT_LINK_USB  1

#define ROBOT_LINK ROBOT_LINK_USB

/* 导出选定链路的 BSP 接口。 */
#if ROBOT_LINK == ROBOT_LINK_USB
#include "bsp_usb.h" // IWYU pragma: export
/* 导出 UART 链路的 BSP 接口。 */
#else
#include "bsp_uart.h" // IWYU pragma: export
#endif

/* 配置通信超时、反馈、握手和状态上报周期。 */
#define ROBOT_COMM_TIMEOUT_MS 3000U

#define ROBOT_FEEDBACK_PERIOD_MS 4U

#define ROBOT_HANDSHAKE_PERIOD_MS 100U

#define ROBOT_LINK_STATUS_PERIOD_MS 100U

#define ROBOT_ZERO_START_TIMEOUT_MS 200U

/* 定义握手、初始化、运行、设零及复位状态。 */
typedef enum {
    robot_state_wait_handshake = 0,
    robot_state_offline_wait,
    robot_state_wait_init,
    robot_state_initing,
    robot_state_normal,
    robot_state_comm_lost,
    robot_state_calibrating,
    robot_state_resetting,
    ROBOT_STATE_NUM, // NOLINT(readability-identifier-naming)
} Robot_state;

/* 保存已校验的协议位置、速度及夹爪目标。 */
typedef struct {
    uint8_t valid;
    uint8_t has_vel;
    float joint_angle[ARM_JOINT_NUM];
    float joint_vel[ARM_JOINT_NUM];
    float gripper_angle;
} Robot_ctrl_cmd;

/* 配置每臂收包环的载荷容量和槽位数。 */
#define ROBOT_RX_DATA_LEN 64U
#define ROBOT_RX_SLOT_NUM 8U

/* 以原始字节或具体报文结构访问收包载荷。 */
typedef union {
    uint8_t raw[ROBOT_RX_DATA_LEN];
    FramePktVersion version;
    FramePktMotorSetZeroAck motor_set_zero_ack;
    FramePktArmCtrlPos arm_ctrl_pos;
    FramePktArmCtrlPosVel arm_ctrl_pos_vel;
} Robot_rx_payload;

/* 保存单条待处理报文的类型、长度和载荷。 */
typedef struct {
    uint8_t type_id;
    uint8_t len;
    Robot_rx_payload payload;
} Robot_rx_slot;

/* 初始化各臂通信状态并登记接收回调。 */
void Robot_State_Init(void);

/* 推进报文消费、业务状态和周期上报。 */
void Robot_State_Loop(void);

/* 读取指定机械臂当前协议控制目标。 */
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm);

/* 提供链路接收、校验和发送异常计数。 */
extern volatile uint32_t robot_rx_frame_cnt;
extern volatile uint32_t robot_rx_drop_cnt;
extern volatile uint32_t robot_rx_over_cnt;
extern volatile uint32_t robot_crc8_err_cnt;
extern volatile uint32_t robot_crc16_err_cnt;

extern volatile uint32_t robot_tx_fail_cnt;

extern volatile uint32_t robot_rx_bad_arm_cnt;
