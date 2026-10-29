#pragma once
#include "controller.h"
#include "monitor.h"

/* 声明通用电机实例及其配置类型。 */
typedef struct Arm_motor Arm_motor; // NOLINT(readability-identifier-naming)
typedef struct Arm_motor_config Arm_motor_config; // NOLINT(readability-identifier-naming)

/* 定义通用驱动的创建、控制和设零操作。
 * 设零结果：0 等待，1 成功，2 零点在容差内，负值失败。 */
typedef struct {
    int (*create)(Arm_motor*, const Arm_motor_config*, uint8_t bus);
    void (*mode)(Arm_motor*, uint8_t enabled, float kp, float kd);
    float (*position_rad)(const Arm_motor*);
    uint8_t (*temperature)(const Arm_motor*);
    void (*disable)(Arm_motor*);
    uint8_t (*stopped)(const Arm_motor*, uint32_t now);
    int (*zero_start)(Arm_motor*, uint32_t now);
    int (*zero_poll)(Arm_motor*, uint32_t now);
    int (*validate)(const Arm_motor_config*);
    uint8_t shared_rx_id;
} Arm_motor_driver;

/* 配置驱动型号、总线标识、编码量程和控制参数。 */
struct Arm_motor_config {
    const Arm_motor_driver* driver;
    const char* model;
    uint16_t tx_id, rx_id;
    uint8_t tx_group;
    float position_max, speed_max, torque_max, kp_max, kd_max;
    float current_max, torque_constant;
    float position_divisor, speed_divisor;
    uint8_t cmd_multiturn;
    PID_Config position_pid, speed_pid;
    float kp, kd, damping_kd;
};

/* 保存驱动实例、控制器、监控和设零进度。 */
struct Arm_motor {
    const Arm_motor_config* config;
    void* context;
    controller* control;
    monitor_item* monitor;
    uint32_t zero_sent_ms;
    float zero_before;
    uint8_t zero_stage;
};

/* 提供 DM 和 ENCOS 的通用驱动操作表。 */
extern const Arm_motor_driver ARM_DRIVER_DM;
extern const Arm_motor_driver ARM_DRIVER_ENCOS;

/* 控制参考值与增益在同一临界区内发布。 */
uint32_t Arm_Motors_Lock(void);
/* 恢复参考值发布前的中断屏蔽状态。 */
void Arm_Motors_Unlock(uint32_t interrupt_mask);
/* 初始化本周期各驱动的发送调度状态。 */
void Arm_Motors_Send_Begin(void);
/* 在指定总线上尝试发送一帧，成功返回 1。 */
uint8_t Arm_Motors_Send_Step(uint8_t bus);
