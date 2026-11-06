#pragma once
#include "robot_def.h"
typedef struct Gimbal_t Gimbal;

/* 定义设零任务从失能到复核结束的执行阶段。 */
typedef enum {
    ARM_ZERO_IDLE = 0, ARM_ZERO_DISABLE, ARM_ZERO_WAIT_DISABLE, // NOLINT(readability-identifier-naming)
    ARM_ZERO_SEND, ARM_ZERO_CHECK, ARM_ZERO_DONE, ARM_ZERO_TIMEOUT, // NOLINT(readability-identifier-naming)
} Arm_zero_state;
/* 定义单电机设零的等待、成功和异常结果。 */
typedef enum {
    ARM_ZERO_RES_NONE = 0, ARM_ZERO_RES_UNCONFIRMED, ARM_ZERO_RES_OK, // NOLINT(readability-identifier-naming)
    ARM_ZERO_RES_FAIL, ARM_ZERO_RES_TIMEOUT, ARM_ZERO_RES_SKIP, // NOLINT(readability-identifier-naming)
} Arm_zero_result;

/* 保存设零请求、参与选择、各电机结果和超时状态。 */
typedef struct {
    volatile uint8_t request, state;
    volatile uint8_t result[ARM_MOTOR_NUM];
    volatile uint32_t elapsed_ms;
    uint8_t selection_valid;
    uint8_t selection[ARM_MOTOR_NUM];
    uint8_t participating[ARM_MOTOR_NUM];
    uint32_t start_ms, phase_ms, retry_ms;
    uint8_t index;
    uint8_t disable_confirmed;
} Arm_zero;

/* MotorTask 每毫秒推进一次；忙时暂停该臂常规发送。 */
void Arm_Zero_Loop(Gimbal* g);
/* 查询设零请求或执行流程是否占用该臂。 */
uint8_t Arm_Zero_Busy(Gimbal* g);
