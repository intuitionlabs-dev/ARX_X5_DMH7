#pragma once
#include "arm_profile.h"
#include "arm_zero.h"

/* 保存单臂电机实例、反馈、参考值及动力学工作区。 */
typedef struct Gimbal_t {
    const Arm_profile* profile;
    uint8_t can_index;
    Arm_motor motors[ARM_MOTOR_NUM];
    uint8_t first_init_flag;
    float motor_pos_init[ARM_JOINT_NUM];  // 电机零点，deg
    Arm_mode arm_mode;
    float damp_comp[ARM_JOINT_NUM];  // 粘性摩擦补偿，N·m
    float tau_comp[ARM_JOINT_NUM];   // 逆动力学力矩，N·m
    Arm_feedback_data arm_feedback_data;
    Arm_control_data arm_control_data;
    Arm_zero zero;
    Arm_dynamics_work dynamics_work[ARM_JOINT_NUM];
    int dynamics_status;
} Gimbal;

/* 根据板级绑定创建指定机械臂控制对象。 */
Gimbal* Gimbal_Create(uint8_t arm_index);
/* 使用指定机型和 CAN 总线创建控制对象。 */
Gimbal* Gimbal_Create_Profile(const Arm_profile* profile, uint8_t can_index);
/* 更新反馈、补偿和电机参考值。 */
void Gimbal_Update(Gimbal* obj);
