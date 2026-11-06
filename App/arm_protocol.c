#include "arm_protocol.h"
#include <math.h>
#include <string.h>

/* 校验控制载荷并写入机械臂控制目标。 */
int Arm_Protocol_Control(Arm* arm, const Robot_rx_payload* payload, uint8_t has_velocity) {
    const float* position = has_velocity ? payload->arm_ctrl_pos_vel.joint_angle
        : payload->arm_ctrl_pos.joint_angle;
    const float* velocity = payload->arm_ctrl_pos_vel.joint_vel;
    const float grip = has_velocity ? payload->arm_ctrl_pos_vel.gripper_angle
        : payload->arm_ctrl_pos.gripper_angle;
    /* 夹爪目标为非有限值时拒绝报文。 */
    if (!isfinite(grip)) return 0;
    /* 校验协议槽位内的关节位置和速度。 */
    for (int i = 0; i < ARM_JOINT_NUM && i < FRAME_PKT_JOINT_NUM; ++i)
        /* 关节目标无效时拒绝整条控制命令。 */
        if (!isfinite(position[i]) || (has_velocity && !isfinite(velocity[i]))) return 0;
    /* 按关节索引复制位置和可选速度目标。 */
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        arm->ctrl.joint_angle[i] = i < FRAME_PKT_JOINT_NUM ? position[i]
            : arm->gimbal->arm_feedback_data.joint_angle[i];
        arm->ctrl.joint_vel[i] = has_velocity && i < FRAME_PKT_JOINT_NUM ? velocity[i] : 0;
    }
    arm->ctrl.gripper_angle = grip;
    arm->ctrl.has_vel = has_velocity;
    arm->ctrl.valid = 1;
    return 1;
}

/* 将机械臂状态转换为反馈报文载荷。 */
void Arm_Protocol_Feedback(const Arm* arm, FramePktArmFeedback* output) {
    memset(output, 0, sizeof(*output));
    output->mode = arm->gimbal ? arm->gimbal->arm_feedback_data.mode : arm->arm_mode;
    output->robot_state = arm->state;
    /* 控制对象尚未创建时仅返回模式和状态。 */
    if (!arm->gimbal) return;
    const Arm_feedback_data* feedback = &arm->gimbal->arm_feedback_data;
    /* 按协议槽位生成电机掩码和温度。 */
    for (int slot = 0; slot < FRAME_PKT_MOTOR_NUM; ++slot) {
        const int motor = slot == FRAME_PKT_JOINT_NUM ? GRIPPER : slot;
        /* 夹爪始终占据协议的第七个电机槽位。 */
        /* 跳过超出关节配置的普通槽位。 */
        if (slot != FRAME_PKT_JOINT_NUM && slot >= ARM_JOINT_NUM) continue;
        output->motor_lost_mask |= (uint8_t)(!!feedback->is_motor_lost[motor] << slot);
        output->motor_inited_mask |= (uint8_t)(!!feedback->motor_inited[motor] << slot);
        output->motor_temp[slot] = feedback->motor_temp[motor];
    }
    /* 复制协议关节槽位内的位置和速度。 */
    for (int i = 0; i < ARM_JOINT_NUM && i < FRAME_PKT_JOINT_NUM; ++i) {
        output->joint_angle[i] = feedback->joint_angle[i];
        output->joint_vel[i] = feedback->joint_vel[i];
    }
    output->gripper_angle = feedback->gripper_angle;
}

/* 将设零状态和各电机结果转换为应答载荷。 */
void Arm_Protocol_Zero_Result(const Gimbal* g, FramePktMotorSetZeroAck* output) {
    memset(output, 0, sizeof(*output));
    output->state = g->zero.state;
    output->elapsed_ms = g->zero.elapsed_ms;
    /* 按协议槽位填充各电机设零结果。 */
    for (int slot = 0; slot < FRAME_PKT_MOTOR_NUM; ++slot) {
        const int motor = slot == FRAME_PKT_JOINT_NUM ? GRIPPER : slot;
        output->result[slot] = slot != FRAME_PKT_JOINT_NUM && slot >= ARM_JOINT_NUM
            ? ARM_ZERO_RES_SKIP : g->zero.result[motor];
    }
}
