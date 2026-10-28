/* 主机测试直接调用固件命令层。 */
#include <string.h>
#include "../App/gimbal_cmd.c"

rc_t rc;
static Robot_ctrl_cmd command;
static Gimbal gimbal;
static Gimbal_cmd cmd;

uint8_t is_Offline(monitor_item* item) {
    (void)item;
    return 1;
}

const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) {
    (void)arm;
    return &command;
}

__attribute__((visibility("default")))
void reference_command_init(const float* feedback) {
    memset(&gimbal, 0, sizeof(gimbal));
    memset(&cmd, 0, sizeof(cmd));
    memset(&command, 0, sizeof(command));
    cmd.gimbal = &gimbal;
    const float r[6] = {1000, 4000, 4000, 2000, 2000, 2000};
    for (int i = 0; i < 6; ++i) {
        TD_Config config = {.r = r[i], .h = .002f, .h0 = .004f};
        TD_Init(&cmd.joint_angle_td[i], &config);
        gimbal.arm_feedback_data.joint_angle[i] = feedback[i];
    }
}

__attribute__((visibility("default")))
void reference_command_step(int mode, int planning, int external_velocity,
    const float* target, const float* velocity, const float* acc, float* output) {
    cmd.mode = mode;
    cmd.plan_running = planning;
    command.valid = command.has_vel = external_velocity;
    for (int i = 0; i < 6; ++i) {
        cmd.ctrl_src.joint_angle_solved[i] = target[i];
        cmd.plan_joint_vel[i] = velocity[i];
        cmd.plan_joint_acc[i] = acc[i];
        command.joint_vel[i] = velocity[i];
    }
    if (mode != arm_run) joint_angle_td_follow(&cmd, gimbal.arm_feedback_data.joint_angle);
    Ctrl_Post_Process(&cmd);
    Arm_Ref_Set(&cmd);
    for (int i = 0; i < 6; ++i) {
        output[i] = gimbal.arm_control_data.joint_angle[i];
        output[6+i] = gimbal.arm_control_data.joint_vel[i];
        output[12+i] = gimbal.arm_control_data.joint_acc[i];
    }
}
