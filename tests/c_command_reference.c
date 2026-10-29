/* 主机测试直接调用固件命令层。 */
#include <string.h>
#include "../App/gimbal_cmd.c" // NOLINT(bugprone-suspicious-include)

rc_t rc;
static Robot_ctrl_cmd cmd;
static Gimbal gimbal;
static Gimbal_cmd gimbal_cmd;
static Arm_profile profile;
static Arm_joint_config joints[ARM_JOINT_NUM];

uint8_t is_Offline(monitor_item* item) {
    (void)item;
    return 1;
}

const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) {
    (void)arm;
    return &cmd;
}

__attribute__((visibility("default")))
void reference_command_init(const float* feedback) { // NOLINT(readability-identifier-naming)
    memset(&gimbal, 0, sizeof(gimbal));
    memset(&gimbal_cmd, 0, sizeof(gimbal_cmd));
    memset(&cmd, 0, sizeof(cmd));
    gimbal_cmd.gimbal = &gimbal;
    gimbal.profile = &profile;
    profile.joints = joints;
    profile.quintic_duration_s = .16f;
    for (int i=0; i<ARM_JOINT_NUM; ++i) joints[i].plan_velocity_scale = 1;
    const float r[6] = {1000, 4000, 4000, 2000, 2000, 2000};
    for (int i = 0; i < 6; ++i) {
        TD_Config config = {.r = r[i], .h = .002f, .h0 = .004f};
        TD_Init(&gimbal_cmd.joint_angle_td[i], &config);
        gimbal.arm_feedback_data.joint_angle[i] = feedback[i];
    }
}

__attribute__((visibility("default")))
void reference_command_step(int mode, int planning, int external_velocity, // NOLINT(readability-identifier-naming)
    const float* target, const float* velocity, const float* acc, float* output) {
    gimbal_cmd.mode = mode;
    gimbal_cmd.plan_running = planning;
    cmd.valid = cmd.has_vel = external_velocity;
    for (int i = 0; i < 6; ++i) {
        gimbal_cmd.ctrl_src.joint_angle_solved[i] = target[i];
        gimbal_cmd.plan_joint_vel[i] = velocity[i];
        gimbal_cmd.plan_joint_acc[i] = acc[i];
        cmd.joint_vel[i] = velocity[i];
    }
    if (mode != arm_run) joint_angle_td_follow(&gimbal_cmd, gimbal.arm_feedback_data.joint_angle);
    Ctrl_Post_Process(&gimbal_cmd);
    Arm_Ref_Set(&gimbal_cmd);
    for (int i = 0; i < 6; ++i) {
        output[i] = gimbal.arm_control_data.joint_angle[i];
        output[6+i] = gimbal.arm_control_data.joint_vel[i];
        output[12+i] = gimbal.arm_control_data.joint_acc[i];
    }
}
