/* 主机测试直接调用固件命令层。 */
#include <string.h>
#include "../App/gimbal_cmd.c" // NOLINT(bugprone-suspicious-include)

rc_t rc;
static Robot_ctrl_cmd cmd;
static Arm arm;
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
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        ruckig_c_destroy(gimbal_cmd.joint_ruckig_ref[i].planner);
    memset(&gimbal, 0, sizeof(gimbal));
    memset(&gimbal_cmd, 0, sizeof(gimbal_cmd));
    memset(&cmd, 0, sizeof(cmd));
    gimbal_cmd.gimbal = &gimbal;
    gimbal_cmd.arm = &arm;
    gimbal.profile = &profile;
    profile.joints = joints;
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        joints[i].plan_velocity_scale = 1;
        joints[i].max_velocity_deg_s = 90;
        joints[i].max_acceleration_deg_s2 = 360;
        joints[i].max_jerk_deg_s3 = 3600;
        gimbal_cmd.joint_ruckig_ref[i].planner = ruckig_c_create(ARM_CONTROL_PERIOD_S);
    }
    for (int i = 0; i < 6; ++i) {
        gimbal.arm_feedback_data.joint_angle[i] = feedback[i];
    }
}

__attribute__((visibility("default")))
void reference_command_step(int mode, int planning, // NOLINT(readability-identifier-naming)
    const float* target, const float* velocity, const float* acc, float* output) {
    gimbal_cmd.mode = mode;
    gimbal_cmd.plan_running = planning;
    for (int i = 0; i < 6; ++i) {
        gimbal_cmd.ctrl_src.joint_angle_solved[i] = target[i];
        gimbal_cmd.plan_joint_vel[i] = velocity[i];
        gimbal_cmd.plan_joint_acc[i] = acc[i];
    }
    Ctrl_Post_Process(&gimbal_cmd);
    Arm_Ref_Set(&gimbal_cmd);
    for (int i = 0; i < 6; ++i) {
        output[i] = gimbal.arm_control_data.joint_angle[i];
        output[6+i] = gimbal.arm_control_data.joint_vel[i];
        output[12+i] = gimbal.arm_control_data.joint_acc[i];
    }
}
