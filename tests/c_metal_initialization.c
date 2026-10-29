/* Exercise production state/command/planning code with simulated feedback and time. */
#include <string.h>
#include "arm.h"
#include "bsp_can.h"
#include "bsp_supervise.h"
#include "move_plan.h"
/* Host tests are single-threaded; replace only the Cortex-M memory barrier. */
#define __DMB() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#include "../App/robot_state.c" // NOLINT(bugprone-suspicious-include)
#define Rc_Online Command_Rc_Online
#include "../App/gimbal_cmd.c" // NOLINT(bugprone-suspicious-include)
#undef Rc_Online
#include "../App/arm_auto.c" // NOLINT(bugprone-suspicious-include)

#define EXPORT __attribute__((visibility("default")))
Arm g_arm[ARM_NUM];
rc_t rc;
static Gimbal gimbals[ARM_NUM];
static monitor_item rc_monitor;
static uint32_t clock_ms;

uint32_t HAL_GetTick(void) { return clock_ms; }

EXPORT void init_fixture(void) { // NOLINT(readability-identifier-naming)
    /* Finish prior test trajectories so the real temporary-plan pool can reuse them. */
    clock_ms += 10000;
    Move_Plan_Update();
    Move_Plan_Init();
    for (unsigned bus=0; bus<ARM_NUM; ++bus) {
        Gimbal_cmd* cmd = g_arm[bus].gimbal_cmd;

        memset(&gimbals[bus], 0, sizeof(Gimbal));
        memset(&g_arm[bus], 0, sizeof(Arm));
        Arm* arm = &g_arm[bus];
        arm->id = bus;
        arm->gimbal = &gimbals[bus];
        arm->gimbal->profile = &ARM_PROFILE_METAL;
        arm->state = robot_state_wait_init;
        if (cmd) {
            Plan_group_temp temp_group = cmd->temp_group;
            memset(cmd, 0, sizeof(*cmd));
            cmd->temp_group = temp_group;
            cmd->arm = arm;
            cmd->gimbal = arm->gimbal;

            const float r[] = {1000,4000,4000,2000,2000,2000};
            for (unsigned i=0; i<6; ++i) {
                TD_Config cfg = {.r=r[i], .h=.002f, .h0=.004f};
                TD_Init(&cmd->joint_angle_td[i], &cfg);
            }
        } else cmd = Gimbal_Cmd_Create(arm);
        arm->gimbal_cmd = cmd;
        memset(arm->gimbal->arm_feedback_data.motor_inited, 1, ARM_JOINT_NUM);
    }
    memset(&rc, 0, sizeof(rc));
    memset(&rc_monitor, 0, sizeof(rc_monitor));
    rc.monitor = &rc_monitor;
}

EXPORT void init_feedback(unsigned bus, const float* angles) { // NOLINT(readability-identifier-naming)
    memcpy(gimbals[bus].arm_feedback_data.joint_angle, angles, 6*sizeof(float));
    gimbals[bus].arm_feedback_data.gripper_angle = angles[6];
}

EXPORT void init_entry(unsigned bus, unsigned state, unsigned mode, unsigned lost_mask, // NOLINT(readability-identifier-naming)
                       unsigned rc_online) {
    Arm* arm = &g_arm[bus];
    arm->state = state;
    arm->arm_mode = mode;
    rc_monitor.count = rc_online;
    for (unsigned i=0; i<7; ++i) arm->gimbal->arm_feedback_data.is_motor_lost[i] = (lost_mask >> i)&1;
    /* Accepted init must discard the old command and gripper target. */
    arm->ctrl.valid = 1;
    arm->ctrl.gripper_angle = 999;
    arm->gimbal->arm_control_data.gripper_angle = 999;
    HandleArmInit(arm);
}

EXPORT void init_motor_lost(unsigned bus, unsigned index) { // NOLINT(readability-identifier-naming)
    gimbals[bus].arm_feedback_data.is_motor_lost[index] = 1;
}

EXPORT void init_tick(unsigned rc_online) { // NOLINT(readability-identifier-naming)
    clock_ms += 2;
    rc_monitor.count = rc_online;
    Move_Plan_Update();
    for (unsigned bus=0; bus<ARM_NUM; ++bus) {
        Arm* arm = &g_arm[bus];
        if (arm->state == robot_state_initing) Init_Step(arm);
        if (arm->state == robot_state_resetting) Reset_Step(arm);
        Gimbal_Cmd_Update(arm->gimbal_cmd);
    }
}

EXPORT void init_observe(unsigned bus, unsigned* state, float* output) { // NOLINT(readability-identifier-naming)
    Arm* arm = &g_arm[bus];
    Gimbal* g = arm->gimbal;
    const unsigned fields[] = {arm->state, arm->arm_mode, arm->init_step,
        arm->gimbal_cmd->auto_mode, g->arm_feedback_data.motor_inited[GRIPPER],
        g->zero.request, g->zero.state, arm->ctrl.valid,
        arm->init_reject_cnt, arm->init_abort_cnt};
    memcpy(state, fields, sizeof(fields));
    memcpy(output, g->arm_control_data.joint_angle, 6*sizeof(float));
    memcpy(output+6, g->arm_control_data.joint_vel, 6*sizeof(float));
    memcpy(output+12, g->arm_control_data.joint_acc, 6*sizeof(float));
    output[18] = g->arm_control_data.gripper_angle;
    output[19] = g->arm_control_data.gripper_torque;
}

EXPORT void init_limits(const float* input, unsigned mask, float* out) { // NOLINT(readability-identifier-naming)
    Gimbal_cmd* cmd = g_arm[0].gimbal_cmd;
    for (unsigned i=0; i<7; ++i) cmd->gimbal->arm_feedback_data.motor_inited[i] = (mask >> i)&1;
    memcpy(cmd->ctrl_src.joint_angle_solved, input, 6*sizeof(float));
    cmd->gimbal->arm_control_data.gripper_angle = input[6];
    Motor_Angle_Limit(cmd);
    memcpy(out, cmd->ctrl_src.joint_angle_solved, 6*sizeof(float));
    out[6] = cmd->gimbal->arm_control_data.gripper_angle;
    memcpy(out+7, cmd->ctrl_src.joint_src.joint_angle, 6*sizeof(float));
}

EXPORT void init_reset(unsigned bus, float gripper) { // NOLINT(readability-identifier-naming)
    Arm* arm = &g_arm[bus];
    arm->state = robot_state_normal;
    arm->arm_mode = arm_damping;
    arm->gimbal->arm_control_data.gripper_angle = gripper;
    HandleArmReset(arm);
}

void FrameRateStatistics(FPS_t* fps) { fps->now = clock_ms; fps->count++; }
void BSP_CAN_RegisterRxCallback(uint8_t bus, can_rx_callback fn) { (void)bus; (void)fn; }
void BSP_CAN_AddFilter(uint8_t bus, uint16_t id) { (void)bus; (void)id; }
uint8_t BSP_CAN_TxGroupMatch(uint8_t group) { return (group >> (clock_ms % 8)) & 1; }
void BSP_CAN_Send(uint8_t bus, uint16_t id, uint8_t* data, basic_data_t len) {
    (void)bus; (void)id; (void)data; (void)len;
}
