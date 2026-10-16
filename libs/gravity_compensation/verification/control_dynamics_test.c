/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <assert.h>
#include <stdio.h>

/* 在主机验证控制层的静态函数；硬件收发函数由链接器回收。 */
#include "../../../App/gimbal_arm.c"
#include "../../../App/gimbal_cmd.c"

rc_t rc;
static Robot_ctrl_cmd test_command;
static uint8_t rc_offline = 1;

uint8_t is_Offline(monitor_item* item) {
    (void)item;
    return rc_offline;
}

const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) {
    (void)arm;
    return &test_command;
}

static void close_to(float actual, float expected) {
    assert(isfinite(actual));
    assert(fabsf(actual-expected) < 1e-4f);
}

static void reference_test(void) {
    Gimbal arm_a = {0}, arm_b = {0};
    Gimbal_cmd a = {0}, b = {0};
    TD_Config config = {.r = 1000.0f, .h = 0.002f, .h0 = 0.004f};
    a.gimbal = &arm_a;
    b.gimbal = &arm_b;
    a.mode = b.mode = arm_run;
    for (int i = 0; i < 6; ++i) {
        TD_Init(&a.joint_angle_td[i], &config);
        TD_Init(&b.joint_angle_td[i], &config);
        close_to(a.joint_angle_td[i].v3, 0.0f);
    }
    a.ctrl_src.joint_angle_solved[0] = 10.0f;
    a.ctrl_src.joint_angle_solved[1] = -10.0f;
    Ctrl_Post_Process(&a);
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_acc[0], 1000.0f);
    close_to(arm_a.arm_control_data.joint_acc[1], -1000.0f);
    close_to(arm_a.arm_control_data.joint_vel[0], 2.0f);
    close_to(arm_a.arm_control_data.joint_vel[1], -2.0f);
    Ctrl_Post_Process(&b);
    Arm_Ref_Set(&b);
    close_to(arm_b.arm_control_data.joint_acc[0], 0.0f);

    /* 外部速度通道的目标加速度为零。 */
    test_command.valid = test_command.has_vel = 1;
    test_command.joint_vel[0] = 80.0f;
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_vel[0], 80.0f);
    close_to(arm_a.arm_control_data.joint_acc[0], 0.0f);
    test_command.joint_vel[0] = 180.0f;
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_acc[0], 0.0f);
    rc_offline = 0;
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_acc[0], 1000.0f);
    rc_offline = 1;

    /* 规划通道使用规划器的解析加速度。 */
    a.plan_running = 1;
    a.plan_joint_acc[0] = -120.0f;
    a.plan_joint_vel[0] = 20.0f;
    Ctrl_Post_Process(&a);
    close_to(a.joint_angle_td[0].v3, 0.0f);
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_acc[0], -120.0f);
    close_to(arm_a.arm_control_data.joint_vel[0], 20.0f);
    a.plan_running = 0;
    test_command.has_vel = 0;
    Ctrl_Post_Process(&a);
    Arm_Ref_Set(&a);
    close_to(arm_a.arm_control_data.joint_acc[0], 0.0f);

    const Arm_mode passive_modes[] = {arm_stop, arm_damping, arm_only_gravity_comp};
    for (unsigned i = 0; i < sizeof(passive_modes)/sizeof(passive_modes[0]); ++i) {
        a.mode = passive_modes[i];
        a.plan_running = 1;
        Arm_Ref_Set(&a);
        close_to(arm_a.arm_control_data.joint_acc[0], 0.0f);
    }
    const float invalid_values[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(invalid_values)/sizeof(invalid_values[0]); ++i) {
        for (unsigned field = 0; field < 3; ++field) {
            a.mode = arm_run;
            a.plan_running = 1;
            arm_a.arm_control_data.mode = arm_run;
            a.ctrl_src.joint_angle_solved[5] = 0.0f;
            a.plan_joint_vel[5] = 0.0f;
            a.plan_joint_acc[5] = 0.0f;
            if (field == 0) a.ctrl_src.joint_angle_solved[5] = invalid_values[i];
            if (field == 1) a.plan_joint_vel[5] = invalid_values[i];
            if (field == 2) a.plan_joint_acc[5] = invalid_values[i];
            Arm_Ref_Set(&a);
            assert(a.mode == arm_damping && arm_a.arm_control_data.mode == arm_damping);
            for (int j = 0; j < ARM_JOINT_NUM; ++j) {
                close_to(arm_a.arm_control_data.joint_angle[j], arm_a.arm_feedback_data.joint_angle[j]);
                close_to(arm_a.arm_control_data.joint_vel[j], 0.0f);
                close_to(arm_a.arm_control_data.joint_acc[j], 0.0f);
            }
        }
    }
    a.mode = arm_run;
    a.plan_running = 0;
    test_command.has_vel = 1;
    test_command.joint_vel[0] = NAN;
    Arm_Ref_Set(&a);
    assert(a.mode == arm_damping && arm_a.arm_control_data.mode == arm_damping);
    close_to(arm_a.arm_control_data.joint_acc[0], 0.0f);
    test_command = (Robot_ctrl_cmd){0};
}

static void torque_test(void) {
    Gimbal arm = {0};
    controller motors[ARM_MOTOR_NUM] = {0};
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) arm.motor_controller_list[i] = &motors[i];
    arm.arm_mode = arm_run;
    arm.arm_feedback_data.joint_angle[1] = 40.0f;
    arm.arm_feedback_data.joint_angle[2] = 60.0f;
    arm.arm_feedback_data.joint_vel[0] = 35.0f;
    arm.arm_control_data.joint_acc[1] = 150.0f;
    arm.arm_control_data.gripper_torque = 0.3f;
    Joint_Dynamics_Comp_Calc(&arm);
    assert(arm.dynamics_status == 0);
    assert(fabsf(arm.tau_comp[0]) > 1e-4f);
    Set_Motor_ref(&arm);
    for (int i = 0; i < ARM_JOINT_NUM; ++i) close_to(motors[i].output_comp, arm.tau_comp[i]);
    close_to(motors[GRIPPER].output_comp, 0.3f);

    arm.arm_mode = arm_only_gravity_comp;
    Joint_Dynamics_Comp_Calc(&arm);
    close_to(arm.tau_comp[0], 0.0f);
    Set_Motor_ref(&arm);
    for (int i = 0; i < ARM_JOINT_NUM; ++i) close_to(motors[i].output_comp, arm.tau_comp[i]);

    arm.arm_mode = arm_damping;
    Joint_Dynamics_Comp_Calc(&arm);
    Set_Motor_ref(&arm);
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) close_to(motors[i].output_comp, 0.0f);

    arm.arm_mode = arm_run;
    arm.arm_control_data.joint_acc[0] = NAN;
    Joint_Dynamics_Comp_Calc(&arm);
    assert(arm.dynamics_status != 0 && arm.arm_mode == arm_stop);
    Set_Motor_ref(&arm);
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) close_to(motors[i].output_comp, 0.0f);
}

int main(void) {
    reference_test();
    torque_test();
    puts("目标加速度、模式切换、双臂隔离和异常保护验证通过");
}
