/* SPDX-License-Identifier: LGPL-2.1-or-later */
#include <assert.h>
#include <stdio.h>

/* 在主机验证控制层的静态函数；硬件收发函数由链接器回收。 */
#include "../../../App/gimbal_arm.c"
#include "../../../App/gimbal_cmd.c"

rc_t rc;
static Robot_ctrl_cmd test_command;
static uint8_t rc_offline = 1;

static const Gimbal* observed_arm;
static float previous_torque[ARM_JOINT_NUM];
static unsigned solver_observations;

int __real_x5_dynamics_compute(x5_model model, const x5_real q[6],
    const x5_real qd[6], const x5_real qdd[6], const x5_real gravity_base[3],
    x5_gravity_mode mode, x5_real torque[6]);

/* 求解期间，公开补偿值保持上一周期的完整结果。 */
int __wrap_x5_dynamics_compute(x5_model model, const x5_real q[6],
    const x5_real qd[6], const x5_real qdd[6], const x5_real gravity_base[3],
    x5_gravity_mode mode, x5_real torque[6]) {
    if (observed_arm) {
        assert(memcmp(observed_arm->tau_comp, previous_torque, sizeof(previous_torque)) == 0);
        assert(torque != observed_arm->tau_comp);
        ++solver_observations;
    }
    const int status = __real_x5_dynamics_compute(model, q, qd, qdd, gravity_base, mode, torque);
    if (observed_arm)
        assert(memcmp(observed_arm->tau_comp, previous_torque, sizeof(previous_torque)) == 0);
    return status;
}

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

static void Reference_Init(Gimbal_cmd* cmd, Gimbal* arm) {
    cmd->gimbal = arm;
    cmd->mode = arm_run;
    arm->arm_feedback_data.joint_angle[1] = 40.0f;
    arm->arm_feedback_data.joint_angle[2] = 60.0f;
    memcpy(cmd->ctrl_src.joint_angle_solved, arm->arm_feedback_data.joint_angle,
        sizeof(cmd->ctrl_src.joint_angle_solved));
    memcpy(cmd->ctrl_src.joint_src.joint_angle, arm->arm_feedback_data.joint_angle,
        sizeof(cmd->ctrl_src.joint_src.joint_angle));
    joint_angle_td_follow(cmd, arm->arm_feedback_data.joint_angle);
}

static void reference_test(void) {
    Gimbal arm_a = {0}, arm_b = {0};
    Gimbal_cmd a = {0}, b = {0};
    TD_Config config = {.r = 1000.0f, .h = 0.002f, .h0 = 0.004f};
    Reference_Init(&a, &arm_a);
    Reference_Init(&b, &arm_b);
    for (int i = 0; i < 6; ++i) {
        TD_Init(&a.joint_angle_td[i], &config);
        TD_Init(&b.joint_angle_td[i], &config);
        close_to(a.joint_angle_td[i].v3, 0.0f);
    }
    joint_angle_td_follow(&a, arm_a.arm_feedback_data.joint_angle);
    joint_angle_td_follow(&b, arm_b.arm_feedback_data.joint_angle);
    a.ctrl_src.joint_angle_solved[0] = 10.0f;
    a.ctrl_src.joint_angle_solved[3] = -10.0f;
    Ctrl_Post_Process(&a);
    TD_t td_before[ARM_JOINT_NUM];
    memcpy(td_before, a.joint_angle_td, sizeof(td_before));
    Arm_Ref_Set(&a);
    assert(memcmp(td_before, a.joint_angle_td, sizeof(td_before)) == 0);
    close_to(a.joint_angle_td[0].v3, 1000.0f);
    close_to(a.joint_angle_td[3].v3, -1000.0f);
    assert(arm_a.arm_control_data.joint_acc[0] > 0.0f);
    assert(arm_a.arm_control_data.joint_acc[0] < 1000.0f);
    assert(arm_a.arm_control_data.joint_acc[3] < 0.0f);
    assert(arm_a.arm_control_data.joint_acc[3] > -1000.0f);
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
    close_to(Quintic_Spline_Calc_dy(&a.joint_quintic_ref[0].curve, 0.0f), 180.0f);
    assert(isfinite(arm_a.arm_control_data.joint_acc[0]));
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
    close_to(Quintic_Spline_Calc_dy(&a.joint_quintic_ref[0].curve, 0.0f), 20.0f);
    close_to(Quintic_Spline_Calc_ddy(&a.joint_quintic_ref[0].curve, 0.0f), -120.0f);

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

static void quintic_reference_test(void) {
    Gimbal arm = {0};
    Gimbal_cmd cmd = {0};
    Reference_Init(&cmd, &arm);
    cmd.ctrl_src.joint_angle_solved[1] = 70.0f;
    Arm_Ref_Set(&cmd);
    assert(arm.arm_control_data.joint_angle[1] > 40.0f);
    assert(arm.arm_control_data.joint_acc[1] > 0.0f);
    assert(arm.arm_control_data.joint_acc[1] < 400.0f);
    close_to(Quintic_Spline_Calc_y(&cmd.joint_quintic_ref[1].curve, 0.0f), 40.0f);

    for (int i = 0; i < 40; ++i) Arm_Ref_Set(&cmd);
    const float position = arm.arm_control_data.joint_angle[1];
    const float velocity = arm.arm_control_data.joint_vel[1];
    const float acc = arm.arm_control_data.joint_acc[1];
    cmd.ctrl_src.joint_angle_solved[1] = 50.0f;
    Arm_Ref_Set(&cmd);
    close_to(Quintic_Spline_Calc_y(&cmd.joint_quintic_ref[1].curve, 0.0f), position);
    close_to(Quintic_Spline_Calc_dy(&cmd.joint_quintic_ref[1].curve, 0.0f), velocity);
    assert(fabsf(Quintic_Spline_Calc_ddy(&cmd.joint_quintic_ref[1].curve, 0.0f) - acc) < 1e-3f);
    const int settle_cycles = (int)ceilf(JOINT_QUINTIC_DURATION_S / GIMBAL_CMD_PERIOD_S) + 1;
    for (int i = 0; i < settle_cycles; ++i) Arm_Ref_Set(&cmd);
    close_to(arm.arm_control_data.joint_angle[1], 50.0f);
    close_to(arm.arm_control_data.joint_vel[1], 0.0f);
    close_to(arm.arm_control_data.joint_acc[1], 0.0f);

    cmd.mode = arm_damping;
    arm.arm_feedback_data.joint_angle[1] = 25.0f;
    cmd.joint_angle_td[1].v1 = 25.0f;
    cmd.joint_angle_td[1].v2 = 0.0f;
    Arm_Ref_Set(&cmd);
    cmd.mode = arm_run;
    cmd.ctrl_src.joint_angle_solved[1] = 30.0f;
    Arm_Ref_Set(&cmd);
    close_to(Quintic_Spline_Calc_y(&cmd.joint_quintic_ref[1].curve, 0.0f), 25.0f);

    for (int i = 0; i < 100; ++i) {
        cmd.ctrl_src.joint_angle_solved[1] += 0.18f;
        Arm_Ref_Set(&cmd);
        assert(cmd.mode == arm_run);
        assert(isfinite(arm.arm_control_data.joint_angle[1]));
        assert(isfinite(arm.arm_control_data.joint_vel[1]));
        assert(isfinite(arm.arm_control_data.joint_acc[1]));
    }
    assert(arm.arm_control_data.joint_angle[1] > 30.0f);
}

static void rc_enable_test(void) {
    Gimbal arm = {0};
    Gimbal_cmd cmd = {0};
    cmd.gimbal = &arm;
    cmd.ctrl_type = arm_ctrl_joint;
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) arm.arm_feedback_data.motor_inited[i] = 1;
    TD_Config config = {.r = 1000, .h = 0.002f, .h0 = 0.004f};
    for (int i = 0; i < ARM_JOINT_NUM; ++i) TD_Init(&cmd.joint_angle_td[i], &config);
    rc_offline = 0;
    rc.sw_a = 0;
    cmd.mode = arm_stop;
    Stop_Mode_Update(&cmd);
    Ctrl_Src_Calc(&cmd);
    Motor_Angle_Limit(&cmd);
    Ctrl_Post_Process(&cmd);
    Arm_Ref_Set(&cmd);
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        close_to(cmd.joint_quintic_ref[i].position, arm.arm_feedback_data.joint_angle[i]);
        close_to(cmd.joint_quintic_ref[i].velocity, 0);
        close_to(cmd.joint_quintic_ref[i].acc, 0);
    }
    rc.sw_a = 1;
    cmd.mode = arm_run;
    Run_Mode_Update(&cmd);
    Ctrl_Src_Calc(&cmd);
    Motor_Angle_Limit(&cmd);
    Ctrl_Post_Process(&cmd);
    Arm_Ref_Set(&cmd);
    assert(cmd.mode == arm_run && arm.arm_control_data.mode == arm_run);
    assert(arm.arm_control_data.joint_angle[1] >= 0.0f);
    assert(arm.arm_control_data.joint_angle[2] >= 0.0f);
    close_to(cmd.joint_quintic_ref[1].curve.poly.a1, 0);
    close_to(cmd.joint_quintic_ref[2].curve.poly.a1, 0);
    for (int k = 0; k < 1000; ++k) {
        Arm_Ref_Set(&cmd);
        assert(cmd.mode == arm_run);
    }
    close_to(arm.arm_control_data.joint_angle[1], JOINT_ANGLE_MIN[1]);
    close_to(arm.arm_control_data.joint_angle[2], JOINT_ANGLE_MIN[2]);
    rc_offline = 1;
}

static void rc_release_test(void) {
    const int hold_cycles[] = {25, 50, 100, 250, 500};
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned trial = 0; trial < sizeof(hold_cycles)/sizeof(hold_cycles[0]); ++trial) {
            Gimbal arm = {0};
            Gimbal_cmd cmd = {0};
            Reference_Init(&cmd, &arm);
            cmd.ctrl_type = arm_ctrl_joint;
            for (int i = 0; i < ARM_JOINT_NUM; ++i) arm.arm_feedback_data.motor_inited[i] = 1;
            rc_offline = 0;
            rc.sw_a = 1;
            rc.ry = direction;
            rc.lx = 0;
            for (int k = 0; k < hold_cycles[trial]; ++k) {
                Run_Mode_Update(&cmd);
                Ctrl_Src_Calc(&cmd);
                Motor_Angle_Limit(&cmd);
                Arm_Ref_Set(&cmd);
                assert(cmd.mode == arm_run);
            }
            rc.ry = 0;
            const float target = cmd.ctrl_src.joint_angle_solved[1];
            float overshoot = 0;
            for (int k = 0; k < 1200; ++k) {
                Run_Mode_Update(&cmd);
                Ctrl_Src_Calc(&cmd);
                Motor_Angle_Limit(&cmd);
                Arm_Ref_Set(&cmd);
                assert(cmd.mode == arm_run);
                const float excess = direction*(arm.arm_control_data.joint_angle[1] - target);
                if (excess > overshoot) overshoot = excess;
            }
            assert(overshoot < 1e-3f);
            close_to(arm.arm_control_data.joint_angle[1], target);
        }
    }
    rc_offline = 1;
}

static void pid_hold_test(void) {
    PID pid = {0};
    PID_SetConfig_Pos(&pid.config, 4, 0, 50, 200, 1000);
    pid.ref = 40.0f;
    pid.fdb = 40.05f;
    pid.error[0] = -0.1f;
    PID_Calc(&pid);
    assert(pid.d_error > 0.0f);
    PID_Calc(&pid);
    close_to(pid.d_error, 0.0f);
    assert(pid.output < 0.0f);
    for (int k = 0; k < 100; ++k) {
        PID_Calc(&pid);
        close_to(pid.output, pid.config.KP*(pid.ref - pid.fdb));
    }
    pid.config.D_ahead = D_AHEAD_USED;
    pid.d_error = -0.03f;
    PID_Calc(&pid);
    close_to(pid.d_error, -0.03f);
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
    for (int i = 0; i < ARM_JOINT_NUM; ++i) arm.tau_comp[i] = previous_torque[i] = 10.0f + i;
    observed_arm = &arm;
    Joint_Dynamics_Comp_Calc(&arm);
    observed_arm = NULL;
    assert(solver_observations == 1);
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

    arm.arm_mode = arm_run;
    arm.arm_control_data.joint_acc[0] = 0.0f;
    Joint_Dynamics_Comp_Calc(&arm);
    assert(arm.dynamics_status == X5_DYNAMICS_OK && arm.arm_mode == arm_run);
    assert(fabsf(arm.tau_comp[1]) > 1e-4f);
}

int main(void) {
    reference_test();
    quintic_reference_test();
    rc_enable_test();
    rc_release_test();
    pid_hold_test();
    torque_test();
    puts("五次样条输出、目标重规划、模式切换、TD 状态和异常保护验证通过");
}
