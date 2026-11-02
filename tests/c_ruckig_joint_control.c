/* Exercise the real firmware reference selection with the real C++ solver. */
#include <assert.h>
#include <stdio.h>
#include "../App/gimbal_cmd.c"

rc_t rc;
static uint8_t rc_offline = 1;
uint8_t is_Offline(monitor_item* item) { (void)item; return rc_offline; }
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
Plan_group_temp Plan_Group_Temp_Create(uint8_t count) {
    return calloc(count, sizeof(Move_plan*));
}

typedef struct {
    Arm arm;
    Gimbal gimbal;
    Arm_profile profile;
    Arm_joint_config joints[ARM_JOINT_NUM];
    Gimbal_cmd* cmd;
} Fixture;

static void init(Fixture* f) {
    memset(f, 0, sizeof(*f));
    f->arm.gimbal = &f->gimbal;
    f->gimbal.profile = &f->profile;
    f->profile.joints = f->joints;
    f->profile.position_min_duration_s = .16f;
    f->profile.rc_joint[0] = 0;
    f->profile.rc_joint[1] = f->profile.rc_joint[2] = -1;
    f->profile.rc_speed_deg_s = 90;
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        f->joints[i] = (Arm_joint_config){.td_r=1000, .plan_velocity_scale=1,
            .minimum_deg=-90, .maximum_deg=90, .max_velocity_deg_s=90,
            .max_acceleration_deg_s2=360, .max_jerk_deg_s3=3600};
        f->gimbal.arm_feedback_data.motor_inited[i] = 1;
        f->gimbal.arm_feedback_data.joint_angle[i] = i;
    }
    f->cmd = Gimbal_Cmd_Create(&f->arm);
    assert(f->cmd);
    f->cmd->mode = arm_run;
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        f->cmd->ctrl_src.joint_angle_solved[i] = i;
}

static void destroy(Fixture* f) {
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        ruckig_c_destroy(f->cmd->joint_ruckig_ref[i].planner);
    free(f->cmd->temp_group);
    free(f->cmd);
}

static void constrained_step(Fixture* f) {
    ruckig_c_state previous[ARM_JOINT_NUM];
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        previous[i] = f->cmd->joint_ruckig_ref[i].initialized
            ? f->cmd->joint_ruckig_ref[i].state
            : (ruckig_c_state){f->gimbal.arm_feedback_data.joint_angle[i], 0, 0};
    }
    Arm_Ref_Set(f->cmd);
    assert(f->cmd->mode == arm_run);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        const ruckig_c_state next = f->cmd->joint_ruckig_ref[i].state;
        const Arm_joint_config* j = &f->joints[i];
        assert(f->cmd->joint_ruckig_ref[i].status >= 0);
        assert(fabs(next.velocity) <= j->max_velocity_deg_s + 1e-7);
        assert(fabs(next.acceleration) <= j->max_acceleration_deg_s2 + 1e-7);
        assert(fabs(next.acceleration - previous[i].acceleration)
            <= j->max_jerk_deg_s3*ARM_CONTROL_PERIOD_S + 1e-5);
        assert(fabs(next.position - previous[i].position)
            <= j->max_velocity_deg_s*ARM_CONTROL_PERIOD_S + 1e-6);
        assert(f->gimbal.arm_control_data.joint_angle[i] == (float)next.position);
        assert(f->gimbal.arm_control_data.joint_vel[i] == (float)next.velocity);
        assert(f->gimbal.arm_control_data.joint_acc[i] == (float)next.acceleration);
    }
}

static void position_replan_isolation(void) {
    Fixture a, b;
    init(&a); init(&b);
    a.cmd->ctrl_src.joint_angle_solved[0] = 60;
    a.cmd->ctrl_src.joint_angle_solved[1] = -40;
    b.cmd->ctrl_src.joint_angle_solved[0] = -30;
    a.joints[1].max_velocity_deg_s = 20;
    for (int k=0; k<100; ++k) {
        constrained_step(&a); constrained_step(&b);
    }
    assert(a.gimbal.arm_control_data.joint_vel[0] > 0);
    assert(b.gimbal.arm_control_data.joint_vel[0] < 0);
    a.cmd->ctrl_src.joint_angle_solved[0] = -10;
    for (int k=0; k<2500; ++k) {
        constrained_step(&a); constrained_step(&b);
    }
    assert(a.gimbal.arm_control_data.joint_angle[0] == -10);
    assert(a.gimbal.arm_control_data.joint_angle[1] == -40);
    assert(b.gimbal.arm_control_data.joint_angle[0] == -30);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(a.cmd->joint_ruckig_ref[i].status == RUCKIG_C_FINISHED);
        assert(a.gimbal.arm_control_data.joint_vel[i] == 0);
        assert(a.gimbal.arm_control_data.joint_acc[i] == 0);
    }
    /* Other axes keep independent trajectories while a target streams in. */
    for (int k=0; k<1000; ++k) {
        a.cmd->ctrl_src.joint_angle_solved[0] = 25*sinf(k*.01f);
        constrained_step(&a);
        assert(a.gimbal.arm_control_data.joint_angle[1] == -40);
    }
    destroy(&a); destroy(&b);
}

static void source_transitions(void) {
    Fixture f; init(&f);
    /* External velocity still uses TD position and protocol velocity. */
    f.arm.ctrl.valid = f.arm.ctrl.has_vel = 1;
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        f.arm.ctrl.joint_vel[i] = 15;
        f.cmd->joint_angle_td[i].v1 = 20;
    }
    Arm_Ref_Set(f.cmd);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(f.gimbal.arm_control_data.joint_angle[i] == 20);
        assert(f.gimbal.arm_control_data.joint_vel[i] == 15);
        assert(f.gimbal.arm_control_data.joint_acc[i] == 0);
        f.cmd->ctrl_src.joint_angle_solved[i] = 40;
    }
    f.arm.ctrl.has_vel = 0;
    constrained_step(&f);
    assert(f.gimbal.arm_control_data.joint_angle[0] > 20);
    /* Automatic motion keeps its own derivatives and velocity scale. */
    f.cmd->plan_running = 1;
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        f.cmd->ctrl_src.joint_angle_solved[i] = 10;
        f.cmd->plan_joint_vel[i] = 5;
        f.cmd->plan_joint_acc[i] = -30;
        f.joints[i].plan_velocity_scale = 2;
    }
    Arm_Ref_Set(f.cmd);
    assert(f.gimbal.arm_control_data.joint_angle[0] == 10);
    assert(f.gimbal.arm_control_data.joint_vel[0] == 10);
    assert(f.gimbal.arm_control_data.joint_acc[0] == -30);
    f.cmd->plan_running = 0;
    constrained_step(&f);
    /* STOP, DAMPING and gravity mode reset the position planner to feedback. */
    const Arm_mode modes[] = {arm_stop, arm_damping, arm_only_gravity_comp};
    for (unsigned m=0; m<3; ++m) {
        f.cmd->mode = modes[m];
        joint_angle_td_follow(f.cmd, f.gimbal.arm_feedback_data.joint_angle);
        Arm_Ref_Set(f.cmd);
        for (int i=0; i<ARM_JOINT_NUM; ++i) {
            assert(f.cmd->joint_ruckig_ref[i].state.position == i);
            assert(f.cmd->joint_ruckig_ref[i].state.velocity == 0);
            assert(f.cmd->joint_ruckig_ref[i].state.acceleration == 0);
        }
        f.cmd->mode = arm_run;
        constrained_step(&f);
        assert(f.gimbal.arm_control_data.joint_angle[0] < .001f);
    }
    destroy(&f);
}

static void errors_and_rc(void) {
    Fixture f; init(&f);
    f.cmd->ctrl_src.joint_angle_solved[0] = 50;
    for (int k=0; k<50; ++k) constrained_step(&f);
    /* Failure on the last axis must also clear already-computed earlier axes. */
    f.joints[ARM_JOINT_NUM-1].max_jerk_deg_s3 = 0;
    Arm_Ref_Set(f.cmd);
    assert(f.cmd->mode == arm_damping && f.gimbal.arm_control_data.mode == arm_damping);
    assert(f.cmd->joint_ruckig_ref[ARM_JOINT_NUM-1].status < 0);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(f.gimbal.arm_control_data.joint_angle[i] == i);
        assert(f.gimbal.arm_control_data.joint_vel[i] == 0);
        assert(f.gimbal.arm_control_data.joint_acc[i] == 0);
        assert(!f.cmd->joint_ruckig_ref[i].initialized);
    }
    f.joints[ARM_JOINT_NUM-1].max_jerk_deg_s3 = 3600;
    f.cmd->mode = arm_run;
    constrained_step(&f);
    f.cmd->ctrl_src.joint_angle_solved[2] = NAN;
    Arm_Ref_Set(f.cmd);
    assert(f.cmd->mode == arm_damping);
    f.cmd->ctrl_src.joint_angle_solved[2] = 2;
    f.cmd->mode = arm_run;
    constrained_step(&f);
    /* The RC position path also reaches Ruckig through the existing limits. */
    rc_offline = 0;
    rc.sw_a = 1; rc.rx = 1;
    for (int k=0; k<300; ++k) {
        Run_Mode_Update(f.cmd);
        Ctrl_Src_Calc(f.cmd);
        Motor_Angle_Limit(f.cmd);
        constrained_step(&f);
    }
    rc.rx = 0;
    const float target = f.cmd->ctrl_src.joint_angle_solved[0];
    for (int k=0; k<2000; ++k) {
        Run_Mode_Update(f.cmd);
        Ctrl_Src_Calc(f.cmd);
        Motor_Angle_Limit(f.cmd);
        constrained_step(&f);
    }
    assert(f.gimbal.arm_control_data.joint_angle[0] == target);
    rc_offline = 1;
    destroy(&f);
}

int main(void) {
    position_replan_isolation();
    source_transitions();
    errors_and_rc();
    puts("Joint Ruckig: bounds, continuous replanning, sources, faults, RC and isolation passed");
    return 0;
}
