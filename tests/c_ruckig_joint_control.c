/* Exercise the real firmware reference selection with the real C++ solver. */
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include "../App/gimbal_cmd.c"
#include "ruckig_c.h"

/* Linker wrappers verify startup failure cleanup and allocation-free updates. */
void* __real_malloc(size_t size);
void* __real_calloc(size_t count, size_t size);
void* __real_realloc(void* memory, size_t size);
void __real_free(void* memory);
static int allocation_budget = -1;
static int live_allocations;
static int updating;
static unsigned update_calls, new_calculations;
int __real_ruckig_c_update(ruckig_c_context* context, const ruckig_c_input* input,
                          ruckig_c_output* output);
int __wrap_ruckig_c_update(ruckig_c_context* context, const ruckig_c_input* input,
                          ruckig_c_output* output) {
    ++update_calls;
    const int status = __real_ruckig_c_update(context, input, output);
    if (status >= 0 && output->new_calculation) ++new_calculations;
    return status;
}
void* __wrap_malloc(size_t size) {
    assert(!updating);
    if (allocation_budget == 0) return NULL;
    if (allocation_budget > 0) --allocation_budget;
    void* memory = __real_malloc(size);
    if (memory) ++live_allocations;
    return memory;
}
void* __wrap_calloc(size_t count, size_t size) {
    assert(!updating);
    if (allocation_budget == 0) return NULL;
    if (allocation_budget > 0) --allocation_budget;
    void* memory = __real_calloc(count, size);
    if (memory) ++live_allocations;
    return memory;
}
void* __wrap_realloc(void* memory, size_t size) {
    assert(!updating);
    return __real_realloc(memory, size);
}
void __wrap_free(void* memory) {
    assert(!updating);
    if (memory) --live_allocations;
    __real_free(memory);
}

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

static void init_profile(Fixture* f, Arm_post_process_algorithm algorithm, float duration_s) {
    memset(f, 0, sizeof(*f));
    f->arm.gimbal = &f->gimbal;
    f->gimbal.profile = &f->profile;
    f->profile.joints = f->joints;
    f->profile.joint_post_process = algorithm;
    f->profile.rc_joint[0] = 0;
    f->profile.rc_joint[1] = f->profile.rc_joint[2] = -1;
    f->profile.rc_speed_deg_s = 90;
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        f->joints[i] = (Arm_joint_config){.plan_velocity_scale=1,
            .minimum_deg=-90, .maximum_deg=90, .quintic_duration_s=duration_s,
            .max_velocity_deg_s=90,
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

static void init(Fixture* f) {
    init_profile(f, ARM_POST_PROCESS_RUCKIG, 0);
}

static void destroy(Fixture* f) {
    if (f->profile.joint_post_process == ARM_POST_PROCESS_RUCKIG) {
        for (int i=0; i<ARM_JOINT_NUM; ++i)
            ruckig_c_destroy(f->cmd->joint_post_ref[i].trajectory.ruckig);
    }
    free(f->cmd->temp_group);
    free(f->cmd);
}

static void reference_step(Fixture* f) {
    updating = 1;
    Ctrl_Post_Process(f->cmd);
    Arm_Ref_Set(f->cmd);
    updating = 0;
}

static void constrained_step(Fixture* f) {
    Joint_motion_state previous[ARM_JOINT_NUM];
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        previous[i] = f->cmd->joint_post_ref[i].initialized
            ? f->cmd->joint_post_ref[i].state
            : (Joint_motion_state){f->gimbal.arm_feedback_data.joint_angle[i], 0, 0};
    }
    reference_step(f);
    assert(f->cmd->mode == arm_run);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        const Joint_motion_state next = f->cmd->joint_post_ref[i].state;
        const Arm_joint_config* j = &f->joints[i];
        assert(f->cmd->joint_post_ref[i].status >= 0);
        const float v_roundoff = 16*FLT_EPSILON*fmaxf(1, j->max_velocity_deg_s);
        const float a_roundoff = 16*FLT_EPSILON*fmaxf(1, j->max_acceleration_deg_s2);
        /* Sampling time is float; at most a few ulps at the trajectory duration. */
        const float dt_roundoff = 4*FLT_EPSILON*10.0f;
        assert(fabsf(next.velocity) <= j->max_velocity_deg_s + v_roundoff);
        assert(fabsf(next.acceleration) <= j->max_acceleration_deg_s2 + a_roundoff);
        assert(fabsf(next.acceleration - previous[i].acceleration)
            <= j->max_jerk_deg_s3*(ARM_CONTROL_PERIOD_S + dt_roundoff) + a_roundoff);
        assert(fabsf(next.position - previous[i].position)
            <= j->max_velocity_deg_s*(ARM_CONTROL_PERIOD_S + dt_roundoff)
                + 8*FLT_EPSILON*fmaxf(1, fabsf(previous[i].position)));
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
        assert(a.cmd->joint_post_ref[i].status == RUCKIG_C_FINISHED);
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

static void short_position_move(void) {
    Fixture f; init(&f);
    f.cmd->ctrl_src.joint_angle_solved[0] = .01f;
    unsigned steps = 0;
    do {
        constrained_step(&f);
        ++steps;
    } while (steps < 80 && f.cmd->joint_post_ref[0].status != RUCKIG_C_FINISHED);
    /* 小角度运动在 80 个控制周期内完成。 */
    assert(steps < 80);
    assert(f.cmd->joint_post_ref[0].status == RUCKIG_C_FINISHED);
    assert(f.gimbal.arm_control_data.joint_angle[0] == .01f);
    assert(f.gimbal.arm_control_data.joint_vel[0] == 0);
    assert(f.gimbal.arm_control_data.joint_acc[0] == 0);
    destroy(&f);
}

static void source_transitions(void) {
    Fixture f; init(&f);
    /* 规划参考逐周期同步到 Ruckig，速度使用机型比例系数。 */
    f.cmd->plan_running = 1;
    const unsigned calls_before_plan = update_calls;
    for (int k=0; k<5; ++k) {
        for (int i=0; i<ARM_JOINT_NUM; ++i) {
            f.cmd->ctrl_src.joint_angle_solved[i] = 10 + k*.02f + i;
            f.cmd->plan_joint_vel[i] = 5 + k;
            f.cmd->plan_joint_acc[i] = -30 + k;
            f.joints[i].plan_velocity_scale = 2;
        }
        reference_step(&f);
        assert(f.cmd->mode == arm_run);
        assert(update_calls == calls_before_plan);
        for (int i=0; i<ARM_JOINT_NUM; ++i) {
            const Joint_motion_state state = f.cmd->joint_post_ref[i].state;
            assert(f.cmd->post_processed_joint_angle[i] == f.cmd->ctrl_src.joint_angle_solved[i]);
            assert(f.cmd->post_processed_joint_vel[i] == f.cmd->plan_joint_vel[i]*2);
            assert(f.cmd->post_processed_joint_acc[i] == f.cmd->plan_joint_acc[i]);
            assert(state.position == f.gimbal.arm_control_data.joint_angle[i]);
            assert(state.velocity == f.gimbal.arm_control_data.joint_vel[i]);
            assert(state.acceleration == f.gimbal.arm_control_data.joint_acc[i]);
        }
    }
    f.cmd->plan_running = 0;
    const unsigned calculations_before_resume = new_calculations;
    constrained_step(&f);
    assert(update_calls == calls_before_plan+ARM_JOINT_NUM);
    assert(new_calculations == calculations_before_resume+ARM_JOINT_NUM);
    /* STOP, DAMPING and gravity mode reset the position planner to feedback. */
    const Arm_mode modes[] = {arm_stop, arm_damping, arm_only_gravity_comp};
    for (unsigned m=0; m<3; ++m) {
        f.cmd->mode = modes[m];
        reference_step(&f);
        for (int i=0; i<ARM_JOINT_NUM; ++i) {
            assert(f.cmd->joint_post_ref[i].state.position == i);
            assert(f.cmd->joint_post_ref[i].state.velocity == 0);
            assert(f.cmd->joint_post_ref[i].state.acceleration == 0);
        }
        f.cmd->mode = arm_run;
        constrained_step(&f);
        assert(f.gimbal.arm_control_data.joint_angle[0] < .001f);
    }
    destroy(&f);
}

static void post_process_and_publication(void) {
    Fixture f; init(&f);
    f.cmd->ctrl_src.joint_angle_solved[0] = 30;
    const unsigned before = update_calls;
    updating = 1;
    Ctrl_Post_Process(f.cmd);
    assert(f.cmd->mode == arm_run);
    assert(update_calls == before+ARM_JOINT_NUM);
    assert(f.gimbal.arm_control_data.joint_angle[0] == 0);
    assert(f.cmd->post_processed_joint_angle[0] > 0);
    Joint_motion_state states[ARM_JOINT_NUM];
    for (int i=0; i<ARM_JOINT_NUM; ++i) states[i] = f.cmd->joint_post_ref[i].state;
    Arm_Ref_Set(f.cmd);
    Arm_Ref_Set(f.cmd);
    assert(update_calls == before+ARM_JOINT_NUM);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(memcmp(&states[i], &f.cmd->joint_post_ref[i].state, sizeof(states[i])) == 0);
        assert(f.gimbal.arm_control_data.joint_angle[i] == states[i].position);
        assert(f.gimbal.arm_control_data.joint_vel[i] == states[i].velocity);
        assert(f.gimbal.arm_control_data.joint_acc[i] == states[i].acceleration);
        assert(f.gimbal.arm_control_data.joint_angle[i] == f.cmd->post_processed_joint_angle[i]);
        assert(f.gimbal.arm_control_data.joint_vel[i] == f.cmd->post_processed_joint_vel[i]);
        assert(f.gimbal.arm_control_data.joint_acc[i] == f.cmd->post_processed_joint_acc[i]);
    }
    updating = 0;
    destroy(&f);
}

static void invalid_source_references(void) {
    Fixture f; init(&f);
    f.cmd->mode = arm_run;
    f.cmd->plan_running = 1;
    f.cmd->plan_joint_acc[ARM_JOINT_NUM-1] = INFINITY;
    reference_step(&f);
    assert(f.cmd->mode == arm_damping);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(f.gimbal.arm_control_data.joint_angle[i] == i);
        assert(f.gimbal.arm_control_data.joint_vel[i] == 0);
        assert(f.gimbal.arm_control_data.joint_acc[i] == 0);
    }
    destroy(&f);
}

static void errors_and_rc(void) {
    Fixture f; init(&f);
    f.cmd->ctrl_src.joint_angle_solved[0] = 50;
    for (int k=0; k<50; ++k) constrained_step(&f);
    /* Failure on the last axis must also clear already-computed earlier axes. */
    f.joints[ARM_JOINT_NUM-1].max_jerk_deg_s3 = 0;
    reference_step(&f);
    assert(f.cmd->mode == arm_damping && f.gimbal.arm_control_data.mode == arm_damping);
    assert(f.cmd->joint_post_ref[ARM_JOINT_NUM-1].status < 0);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        assert(f.gimbal.arm_control_data.joint_angle[i] == i);
        assert(f.gimbal.arm_control_data.joint_vel[i] == 0);
        assert(f.gimbal.arm_control_data.joint_acc[i] == 0);
        assert(!f.cmd->joint_post_ref[i].initialized);
    }
    f.joints[ARM_JOINT_NUM-1].max_jerk_deg_s3 = 3600;
    f.cmd->mode = arm_run;
    constrained_step(&f);
    f.cmd->ctrl_src.joint_angle_solved[2] = NAN;
    reference_step(&f);
    assert(f.cmd->mode == arm_damping);
    f.cmd->ctrl_src.joint_angle_solved[2] = 2;
    f.cmd->mode = arm_run;
    constrained_step(&f);
    Edge_trigger_continuous left = {.flag = 1};
    Edge_trigger_continuous right = {0};
    rc_offline = 0;
    rc.sw_a = 1;
    rc.trig.lx_left = &left;
    rc.trig.lx_right = &right;
    Run_Mode_Update(f.cmd);
    assert(f.gimbal.arm_control_data.gripper_angle == -60);
    Run_Mode_Update(f.cmd);
    assert(f.gimbal.arm_control_data.gripper_angle == -60);
    right.flag = 1;
    Run_Mode_Update(f.cmd);
    assert(f.gimbal.arm_control_data.gripper_angle == 0);
    rc.trig.lx_left = NULL;
    rc.trig.lx_right = NULL;
    rc_offline = 1;
    destroy(&f);
}

static void check_quintic_boundary(const Fixture* fixture, Joint_motion_state expected) {
    const Quintic_poly* poly = &fixture->cmd->joint_post_ref[0].trajectory.quintic.curve.poly;
    assert(fabsf(poly->a0 - expected.position) < 1e-4f);
    assert(fabsf(poly->a1 - expected.velocity) < 1e-4f);
    assert(fabsf(2 * poly->a2 - expected.acceleration) < 1e-3f);
}

static void quintic_replanning_and_handoff(void) {
    Fixture a, b;
    allocation_budget = 2;
    init_profile(&a, ARM_POST_PROCESS_QUINTIC, 0.1f);
    assert(allocation_budget == 0);
    allocation_budget = -1;
    init(&b);

    a.joints[1].quintic_duration_s = 0.2f;
    a.cmd->ctrl_src.joint_angle_solved[0] = 10;
    a.cmd->ctrl_src.joint_angle_solved[1] = 11;
    b.cmd->ctrl_src.joint_angle_solved[0] = -5;
    const unsigned calls_before = update_calls;
    for (int k = 0; k < 25; ++k) {
        reference_step(&a);
        constrained_step(&b);
    }
    assert(update_calls == calls_before + 25 * ARM_JOINT_NUM);
    assert(b.gimbal.arm_control_data.joint_angle[0] < 0);
    /* 两轴在相同位移下按各自配置的时长推进。 */
    assert(fabsf(a.gimbal.arm_control_data.joint_angle[0] - 5) < 1e-4f);
    assert(fabsf(a.gimbal.arm_control_data.joint_vel[0] - 187.5f) < 1e-3f);
    assert(fabsf(a.gimbal.arm_control_data.joint_acc[0]) < 0.02f);
    assert(fabsf(a.gimbal.arm_control_data.joint_angle[1] - 2.03515625f) < 1e-4f);

    const Joint_motion_state last = a.cmd->joint_post_ref[0].state;
    a.cmd->ctrl_src.joint_angle_solved[0] = -3;
    reference_step(&a);
    check_quintic_boundary(&a, last);
    for (int k = 0; k < 51; ++k) reference_step(&a);
    assert(a.gimbal.arm_control_data.joint_angle[0] == -3);
    assert(a.gimbal.arm_control_data.joint_vel[0] == 0);
    assert(a.gimbal.arm_control_data.joint_acc[0] == 0);

    a.cmd->plan_running = 1;
    a.joints[0].plan_velocity_scale = 2;
    a.cmd->ctrl_src.joint_angle_solved[0] = 12;
    a.cmd->plan_joint_vel[0] = 3;
    a.cmd->plan_joint_acc[0] = 5;
    reference_step(&a);
    assert(a.gimbal.arm_control_data.joint_angle[0] == 12);
    assert(a.gimbal.arm_control_data.joint_vel[0] == 6);
    assert(a.gimbal.arm_control_data.joint_acc[0] == 5);
    a.cmd->plan_running = 0;
    reference_step(&a);
    check_quintic_boundary(&a, (Joint_motion_state){12, 6, 5});

    a.cmd->ctrl_src.joint_angle_solved[ARM_JOINT_NUM - 1] = NAN;
    reference_step(&a);
    assert(a.gimbal.arm_control_data.mode == arm_damping);
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        assert(a.gimbal.arm_control_data.joint_angle[i] == i);
        assert(a.gimbal.arm_control_data.joint_vel[i] == 0);
        assert(a.gimbal.arm_control_data.joint_acc[i] == 0);
        assert(!a.cmd->joint_post_ref[i].initialized);
    }
    a.cmd->mode = arm_run;
    a.cmd->ctrl_src.joint_angle_solved[ARM_JOINT_NUM - 1] = ARM_JOINT_NUM - 1;
    reference_step(&a);
    check_quintic_boundary(&a, (Joint_motion_state){0, 0, 0});
    destroy(&a);
    destroy(&b);
}

int main(void) {
    Fixture failed = {0};
    failed.arm.gimbal = &failed.gimbal;
    failed.gimbal.profile = &failed.profile;
    /* Object, temporary group, then six independent Ruckig instances. */
    for (int i=0; i<2+ARM_JOINT_NUM; ++i) {
        allocation_budget = i;
        assert(!Gimbal_Cmd_Create(&failed.arm));
        assert(live_allocations == 0);
    }
    allocation_budget = -1;
    position_replan_isolation();
    short_position_move();
    source_transitions();
    post_process_and_publication();
    invalid_source_references();
    errors_and_rc();
    quintic_replanning_and_handoff();
    assert(live_allocations == 0);
    puts("Joint Ruckig and quintic control checks passed");
    return 0;
}
