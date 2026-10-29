/* 使用可观测驱动验证控制核心、协议边界和初始化策略。 */
#include "arm.h"
#include "arm_initialization.h"
#include "arm_protocol.h"
#include "rc.h"
#include <math.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
Arm g_arm[ARM_NUM];
rc_t rc;
const Arm_motor_driver ARM_DRIVER_DM = {0}, ARM_DRIVER_ENCOS = {0};
const Arm_binding ARM_BINDINGS[ARM_NUM] = {0};
static uint32_t clock_ms;
static unsigned create_count, disabled_count, zero_count;
static float last_kp[ARM_NUM*ARM_MOTOR_NUM], last_kd[ARM_NUM*ARM_MOTOR_NUM];
uint32_t HAL_GetTick(void) { return clock_ms; }
uint32_t Arm_Motors_Lock(void) { return 0; }
void Arm_Motors_Unlock(uint32_t mask) { (void)mask; }
const Robot_ctrl_cmd* Robot_State_Ctrl(const Arm* arm) { return &arm->ctrl; }
static void on_lost(void* context) { (void)context; }
static int create(Arm_motor* motor, const Arm_motor_config* cfg, uint8_t bus) {
    (void)bus;
    motor->context = (void*)(uintptr_t)create_count++; // NOLINT(performance-no-int-to-ptr)
    motor->control = calloc(1, sizeof(controller));
    motor->monitor = Monitor_Register(on_lost, 10, NULL);
    motor->monitor->reset(motor->monitor);
    motor->control->pid_pos_data.config = cfg->position_pid;
    motor->control->pid_speed_data.config = cfg->speed_pid;
    return 1;
}
static void mode(Arm_motor* motor, uint8_t enabled, float kp, float kd) {
    (void)enabled;
    unsigned i = (unsigned)(uintptr_t)motor->context;
    last_kp[i] = kp;
    last_kd[i] = kd;
}
static float position(const Arm_motor* m) { return m->control->fdb_position*0.01745329252f; }
static uint8_t temperature(const Arm_motor* m) { (void)m; return 45; }
static void disable(Arm_motor* m) { (void)m; ++disabled_count; }
static uint8_t stopped(const Arm_motor* m, uint32_t now) { (void)m; (void)now; return 1; }
static int zero_start(Arm_motor* m, uint32_t now) {
    (void)now; m->control->fdb_position = 0; ++zero_count; return 1;
}
static int zero_poll(Arm_motor* m, uint32_t now) { (void)m; (void)now; return 1; }
static const Arm_motor_driver driver = { // NOLINT(readability-identifier-naming)
    create, mode, position, temperature, disable, stopped, zero_start, zero_poll,
};

int main(void) {
    Monitor_Init();
    Move_Plan_Init();
    static Arm_motor_config motors[ARM_MOTOR_NUM];
    static Arm_joint_config joints[ARM_JOINT_NUM];
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        motors[i] = (Arm_motor_config){.driver=&driver, .tx_id=i+1, .rx_id=i+100,
            .tx_group=1u<<(i%8), .position_max=10, .speed_max=10, .torque_max=10,
            .kp_max=500, .kd_max=50, .kp=20+i, .kd=1, .damping_kd=2};
    }
    for (int i = 0; i < ARM_JOINT_NUM; ++i)
        joints[i] = (Arm_joint_config){.direction=-1, .reduction=2, .position_zero_deg=3,
            .minimum_deg=-90, .maximum_deg=90, .reset_deg=5+i, .torque_scale=2,
            .velocity_scale=1, .plan_velocity_scale=1, .td_r=1000, .absolute_encoder=1};
    Arm_profile profile = {.name="test", .joint_count=ARM_JOINT_NUM, .motor_count=ARM_MOTOR_NUM,
        .motors=motors, .joints=joints, .gripper={.cmd={0,100}, .motor_deg={80,-120},
            .torque_scale=-2, .minimum=0, .maximum=100},
        .init={.reset_time_s=.02f, .pose_tolerance_deg=20, .zero_total_ms=1000,
            .zero_motor_ms=100, .zero_disable_timeout_ms=1500, .zero_disable_retry_ms=100, .gripper_home_torque=.5f, .gripper_speed_deg_s=100,
            .gripper_backoff_deg=1, .gripper_follow_error_deg=20, .gripper_backoff_tolerance_deg=4, .gripper_settle_ms=400, .stall_tolerance_deg=.5f, .stall_window_ms=2,
            .stall_windows=2, .seek_timeout_ms=100, .backoff_timeout_ms=100},
        .quintic_duration_s=.16f, .stop_on_motor_lost=1, .initialize=Arm_Initialize_Hold,
        .rc_joint={-1,-1,-1}};
    CHECK(Arm_Profile_Validate(&profile));
    Arm_profile invalid = profile;
    invalid.joint_count++;
    CHECK(!Arm_Profile_Validate(&invalid));
    invalid = profile;
    invalid.gripper.motor_deg[1] = invalid.gripper.motor_deg[0];
    CHECK(!Arm_Profile_Validate(&invalid));
    CHECK(fabsf(Arm_Gripper_To_Motor(&profile, 25)-30) < .001f);
    CHECK(fabsf(Arm_Gripper_From_Motor(&profile, -70)-75) < .001f);
    static monitor_item rc_monitor;
    rc.monitor = &rc_monitor;
    for (int arm = 0; arm < ARM_NUM; ++arm) {
        g_arm[arm].gimbal = Gimbal_Create_Profile(&profile, arm);
        CHECK(g_arm[arm].gimbal);
        g_arm[arm].gimbal_cmd = Gimbal_Cmd_Create(&g_arm[arm]);
        CHECK(g_arm[arm].gimbal_cmd);
    }
    for (int a=0; a<ARM_NUM; ++a)
        for (int i=0; i<ARM_MOTOR_NUM; ++i)
            g_arm[a].gimbal->motors[i].monitor->reset(g_arm[a].gimbal->motors[i].monitor);
    Gimbal* g = g_arm[0].gimbal;
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        g->motors[i].control->fdb_position = 3;
        g->arm_control_data.joint_angle[i] = 10+i;
        g->arm_control_data.joint_vel[i] = 2;
    }
    g->arm_control_data.gripper_angle = 25;
    g->arm_control_data.gripper_torque = 3;
    g->arm_control_data.mode = arm_run;
    Gimbal_Update(g);
    CHECK(g->arm_feedback_data.mode == arm_run);
    CHECK(fabsf(g->motors[0].control->ref_position+17) < .001f);
    CHECK(fabsf(g->motors[GRIPPER].control->ref_position-30) < .001f);
    CHECK(g->motors[GRIPPER].control->output_comp == -6);
    g->arm_control_data.mode = arm_damping;
    Gimbal_Update(g);
    CHECK(last_kp[0] == 0 && last_kd[0] == 2);
    CHECK(g->motors[0].control->output_comp == 0);
    g->motors[ARM_JOINT_NUM-1].monitor->count = 0;
    g->arm_control_data.mode = arm_run;
    Gimbal_Update(g);
    CHECK(g->arm_feedback_data.mode == arm_stop);
    g->motors[ARM_JOINT_NUM-1].monitor->reset(g->motors[ARM_JOINT_NUM-1].monitor);

    Arm_link links[ARM_JOINT_NUM];
    float q[ARM_JOINT_NUM]={0}, qd[ARM_JOINT_NUM]={0}, qdd[ARM_JOINT_NUM]={0}, tau[ARM_JOINT_NUM];
    Arm_dynamics_work work[ARM_JOINT_NUM];
    for (int i=0; i<ARM_JOINT_NUM; ++i) links[i]=(Arm_link){.mass=1, .com={.5f,0,0},
        .origin={1,0,0}, .axis={0,1,0}, .fixed_rotation={{{1,0,0},{0,1,0},{0,0,1}}}};
    const Arm_dynamics_model dynamics={.joint_count=ARM_JOINT_NUM,.links=links};
    CHECK(Arm_Dynamics_Compute(&dynamics,q,qd,qdd,NULL,1,tau,work,ARM_JOINT_NUM)==0);
    for (int i=0; i<ARM_JOINT_NUM; ++i) {
        const float length=ARM_JOINT_NUM-i;
        CHECK(fabsf(tau[i]+4.905f*length*length)<.005f);
    }
    q[0]=NAN;
    tau[0]=123;
    CHECK(Arm_Dynamics_Compute(&dynamics,q,qd,qdd,NULL,1,tau,work,ARM_JOINT_NUM)<0 && tau[0]==123);

    Robot_rx_payload payload = {0};
    for (int i=0; i<FRAME_PKT_JOINT_NUM; ++i) {
        payload.arm_ctrl_pos_vel.joint_angle[i] = 11+i;
        payload.arm_ctrl_pos_vel.joint_vel[i] = 2+i;
    }
    payload.arm_ctrl_pos_vel.gripper_angle = 35;
    CHECK(Arm_Protocol_Control(&g_arm[0], &payload, 1));
    CHECK(g_arm[0].ctrl.joint_angle[0] == 11);
    for (int i=FRAME_PKT_JOINT_NUM; i<ARM_JOINT_NUM; ++i)
        CHECK(g_arm[0].ctrl.joint_angle[i] == g->arm_feedback_data.joint_angle[i]);
    payload.arm_ctrl_pos_vel.joint_angle[0] = NAN;
    CHECK(!Arm_Protocol_Control(&g_arm[0], &payload, 1));
    struct { unsigned before; FramePktArmFeedback wire; unsigned after; } guard = {.before=123, .after=456};
    g->arm_feedback_data.motor_inited[GRIPPER] = 1;
    Arm_Protocol_Feedback(&g_arm[0], &guard.wire);
    CHECK(sizeof(guard.wire) == 63 && guard.before == 123 && guard.after == 456);
    CHECK(guard.wire.motor_temp[6] == 45 && (guard.wire.motor_inited_mask & 64));
    CHECK(guard.wire.joint_angle[0] == g->arm_feedback_data.joint_angle[0]);

    g->zero.request = 1;
    for (int i=0; i<1000 && Arm_Zero_Busy(g); ++i, ++clock_ms) Arm_Zero_Loop(g);
    CHECK(g->zero.state == ARM_ZERO_DONE && zero_count == ARM_JOINT_NUM);
    CHECK(g->zero.result[GRIPPER] == ARM_ZERO_RES_SKIP);
    CHECK(g_arm[1].gimbal->zero.state == ARM_ZERO_IDLE);
    CHECK(disabled_count >= ARM_MOTOR_NUM);

    Gimbal_cmd* cmd = g_arm[0].gimbal_cmd;
    g_arm[0].ctrl.valid = 0;
    g_arm[0].arm_mode = arm_run;
    CHECK(Arm_Initialize_Hold(cmd, 1) == 0);
    for (int i=0; i<30; ++i) {
        clock_ms+=2;
        Move_Plan_Update();
        Gimbal_Cmd_Update(cmd);
        if (i == 4) {
            CHECK(cmd->plan_running);
            CHECK(fabsf(g->arm_control_data.joint_angle[0]-cmd->temp_group[0]->current_pos)<.001f);
            CHECK(fabsf(g->arm_control_data.joint_vel[0]-cmd->temp_group[0]->current_vel)<.001f);
            CHECK(fabsf(g->arm_control_data.joint_acc[0]-cmd->temp_group[0]->current_acc)<.001f);
        }
    }
    CHECK(Arm_Initialize_Hold(cmd, 0) == 1);
    for (int i=0; i<ARM_JOINT_NUM; ++i)
        CHECK(fabsf(g->arm_control_data.joint_angle[i]-joints[i].reset_deg)<.001f);

    profile.initialize = Arm_Initialize_Home;
    CHECK(Arm_Initialize_Home(cmd, 1) == 0);
    int initialized = 0;
    for (int i=0; i<300; ++i) {
        clock_ms+=2;
        Move_Plan_Update();
        Gimbal_Cmd_Update(cmd);
        if (cmd->initialization.step == 2)
            g->motors[GRIPPER].control->fdb_position = cmd->initialization.backoff_target;
        if (Arm_Zero_Busy(g)) Arm_Zero_Loop(g);
        const int result = Arm_Initialize_Home(cmd, 0);
        CHECK(result >= 0);
        if (result > 0) { initialized = 1; break; }
    }
    CHECK(initialized);
    while (get_Rest_Temp_Plan_cnt() > 0)
        CHECK(Move_Plan_Temp_Create(0,0,0,1,0,0,10,quintic_spline));
    g->arm_feedback_data.motor_inited[GRIPPER] = 0;
    CHECK(Arm_Initialize_Hold(cmd, 1) == 0);
    Gimbal_Cmd_Update(cmd);
    CHECK(Arm_Initialize_Hold(cmd, 0) < 0);
    CHECK(cmd->auto_failed && !cmd->plan_running);
    CHECK(!g->arm_feedback_data.motor_inited[GRIPPER]);
    return 0;
}
