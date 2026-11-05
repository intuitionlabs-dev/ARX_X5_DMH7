/* Exercise the actual diagnostic callbacks/guards with fake hardware registers. */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "gimbal_cmd.h"
#include "arm.h"
#include "rc.h"
#include <assert.h>
#include <stdlib.h>

static DWT_Type test_dwt;
static CoreDebug_Type test_debug;
static SCB_Type test_scb;
static SysTick_Type test_systick;
static RCC_TypeDef test_rcc;
static TIM_TypeDef test_tim;
#undef DWT
#undef CoreDebug
#undef SCB
#undef SysTick
#undef RCC
#undef TIM23
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define SCB (&test_scb)
#define SysTick (&test_systick)
#define RCC (&test_rcc)
#define TIM23 (&test_tim)
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define SCB_CleanDCache_by_Addr(a, b) ((void)0)

#include "h7_joint_diagnostics_impl.h"

Arm g_arm[ARM_NUM];
rc_t rc;
uint32_t SystemCoreClock = 480000000;
volatile ruckig_c_input g_ruckig_input;
volatile ruckig_c_output g_ruckig_output;
volatile int g_ruckig_status;
volatile uint8_t g_ruckig_active;
const Arm_profile ARM_PROFILE_ARX_X5 = {0};
const Arm_motor_driver ARM_DRIVER_ENCOS = {0};

static uint32_t test_ms = 100, test_lock;
static unsigned called, damping_calls;
static uint8_t rc_offline = 1;
uint32_t HAL_GetTick(void) { return test_ms; }
TickType_t xTaskGetTickCount(void) { return test_ms; }
uint32_t HAL_RCC_GetHCLKFreq(void) { return 240000000; }
uint32_t HAL_RCC_GetPCLK1Freq(void) { return 120000000; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 120000000; }
osStatus_t osDelay(uint32_t ticks) { test_ms += ticks; return osOK; }
void osThreadExit(void) { abort(); }
uint8_t is_Offline(monitor_item* monitor) { (void)monitor; return rc_offline; }
uint32_t Arm_Motors_Lock(void) { return test_lock++; }
void Arm_Motors_Unlock(uint32_t mask) { assert(test_lock == mask+1); test_lock = mask; }

static void motor_mode(Arm_motor* motor, uint8_t enabled, float kp, float kd) {
    (void)motor;
    assert(enabled == 1 && kp == 0 && kd == 30);
    ++damping_calls;
}
static void original_control(ENCOS_motor* motor) {
    ++called;
    controller* c = motor->motor_controller;
    c->pid_pos_data.error[0] = c->ref_position-c->fdb_position;
    c->pid_pos_data.output = c->pid_pos_data.config.KP*c->pid_pos_data.error[0];
    c->ref_speed = c->speed_comp+c->pid_pos_data.output;
    motor->ref_position = c->ref_position*ENCOS_PI/180;
    motor->ref_speed = c->ref_speed*ENCOS_PI/180;
}

int main(void) {
    static controller controls[ARM_MOTOR_NUM];
    static Arm_motor_config motors[ARM_MOTOR_NUM];
    static Arm_joint_config joints[ARM_JOINT_NUM];
    static const Arm_motor_driver driver = {.mode = motor_mode};
    static Arm_profile profile = {.joints = joints, .motors = motors};
    static Gimbal gimbal;
    static Gimbal_cmd command;
    static ENCOS_motor encos;
    gimbal.profile = &profile;
    for (int i = 0; i < ARM_MOTOR_NUM; ++i) {
        motors[i].driver = &driver;
        motors[i].damping_kd = 30;
        gimbal.motors[i].config = &motors[i];
        gimbal.motors[i].control = &controls[i];
    }
    joints[0] = (Arm_joint_config){.minimum_deg=-115, .maximum_deg=90, .direction=1, .reduction=1};
    g_arm[0].gimbal = &gimbal;
    g_arm[0].gimbal_cmd = &command;
    g_arm[0].arm_mode = arm_run;
    g_arm[0].state = robot_state_normal;
    g_arm[0].ctrl.valid = 1;
    command.gimbal = &gimbal;
    command.mode = arm_run;
    encos.motor_controller = &controls[0];
    encos.last_rx_ms = encos.last_position_ms = encos.last_speed_ms = test_ms;
    encos.feedback_valid = ENCOS_HAS_POSITION | ENCOS_HAS_SPEED;
    encos.feedback.type = 1;
    encos.enable = encos_enable;
    diag_motor = &encos;
    diag_original_control = original_control;
    diag_ready = 1;
    controls[0].ref_position = 4;
    controls[0].speed_comp = 5;
    controls[0].pid_pos_data.config.KP = 6;
    h7_diag.state = 2;
    DWT->CYCCNT = 100;
    H7_Diag_Cmd_Begin();
    DWT->CYCCNT = 150;
    diag_control(&encos);
    assert(called == 1 && controls[0].ref_speed == 29 && !damping_calls);
    command.post_processed_joint_angle[0] = 4;
    command.post_processed_joint_vel[0] = 5;
    DWT->CYCCNT = 200;
    H7_Diag_Cmd_End();
    assert(h7_diag.count == 1 && h7_diag_samples[0].cycles == 100);
    assert(h7_diag_samples[0].motor.cycles == 150 && h7_diag_samples[0].wall_cycles == 100);
    assert(h7_diag_samples[0].motor.ref_speed == 29 && h7_diag_samples[0].q == 4);
    assert(h7_diag_samples[0].motor.flags & (1U << 16));
    assert(h7_diag_samples[0].motor.flags & (1U << 17));
    H7_Diag_Deadline_Miss();
    assert(h7_diag_clock.deadline_misses == 1 && !(h7_diag_clock.lock & 1));
    diag_freeze();
    H7_Diag_Cmd_End();
    assert(h7_diag.state == 3 && h7_diag.count == 1);

    h7_diag.guard_enable = 1;
    h7_diag.guard_margin_deg = 5;
    h7_diag.guard_error_deg = 20;
    encos.real_fdb_position = -113*ENCOS_PI/180;
    controls[0].fdb_position = -113;
    diag_control(&encos);
    assert(h7_diag.guard_fault == 2 && g_arm[0].arm_mode == arm_damping);
    assert(damping_calls == ARM_MOTOR_NUM && controls[0].ref_speed == 0);
    encos.real_fdb_position = controls[0].fdb_position = 0;
    controls[0].ref_position = 2;
    diag_control(&encos);
    assert(h7_diag.guard_fault == 2 && controls[0].ref_position == 0); /* latched */
    h7_diag.guard_enable = 0;
    diag_control(&encos);
    assert(!h7_diag.guard_fault);
    h7_diag.guard_enable = 1;
    controls[0].ref_position = 91;
    diag_control(&encos);
    assert(h7_diag.guard_fault == 3);
    h7_diag.guard_enable = 0;
    diag_control(&encos);
    h7_diag.guard_enable = 1;
    test_ms += 51;
    diag_control(&encos);
    assert(h7_diag.guard_fault == 1);
    /* A rejected benchmark must never leave old 'completed' results visible. */
    h7_diag_bench.completed = 1;
    diag_benchmark();
    assert(h7_diag.error == 2 && !h7_diag_bench.completed);
    h7_diag.guard_enable = 0;
    diag_control(&encos);
    encos.last_position_ms = encos.last_speed_ms = test_ms;
    h7_diag.guard_enable = 1;
    h7_diag.guard_cycle_hz = 480000000;
    h7_diag.guard_timeout_ms = 18000;
    diag_control(&encos);
    /* The independent time guard must survive multiple CYCCNT wraps. */
    for (int i = 0; i < 37; ++i) {
        DWT->CYCCNT += 240000000U;
        diag_control(&encos);
    }
    assert(h7_diag.guard_fault == 5);
    h7_diag.guard_enable = 0;
    diag_control(&encos);
    g_arm[0].arm_mode = g_arm[1].arm_mode = gimbal.arm_mode = arm_stop;
    g_arm[1].gimbal = &gimbal;
    h7_diag.max_velocity = 400;
    h7_diag.max_acceleration = 2500;
    h7_diag.max_jerk = 10000;
    h7_diag.error = 0;
    diag_benchmark();
    assert(h7_diag_bench.completed && h7_diag_bench.status == 0);
    for (int i = 0; i < 4; ++i) assert(h7_diag_bench.cost[i].count == 128);
    h7_diag.request = 2;
    diag_benchmark();
    assert(h7_diag.error == 6 && h7_diag_bench.status == 6 && !h7_diag.request);

    uint8_t guard[] = {H7_DIAG_OP_GUARD, 1, 0, 0, 0};
    const uint8_t capture[] = {H7_DIAG_OP_CAPTURE, 1, 0, 0, 0};
    const uint8_t freeze[] = {H7_DIAG_OP_CAPTURE, 2, 0, 0, 0};
    const uint8_t clock_hz[] = {H7_DIAG_OP_CLOCK_HZ, 0x00, 0x38, 0x9c, 0x1c};
    g_arm[0].arm_mode = arm_damping;
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard);
    assert(!h7_diag.guard_enable);
    memset(gimbal.arm_feedback_data.motor_inited, 1, ARM_MOTOR_NUM);
    H7_Diag_Link_Command(ARM_B, guard, sizeof guard);
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard - 1);
    rc_offline = 0;
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard);
    assert(!h7_diag.guard_enable);
    rc_offline = 1;
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard);
    assert(h7_diag.guard_enable);
    h7_diag.guard_cycle_hz = 0;
    H7_Diag_Link_Command(ARM_A, clock_hz, sizeof clock_hz);
    assert(h7_diag.guard_cycle_hz == 480000000);
    guard[1] = 0;
    g_arm[0].arm_mode = arm_run;
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard);
    assert(h7_diag.guard_enable);
    g_arm[0].arm_mode = arm_damping;
    H7_Diag_Link_Command(ARM_A, guard, sizeof guard);
    assert(!h7_diag.guard_enable);
    h7_diag.state = 3;
    H7_Diag_Link_Command(ARM_A, capture, sizeof capture);
    assert(h7_diag.request == 1);
    h7_diag.state = 2;
    h7_diag.request = 0;
    H7_Diag_Link_Command(ARM_A, freeze, sizeof freeze);
    assert(h7_diag.request == 2);
    assert(!test_lock);
    puts("diagnostic callback, timestamp, freeze and guard checks passed");
    return 0;
}
