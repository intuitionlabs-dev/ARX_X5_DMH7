#pragma once
/* Included once by freertos.c; keep test code out of production source lists. */
#include "h7_joint_diagnostics.h"
#include "ENCOS_motor.h"
#include "arm.h"
#include "gimbal_cmd.h"
#include "rc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern volatile ruckig_c_input g_ruckig_input;
extern volatile ruckig_c_output g_ruckig_output;
extern volatile int g_ruckig_status;
extern volatile uint8_t g_ruckig_active;

volatile H7_Diag_Info h7_diag;
volatile H7_Diag_Clock h7_diag_clock;
volatile H7_Diag_Benchmark h7_diag_bench;
H7_Diag_Sample h7_diag_samples[H7_DIAG_CAPACITY]
    __attribute__((section(".RAM_D1"), aligned(32)));
#ifdef __OPTIMIZE__
#define H7_DIAG_OPT "optimized"
#else
#define H7_DIAG_OPT "O0"
#endif
const char h7_diag_build[] = "joint-diag ABI1 " __DATE__ " " __TIME__
    " GCC " __VERSION__ " " H7_DIAG_OPT;

_Static_assert(sizeof(H7_Diag_Sample) == 156, "diagnostic ABI");
_Static_assert(sizeof(H7_Diag_Motor) == 92, "diagnostic ABI");

static ENCOS_motor* diag_motor;
static control_encos_func diag_original_control;
static H7_Diag_Motor diag_latest_motor;
static uint32_t diag_begin_cycles, diag_cmd_seq, diag_in_cmd, diag_ready;

/* A latched guard acts only when explicitly enabled by the test runner.
 * Use the existing damping behavior, including all axes on A. No startup move. */
static void diag_damping(void) {
    Arm* arm = &g_arm[ARM_A];
    Gimbal* g = arm->gimbal;
    arm->arm_mode = arm_damping;
    arm->gimbal_cmd->mode = arm_damping;
    g->arm_mode = g->arm_control_data.mode = arm_damping;
    g->arm_feedback_data.mode = arm_damping;
    for (int j = 0; j < ARM_MOTOR_NUM; ++j) {
        Arm_motor* m = &g->motors[j];
        controller* c = m->control;
        c->ref_position = c->fdb_position;
        c->speed_comp = c->output_comp = 0;
        c->pid_pos_data.config = (PID_Config){0};
        c->pid_speed_data.config = (PID_Config){0};
        m->config->driver->mode(m, 1, 0, m->config->damping_kd);
    }
}

static void diag_guard(void) {
    if (!h7_diag.guard_enable) { h7_diag.guard_fault = 0; return; }
    Gimbal* g = g_arm[ARM_A].gimbal;
    const Arm_joint_config* j = &g->profile->joints[JOINT1];
    const controller* c = diag_motor->motor_controller;
    const float q = (diag_motor->real_fdb_position * (180.0f / ENCOS_PI)
        - g->motor_pos_init[JOINT1]) * j->direction / j->reduction;
    const float ref = (c->ref_position - g->motor_pos_init[JOINT1])
        * j->direction / j->reduction;
    if (!h7_diag.guard_fault) {
        if (!isfinite(q) || !isfinite(ref) || !isfinite(c->speed_comp)
            || !(diag_motor->feedback_valid & ENCOS_HAS_POSITION)
            || !(diag_motor->feedback_valid & ENCOS_HAS_SPEED)
            || !isfinite(diag_motor->fdb_speed)
            || (uint32_t)(HAL_GetTick() - diag_motor->last_position_ms) > 50U
            || (uint32_t)(HAL_GetTick() - diag_motor->last_speed_ms) > 50U)
            h7_diag.guard_fault = 1;
        else if (q < j->minimum_deg + h7_diag.guard_margin_deg
                 || q > j->maximum_deg - h7_diag.guard_margin_deg)
            h7_diag.guard_fault = 2;
        else if (ref < j->minimum_deg || ref > j->maximum_deg)
            h7_diag.guard_fault = 3;
        else if (fabsf(ref - q) > h7_diag.guard_error_deg)
            h7_diag.guard_fault = 4;
    }
    if (h7_diag.guard_fault) diag_damping();
}

/* Called from the existing ENCOS callback, after feedback was copied into the
 * controller. Keep that callback and its control math unchanged. RX can run
 * between the driver's copy and here: bit 16 identifies matching raw/control
 * feedback; timestamps must not be attributed to old controller data otherwise. */
static void diag_control(ENCOS_motor* motor) {
    H7_Diag_Motor s = {0};
    const uint32_t mask = Arm_Motors_Lock();
    diag_guard();
    s.seq = diag_latest_motor.seq + 1U;
    s.cycles = DWT->CYCCNT;
    s.hal_ms = HAL_GetTick();
    s.pos_ms = motor->last_position_ms;
    s.speed_ms = motor->last_speed_ms;
    s.rx_ms = motor->last_rx_ms;
    s.raw_pos_rad = motor->real_fdb_position;
    s.raw_speed_rad_s = motor->fdb_speed;
    s.flags = motor->feedback.type | ((uint32_t)motor->feedback_valid << 8)
        | (diag_in_cmd << 17) | ((uint32_t)motor->enable << 18)
        | ((uint32_t)motor->errcode << 24);
    s.cmd_seq = diag_cmd_seq;
    const controller* c = motor->motor_controller;
    if (fabsf(c->fdb_position - s.raw_pos_rad * (180.0f / ENCOS_PI)) < 1e-4f
        && fabsf(c->fdb_speed - s.raw_speed_rad_s * (180.0f / ENCOS_PI)) < 1e-4f)
        s.flags |= 1U << 16;
    Arm_Motors_Unlock(mask);
    diag_original_control(motor);
    /* CmdTask cannot preempt MotorTask; controller fields are stable here. */
    s.ref_pos = c->ref_position;
    s.speed_comp = c->speed_comp;
    s.ref_speed = c->ref_speed;
    s.fdb_pos = c->fdb_position;
    s.fdb_speed = c->fdb_speed;
    s.pid_out = c->pid_pos_data.output;
    s.pid_error = c->pid_pos_data.error[0];
    s.pid_d_error = c->pid_pos_data.d_error;
    s.tx_pos_rad = motor->ref_position;
    s.tx_speed_rad_s = motor->ref_speed;
    s.tx_torque = motor->ref_torque;
    s.mit_kp = motor->kp;
    s.mit_kd = motor->kd;
    diag_latest_motor = s;
}

void H7_Diag_Cmd_Begin(void) {
    if (!diag_ready) return;
    diag_begin_cycles = DWT->CYCCNT;
    ++diag_cmd_seq;
    diag_in_cmd = 1;
}

void H7_Diag_Deadline_Miss(void) {
    if (diag_ready) ++h7_diag_clock.deadline_misses;
}

void H7_Diag_Cmd_End(void) {
    if (!diag_ready) return;
    const uint32_t start = DWT->CYCCNT;
    const uint32_t wall = start - diag_begin_cycles;
    const uint32_t mask = Arm_Motors_Lock();
    diag_in_cmd = 0;
    diag_guard();
    ++h7_diag_clock.lock;
    __DMB();
    h7_diag_clock.cycles = start;
    h7_diag_clock.hal_ms = HAL_GetTick();
    h7_diag_clock.rtos_tick = xTaskGetTickCount();
    h7_diag_clock.cmd_count = diag_cmd_seq;
    if (wall > h7_diag_clock.wall_max_cycles) h7_diag_clock.wall_max_cycles = wall;
    __DMB();
    ++h7_diag_clock.lock;
    if (h7_diag.state == 2 && h7_diag.count < H7_DIAG_CAPACITY) {
        Gimbal_cmd* cmd = g_arm[ARM_A].gimbal_cmd;
        H7_Diag_Sample* s = &h7_diag_samples[h7_diag.count];
        s->seq = diag_cmd_seq;
        s->cycles = diag_begin_cycles;
        s->hal_ms = h7_diag_clock.hal_ms;
        s->rtos_tick = h7_diag_clock.rtos_tick;
        s->wall_cycles = wall;
        s->flags = (uint32_t)cmd->mode | ((uint32_t)g_arm[ARM_A].state << 4)
            | ((uint32_t)(!is_Offline(rc.monitor)) << 8)
            | ((uint32_t)g_arm[ARM_A].ctrl.valid << 9)
            | ((uint32_t)g_arm[ARM_A].ctrl.has_vel << 10)
            | ((uint32_t)cmd->plan_running << 11)
            | ((uint32_t)cmd->external_velocity_active << 12)
            | ((uint32_t)g_ruckig_active << 13)
            | ((uint32_t)(g_ruckig_output.new_calculation != 0) << 14)
            | (h7_diag.guard_fault << 16);
        s->ruckig_status = g_ruckig_status;
        s->target = cmd->ctrl_src.joint_angle_solved[JOINT1];
        s->q = cmd->post_processed_joint_angle[JOINT1];
        s->v = cmd->post_processed_joint_vel[JOINT1];
        s->a = cmd->post_processed_joint_acc[JOINT1];
        s->ruckig_time = g_ruckig_output.time;
        s->input_q = g_ruckig_input.current.position;
        s->input_v = g_ruckig_input.current.velocity;
        s->ruckig_q = g_ruckig_output.state.position;
        s->ruckig_v = g_ruckig_output.state.velocity;
        s->motor = diag_latest_motor;
        __DMB();
        ++h7_diag.count;
        /* Publication of state=3 happens in the test task after cache clean. */
    }
    const uint32_t hook = DWT->CYCCNT - start;
    if (hook > h7_diag_clock.hook_max_cycles) h7_diag_clock.hook_max_cycles = hook;
    Arm_Motors_Unlock(mask);
}

static void diag_config(void) {
    Gimbal* g = g_arm[ARM_A].gimbal;
    const Arm_joint_config* j = &g->profile->joints[JOINT1];
    const Arm_motor_config* m = g->motors[JOINT1].config;
    const controller* c = g->motors[JOINT1].control;
    h7_diag.core_hz = SystemCoreClock;
    h7_diag.hclk_hz = HAL_RCC_GetHCLKFreq();
    h7_diag.pclk1_hz = HAL_RCC_GetPCLK1Freq();
    h7_diag.pclk2_hz = HAL_RCC_GetPCLK2Freq();
    h7_diag.hse_hz = HSE_VALUE;
    h7_diag.tick_hz = configTICK_RATE_HZ;
    h7_diag.systick_load = SysTick->LOAD;
    h7_diag.tim23_psc = TIM23->PSC;
    h7_diag.tim23_arr = TIM23->ARR;
    h7_diag.rcc_cfgr = RCC->CFGR;
    h7_diag.rcc_d1cfgr = RCC->D1CFGR;
    h7_diag.rcc_d2cfgr = RCC->D2CFGR;
    h7_diag.rcc_pllckselr = RCC->PLLCKSELR;
    h7_diag.rcc_pll1divr = RCC->PLL1DIVR;
    h7_diag.cpuid = SCB->CPUID;
    h7_diag.ccr = SCB->CCR;
    h7_diag.rc_online = !is_Offline(rc.monitor);
    h7_diag.robot_state = g_arm[ARM_A].state;
    h7_diag.arm_mode = g->arm_mode;
    h7_diag.tx_group = m->tx_group;
    h7_diag.control_type = c->config.control_type;
    h7_diag.control_depth = c->config.control_depth;
    h7_diag.td_status = c->config.td_status;
    h7_diag.pid_mode = m->position_pid.PID_mode;
    h7_diag.d_ahead = m->position_pid.D_ahead;
    h7_diag.period_s = ARM_CONTROL_PERIOD_S;
    h7_diag.direction = j->direction;
    h7_diag.reduction = j->reduction;
    h7_diag.zero_deg = g->motor_pos_init[JOINT1];
    h7_diag.velocity_scale = j->velocity_scale;
    h7_diag.plan_velocity_scale = j->plan_velocity_scale;
    h7_diag.max_velocity = j->max_velocity_deg_s;
    h7_diag.max_acceleration = j->max_acceleration_deg_s2;
    h7_diag.max_jerk = j->max_jerk_deg_s3;
    h7_diag.pos_kp = m->position_pid.KP;
    h7_diag.pos_ki = m->position_pid.KI;
    h7_diag.pos_kd = m->position_pid.KD;
    h7_diag.pos_output_max = m->position_pid.outputMax;
    h7_diag.speed_kp = m->speed_pid.KP;
    h7_diag.speed_ki = m->speed_pid.KI;
    h7_diag.speed_kd = m->speed_pid.KD;
    h7_diag.mit_kp = m->kp;
    h7_diag.mit_kd = m->kd;
    h7_diag.position_max = m->position_max;
    h7_diag.speed_max = m->speed_max;
    h7_diag.feedback_offset_rad = diag_motor->config.position_offset;
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        h7_diag.minimum_deg[i] = g->profile->joints[i].minimum_deg;
        h7_diag.maximum_deg[i] = g->profile->joints[i].maximum_deg;
    }
    snprintf((char*)h7_diag.profile, sizeof(h7_diag.profile), "%s", g->profile->name);
    snprintf((char*)h7_diag.version, sizeof(h7_diag.version), "%s", g->profile->version_string);
    snprintf((char*)h7_diag.build, sizeof(h7_diag.build), "%s", h7_diag_build);
}

static int diag_all_stopped(void) {
    for (int i = 0; i < ARM_NUM; ++i) {
        const Gimbal* g = g_arm[i].gimbal;
        if (!g || g->arm_mode != arm_stop || g_arm[i].arm_mode != arm_stop) return 0;
    }
    return !h7_diag.guard_enable;
}

/* Mathematical baseline only: no motor reference is bypassed. */
__attribute__((noinline)) static float diag_baseline(float target) { return target; }
__attribute__((noinline)) static float diag_quintic(float t) {
    return 20.0f * t*t*t * (10.0f + t*(-15.0f + 6.0f*t));
}

static void diag_benchmark(void) {
    memset((void*)&h7_diag_bench, 0, sizeof(h7_diag_bench));
    if (!diag_all_stopped()) { h7_diag.error = 2; return; }
    ruckig_c_context* ctx = ruckig_c_create(ARM_CONTROL_PERIOD_S);
    if (!ctx) { h7_diag.error = 3; return; }
    h7_diag.state = 4;
    uint32_t mask = Arm_Motors_Lock();
    uint32_t start = DWT->CYCCNT;
    h7_diag_bench.counter_overhead_cycles = DWT->CYCCNT - start;
    Arm_Motors_Unlock(mask);
    for (unsigned kind = 0; kind < 4 && !h7_diag.error; ++kind) {
        ruckig_c_input in = {.target = {.position = 20.0f},
            .max_velocity = h7_diag.max_velocity,
            .max_acceleration = h7_diag.max_acceleration, .max_jerk = h7_diag.max_jerk};
        ruckig_c_output out;
        ruckig_c_reset(ctx);
        /* Cached case excludes the first solve and repeats an identical
         * interior evaluation, so endpoint holds do not dominate its mean. */
        if (kind == 2 && ruckig_c_update(ctx, &in, &out) < 0) {
            h7_diag.error = 3; break;
        }
        uint64_t total = 0;
        H7_Diag_Cost cost = {.min_cycles = UINT32_MAX};
        for (unsigned n = 0; n < 1000; ++n) {
            if (!diag_all_stopped()) { h7_diag.error = 2; break; }
            if (kind == 2) {
                in.current = out.state;
                /* Restart cached motion before it becomes a terminal hold. */
                if (out.time >= 0.1f) {
                    in.current = (ruckig_c_state){0};
                    ruckig_c_reset(ctx);
                    if (ruckig_c_update(ctx, &in, &out) < 0) { h7_diag.error = 3; break; }
                    in.current = out.state;
                }
            } else if (kind == 3) {
                in.current = (ruckig_c_state){0};
                in.target.position = (n & 1U) ? 20.0f : -20.0f;
                ruckig_c_reset(ctx);
            }
            /* The only IRQ-off solver measurement. Both arms MUST be stopped.
             * Yield after every call; never mask a full batch. */
            mask = Arm_Motors_Lock();
            if (!diag_all_stopped()) { Arm_Motors_Unlock(mask); h7_diag.error = 2; break; }
            start = DWT->CYCCNT;
            int result = 0;
            float value;
            if (kind == 0) value = diag_baseline((float)n);
            else if (kind == 1) value = diag_quintic((float)n / 1000.0f);
            else { result = ruckig_c_update(ctx, &in, &out); value = result < 0 ? 0 : out.state.position; }
            const uint32_t elapsed = DWT->CYCCNT - start;
            Arm_Motors_Unlock(mask);
            h7_diag_bench.sink = value;
            if (result < 0) { h7_diag.error = 3; break; }
            total += elapsed;
            ++cost.count;
            if (elapsed < cost.min_cycles) cost.min_cycles = elapsed;
            if (elapsed > cost.max_cycles) cost.max_cycles = elapsed;
            if (elapsed > SystemCoreClock / 1000U) { h7_diag.error = 4; break; }
            osDelay(1);
        }
        cost.mean_cycles = cost.count ? (float)total / (float)cost.count : 0;
        h7_diag_bench.cost[kind] = cost;
    }
    ruckig_c_destroy(ctx);
    h7_diag_bench.status = h7_diag.error;
    h7_diag_bench.completed = 1;
    h7_diag.state = 5;
}

static void diag_freeze(void) {
    /* Stop CmdTask writes before cache maintenance. */
    uint32_t mask = Arm_Motors_Lock();
    h7_diag.state = 1;
    Arm_Motors_Unlock(mask);
    if (SCB->CCR & SCB_CCR_DC_Msk)
        SCB_CleanDCache_by_Addr((uint32_t*)h7_diag_samples, sizeof(h7_diag_samples));
    __DSB();
    h7_diag.state = 3;
}

void H7_Diag_Task(void* argument) {
    (void)argument;
    /* Let CAN perform its one-time DWT calibration/reset first. Never reset
     * CYCCNT here: it is shared by CAN TX spacing and this diagnostic. */
    osDelay(2000);
    while (!g_arm[ARM_A].gimbal || !g_arm[ARM_A].gimbal_cmd) osDelay(20);
    Gimbal* g = g_arm[ARM_A].gimbal;
    h7_diag.magic = H7_DIAG_MAGIC;
    h7_diag.abi = 1;
    h7_diag.record_bytes = sizeof(H7_Diag_Sample);
    h7_diag.capacity = H7_DIAG_CAPACITY;
    h7_diag.guard_margin_deg = 5.0f;
    h7_diag.guard_error_deg = 20.0f;
    if (g->profile != &ARM_PROFILE_ARX_X5
        || g->motors[JOINT1].config->driver != &ARM_DRIVER_ENCOS) {
        h7_diag.error = 1;
        osThreadExit();
    }
    diag_motor = g->motors[JOINT1].context;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t mask = Arm_Motors_Lock();
    diag_original_control = diag_motor->control_calc;
    diag_motor->control_calc = diag_control;
    diag_config();
    diag_ready = 1;
    h7_diag.state = 1;
    Arm_Motors_Unlock(mask);
    for (;;) {
        const uint32_t request = h7_diag.request;
        if (request) {
            h7_diag.request = 0;
            if (request == 1 && h7_diag.state != 2) {
                const uint32_t lock = Arm_Motors_Lock();
                diag_config();
                h7_diag.count = h7_diag.error = 0;
                ++h7_diag.generation;
                h7_diag.state = 2;
                Arm_Motors_Unlock(lock);
            } else if (request == 2 && h7_diag.state == 2) diag_freeze();
            else if (request == 3 && h7_diag.state != 2) { h7_diag.error = 0; diag_config(); diag_benchmark(); }
        }
        if (h7_diag.state == 2 && h7_diag.count == H7_DIAG_CAPACITY) diag_freeze();
        osDelay(5);
    }
}
