#pragma once

#include <stdint.h>
#include "test_config.h"

#if TEST_JOINT_DIAGNOSTICS_ENABLE
/* ABI 1: all fields are 32-bit words, little endian. Units: deg, deg/s,
 * deg/s^2 unless the field explicitly ends in _rad or _rad_s. */
#define H7_DIAG_CAPACITY 1280U
#define H7_DIAG_MAGIC 0x4a314447U
#define H7_DIAG_COMMAND_TID 0x70U
#define H7_DIAG_OP_CAPTURE 1U
#define H7_DIAG_OP_GUARD 2U
#define H7_DIAG_OP_CLOCK_HZ 3U

typedef struct {
    uint32_t seq, cycles, hal_ms, pos_ms, speed_ms, rx_ms, flags, cmd_seq;
    float ref_pos, speed_comp, ref_speed, fdb_pos, fdb_speed;
    float pid_out, pid_error, pid_d_error, raw_pos_rad, raw_speed_rad_s;
    float tx_pos_rad, tx_speed_rad_s, tx_torque, mit_kp, mit_kd;
} H7_Diag_Motor;

typedef struct {
    uint32_t seq, cycles, hal_ms, rtos_tick, wall_cycles, flags;
    int32_t ruckig_status;
    float target, q, v, a, ruckig_time, input_q, input_v, ruckig_q, ruckig_v;
    H7_Diag_Motor motor;
} H7_Diag_Sample;

/* request: 1 capture, 2 freeze, 3 isolated CPU benchmark. state:
 * 0 starting, 1 ready, 2 recording, 3 frozen, 4 benchmark, 5 benchmark done.
 * error: 1 incompatible profile, 2 benchmark needs both arms stopped,
 * 3 allocation/solver error, 4 excessive benchmark IRQ-off time,
 * 5 thread creation failed, 6 benchmark cancelled.
 * guard_fault: 1 invalid/stale feedback, 2 soft-limit margin, 3 reference
 * outside soft limits, 4 tracking error, 5 test time limit.
 * Faults latch until guard_enable=0.
 * guard_cycle_hz is updated from PC clock calibration before recording.
 * request=2 may cancel an active recording/benchmark. */
typedef struct {
    uint32_t magic, abi, record_bytes, capacity;
    volatile uint32_t request, state, count, generation, error;
    volatile uint32_t guard_enable, guard_fault;
    volatile uint32_t guard_cycle_hz;
    uint32_t guard_timeout_ms;
    float guard_margin_deg, guard_error_deg;
    uint32_t core_hz, hclk_hz, pclk1_hz, pclk2_hz, hse_hz, tick_hz;
    uint32_t systick_load, tim23_psc, tim23_arr;
    uint32_t rcc_cfgr, rcc_d1cfgr, rcc_d2cfgr, rcc_pllckselr, rcc_pll1divr;
    uint32_t cpuid, ccr, rc_online, robot_state, arm_mode, tx_group;
    uint32_t control_type, control_depth, td_status, pid_mode, d_ahead;
    float period_s, direction, reduction, zero_deg, velocity_scale, plan_velocity_scale;
    float max_velocity, max_acceleration, max_jerk;
    float pos_kp, pos_ki, pos_kd, pos_output_max, speed_kp, speed_ki, speed_kd;
    float mit_kp, mit_kd, position_max, speed_max, feedback_offset_rad;
    float minimum_deg[6], maximum_deg[6];
    char profile[32], version[64], build[96];
} H7_Diag_Info;

/* Read with seqlock: lock must be the same even value before/after the read.
 * DWT wraps ~8.95 s at 480 MHz. The PC must read at < 4 s intervals. */
typedef struct {
    volatile uint32_t lock;
    uint32_t cycles, hal_ms, rtos_tick, cmd_count, deadline_misses;
    uint32_t wall_max_cycles, hook_max_cycles;
} H7_Diag_Clock;

typedef struct {
    uint32_t count, min_cycles, max_cycles;
    float mean_cycles;
} H7_Diag_Cost;

typedef struct {
    uint32_t completed, status, counter_overhead_cycles;
    /* passthrough, quintic evaluate, cached Ruckig, forced replan */
    H7_Diag_Cost cost[4];
    float sink;
} H7_Diag_Benchmark;

extern volatile H7_Diag_Info h7_diag;
extern volatile H7_Diag_Clock h7_diag_clock;
extern volatile H7_Diag_Benchmark h7_diag_bench;
extern H7_Diag_Sample h7_diag_samples[H7_DIAG_CAPACITY];
extern const char h7_diag_build[];

void H7_Diag_Cmd_Begin(void);
void H7_Diag_Cmd_End(void);
void H7_Diag_Deadline_Miss(void);
void H7_Diag_Task(void* argument);
void H7_Diag_Link_Command(uint8_t arm_id, const uint8_t* payload, uint8_t len);
#else
#define H7_Diag_Cmd_Begin() ((void)0)
#define H7_Diag_Cmd_End() ((void)0)
#define H7_Diag_Deadline_Miss() ((void)0)
#endif
