/* Host harness: production application and unchanged DM driver, simulated CAN/HAL. */
#include <string.h>
#include "../App/gimbal_arm.c"
#include "../App/arm_zero.c"
#include "../libs/dm_motor/DM_motor.c"

#define EXPORT __attribute__((visibility("default")))
Arm g_arm[ARM_NUM];
static uint32_t clock_ms;
static can_rx_callback receivers[DEVICE_CAN_CNT];
static uint8_t frames[ARM_NUM][ARM_MOTOR_NUM][8];
static unsigned controls[ARM_NUM][ARM_MOTOR_NUM], enables[ARM_NUM][ARM_MOTOR_NUM];
static unsigned burst[ARM_NUM], peak[ARM_NUM];
static uint8_t saved_zero[ARM_NUM], emulate_zero;

uint32_t HAL_GetTick(void) { return clock_ms; }
void FrameRateStatistics(FPS_t* fps) { fps->now = clock_ms; fps->count++; }
void BSP_CAN_RegisterRxCallback(uint8_t bus, can_rx_callback fn) { receivers[bus] = fn; }
void BSP_CAN_AddFilter(uint8_t bus, uint16_t id) { (void)bus; (void)id; }
uint8_t BSP_CAN_TxGroupMatch(uint8_t group) { return (group >> (clock_ms % 8)) & 1; }

static DM_motor* motor_at(unsigned bus, unsigned index) { return dm_at(g_arm[bus].gimbal, index); }

void BSP_CAN_Send(uint8_t bus, uint16_t id, uint8_t* data, basic_data_t len) {
    if (bus >= ARM_NUM || id < 1 || id > 7 || len != 8) abort();
    const uint8_t special[7] = {255,255,255,255,255,255,255};
    unsigned i = id-1;
    burst[bus]++;
    if (burst[bus] > peak[bus]) peak[bus] = burst[bus];
    if (!memcmp(data, special, 7) && data[7] >= 0xfc && data[7] <= 0xfe) {
        if (data[7] == 0xfc) enables[bus][i]++;
        if (data[7] == 0xfe) saved_zero[bus] |= 1u << i;
        if (emulate_zero) {
            uint8_t feedback[8];
            memcpy(feedback, motor_at(bus,i)->rx_data, 8);
            feedback[0] = id | (data[7] == 0xfc ? 0x10 : 0);
            if (data[7] == 0xfe) { feedback[1] = 0x7f; feedback[2] = 0xff; }
            receivers[bus](bus, 0x11+i, feedback, 8);
        }
    } else {
        memcpy(frames[bus][i], data, 8);
        controls[bus][i]++;
    }
}

EXPORT void metal_fixture_init(void) {
    Monitor_Init();
    DMmotor_Driver_Init();
    for (unsigned bus = 0; bus < ARM_NUM; ++bus) {
        g_arm[bus].gimbal = Gimbal_Create(bus);
        if (!g_arm[bus].gimbal) abort();
    }
}

EXPORT void metal_config(unsigned bus, unsigned i, float* out) {
    DM_motor* m = motor_at(bus,i);
    float values[] = {m->config.can_tx_id, m->config.can_rx_id,
        m->config.max_position, m->config.max_speed, m->config.max_torque,
        m->config.max_kp, m->config.max_kd, m->kp, m->kd, m->config.tx_group,
        m->motor_controller->pid_pos_data.config.KP,
        m->motor_controller->pid_speed_data.config.KP};
    memcpy(out, values, sizeof(values));
}

EXPORT void metal_feedback(unsigned bus, unsigned id, const uint8_t* data) {
    receivers[bus](bus, id, (uint8_t*)data, 8);
}

EXPORT void metal_position_speed(unsigned bus, unsigned i, float* out) {
    DM_motor* m = motor_at(bus,i);
    out[0] = m->config.pos_fdb_calc(m->config.arg) * DEG2RAD;
    out[1] = m->config.speed_fdb_calc(m->config.arg) * DEG2RAD;
}

EXPORT void metal_encode(unsigned bus, unsigned i, const float* refs, int rounds, uint8_t* out) {
    DM_motor* m = motor_at(bus,i);
    set_pid_normal(g_arm[bus].gimbal);
    m->enable = dm_enable;
    m->round = rounds;
    m->motor_controller->ref_position = refs[0] * RAD2DEG;
    m->motor_controller->speed_comp = refs[1] * RAD2DEG;
    m->motor_controller->output_comp = refs[2];
    /* Conversion round trips are part of the application path. */
    clock_ms++;
    while (!BSP_CAN_TxGroupMatch(m->config.tx_group)) clock_ms++;
    DMmotor_Send_Begin();
    for (unsigned n=0; n<16 && DMmotor_Send_Step(bus); ++n) {}
    memcpy(out, frames[bus][i], 8);
}

EXPORT void metal_rates(unsigned duration, unsigned* out) {
    clock_ms = (clock_ms+8) & ~7u;
    memset(controls, 0, sizeof(controls));
    memset(enables, 0, sizeof(enables));
    memset(peak, 0, sizeof(peak));
    memset(dm_enable_time, 0, sizeof(dm_enable_time));
    for (unsigned tick=0; tick<duration; ++tick, ++clock_ms) {
        memset(burst, 0, sizeof(burst));
        DMmotor_Send_Begin();
        /* Same four-round budget and two-bus order as MotorTask. */
        for (unsigned round=0; round<CAN_TX_MAX_FRAMES_PER_TICK; ++round)
            for (unsigned bus=0; bus<ARM_NUM; ++bus) DMmotor_Send_Step(bus);
    }
    for (unsigned bus=0; bus<ARM_NUM; ++bus) {
        memcpy(out+bus*15, controls[bus], 7*sizeof(unsigned));
        memcpy(out+bus*15+7, enables[bus], 7*sizeof(unsigned));
        out[bus*15+14] = peak[bus];
    }
}

EXPORT void metal_mode(unsigned bus, unsigned mode, float* out) {
    Gimbal* g = g_arm[bus].gimbal;
    for (unsigned i=0; i<7; ++i) motor_at(bus,i)->monitor->reset(motor_at(bus,i)->monitor);
    g->arm_control_data.mode = mode;
    Gimbal_Update(g);
    for (unsigned i=0; i<7; ++i) {
        DM_motor* m = motor_at(bus,i);
        out[i*2] = m->kp; out[i*2+1] = m->kd;
    }
}

EXPORT void metal_zero(unsigned bus, unsigned mask, unsigned* out) {
    Gimbal* g = g_arm[bus].gimbal;
    memset(&g->zero, 0, sizeof(g->zero));
    memset(saved_zero, 0, sizeof(saved_zero));
    for (unsigned i=0; i<7; ++i) {
        uint8_t data[8] = {i+1, 0x70, 0x00, 0x7f, 0xf7, 0xff, 25, 25};
        receivers[bus](bus, 0x11+i, data, 8);
    }
    g->zero.request = 1;
    g->zero.participate_override = mask;
    emulate_zero = 1;
    for (unsigned n=0; n<3000; ++n, ++clock_ms) {
        Arm_Zero_Loop(g);
        if (!Arm_Zero_Busy(g) && !g->zero.request) break;
    }
    emulate_zero = 0;
    out[0] = g->zero.state;
    out[1] = g->zero.participate_mask;
    out[2] = saved_zero[bus];
    out[3] = saved_zero[1-bus];
    for (unsigned i=0; i<7; ++i) out[4+i] = g->zero.result[i];
}
