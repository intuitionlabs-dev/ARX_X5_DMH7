/* 主机测试封装固件的 ENCOS 编解码和达妙量化。 */
#include "encos_protocol.h"
#include "common.h"

static const ENCOS_Ranges ranges = {{0, 500}, {0, 50}, {-12.5f, 12.5f},
                                   {-18, 18}, {-30, 30}, {-30, 30}};

int reference_encos(float p, float v, float t, float kp, float kd, uint8_t* out) {
    ENCOS_Frame frame;
    int status = ENCOS_PackMixed(&frame, 1, &ranges, kp, kd, p, v, t);
    if (!status) for (int i = 0; i < 8; ++i) out[i] = frame.data[i];
    return status;
}

int reference_encos_decode(const uint8_t* data, int length, float* out) {
    ENCOS_Feedback feedback;
    int status = ENCOS_Decode(data, length, &ranges, &feedback);
    if (status) return status;
    out[0] = feedback.position;
    out[1] = feedback.speed;
    out[2] = feedback.current;
    out[3] = feedback.motor_temp;
    out[4] = feedback.mos_temp;
    return feedback.valid;
}

void reference_dm(float p, float v, float t, float kp, float kd, uint8_t* out) {
    uint16_t position = float_to_uint(p, -12.5f, 12.5f, 16);
    uint16_t speed = float_to_uint(v, -45, 45, 12);
    uint16_t torque = float_to_uint(t, -10, 10, 12);
    uint16_t k = float_to_uint(kp, 0, 500, 12);
    uint16_t d = float_to_uint(kd, 0, 5, 12);
    out[0] = position >> 8;
    out[1] = position;
    out[2] = speed >> 4;
    out[3] = (speed << 4) | (k >> 8);
    out[4] = k;
    out[5] = d >> 4;
    out[6] = (d << 4) | (torque >> 8);
    out[7] = torque;
}
