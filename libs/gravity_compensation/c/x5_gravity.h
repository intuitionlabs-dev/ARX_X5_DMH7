#ifndef X5_GRAVITY_H
#define X5_GRAVITY_H

/* 输入采用 URDF 关节坐标，输出 joint1～joint6 的重力力矩，单位 N·m。 */
typedef float x5_real;

#define X5_GRAVITY_DOF 6
/* Bounded range reduction, deliberately not a general-purpose sin/cos API. */
#define X5_GRAVITY_MAX_ANGLE 128.0f

typedef enum {
    X5_MODEL_2023 = 0,
    X5_MODEL_MASTER = 1,
    X5_MODEL_2025 = 2
} x5_model;

typedef enum {
    X5_GRAVITY_PHYSICAL = 0,
    X5_GRAVITY_SDK = 1
} x5_gravity_mode;

enum {
    X5_GRAVITY_OK = 0,
    X5_GRAVITY_INVALID_ARGUMENT = -1,
    X5_GRAVITY_ANGLE_RANGE = -2
};

#ifdef __cplusplus
extern "C" {
#endif

#ifndef X5_GRAVITY_NO_TRIG
/* q 为六个有限 rad 值且绝对值不超过 MAX_ANGLE；基座重力为 m/s²，NULL 使用 (0,0,-9.81)。
 * torque 须可写入六个元素，参数校验失败时保持输出。 */
int x5_gravity_compute(x5_model model, const x5_real q[6],
                       const x5_real gravity_base[3], x5_gravity_mode mode,
                       x5_real torque[6]);
#endif

/* 调用方须提供一致的单位圆正余弦对；接口仅检查有限值及 [-1,1] 范围。
 * X5_GRAVITY_NO_TRIG 可关闭角度输入接口。 */
int x5_gravity_compute_sincos(x5_model model, const x5_real sin_q[6],
                              const x5_real cos_q[6], const x5_real gravity_base[3],
                              x5_gravity_mode mode, x5_real torque[6]);

#ifdef __cplusplus
}
#endif
#endif
