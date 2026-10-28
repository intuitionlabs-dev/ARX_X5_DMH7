/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef METAL_DYNAMICS_H
#define METAL_DYNAMICS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef float metal_real;
#define METAL_GRAVITY_DOF 6
#define METAL_GRAVITY_MAX_ANGLE 128.0f

typedef enum {
    METAL_MODEL_WITH_GRIPPER = 0,
    METAL_MODEL_NO_GRIPPER = 1
} metal_model;

typedef enum {
    METAL_GRAVITY_PHYSICAL = 0,
    METAL_GRAVITY_SDK = 1
} metal_gravity_mode;

enum {
    METAL_DYNAMICS_OK = 0,
    METAL_DYNAMICS_INVALID_ARGUMENT = -1,
    METAL_DYNAMICS_ANGLE_RANGE = -2,
    METAL_DYNAMICS_NONFINITE_RESULT = -3
};

/* 关节顺序为 J1～J6；q、qd、qdd 单位为 rad、rad/s、rad/s²。
 * torque 单位为 N·m。gravity_base 为基座系重力，单位 m/s²。
 * gravity_base 为 NULL 时使用 (0, 0, -9.81)。输入与输出数组须独立。
 * SDK 模式缩放重力与科氏力，并补偿摩擦。错误时保持输出数组。
 */
int metal_dynamics_compute(metal_model model, const metal_real q[6],
    const metal_real qd[6], const metal_real qdd[6], const metal_real gravity_base[3],
    metal_gravity_mode mode, metal_real torque[6]);

/* 计算静态支撑力矩；SDK 模式使用重力缩放系数。 */
int metal_gravity_compute(metal_model model, const metal_real q[6],
    const metal_real gravity_base[3], metal_gravity_mode mode, metal_real torque[6]);

#ifdef __cplusplus
}
#endif
#endif
