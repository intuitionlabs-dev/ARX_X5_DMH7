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

/* J1～J6 顺序；q、qd、qdd 为 rad、rad/s、rad/s²，输出为 N·m。
 * 基座重力为 m/s²，NULL 使用 (0,0,-9.81)；输入输出数组独立，错误时保持输出。
 * SDK 模式按标定系数缩放重力、科氏力并补偿摩擦。 */
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
