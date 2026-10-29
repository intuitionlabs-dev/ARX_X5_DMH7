#pragma once
#include <stddef.h>

typedef struct { float x, y, z; } Arm_vector3;
typedef struct { float data[3][3]; } Arm_rotation3;
typedef struct { Arm_vector3 force, torque; } Arm_wrench;
typedef struct {
    float mass;
    Arm_vector3 com, origin, axis;
    Arm_rotation3 fixed_rotation, inertia;
} Arm_link;

typedef struct {
    size_t joint_count;
    const Arm_link* links;
    const float* gravity_scale;
    const float* coriolis_scale;
    const float* friction;
    const float* friction_speed_limit;
} Arm_dynamics_model;

/* 工作区由调用方按关节数分配，每台臂独立持有。 */
typedef struct {
    Arm_rotation3 rotation;
    Arm_wrench force;
    float total, gravity, velocity;
} Arm_dynamics_work;

enum {
    ARM_DYNAMICS_OK = 0, // NOLINT(readability-identifier-naming)
    ARM_DYNAMICS_INVALID = -1, // NOLINT(readability-identifier-naming)
    ARM_DYNAMICS_RANGE = -2, // NOLINT(readability-identifier-naming)
    ARM_DYNAMICS_NONFINITE = -3, // NOLINT(readability-identifier-naming)
};

/* q、qd、qdd 使用 rad、rad/s、rad/s²；tau 使用 N·m。 */
int Arm_Dynamics_Compute(const Arm_dynamics_model* model, const float* q,
    const float* qd, const float* qdd, const float gravity[3], int gravity_only,
    float* tau, Arm_dynamics_work* work, size_t capacity);
