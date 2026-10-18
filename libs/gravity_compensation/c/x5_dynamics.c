/* SPDX-License-Identifier: LGPL-2.1-or-later
 * Copyright (C) 2009 Ruben Smits and Dominick Vanthienen.
 * 基于 Orocos KDL v1.5.1 ChainIdSolver_RNE 的六轴 C 实现。
 */
#include "x5_dynamics.h"
#include <math.h>

typedef struct { x5_real x, y, z; } vector3;
typedef struct { x5_real data[3][3]; } rotation3;
typedef struct { vector3 vel, rot; } twist;
typedef struct { vector3 force, torque; } wrench;
typedef struct {
    x5_real mass;
    vector3 com, origin, axis;
    rotation3 fixed_rotation, inertia;
} link_data;

#include "x5_dynamics_data.h"

static vector3 add(vector3 a, vector3 b) {
    vector3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static vector3 scale(vector3 a, x5_real s) {
    vector3 r = {a.x * s, a.y * s, a.z * s};
    return r;
}

static vector3 cross(vector3 a, vector3 b) {
    vector3 r = {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
    return r;
}

static x5_real dot(vector3 a, vector3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static vector3 rotate(const rotation3* r, vector3 v) {
    vector3 out = {
        r->data[0][0]*v.x + r->data[0][1]*v.y + r->data[0][2]*v.z,
        r->data[1][0]*v.x + r->data[1][1]*v.y + r->data[1][2]*v.z,
        r->data[2][0]*v.x + r->data[2][1]*v.y + r->data[2][2]*v.z
    };
    return out;
}

static vector3 inverse_rotate(const rotation3* r, vector3 v) {
    vector3 out = {
        r->data[0][0]*v.x + r->data[1][0]*v.y + r->data[2][0]*v.z,
        r->data[0][1]*v.x + r->data[1][1]*v.y + r->data[2][1]*v.z,
        r->data[0][2]*v.x + r->data[1][2]*v.y + r->data[2][2]*v.z
    };
    return out;
}

static rotation3 joint_rotation(const link_data* link, x5_real q) {
    const x5_real s = sinf(q), c = cosf(q), d = 1.0f - c;
    const x5_real x = link->axis.x, y = link->axis.y, z = link->axis.z;
    const rotation3 joint = {{{c+x*x*d, x*y*d-z*s, x*z*d+y*s},
                              {y*x*d+z*s, c+y*y*d, y*z*d-x*s},
                              {z*x*d-y*s, z*y*d+x*s, c+z*z*d}}};
    rotation3 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            out.data[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
                out.data[i][j] += link->fixed_rotation.data[i][k] * joint.data[k][j];
        }
    return out;
}

/* KDL Frame::Inverse(Twist)：空间量的参考点随关节原点平移。 */
static twist inverse_transform(const rotation3* r, vector3 p, twist t) {
    twist out = {inverse_rotate(r, add(t.vel, cross(t.rot, p))),
                 inverse_rotate(r, t.rot)};
    return out;
}

/* KDL RigidBodyInertia * Twist，inertia 为质心处的连杆系惯量。 */
static wrench inertia_multiply(const link_data* link, twist t) {
    wrench out;
    out.force = scale(add(t.vel, cross(t.rot, link->com)), link->mass);
    out.torque = add(rotate(&link->inertia, t.rot), cross(link->com, out.force));
    return out;
}

int x5_dynamics_compute(x5_model model, const x5_real q[6],
    const x5_real qd[6], const x5_real qdd[6], const x5_real gravity_base[3],
    x5_gravity_mode mode, x5_real torque[6]) {
    if ((unsigned)model > X5_MODEL_2025 || !q || !qd || !qdd || !torque
        || (mode != X5_GRAVITY_PHYSICAL && mode != X5_GRAVITY_SDK))
        return X5_DYNAMICS_INVALID_ARGUMENT;
    for (int i = 0; i < 6; ++i) {
        if (!isfinite(q[i]) || !isfinite(qd[i]) || !isfinite(qdd[i]))
            return X5_DYNAMICS_INVALID_ARGUMENT;
        if (fabsf(q[i]) > X5_GRAVITY_MAX_ANGLE)
            return X5_DYNAMICS_ANGLE_RANGE;
    }
    if (gravity_base)
        for (int i = 0; i < 3; ++i)
            if (!isfinite(gravity_base[i])) return X5_DYNAMICS_INVALID_ARGUMENT;

    rotation3 rotations[6];
    wrench forces[6];
    const vector3 zero = {0.0f, 0.0f, 0.0f};
    twist velocity = {zero, zero};
    twist acceleration = {{0.0f, 0.0f, 9.81f}, {0.0f, 0.0f, 0.0f}};
    const link_data* links = x5_dynamic_links[model];
    x5_real result[6];
    if (gravity_base) {
        acceleration.vel.x = -gravity_base[0];
        acceleration.vel.y = -gravity_base[1];
        acceleration.vel.z = -gravity_base[2];
    }

    /* KDL 正向递推：v=X⁻¹v_parent+S*qdot，a=X⁻¹a_parent+S*qddot+v×vj。 */
    for (int i = 0; i < 6; ++i) {
        const link_data* link = &links[i];
        rotations[i] = joint_rotation(link, q[i]);
        velocity = inverse_transform(&rotations[i], link->origin, velocity);
        acceleration = inverse_transform(&rotations[i], link->origin, acceleration);
        const vector3 joint_speed = scale(link->axis, qd[i]);
        velocity.rot = add(velocity.rot, joint_speed);
        acceleration.vel = add(acceleration.vel, cross(velocity.vel, joint_speed));
        acceleration.rot = add(acceleration.rot,
            add(scale(link->axis, qdd[i]), cross(velocity.rot, joint_speed)));
        forces[i] = inertia_multiply(link, acceleration);
        const wrench momentum = inertia_multiply(link, velocity);
        forces[i].force = add(forces[i].force, cross(velocity.rot, momentum.force));
        forces[i].torque = add(forces[i].torque,
            add(cross(velocity.rot, momentum.torque), cross(velocity.vel, momentum.force)));
    }

    /* KDL 反向递推：tau=Sᵀf，f_parent+=X*f。 */
    for (int i = 5; i >= 0; --i) {
        result[i] = dot(links[i].axis, forces[i].torque);
        if (i > 0) {
            const vector3 parent_force = rotate(&rotations[i], forces[i].force);
            forces[i-1].force = add(forces[i-1].force, parent_force);
            forces[i-1].torque = add(forces[i-1].torque,
                add(rotate(&rotations[i], forces[i].torque),
                    cross(links[i].origin, parent_force)));
        }
    }
    if (mode == X5_GRAVITY_SDK) {
        x5_real gravity[6];
        const int status = x5_gravity_compute(model, q, gravity_base,
            X5_GRAVITY_PHYSICAL, gravity);
        if (status != X5_GRAVITY_OK) return status;
        for (int i = 0; i < 6; ++i)
            result[i] += ((i < 3 ? 0.8f : 1.32f) - 1.0f) * gravity[i];
    }
    for (int i = 0; i < 6; ++i)
        if (!isfinite(result[i])) return X5_DYNAMICS_NONFINITE_RESULT;
    for (int i = 0; i < 6; ++i) torque[i] = result[i];
    return X5_DYNAMICS_OK;
}
