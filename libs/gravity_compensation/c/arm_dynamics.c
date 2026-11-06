/* 使用连杆坐标递归计算串联机械臂的逆动力学。 */
#include "arm_dynamics.h"
#include <math.h>
typedef struct { Arm_vector3 vel, rot; } twist; // NOLINT(readability-identifier-naming)

static Arm_vector3 add(Arm_vector3 a, Arm_vector3 b) {
    Arm_vector3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static Arm_vector3 scale(Arm_vector3 a, float s) {
    Arm_vector3 r = {a.x * s, a.y * s, a.z * s};
    return r;
}

static Arm_vector3 cross(Arm_vector3 a, Arm_vector3 b) {
    Arm_vector3 r = {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
    return r;
}

static float dot(Arm_vector3 a, Arm_vector3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

static Arm_vector3 rotate(const Arm_rotation3* r, Arm_vector3 v) {
    Arm_vector3 out = {
        r->data[0][0]*v.x + r->data[0][1]*v.y + r->data[0][2]*v.z,
        r->data[1][0]*v.x + r->data[1][1]*v.y + r->data[1][2]*v.z,
        r->data[2][0]*v.x + r->data[2][1]*v.y + r->data[2][2]*v.z
    };
    return out;
}

static Arm_vector3 inverse_rotate(const Arm_rotation3* r, Arm_vector3 v) {
    Arm_vector3 out = {
        r->data[0][0]*v.x + r->data[1][0]*v.y + r->data[2][0]*v.z,
        r->data[0][1]*v.x + r->data[1][1]*v.y + r->data[2][1]*v.z,
        r->data[0][2]*v.x + r->data[1][2]*v.y + r->data[2][2]*v.z
    };
    return out;
}

static Arm_rotation3 joint_rotation(const Arm_link* link, float q) {
    const float s = sinf(q), c = cosf(q), d = 1.0f - c;
    const float x = link->axis.x, y = link->axis.y, z = link->axis.z;
    const Arm_rotation3 joint = {{{c+x*x*d, x*y*d-z*s, x*z*d+y*s},
                              {y*x*d+z*s, c+y*y*d, y*z*d-x*s},
                              {z*x*d-y*s, z*y*d+x*s, c+z*z*d}}};
    Arm_rotation3 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            out.data[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k)
                out.data[i][j] += link->fixed_rotation.data[i][k] * joint.data[k][j];
        }
    return out;
}

/* KDL Frame::Inverse(Twist)：空间量的参考点随关节原点平移。 */
static twist inverse_transform(const Arm_rotation3* r, Arm_vector3 p, twist t) {
    twist out = {inverse_rotate(r, add(t.vel, cross(t.rot, p))),
                 inverse_rotate(r, t.rot)};
    return out;
}

/* KDL RigidBodyInertia * Twist，inertia 为质心处的连杆系惯量。 */
static Arm_wrench inertia_multiply(const Arm_link* link, twist t) {
    Arm_wrench out;
    out.force = scale(add(t.vel, cross(t.rot, link->com)), link->mass);
    out.torque = add(rotate(&link->inertia, t.rot), cross(link->com, out.force));
    return out;
}

static void rne(const Arm_dynamics_model* model, Arm_dynamics_work* work,
    const float* qd, const float* qdd, const float gravity_base[3], int output) {
    const Arm_link* links = model->links;
    const int count = (int)model->joint_count;
    const Arm_vector3 zero = {0.0f, 0.0f, 0.0f};
    twist velocity = {zero, zero};
    twist acceleration = {{0.0f, 0.0f, 9.81f}, {0.0f, 0.0f, 0.0f}};
    if (gravity_base) {
        acceleration.vel.x = -gravity_base[0];
        acceleration.vel.y = -gravity_base[1];
        acceleration.vel.z = -gravity_base[2];
    }

    /* KDL 正向递推：v=X⁻¹v_parent+S*qdot，a=X⁻¹a_parent+S*qddot+v×vj。 */
    for (int i = 0; i < count; ++i) {
        const Arm_link* link = &links[i];
        velocity = inverse_transform(&work[i].rotation, link->origin, velocity);
        acceleration = inverse_transform(&work[i].rotation, link->origin, acceleration);
        const Arm_vector3 joint_speed = scale(link->axis, qd ? qd[i] : 0.0f);
        velocity.rot = add(velocity.rot, joint_speed);
        acceleration.vel = add(acceleration.vel, cross(velocity.vel, joint_speed));
        acceleration.rot = add(acceleration.rot,
            add(scale(link->axis, qdd ? qdd[i] : 0.0f), cross(velocity.rot, joint_speed)));
        work[i].force = inertia_multiply(link, acceleration);
        const Arm_wrench momentum = inertia_multiply(link, velocity);
        work[i].force.force = add(work[i].force.force, cross(velocity.rot, momentum.force));
        work[i].force.torque = add(work[i].force.torque,
            add(cross(velocity.rot, momentum.torque), cross(velocity.vel, momentum.force)));
    }

    /* KDL 反向递推：tau=Sᵀf，f_parent+=X*f。 */
    for (int i = count - 1; i >= 0; --i) {
        const float torque = dot(links[i].axis, work[i].force.torque);
        if (output == 0) work[i].total = torque;
        else if (output == 1) work[i].gravity = torque;
        else work[i].velocity = torque;
        if (i > 0) {
            const Arm_vector3 parent_force = rotate(&work[i].rotation, work[i].force.force);
            work[i-1].force.force = add(work[i-1].force.force, parent_force);
            work[i-1].force.torque = add(work[i-1].force.torque,
                add(rotate(&work[i].rotation, work[i].force.torque),
                    cross(links[i].origin, parent_force)));
        }
    }
}

int Arm_Dynamics_Compute(const Arm_dynamics_model* model, const float* q,
    const float* qd, const float* qdd, const float gravity[3], int gravity_only,
    float* tau, Arm_dynamics_work* work, size_t capacity) {
    if (!model || !model->links || !q || !qd || !qdd || !tau || !work
        || model->joint_count == 0 || model->joint_count > capacity)
        return ARM_DYNAMICS_INVALID;
    for (size_t i = 0; i < model->joint_count; ++i) {
        if (!isfinite(q[i]) || !isfinite(qd[i]) || !isfinite(qdd[i]))
            return ARM_DYNAMICS_INVALID;
        if (fabsf(q[i]) > 128.0f) return ARM_DYNAMICS_RANGE;
        work[i].rotation = joint_rotation(&model->links[i], q[i]);
    }
    if (gravity)
        for (int i = 0; i < 3; ++i)
            if (!isfinite(gravity[i])) return ARM_DYNAMICS_INVALID;
    rne(model, work, NULL, NULL, gravity, 1);
    if (!gravity_only) {
        rne(model, work, qd, qdd, gravity, 0);
        rne(model, work, qd, NULL, gravity, 2);
    }
    for (size_t i = 0; i < model->joint_count; ++i) {
        const float gs = model->gravity_scale ? model->gravity_scale[i] : 1.0f;
        const float cs = model->coriolis_scale ? model->coriolis_scale[i] : 1.0f;
        float torque = work[i].gravity * gs;
        if (!gravity_only) {
            float speed = qd[i];
            const float limit = model->friction_speed_limit ? model->friction_speed_limit[i] : 0;
            if (limit > 0) speed = fmaxf(-limit, fminf(limit, speed));
            torque += work[i].total - work[i].velocity
                + (work[i].velocity - work[i].gravity) * cs
                + speed * (model->friction ? model->friction[i] : 0);
        }
        if (!isfinite(torque)) return ARM_DYNAMICS_NONFINITE;
        work[i].total = torque;
    }
    for (size_t i = 0; i < model->joint_count; ++i) tau[i] = work[i].total;
    return ARM_DYNAMICS_OK;
}
