/* SPDX-License-Identifier: LGPL-2.1-or-later */
#ifndef X5_DYNAMICS_H
#define X5_DYNAMICS_H

#include "x5_gravity.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    X5_DYNAMICS_OK = 0,
    X5_DYNAMICS_INVALID_ARGUMENT = -1,
    X5_DYNAMICS_ANGLE_RANGE = -2,
    X5_DYNAMICS_NONFINITE_RESULT = -3
};

/* 固定基座六轴逆动力学，关节顺序 joint1..joint6。
 * q、qd、qdd 单位为 rad、rad/s、rad/s²，torque 单位为 N·m。
 * gravity_base 为基座系重力加速度，NULL 表示 (0, 0, -9.81) m/s²。
 * SDK 模式的重力项乘现有标定系数。模型采用 base_link 到 link6 的 URDF 惯量。
 * 输入和输出须为独立数组；错误时保持输出。计算使用固定栈空间。
 */
int x5_dynamics_compute(x5_model model, const x5_real q[6],
    const x5_real qd[6], const x5_real qdd[6],
    const x5_real gravity_base[3], x5_gravity_mode mode, x5_real torque[6]);

#ifdef __cplusplus
}
#endif
#endif
