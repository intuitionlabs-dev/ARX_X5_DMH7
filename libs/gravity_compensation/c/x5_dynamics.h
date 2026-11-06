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

/* joint1～joint6 顺序；q、qd、qdd 为 rad、rad/s、rad/s²，输出为 N·m。
 * 基座重力为 m/s²，NULL 使用 (0,0,-9.81)；输入输出数组独立，错误时保持输出。
 * SDK 模式缩放重力项，计算使用固定栈空间。 */
int x5_dynamics_compute(x5_model model, const x5_real q[6],
    const x5_real qd[6], const x5_real qdd[6],
    const x5_real gravity_base[3], x5_gravity_mode mode, x5_real torque[6]);

#ifdef __cplusplus
}
#endif
#endif
