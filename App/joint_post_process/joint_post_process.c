#include "joint_post_process.h"

#include <stddef.h>
#include "algorithm.h"
#include "gimbal_cmd.h"

static const struct Joint_post_process_ops* select_joint_post_process(Arm_post_process_algorithm algorithm) {
    switch (algorithm) {
    case ARM_POST_PROCESS_RUCKIG: return &joint_ruckig_ops;
    case ARM_POST_PROCESS_QUINTIC: return &joint_quintic_ops;
    default: return NULL;
    }
}

/* 根据机型配置初始化关节轨迹算法。 */
int Joint_Post_Process_Init(Gimbal_cmd* obj) {
    const struct Joint_post_process_ops* ops = select_joint_post_process(obj->gimbal->profile->joint_post_process);
    if (ops == NULL || (ops->init && !ops->init(obj->joint_post_ref))) return 0;
    obj->post_process = ops;
    return 1;
}
