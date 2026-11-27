#include "joint_post_process.h"

#include <stddef.h>
#include <string.h>
#include "algorithm.h"
#include "gimbal_cmd.h"

static const struct Joint_post_process_ops* select_joint_post_process(uint32_t algorithm) {
    switch (algorithm) {
    case ARM_POST_PROCESS_RUCKIG: return &joint_ruckig_ops;
    case ARM_POST_PROCESS_QUINTIC: return &joint_quintic_ops;
    case ARM_POST_PROCESS_DIRECT: return &joint_direct_ops;
    default: return NULL;
    }
}

/* 根据机型配置初始化关节轨迹算法。 */
int Joint_Post_Process_Init(Gimbal_cmd* obj) {
    return Joint_Post_Process_Set(obj, obj->gimbal->profile->joint_post_process);
}

int Joint_Post_Process_Set(Gimbal_cmd* obj, uint32_t algorithm) {
    const struct Joint_post_process_ops* ops = select_joint_post_process(algorithm);
    if (ops == NULL) return 0;
    if (obj->post_process == ops) return 1;
    for (int i = 0; i < ARM_JOINT_NUM; ++i) {
        const Arm_joint_config* joint = &obj->gimbal->profile->joints[i];
        if (algorithm == ARM_POST_PROCESS_RUCKIG
            && (!isfinite(joint->max_velocity_deg_s) || joint->max_velocity_deg_s <= 0
                || !isfinite(joint->max_acceleration_deg_s2) || joint->max_acceleration_deg_s2 <= 0
                || !isfinite(joint->max_jerk_deg_s3) || joint->max_jerk_deg_s3 <= 0)) return 0;
        if (algorithm == ARM_POST_PROCESS_QUINTIC
            && (!isfinite(joint->quintic_duration_s) || joint->quintic_duration_s <= 0)) return 0;
    }
    Joint_post_process_ref refs[ARM_JOINT_NUM] = {0};
    if (ops->init && !ops->init(refs)) return 0;
    if (obj->post_process && obj->post_process->deinit)
        obj->post_process->deinit(obj->joint_post_ref);
    memcpy(obj->joint_post_ref, refs, sizeof(refs));
    obj->post_process = ops;
    obj->post_process_algorithm = (Arm_post_process_algorithm)algorithm;
    return 1;
}
