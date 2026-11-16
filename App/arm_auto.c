#include "arm_auto.h"

/* 按步骤启动并跟踪关节复位轨迹。 */
void Arm_Reset(Gimbal_cmd* obj) {
    const float duration = obj->gimbal->profile->init.reset_time_s;
    /* 自动模式切入时从首步骤开始。 */
    if (obj->last_auto_mode != obj->auto_mode) obj->auto_step = 0;
    /* 按复位步骤分派规划和结束处理。 */
    switch (obj->auto_step) {
    /* 创建从反馈位置到配置复位姿态的规划。 */
    case 0: {
        obj->auto_failed = 0;
        /* 规划资源不足时报告自动动作失败。 */
        if (obj->temp_group == NULL || get_Rest_Temp_Plan_cnt() < ARM_JOINT_NUM) {
            obj->auto_failed = 1;
            obj->auto_mode = arm_auto_none;
            break;
        }
        float start[ARM_JOINT_NUM], end[ARM_JOINT_NUM];
        /* 收集各轴起点、终点并清空规划句柄。 */
        for (int i = 0; i < ARM_JOINT_NUM; ++i) {
            start[i] = obj->gimbal->arm_feedback_data.joint_angle[i];
            end[i] = obj->gimbal->profile->joints[i].reset_deg;
            obj->temp_group[i] = NULL;
        }
        /* 由 MovePlan 同时启动各轴五次规划。 */
        Plan_Group_Temp_Calc_And_Run(obj->temp_group, ARM_JOINT_NUM, start, end,
            duration, quintic_spline);
        for (int i = 0; i < ARM_JOINT_NUM; ++i) {
            if (obj->temp_group[i] == NULL) {
                obj->auto_failed = 1;
                obj->auto_mode = arm_auto_none;
                return;
            }
        }
        for (int i = 0; i < ARM_JOINT_NUM; ++i)
            obj->temp_group[i]->retain_output = 1;
        obj->auto_step = 1;
        break;
    }
    /* 等待所有轴发布终点。 */
    case 1:
        for (int i = 0; i < ARM_JOINT_NUM; ++i)
            if (obj->temp_group[i]->status == move_plan_run) return;
        obj->auto_step = 2;
        break;
    /* 清除自动模式并复位步骤索引。 */
    default:
        obj->auto_mode = arm_auto_none;
        obj->auto_step = 0;
        break;
    }
}
