#pragma once
#include <stdint.h>
#include "move_plan.h"
/**
 * @file plan_group.h
 * @author zsy
 * @brief 轨迹规划组接口使用指南
 * @version 2.0
 * @date 2026-03-23
 *
 * 本库基于move_plan,用于批量创建和管理多个独立的运动规划（Move_plan），适合多电机同步轨迹控制。
 * 类型说明：
    * Plan_group：常规规划组，包含多个Move_plan对象，适用于需要重复执行的轨迹。
    * Plan_group_temp：临时规划组，包含多个Move_plan对象，适用于一次性的轨迹调整。
 * 

 */

typedef Move_plan** Plan_group;
typedef Move_plan** Plan_group_temp;

/* Plan_Group 常规规划组 */
Plan_group Plan_Group_Create(const uint8_t plan_cnt, const uint8_t via_pos_cnt,
    const float via_pos[plan_cnt][via_pos_cnt], const float duration_s[via_pos_cnt - 1],
    const Line_type line_type);
void Plan_Group_Run(Plan_group group, uint8_t plan_cnt);
void Plan_Group_Reset(Plan_group group, uint8_t plan_cnt);

/* Plan_Group_Temp 临时规划组 */
Plan_group_temp Plan_Group_Temp_Create(uint8_t plan_cnt);
void Plan_Group_Temp_Calc_And_Run(Plan_group_temp group_temp, uint8_t plan_cnt,
    float start_pos[plan_cnt], float end_pos[plan_cnt], float duration_s,
    Line_type line_type);
void Plan_Group_Temp_Reset(Plan_group_temp group_temp, uint8_t plan_cnt);
