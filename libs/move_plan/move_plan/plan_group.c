#include "plan_group.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "cvector.h"
/* #include "debug/debug_log.h" */  /* 本工程 printf 是坏的（__io_putchar 没定义），
                                     * 下面所有 printf_log 都注释掉了 */
#include "move_plan.h"

/* 源工程这里原本是 `#if _RT_MOVEPLAN_ENABLE` ... `#endif` 整个包起来的。
 * 本工程没有这个宏，条件恒为 0，整个文件会被编成一个空的目标文件（实测 nm 一个符号都没有），
 * 所以把这对条件编译删掉。move_plan.c 里的同一对宏也已经删了，保持一致。 */

Plan_group Plan_Group_Create(const uint8_t plan_cnt, const uint8_t via_pos_cnt,
    const float via_pos[plan_cnt][via_pos_cnt], const float duration_s[via_pos_cnt - 1],
    const Line_type line_type) {
    const uint8_t via_line_cnt = via_pos_cnt - 1;  // 通过段数量比通过点数量少1

    if (via_pos_cnt < 2) {
        /* printf_log("via_pos_cnt must be at least 2 in %s", __func__); */
        return NULL;  // 如果通过点数量小于2返回NULL
    }
    for (uint8_t j = 0; j < via_line_cnt; j++) {
        if (duration_s[j] <= 0) {
            /* printf_log("invalid duration %d in %s", j, __func__); */
            return NULL;  // 如果持续时间不合法，返回NULL
        }
    }

    Move_plan_config** config_array =
        (Move_plan_config**)malloc(sizeof(Move_plan_config*) * plan_cnt);
    memset(config_array, 0, sizeof(Move_plan_config*) * plan_cnt);
    Move_plan** plangroup = (Move_plan**)malloc(sizeof(Move_plan*) * plan_cnt);
    memset(plangroup, 0, sizeof(Move_plan*) * plan_cnt);

    for (uint8_t i = 0; i < plan_cnt; i++) {
        config_array[i] = Move_Plan_Config_Create(line_type);
        if (config_array[i] == NULL) {
            /* printf_log("failed to create move plan config %d in %s", i, __func__); */
            return NULL;  // 如果配置创建失败，返回NULL
        }
        /* 添加点与段 */
        Add_Via_Point(config_array[i], via_pos[i][0], 0, 0);  // 添加起始点,默认速度和加速度为0
        for (uint8_t j = 1; j < via_pos_cnt; j++) {
            Add_Via_Point(config_array[i], via_pos[i][j], 0, 0);
            Add_Via_Line(config_array[i], duration_s[j-1]);
        }

        /* 自动计算点的速度与加速度 */
        float ave_vel[via_line_cnt];  // 每段的平均速度
        float via_vel[via_pos_cnt];   // 每个通过点的速度
        memset(ave_vel, 0, sizeof(float) * via_line_cnt);
        memset(via_vel, 0, sizeof(float) * via_pos_cnt);

        // 计算每段的平均速度
        for (uint8_t j = 0; j < via_line_cnt; j++) {  // 计算后续段的平均速度
            ave_vel[j] = (via_pos[i][j + 1] - via_pos[i][j]) / duration_s[j];
        }
        // 计算除了始终点外每个通过点的速度，使用前后两段的平均速度，如果前后两段速度方向相反或为0，则设置为0
        // 始终点的速度不计算，已经初始化为0
        for (uint8_t j = 1; j < via_pos_cnt - 1; j++) {
            Via_point* point = (Via_point*)cvector_val_at(config_array[i]->via_points, j);
            if (ave_vel[j - 1] * ave_vel[j] < 0 || fabsf(ave_vel[j - 1]) < 1e-4
                || fabsf(ave_vel[j]) < 1e-4) {
                // 如果前后两段速度方向相反或为0，设置为0
                point->vel = 0;
            } else {
                // 否则设置为前后两段平均速度的平均值
                point->vel = (ave_vel[j - 1] + ave_vel[j]) / 2.0;
            }
            via_vel[j] = point->vel;  // 存储通过点的速度
        }

        // 计算每段的平均加速度
        float ave_acc[via_line_cnt];
        memset(ave_acc, 0, sizeof(float) * via_line_cnt);
        for (uint8_t j = 0; j < via_line_cnt; j++) {  // 计算每一段的平均加速度
            ave_acc[j] = (via_vel[j + 1] - via_vel[j]) / duration_s[j];
        }
        // 计算除了始终点外每个通过点的加速度，使用前后两段的平均速度，如果前后两段加速度方向相反或为0，则设置为0
        // 始终点的加速度不计算，已经初始化为0
        for (uint8_t j = 1; j < via_pos_cnt - 1; j++) {
            Via_point* point = (Via_point*)cvector_val_at(config_array[i]->via_points, j);
            if (ave_acc[j - 1] * ave_acc[j] < 0 || fabsf(ave_acc[j - 1]) < 1e-4
                || fabsf(ave_acc[j]) < 1e-4) {
                // 如果前后两段加速度方向相反或为0，设置为0
                point->acc = 0;
            } else {
                // 否则设置为前后两段平均加速度的平均值
                point->acc = (ave_acc[j - 1] + ave_acc[j]) / 2.0;
            }
        }

        plangroup[i] = Move_Plan_Create(config_array[i]);
    }
    free(config_array);  // 释放配置数组内存
    return plangroup;
}

Plan_group_temp Plan_Group_Temp_Create(uint8_t plan_cnt) {
    if (plan_cnt < 1) {
        /* printf_log("plan_cnt must be at least 1 in %s", __func__); */
        return NULL;  // 如果规划数量小于1返回NULL
    }
    Plan_group_temp temp_group = (Plan_group_temp)malloc(sizeof(Move_plan*) * plan_cnt);
    memset(temp_group, 0, sizeof(Move_plan*) * plan_cnt);
    return temp_group;
}

void Plan_Group_Temp_Calc_And_Run(Plan_group_temp group_temp, uint8_t plan_cnt,
    float start_pos[plan_cnt], float end_pos[plan_cnt], float duration_s,
    Line_type line_type) {
    if (duration_s <= 0 || plan_cnt < 1 || start_pos == NULL || end_pos == NULL) {
        /* printf_log("invalid parameters in %s", __func__); */
        return;  // 如果持续时间不合法或数量不合法，返回
    }
    if (get_Rest_Temp_Plan_cnt() < plan_cnt) {
        /* printf_log("not enough temporary plans available in %s", __func__); */
        return;  // 如果剩余临时规划数量不足，返回
    }

    for (uint8_t i = 0; i < plan_cnt; i++) {
        // 创建临时规划，默认起始速度和加速度为0
        group_temp[i] = Move_Plan_Temp_Create(start_pos[i], 0, 0, end_pos[i], 0, 0,
            duration_s, line_type);
    }
}

void Plan_Group_Run(Plan_group group, uint8_t plan_cnt) {
    for (uint8_t i = 0; i < plan_cnt; i++) {
        if (group[i] != NULL) {
            Run_Move_Plan(group[i]);
        }
    }
}

void Plan_Group_Reset(Plan_group group, uint8_t plan_cnt) {
    for (uint8_t i = 0; i < plan_cnt; i++) {
        if (group[i] != NULL) {
            Reset_Move_Plan(group[i]);
        }
    }
}

void Plan_Group_Temp_Reset(Plan_group_temp group_temp, uint8_t plan_cnt) {
    Plan_Group_Reset((Plan_group)group_temp, plan_cnt);
}