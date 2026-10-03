#include "move_plan.h"
#include <stdint.h>
#include <string.h>
#include <time.h>
/* 源工程这里 include 的是 osal/time/bsp_time.h，拿 HAL_GetTick()。
 * 那个函数的函数体就是一句 `return HAL_GetTick();`（osal/time/bsp_time.c:13），
 * 所以本工程直接调 HAL_GetTick()，语义完全一样。 */
#include "cvector.h"
#include "main.h"
/* #include "debug/debug_log.h" */ /* 本工程 printf 是坏的（__io_putchar 没定义），
                                     * 下面所有 printf_log 都注释掉了 */

/* 私有函数 */
static void line_coef_calc(Move_plan_config *config, Line_type old_line_type);  // 计算线段系数
static uint8_t is_line_type_valid(Line_type line_type);  // 判断线段类型是否合法

#define MAX_TEMP_PLAN_NUM 30  // 最大临时规划数量
cvector *move_plan_list = NULL;
cvector *move_plan_temp_list = NULL;  // 临时规划列表

void Move_Plan_Init(void) {
    if (move_plan_list == NULL) {
        move_plan_list = cvector_create(sizeof(Move_plan *));
    }
    if (move_plan_temp_list == NULL) {
        move_plan_temp_list = cvector_create(sizeof(Move_plan *));
    }
}

Move_plan_config *Move_Plan_Config_Create(Line_type line_type) {
    // 判断线段类型是否合法
    if (!is_line_type_valid(line_type)) {
        /* printf_log("invalid line type in %s", __func__); */
        return NULL;
    }

    Move_plan_config *config = (Move_plan_config *)malloc(sizeof(Move_plan_config));
    memset(config, 0, sizeof(Move_plan_config));
    config->via_points = cvector_create(sizeof(Via_point));
    config->via_lines = cvector_create(sizeof(Via_line));
    config->type = move_plan_normal;  // 默认一般规划
    config->line_type = line_type;    // 使用传入的线段类型
    return config;
}

void Add_Via_Point(Move_plan_config *config, float pos, float vel, float acc) {
    if (config == NULL || config->via_points == NULL) {
        /* printf_log("config or via_points is NULL in %s", __func__); */
        return;
    }

    Via_point point = {
        .id = config->via_points->cv_len,  // ID自动分配
        .pos = pos,
        .vel = vel,
        .acc = acc,
    };
    cvector_pushback(config->via_points, &point);
}

void Add_Via_Line(Move_plan_config *config, float duration_s) {
    if (config == NULL || config->via_lines == NULL) {
        /* printf_log("config or via_lines is NULL in %s", __func__); */
        return;
    }

    if (duration_s <= 0) {
        /* printf_log("invalid duration in %s", __func__); */
        return;  // 持续时间必须大于0
    }

    Via_line line = {
        .id = config->via_lines->cv_len + 1,  // 自动分配ID
        .duration = duration_s,
        .line_data.cubic_spline = NULL,  // 线段数据将在Move_Plan_Create中分配与计算
    };
    cvector_pushback(config->via_lines, &line);
    config->lines_in_total++;
    config->time_in_total += duration_s;
}

Move_plan *Move_Plan_Create(Move_plan_config *config) {
    if (config == NULL || config->via_points == NULL || config->via_lines == NULL) {
        /* printf_log("config or via_points/via_lines is NULL in %s", __func__); */
        return NULL;  // 如果配置或通过点、通过段为空，返回NULL
    }
    Via_point *s_point = (Via_point *)cvector_val_at(config->via_points, 0);
    if (s_point == NULL || s_point->id != 0) {
        /* printf_log("start point missing or id mismatch in %s", __func__); */
        return NULL;
    }
    for (int i = 0; i < config->lines_in_total; i++) {
        Via_point *point = (Via_point *)cvector_val_at(config->via_points, i + 1);
        Via_line *line = (Via_line *)cvector_val_at(config->via_lines, i);
        if (point == NULL || line == NULL || point->id != i + 1 || line->id != i + 1) {
            /* printf_log("via point or line mismatch in %s", __func__); */
            return NULL;  // 如果通过点或通过段不匹配，返回NULL
        }
    }

    Move_plan *plan = (Move_plan *)malloc(sizeof(Move_plan));
    memset(plan, 0, sizeof(Move_plan));

    plan->status = move_plan_reset;
    plan->config = config;
    plan->current_pos = 0.0f;
    plan->current_vel = 0.0f;
    plan->start_time = 0;
    plan->now_line_id = 0;          // 初始化当前线段ID
    plan->now_line_delta_time = 0;  // 初始化当前线段内的时间

    //计算线段系数，第一次创建不需要改变线段类型
    line_coef_calc(plan->config, plan->config->line_type);

    if (plan->config->type == move_plan_normal) {
        // 如果是一般规划，将规划添加到全局规划列表
        cvector_pushback(move_plan_list, &plan);
    } else if (plan->config->type == move_plan_temp) {
        // 如果是临时规划，将规划添加到临时规划列表
        cvector_pushback(move_plan_temp_list, &plan);
    }

    return plan;
}

void Reset_Move_Plan(Move_plan *plan) {
    if (plan->config->type == move_plan_temp && plan->status == move_plan_run) {
        // 如果是临时规划且正在运行，不允许重置
        /* printf_log("cannot reset a running temp plan in %s", __func__); */
        return;
    }

    plan->status = move_plan_reset;
    // plan->current_pos = 0.0f;
    // plan->current_vel = 0.0f;
    plan->start_time = 0;
    plan->now_line_id = 0;          // 初始化当前线段ID
    plan->now_line_start_time = 0;  // 初始化当前线段开始时间
    plan->now_line_delta_time = 0;  // 初始化当前线段内的时间
}

void Run_Move_Plan(Move_plan *plan) {
    if (plan->status != move_plan_reset || plan->config->type == move_plan_none) {
        // 如果规划不是重置状态，或者配置类型为无规划，直接返回
        return;
    }

    plan->status = move_plan_run;
    plan->current_pos = ((Via_point *)cvector_val_at(plan->config->via_points, 0))->pos;
    plan->current_vel = 0;                      // 起始点速度默认为0
    plan->current_acc = 0;                      // 起始点加速度默认为0
    plan->now_line_id = 1;                      // 初始化当前线段ID
    plan->now_line_start_time = HAL_GetTick();  // 记录当前线段开始时间
    plan->start_time = HAL_GetTick();
}

void Move_Plan_Update(void) {
    if (move_plan_list == NULL && move_plan_temp_list == NULL)
        return;  // 如果没有运动规划，直接返回

    uint32_t current_time = HAL_GetTick();
    cvector *now_list = NULL;
    for (uint8_t k = 0; k < 2; k++) {
        if (k == 0) {
            now_list = move_plan_list;  // 处理全局规划列表
        } else {
            now_list = move_plan_temp_list;  // 处理临时规划列表
        }
        if (now_list == NULL || now_list->cv_len == 0) continue;  // 如果列表为空，跳过

        for (size_t i = 0; i < now_list->cv_len; i++) {
            Move_plan *plan = *(Move_plan **)cvector_val_at(now_list, i);
            if (plan->status != move_plan_run) continue;  // 只处理运行中的规划
            if (current_time - plan->start_time
                > (uint32_t)(plan->config->time_in_total * 1000)) {  // 如果超时，重置规划
                if (plan->config->type == move_plan_normal) {
                    Reset_Move_Plan(plan);
                } else if (plan->config->type == move_plan_temp) {  // 如果是临时规划,重置config
                    plan->config->type = move_plan_none;            // 设置为无规划
                    Reset_Move_Plan(plan);
                }
                continue;
            }

            Via_line *line =
                (Via_line *)cvector_val_at(plan->config->via_lines, plan->now_line_id - 1);
            if (current_time - plan->now_line_start_time >= line->duration * 1000) {
                // 如果当前线段时间超过了预定时间，切换到下一线段
                plan->now_line_id++;
                plan->now_line_start_time = current_time;  // 更新当前线段开始时间
                plan->now_line_delta_time = 0;             // 重置当前线段内的时间
                continue;                                  // 继续处理下一线段
            } else {
                plan->now_line_delta_time =
                    current_time - plan->now_line_start_time;  // 更新当前线段内的时间
            }
            if (plan->now_line_id
                > plan->config->lines_in_total) {  // 如果当前线段ID超过总线段数，重置规划
                if (plan->config->type == move_plan_normal) {
                    Reset_Move_Plan(plan);
                } else if (plan->config->type == move_plan_temp) {  // 如果是临时规划,重置config
                    plan->config->type = move_plan_none;            // 设置为无规划
                    Reset_Move_Plan(plan);
                }
                continue;  // 继续处理下一规划
            }

            float t_s = (plan->now_line_delta_time / 1000.0f);  // 当前线段内的时间，单位为秒
            // 计算当前位置、速度、加速度
            switch (plan->config->line_type) {
            case linear_spline:  // 计算一次样条
                {
                    plan->current_pos =
                        Linear_Spline_Calc_y(line->line_data.linear_spline, t_s);
                    plan->current_vel =
                        Linear_Spline_Calc_dy(line->line_data.linear_spline, t_s);
                    plan->current_acc =
                        Linear_Spline_Calc_ddy(line->line_data.linear_spline, t_s);
                }
                break;
            case cubic_spline:  // 计算三次样条
                {
                    plan->current_pos =
                        Cubic_Spline_Calc_y(line->line_data.cubic_spline, t_s);
                    plan->current_vel =
                        Cubic_Spline_Calc_dy(line->line_data.cubic_spline, t_s);
                    plan->current_acc =
                        Cubic_Spline_Calc_ddy(line->line_data.cubic_spline, t_s);
                }
                break;
            case quintic_spline:  // 计算五次样条
                {
                    plan->current_pos =
                        Quintic_Spline_Calc_y(line->line_data.quintic_spline, t_s);
                    plan->current_vel =
                        Quintic_Spline_Calc_dy(line->line_data.quintic_spline, t_s);
                    plan->current_acc =
                        Quintic_Spline_Calc_ddy(line->line_data.quintic_spline, t_s);
                }
                break;
                // 不使用default，以便自动检查是否覆盖了所有线段类型
            }
        }
    }
}

Move_plan *Move_Plan_Temp_Create(float start_pos, float start_vel, float start_acc,
    float end_pos, float end_vel, float end_acc, float duration_s, Line_type line_type) {
    if (duration_s <= 0) {
        /* printf_log("invalid duration in %s", __func__); */
        return NULL;  // 如果持续时间小于等于0，返回NULL
    }
    // 判断线段类型是否合法
    if (!is_line_type_valid(line_type)) {
        /* printf_log("invalid line type in %s", __func__); */
        return NULL;
    }
    // 查找临时规划列表中是否存在空的规划
    for (size_t i = 0; i < move_plan_temp_list->cv_len; i++) {
        Move_plan *plan = *(Move_plan **)cvector_val_at(move_plan_temp_list, i);
        if (plan->config->type == move_plan_none) {  // 找到一个空的临时规划，重用它
            plan->config->type = move_plan_temp;     // 设置为临时规划类型
            // 修改原有配置
            Via_line *line = (Via_line *)cvector_val_at(plan->config->via_lines, 0);
            Via_point *start_point =
                (Via_point *)cvector_val_at(plan->config->via_points, line->id - 1);
            Via_point *end_point =
                (Via_point *)cvector_val_at(plan->config->via_points, line->id);
            // 通过指针更新信息
            start_point->pos = start_pos;  // 设置起始点位置
            start_point->vel = start_vel;  // 设置起始点速度
            start_point->acc = start_acc;  // 设置起始点加速度
            end_point->pos = end_pos;      // 设置终点位置
            end_point->vel = end_vel;      // 设置终点速度
            end_point->acc = end_acc;      // 设置终点加速度
            line->duration = duration_s;   // 设置持续时间

            // 重新计算线段系数
            // 如果线段类型与先前不同，原有的线段数据将销毁并重新创建新的线段数据
            Line_type old_line_type = plan->config->line_type;  // 记录旧的线段类型
            plan->config->line_type = line_type;                // 更新线段类型
            line_coef_calc(plan->config, old_line_type);

            plan->config->time_in_total = duration_s;  // 更新总时间
            Run_Move_Plan(plan);                       // 立刻启动临时规划
            return plan;
        }
    }

    // 如果没有找到空的临时规划，创建一个新的临时规划
    if (move_plan_temp_list->cv_len >= MAX_TEMP_PLAN_NUM) {
        // 如果临时规划列表已满，返回NULL
        /* printf_log("temp plan list is full in %s", __func__); */
        return NULL;
    }
    Move_plan_config *config = Move_Plan_Config_Create(line_type);
    if (config == NULL) {
        /* printf_log("failed to create config in %s", __func__); */
        return NULL;  // 如果配置创建失败，返回NULL
    }
    config->type = move_plan_temp;                           // 设置为临时规划类型
    Add_Via_Point(config, start_pos, start_vel, start_acc);  // 初始点
    Add_Via_Point(config, end_pos, end_vel, end_acc);        // 终点
    Add_Via_Line(config, duration_s);                        // 添加一条线段

    Move_plan *plan = Move_Plan_Create(config);
    Run_Move_Plan(plan);  // 立刻启动临时规划
    return plan;
}

void Move_Plan_Recalc(Move_plan_config *config) {
    if (config == NULL || config->via_points == NULL || config->via_lines == NULL) {
        return;  // 如果配置或通过点、通过段为空，直接返回
    }
    line_coef_calc(config, config->line_type);  // 重新计算线段系数，不改变线段类型
    // 更新总时间
    config->time_in_total = 0;
    for (size_t i = 0; i < config->via_lines->cv_len; i++) {
        Via_line *line = (Via_line *)cvector_val_at(config->via_lines, i);
        config->time_in_total += line->duration;
    }
}

uint8_t get_Rest_Temp_Plan_cnt(void) {
    if (move_plan_temp_list == NULL) {
        return 0;  // 如果临时规划列表为空，返回0
    }
    uint8_t rest_temp_plan_cnt = 0;
    for (uint8_t i = 0; i < move_plan_temp_list->cv_len; i++) {
        Move_plan *plan = *(Move_plan **)cvector_val_at(move_plan_temp_list, i);
        if (plan->config->type == move_plan_none) {
            rest_temp_plan_cnt++;
        }
    }
    return rest_temp_plan_cnt + MAX_TEMP_PLAN_NUM
        - move_plan_temp_list->cv_len;  // 返回剩余的可用临时规划数量
}

static void line_coef_calc(Move_plan_config *config, Line_type old_line_type) {
    for (size_t i = 0; i < config->via_lines->cv_len; i++) {
        Via_line *line = (Via_line *)cvector_val_at(config->via_lines, i);
        Via_point *start_point =
            (Via_point *)cvector_val_at(config->via_points, line->id - 1);
        Via_point *end_point = (Via_point *)cvector_val_at(config->via_points, line->id);

        // 检查是否改变了线段类型，如果改变了，销毁原有的线段数据
        if (config->line_type != old_line_type) {
            switch (old_line_type) {
            case linear_spline:
                Linear_Spline_Destroy(line->line_data.linear_spline);
                line->line_data.linear_spline = NULL;
                break;
            case cubic_spline:
                Cubic_Spline_Destroy(line->line_data.cubic_spline);
                line->line_data.cubic_spline = NULL;
                break;
            case quintic_spline:
                Quintic_Spline_Destroy(line->line_data.quintic_spline);
                line->line_data.quintic_spline = NULL;
                break;
            }
        }

        // 根据线段类型计算线段数据
        switch (config->line_type) {
        case linear_spline:  // 计算一次样条系数
            {
                Linear_spline_config linear_spline_config = {
                    .x0 = 0,
                    .y0 = start_point->pos,
                    .x1 = line->duration,
                    .y1 = end_point->pos,
                    .xmin = 0,
                    .xmax = line->duration,
                };
                // 如果线段数据已经存在，重新计算系数；如果不存在，创建新的对象
                if (line->line_data.linear_spline != NULL) {
                    Linear_Spline_Reinit(line->line_data.linear_spline,
                        &linear_spline_config);
                } else {
                    line->line_data.linear_spline =
                        Linear_Spline_Create(&linear_spline_config);
                }
            }
            break;
        case cubic_spline:  // 计算三次样条系数
            {
                Cubic_spline_config cubic_spline_config = {
                    .x0 = 0,
                    .y0 = start_point->pos,
                    .dy0 = start_point->vel,
                    .x1 = line->duration,
                    .y1 = end_point->pos,
                    .dy1 = end_point->vel,
                    .xmin = 0,
                    .xmax = line->duration,
                };
                // 如果线段数据已经存在，重新计算系数；如果不存在，创建新的对象
                if (line->line_data.cubic_spline != NULL) {
                    Cubic_Spline_Reinit(line->line_data.cubic_spline, &cubic_spline_config);
                } else {
                    line->line_data.cubic_spline = Cubic_Spline_Create(&cubic_spline_config);
                }
            }
            break;
        case quintic_spline:
            {
                Quintic_spline_config quintic_spline_config = {
                    .x0 = 0,
                    .y0 = start_point->pos,
                    .dy0 = start_point->vel,
                    .ddy0 = start_point->acc,
                    .x1 = line->duration,
                    .y1 = end_point->pos,
                    .dy1 = end_point->vel,
                    .ddy1 = end_point->acc,
                    .xmin = 0,
                    .xmax = line->duration,
                };
                // 如果线段数据已经存在，重新计算系数；如果不存在，创建新的对象
                if (line->line_data.quintic_spline != NULL) {
                    Quintic_Spline_Reinit(line->line_data.quintic_spline,
                        &quintic_spline_config);
                } else {
                    line->line_data.quintic_spline =
                        Quintic_Spline_Create(&quintic_spline_config);
                }
            }
            break;
            // 不使用default，以便自动检查是否覆盖了所有线段类型
        }
    }
}

static uint8_t is_line_type_valid(Line_type line_type) {
    return line_type == linear_spline || line_type == cubic_spline
        || line_type == quintic_spline;
}
