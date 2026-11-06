#pragma once
#include <stdint.h>
#include "cubic_spline.h"
#include "cvector.h"
#include "linear_spline.h"
#include "quintic_spline.h"

/* 运动规划状态 */
typedef enum {
    move_plan_reset = 0,
    move_plan_run,
} Move_plan_status;

/* 运动规划类型 */
typedef enum {
    move_plan_none = 0,  /* 无规划 */
    move_plan_temp,      /* 临时规划 */
    move_plan_normal,    /* 一般规划 */
} Move_plan_type;

/* 通过点结构体 */
typedef struct {
    uint8_t id;  /* 通过点ID,[0,255],0为初始点,最后一个点为终点 */
    float pos;
    float vel;
    float acc;
} Via_point;

/* 线段类型 */
typedef enum {
    linear_spline = 0,  /* 一次样条 */
    cubic_spline,       /* 三次样条 */
    quintic_spline,     /* 五次样条 */
} Line_type;

/* 线段数据联合体 */
typedef union {
    Linear_spline *linear_spline;    /* 一次样条 */
    Cubic_spline *cubic_spline;      /* 三次样条 */
    Quintic_spline *quintic_spline;  /* 五次样条 */
} Line_data;

/* 通过段结构体 */
typedef struct {
    uint8_t id;  /* 通过段ID,[1,255],1为初始段,最后一段编号与终点一致 */
    Line_data line_data;  /* 线段数据 */
    float duration;       /* 持续时间s */
} Via_line;

/* 运动规划配置结构体 */
typedef struct {
    Move_plan_type type;  /* 规划类型 */
    Line_type line_type;  /* 线段类型 */
    cvector *via_points;
    cvector *via_lines;
    uint8_t lines_in_total;
    float time_in_total;
} Move_plan_config;

/* 运动规划结构体 */
typedef struct {
    Move_plan_status status;
    Move_plan_config *config;
    uint8_t now_line_id;               /* 当前线段ID */
    uint32_t start_time;           // ms
    uint32_t now_line_start_time;  /* 当前线段开始时间 */
    uint32_t now_line_delta_time;  /* 当前线段内的时间 */
    float current_pos;
    float current_vel;
    float current_acc;
} Move_plan;

/* 初始化函数 */
void Move_Plan_Init(void);
Move_plan_config *Move_Plan_Config_Create(Line_type line_type);
void Add_Via_Point(Move_plan_config *config, float pos, float vel, float acc);
void Add_Via_Line(Move_plan_config *config, float duration_s);
Move_plan *Move_Plan_Create(Move_plan_config *config);
void Move_Plan_Recalc(Move_plan_config *config);
/* 执行函数 */
void Reset_Move_Plan(Move_plan *plan);
void Run_Move_Plan(Move_plan *plan);

void Move_Plan_Update(void);

/* 临时规划 */
Move_plan *Move_Plan_Temp_Create(float start_pos, float start_vel, float start_acc,
    float end_pos, float end_vel, float end_acc, float duration_s, Line_type line_type);

/* 查询函数 */
uint8_t get_Rest_Temp_Plan_cnt(void);
