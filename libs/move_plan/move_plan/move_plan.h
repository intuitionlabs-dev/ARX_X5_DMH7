/**
 * @file move_plan.h
 * @author 赵苏亚 (2779776204@qq.com)
 * @brief 轨迹规划器
 * @version 4.0
 * @date 2026-03-22
 * 
 * @copyright Copyright (c) 2025
 * 本库提供了一个通用的轨迹规划器，支持一次样条、三次样条和五次样条线段类型（可扩展）。
    * 规划器支持两种类型的规划：一般规划（Move_plan_normal）和临时规划（Move_plan_temp）。一般规划适用于需要重复执行的轨迹，而临时规划适用于一次性的轨迹调整。
    * 时间是轨迹的自变量，位置、速度和加速度是因变量。规划器根据通过点和通过段的信息计算出每个线段的系数，并在运行时根据当前时间计算出当前位置、速度和加速度。
    * 通过点（Via_point）和通过段（Via_line）是规划的基本元素，任何一段轨迹都包含1+i个通过点与i个通过段。通过点定义了位置、速度和加速度，而通过段定义了线段类型和持续时间。规划器根据这些信息计算出每个线段的系数，并在运行时根据当前时间计算出当前位置、速度和加速度。
        * 线段类型说明：
            * 一次样条（linear_spline）：每条通过段都由一次多项式定义，系数仅由通过点的位置决定。
            * 三次样条（cubic_spline）：每条通过段都由三次多项式定义，系数由通过点的位置、速度决定。
            * 五次样条（quintic_spline）：每条通过段都由五次多项式定义，系数由通过点的位置、速度、加速度决定。
    * 一般APP层不直接使用本文件的api，而是使用plan_group提供的批量规划api。

 */
#pragma once
#include <stdint.h>
#include "cubic_spline.h"
#include "cvector.h"
#include "linear_spline.h"
#include "quintic_spline.h"

//运动规划状态
typedef enum {
    move_plan_reset = 0,
    move_plan_run,
} Move_plan_status;

// 运动规划类型
typedef enum {
    move_plan_none = 0,  // 无规划
    move_plan_temp,      // 临时规划
    move_plan_normal,    // 一般规划
} Move_plan_type;

//通过点结构体
typedef struct {
    uint8_t id;  // 通过点ID,[0,255],0为初始点,最后一个点为终点
    float pos;
    float vel;
    float acc;
} Via_point;

// 线段类型
typedef enum {
    linear_spline = 0,  // 一次样条
    cubic_spline,       // 三次样条
    quintic_spline,     // 五次样条
} Line_type;

// 线段数据联合体
typedef union {
    Linear_spline *linear_spline;    // 一次样条
    Cubic_spline *cubic_spline;      // 三次样条
    Quintic_spline *quintic_spline;  // 五次样条
} Line_data;

//通过段结构体
typedef struct {
    uint8_t id;  // 通过段ID,[1,255],1为初始段,最后一段编号与终点一致
    Line_data line_data;  // 线段数据
    float duration;       // 持续时间s
} Via_line;

//运动规划配置结构体
typedef struct {
    Move_plan_type type;  // 规划类型
    Line_type line_type;  // 线段类型
    cvector *via_points;
    cvector *via_lines;
    uint8_t lines_in_total;
    float time_in_total;  // s
} Move_plan_config;

//运动规划结构体
typedef struct {
    Move_plan_status status;
    Move_plan_config *config;
    uint8_t now_line_id;               // 当前线段ID
    uint32_t start_time;           // ms
    uint32_t now_line_start_time;  // 当前线段开始时间
    uint32_t now_line_delta_time;  // 当前线段内的时间
    float current_pos;
    float current_vel;
    float current_acc;
} Move_plan;

// 初始化函数
void Move_Plan_Init(void);
Move_plan_config *Move_Plan_Config_Create(Line_type line_type);
void Add_Via_Point(Move_plan_config *config, float pos, float vel, float acc);
void Add_Via_Line(Move_plan_config *config, float duration_s);
Move_plan *Move_Plan_Create(Move_plan_config *config);
void Move_Plan_Recalc(Move_plan_config *config);
//执行函数
void Reset_Move_Plan(Move_plan *plan);
void Run_Move_Plan(Move_plan *plan);
//更新函数
void Move_Plan_Update(void);

// 临时规划
Move_plan *Move_Plan_Temp_Create(float start_pos, float start_vel, float start_acc,
    float end_pos, float end_vel, float end_acc, float duration_s, Line_type line_type);

// 查询函数
uint8_t get_Rest_Temp_Plan_cnt(void);
