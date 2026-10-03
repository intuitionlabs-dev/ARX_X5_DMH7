#pragma once

// 五次样条配置结构体
typedef struct {
    float x0;    // 第一个点的x坐标
    float y0;    // 第一个点的y坐标
    float dy0;   // 第一个点的y对x的导数
    float ddy0;  // 第一个点的y对x的二阶导数
    float x1;    // 第二个点的x坐标
    float y1;    // 第二个点的y坐标
    float dy1;   // 第二个点的y对x的导数
    float ddy1;  // 第二个点的y对x的二阶导数
    float xmin;  // 样条的x坐标下限
    float xmax;  // 样条的x坐标上限
} Quintic_spline_config;
//五次多项式结构体
// y = a5*x^5 + a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0
typedef struct {
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    float a5;
} Quintic_poly;
// 五次样条结构体，包含多项式系数和x坐标范围
typedef struct {
    Quintic_poly poly;
    float xmin;
    float xmax;
} Quintic_spline;

Quintic_spline* Quintic_Spline_Create(Quintic_spline_config* config);
void Quintic_Spline_Destroy(Quintic_spline* spline);
float Quintic_Spline_Calc_y(Quintic_spline* spline, float x);    //NOLINT
float Quintic_Spline_Calc_dy(Quintic_spline* spline, float x);   //NOLINT
float Quintic_Spline_Calc_ddy(Quintic_spline* spline, float x);  //NOLINT
void Quintic_Spline_Reinit(Quintic_spline* spline, Quintic_spline_config* config);