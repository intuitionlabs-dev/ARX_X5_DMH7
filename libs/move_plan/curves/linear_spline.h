#pragma once
#pragma once

// 一次样条配置结构体
typedef struct {
    float x0;    //第一个点的x坐标
    float y0;    //第一个点的y坐标
    float x1;    //第二个点的x坐标
    float y1;    //第二个点的y坐标
    float xmin;  //样条的x坐标下限
    float xmax;  //样条的x坐标上限
} Linear_spline_config;
//一次多项式结构体
// y = a1*x + a0
typedef struct {
    float a0;
    float a1;
} Linear_poly;
// 一次样条结构体，包含多项式系数和x坐标范围
typedef struct {
    Linear_poly poly;
    float xmin;
    float xmax;
} Linear_spline;

Linear_spline* Linear_Spline_Create(Linear_spline_config* config);
void Linear_Spline_Destroy(Linear_spline* spline);
float Linear_Spline_Calc_y(Linear_spline* spline, float x);    //NOLINT
float Linear_Spline_Calc_dy(Linear_spline* spline, float x);   //NOLINT
float Linear_Spline_Calc_ddy(Linear_spline* spline, float x);  //NOLINT
void Linear_Spline_Reinit(Linear_spline* spline, Linear_spline_config* config);