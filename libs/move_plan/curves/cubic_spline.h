#pragma once

/* 三次样条配置结构体 */
typedef struct {
    float x0;    /* 第一个点的x坐标 */
    float y0;    /* 第一个点的y坐标 */
    float dy0;   /* 第一个点的y对x的导数 */
    float x1;    /* 第二个点的x坐标 */
    float y1;    /* 第二个点的y坐标 */
    float dy1;   /* 第二个点的y对x的导数 */
    float xmin;  /* 样条的x坐标下限 */
    float xmax;  /* 样条的x坐标上限 */
} Cubic_spline_config;
/* 三次多项式结构体 */
// y = a3*x^3 + a2*x^2 + a1*x + a0
typedef struct {
    float a0;
    float a1;
    float a2;
    float a3;
} Cubic_poly;
/* 三次样条结构体，包含多项式系数和x坐标范围 */
typedef struct {
    Cubic_poly poly;
    float xmin;
    float xmax;
} Cubic_spline;

Cubic_spline* Cubic_Spline_Create(Cubic_spline_config* config);
void Cubic_Spline_Destroy(Cubic_spline* spline);
float Cubic_Spline_Calc_y(Cubic_spline* spline, float x);    //NOLINT
float Cubic_Spline_Calc_dy(Cubic_spline* spline, float x);   //NOLINT
float Cubic_Spline_Calc_ddy(Cubic_spline* spline, float x);  //NOLINT
void Cubic_Spline_Reinit(Cubic_spline* spline, Cubic_spline_config* config);
