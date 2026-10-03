#include "linear_spline.h"
#include <stdlib.h>

/* 私有函数 */
// 计算一次多项式系数
static Linear_poly poly_calc(float x0, float y0, float x1, float y1);

Linear_spline* Linear_Spline_Create(Linear_spline_config* config) {
    Linear_spline* spline = malloc(sizeof(Linear_spline));
    spline->poly =
        poly_calc(config->x0, config->y0, config->x1, config->y1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
    return spline;
}

void Linear_Spline_Destroy(Linear_spline* spline) { free(spline); }

float Linear_Spline_Calc_y(Linear_spline* spline, float x) {
    return spline->poly.a0 + spline->poly.a1 * x;
}

float Linear_Spline_Calc_dy(Linear_spline* spline, float x) { return spline->poly.a1; }

float Linear_Spline_Calc_ddy(Linear_spline* spline, float x) { return 0.0f; }

void Linear_Spline_Reinit(Linear_spline* spline, Linear_spline_config* config) {
    spline->poly = poly_calc(config->x0, config->y0, config->x1, config->y1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
}

static Linear_poly poly_calc(float x0, float y0, float x1, float y1) {
    Linear_poly poly = {
        .a0 = (y0 * x1 - y1 * x0) / (x1 - x0),
        .a1 = (y1 - y0) / (x1 - x0),
    };
    return poly;
}