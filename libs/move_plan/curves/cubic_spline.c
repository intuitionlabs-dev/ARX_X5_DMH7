#include "cubic_spline.h"
#include <stdlib.h>
#include "common.h"

/* 计算三次多项式系数 */
static Cubic_poly poly_calc(float x0, float y0, float dy0, float x1, float y1, float dy1);

/* 由两端位置和一阶导数计算三次多项式系数。 */
Cubic_spline* Cubic_Spline_Create(Cubic_spline_config* config) {
    Cubic_spline* spline = malloc(sizeof(Cubic_spline));
    spline->poly =
        poly_calc(config->x0, config->y0, config->dy0, config->x1, config->y1, config->dy1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
    return spline;
}

void Cubic_Spline_Destroy(Cubic_spline* spline) { free(spline); }

float Cubic_Spline_Calc_y(Cubic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return spline->poly.a3 * x_limited * x_limited * x_limited
        + spline->poly.a2 * x_limited * x_limited + spline->poly.a1 * x_limited
        + spline->poly.a0;
}

float Cubic_Spline_Calc_dy(Cubic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return 3.0 * spline->poly.a3 * x_limited * x_limited
        + 2.0 * spline->poly.a2 * x_limited + spline->poly.a1;
}

float Cubic_Spline_Calc_ddy(Cubic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return 6.0 * spline->poly.a3 * x_limited + 2.0 * spline->poly.a2;
}

void Cubic_Spline_Reinit(Cubic_spline* spline, Cubic_spline_config* config) {
    spline->poly =
        poly_calc(config->x0, config->y0, config->dy0, config->x1, config->y1, config->dy1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
}

static Cubic_poly poly_calc(float x0, float y0, float dy0, float x1, float y1, float dy1) {
    const float x0_2 = x0 * x0;
    const float x1_2 = x1 * x1;
    const float x0_3 = x0 * x0 * x0;
    const float x1_3 = x1 * x1 * x1;
    const float delta_x = x1 - x0;
    const float delta_x_3 = delta_x * delta_x * delta_x;
    Cubic_poly poly = {.a0 = ((x0_3)*y1 - (x1_3)*y0 + dy0 * x0 * (x1_3)-dy1 * (x0_3)*x1
                                 + x0 * (x1_2)*y0 * 3.0 - (x0_2)*x1 * y1 * 3.0
                                 - dy0 * (x0_2) * (x1_2) + dy1 * (x0_2) * (x1_2))
            / ((x0 - x1) * (x0 * x1 * -2.0 + x0_2 + x1_2)),
        .a1 = 1.0 / delta_x_3
            * (dy0 * (x1_3)-dy1 * (x0_3) + dy0 * x0 * (x1_2)-dy0 * (x0_2)*x1 * 2.0
                + dy1 * x0 * (x1_2) * 2.0 - dy1 * (x0_2)*x1 + x0 * x1 * y0 * 6.0
                - x0 * x1 * y1 * 6.0),
        .a2 = -1.0 / delta_x_3
            * (x0 * y0 * 3.0 - x0 * y1 * 3.0 + x1 * y0 * 3.0 - x1 * y1 * 3.0
                - dy0 * (x0_2) + dy0 * (x1_2) * 2.0 - dy1 * (x0_2) * 2.0
                + dy1 * (x1_2)-dy0 * x0 * x1 + dy1 * x0 * x1),
        .a3 = 1.0 / delta_x_3
            * (y0 * 2.0 - y1 * 2.0 - dy0 * x0 + dy0 * x1 - dy1 * x0 + dy1 * x1)};
    return poly;
}
