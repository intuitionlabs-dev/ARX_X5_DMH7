#include "quintic_spline.h"
#include <stdlib.h>
#include "common.h"

/* 计算五次多项式系数 */
static Quintic_poly poly_calc(float x0, float y0, float dy0, float ddy0, float x1,
    float y1, float dy1, float ddy1);

/* 由两端位置、一阶和二阶导数计算五次多项式系数。 */
Quintic_spline* Quintic_Spline_Create(Quintic_spline_config* config) {
    Quintic_spline* spline = malloc(sizeof(Quintic_spline));
    spline->poly = poly_calc(config->x0, config->y0, config->dy0, config->ddy0,
        config->x1, config->y1, config->dy1, config->ddy1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
    return spline;
}

void Quintic_Spline_Destroy(Quintic_spline* spline) { free(spline); }

float Quintic_Spline_Calc_y(Quintic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return spline->poly.a5 * x_limited * x_limited * x_limited * x_limited * x_limited
        + spline->poly.a4 * x_limited * x_limited * x_limited * x_limited
        + spline->poly.a3 * x_limited * x_limited * x_limited
        + spline->poly.a2 * x_limited * x_limited + spline->poly.a1 * x_limited
        + spline->poly.a0;
}

float Quintic_Spline_Calc_dy(Quintic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return 5.0 * spline->poly.a5 * x_limited * x_limited * x_limited * x_limited
        + 4.0 * spline->poly.a4 * x_limited * x_limited * x_limited
        + 3.0 * spline->poly.a3 * x_limited * x_limited
        + 2.0 * spline->poly.a2 * x_limited + spline->poly.a1;
}

float Quintic_Spline_Calc_ddy(Quintic_spline* spline, float x) {
    float x_limited = fclip(x, spline->xmin, spline->xmax);
    return 20.0 * spline->poly.a5 * x_limited * x_limited * x_limited
        + 12.0 * spline->poly.a4 * x_limited * x_limited
        + 6.0 * spline->poly.a3 * x_limited + 2.0 * spline->poly.a2;
}

void Quintic_Spline_Reinit(Quintic_spline* spline, Quintic_spline_config* config) {
    spline->poly = poly_calc(config->x0, config->y0, config->dy0, config->ddy0,
        config->x1, config->y1, config->dy1, config->ddy1);
    spline->xmin = config->xmin;
    spline->xmax = config->xmax;
}

static Quintic_poly poly_calc(float x0, float y0, float dy0, float ddy0, float x1, float y1,
    float dy1, float ddy1) {
    const float x0_2 = x0 * x0;
    const float x1_2 = x1 * x1;
    const float x0_3 = x0_2 * x0;
    const float x1_3 = x1_2 * x1;
    const float x0_4 = x0_3 * x0;
    const float x1_4 = x1_3 * x1;
    const float x0_5 = x0_4 * x0;
    const float x1_5 = x1_4 * x1;
    const float delta_x = x1 - x0;
    const float delta_x_2 = delta_x * delta_x;

    Quintic_poly poly = {
        .a0 = (1.0 / delta_x_2
                  * ((x0_5)*y1 * 2.0 - (x1_5)*y0 * 2.0 + dy0 * x0 * (x1_5) * 2.0
                      - dy1 * (x0_5)*x1 * 2.0 + x0 * (x1_4)*y0 * 1.0E+1
                      - (x0_4)*x1 * y1 * 1.0E+1 - ddy0 * (x0_2) * (x1_5)
                      + ddy0 * (x0_3) * (x1_4) * 2.0 - ddy0 * (x0_4) * (x1_3)
                      + ddy1 * (x0_3) * (x1_4)-ddy1 * (x0_4) * (x1_3) * 2.0
                      + ddy1 * (x0_5) * (x1_2)-dy0 * (x0_2) * (x1_4) * 1.0E+1
                      + dy0 * (x0_3) * (x1_3) * 8.0 - dy1 * (x0_3) * (x1_3) * 8.0
                      + dy1 * (x0_4) * (x1_2) * 1.0E+1 - (x0_2) * (x1_3)*y0 * 2.0E+1
                      + (x0_3) * (x1_2)*y1 * 2.0E+1))
            / (x0 * (x1_2) * 6.0 - (x0_2)*x1 * 6.0 + (x0_3) * 2.0 - (x1_3) * 2.0),
        .a1 = (1.0 / delta_x_2
                  * (dy0 * (x1_5) * 2.0 - dy1 * (x0_5) * 2.0 - ddy0 * x0 * (x1_5) * 2.0
                      + ddy1 * (x0_5)*x1 * 2.0 - dy0 * x0 * (x1_4) * 1.0E+1
                      + dy1 * (x0_4)*x1 * 1.0E+1 + ddy0 * (x0_2) * (x1_4)
                      + ddy0 * (x0_3) * (x1_3) * 4.0 - ddy0 * (x0_4) * (x1_2) * 3.0
                      + ddy1 * (x0_2) * (x1_4) * 3.0 - ddy1 * (x0_3) * (x1_3) * 4.0
                      - ddy1 * (x0_4) * (x1_2)-dy0 * (x0_2) * (x1_3) * 1.6E+1
                      + dy0 * (x0_3) * (x1_2) * 2.4E+1 - dy1 * (x0_2) * (x1_3) * 2.4E+1
                      + dy1 * (x0_3) * (x1_2) * 1.6E+1 - (x0_2) * (x1_2)*y0 * 6.0E+1
                      + (x0_2) * (x1_2)*y1 * 6.0E+1)
                  * (-1.0 / 2.0))
            / (x0 * (x1_2) * 3.0 - (x0_2)*x1 * 3.0 + x0_3 - x1_3),
        .a2 = (1.0 / delta_x_2
                  * (ddy0 * (x1_5)-ddy1 * (x0_5) + ddy0 * x0 * (x1_4) * 4.0
                      + ddy0 * (x0_4)*x1 * 3.0 - ddy1 * x0 * (x1_4) * 3.0
                      - ddy1 * (x0_4)*x1 * 4.0 + dy0 * x0 * (x1_3) * 3.6E+1
                      - dy0 * (x0_3)*x1 * 2.4E+1 + dy1 * x0 * (x1_3) * 2.4E+1
                      - dy1 * (x0_3)*x1 * 3.6E+1 + x0 * (x1_2)*y0 * 6.0E+1
                      + (x0_2)*x1 * y0 * 6.0E+1 - x0 * (x1_2)*y1 * 6.0E+1 - (x0_2)*x1 * y1 * 6.0E+1
                      - ddy0 * (x0_2) * (x1_3) * 8.0 + ddy1 * (x0_3) * (x1_2) * 8.0
                      - dy0 * (x0_2) * (x1_2) * 1.2E+1 + dy1 * (x0_2) * (x1_2) * 1.2E+1)
                  * (-1.0 / 2.0))
            / (x0 * (x1_2) * 3.0 - (x0_2)*x1 * 3.0 + x0_3 - x1_3),
        .a3 = (ddy0 * (x0_4) + ddy0 * (x1_4) * 3.0 - ddy1 * (x0_4) * 3.0
                  - ddy1 * (x1_4)-dy0 * (x0_3) * 8.0 + dy0 * (x1_3) * 1.2E+1
                  - dy1 * (x0_3) * 1.2E+1 + dy1 * (x1_3) * 8.0 + (x0_2)*y0 * 2.0E+1
                  - (x0_2)*y1 * 2.0E+1 + (x1_2)*y0 * 2.0E+1 - (x1_2)*y1 * 2.0E+1
                  + ddy0 * (x0_3)*x1 * 4.0 - ddy1 * x0 * (x1_3) * 4.0
                  + dy0 * x0 * (x1_2) * 2.8E+1 - dy0 * (x0_2)*x1 * 3.2E+1
                  + dy1 * x0 * (x1_2) * 3.2E+1 - dy1 * (x0_2)*x1 * 2.8E+1
                  - ddy0 * (x0_2) * (x1_2) * 8.0 + ddy1 * (x0_2) * (x1_2) * 8.0
                  + x0 * x1 * y0 * 8.0E+1 - x0 * x1 * y1 * 8.0E+1)
            / ((x0 * x1 * -2.0 + x0_2 + x1_2)
                * (x0 * (x1_2) * 3.0 - (x0_2)*x1 * 3.0 + x0_3 - x1_3) * 2.0),
        .a4 = ((x0 * y0 * 3.0E+1 - x0 * y1 * 3.0E+1 + x1 * y0 * 3.0E+1 - x1 * y1 * 3.0E+1
                   + ddy0 * (x0_3) * 2.0 + ddy0 * (x1_3) * 3.0 - ddy1 * (x0_3) * 3.0
                   - ddy1 * (x1_3) * 2.0 - dy0 * (x0_2) * 1.4E+1 + dy0 * (x1_2) * 1.6E+1
                   - dy1 * (x0_2) * 1.6E+1 + dy1 * (x1_2) * 1.4E+1
                   - ddy0 * x0 * (x1_2) * 4.0 - ddy0 * (x0_2)*x1 + ddy1 * x0 * (x1_2)
                   + ddy1 * (x0_2)*x1 * 4.0 - dy0 * x0 * x1 * 2.0 + dy1 * x0 * x1 * 2.0)
                  * (-1.0 / 2.0))
            / ((x0 - x1)
                * ((x0_2) * (x1_2) * 6.0 - x0 * (x1_3) * 4.0 - (x0_3)*x1 * 4.0 + x0_4 + x1_4)),
        .a5 = (1.0 / delta_x_2
                  * (y0 * 1.2E+1 - y1 * 1.2E+1 - dy0 * x0 * 6.0 + dy0 * x1 * 6.0
                      - dy1 * x0 * 6.0 + dy1 * x1 * 6.0 + ddy0 * (x0_2)
                      + ddy0 * (x1_2)-ddy1 * (x0_2)-ddy1 * (x1_2)-ddy0 * x0 * x1 * 2.0
                      + ddy1 * x0 * x1 * 2.0))
            / (x0 * (x1_2) * 6.0 - (x0_2)*x1 * 6.0 + (x0_3) * 2.0 - (x1_3) * 2.0)};
    return poly;
}
