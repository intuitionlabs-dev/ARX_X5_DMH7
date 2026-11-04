#include "ruckig_c.h"
#include <stdio.h>

int main(void) {
    ruckig_c_context* planner = ruckig_c_create(0.002f);
    if (!planner) return 1;
    ruckig_c_input input = {
        .current = {0, 0, 0}, .target = {30, 0, 0},
        .max_velocity = 90, .max_acceleration = 360, .max_jerk = 3600
    };
    ruckig_c_output output;
    int status;
    do {
        status = ruckig_c_update(planner, &input, &output);
        if (status < 0) break;
        /* Publish output.state to the controller here. */
        input.current = output.state;
    } while (status == RUCKIG_C_WORKING);
    if (status == RUCKIG_C_FINISHED)
        printf("position=%.6f duration=%.6f s context=%zu bytes\n",
               output.state.position, output.duration, ruckig_c_context_size());
    ruckig_c_destroy(planner);
    return status < 0;
}
