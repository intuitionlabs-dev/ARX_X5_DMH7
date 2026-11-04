#ifndef RUCKIG_C_H
#define RUCKIG_C_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Independent single-axis, position-control Ruckig instance. Not thread-safe;
 * use one instance per axis and do not copy the opaque object. Units are chosen
 * by the caller, with seconds as the time unit. All scalars are IEEE FP32.
 * ABI changed from the earlier double interface: rebuild all callers. */
typedef struct ruckig_c_context ruckig_c_context;

typedef struct {
    float position, velocity, acceleration;
} ruckig_c_state;

typedef struct {
    ruckig_c_state current, target;
    float max_velocity, max_acceleration, max_jerk;
    /* Zero selects the fastest feasible trajectory; otherwise a lower bound. */
    float minimum_duration;
} ruckig_c_input;

typedef struct {
    ruckig_c_state state;
    float time, duration;
    int new_calculation;
} ruckig_c_output;

enum {
    RUCKIG_C_WORKING = 0,
    RUCKIG_C_FINISHED = 1,
    RUCKIG_C_ERROR = -1,
    RUCKIG_C_INVALID_INPUT = -100,
    RUCKIG_C_DURATION_ERROR = -101,
    RUCKIG_C_POSITION_LIMIT_ERROR = -102,
    RUCKIG_C_ZERO_LIMIT_ERROR = -104,
    RUCKIG_C_EXECUTION_ERROR = -110,
    RUCKIG_C_SYNCHRONIZATION_ERROR = -111
};

/* Allocates once. Returns NULL for invalid period or allocation failure. */
ruckig_c_context* ruckig_c_create(float period_s);
void ruckig_c_destroy(ruckig_c_context* context);
/* Invalidate the cached trajectory. The next update starts at input.current. */
void ruckig_c_reset(ruckig_c_context* context);
/* Per-instance allocation size, excluding the allocator's bookkeeping. */
size_t ruckig_c_context_size(void);

/* Advance one period and return WORKING / FINISHED or a negative error.
 * Feed output.state into input.current on the next call. Changed targets,
 * limits, or current state trigger replanning. All input values must be finite;
 * limits must be positive and minimum_duration nonnegative. An initially
 * out-of-limit current velocity/acceleration is allowed for braking recovery.
 * No allocation, OS clock, cloud calls, or exceptions during update.
 * On error, output is untouched and the cached trajectory is invalidated.
 * Time is clamped at the endpoint: FINISHED repeatedly returns the target state
 * (even for nonzero target derivatives) until the caller changes the input. */
int ruckig_c_update(ruckig_c_context* context, const ruckig_c_input* input,
                    ruckig_c_output* output);

#ifdef __cplusplus
}
#endif
#endif
