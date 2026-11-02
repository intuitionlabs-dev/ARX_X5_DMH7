#include "ruckig_c.h"

#include <cmath>
#include <cstdlib>
#include <new>
#include <ruckig/ruckig.hpp>

struct ruckig_c_context {
    ruckig::Ruckig<1> planner;
    ruckig::InputParameter<1> input;
    ruckig::Trajectory<1> trajectory;
    ruckig_c_input expected {};
    double time {0.0};
    bool valid {false};

    explicit ruckig_c_context(double period): planner(period) {}
};

static bool finite_state(const ruckig_c_state& state) {
    return std::isfinite(state.position) && std::isfinite(state.velocity)
        && std::isfinite(state.acceleration);
}

static bool same_state(const ruckig_c_state& a, const ruckig_c_state& b) {
    return a.position == b.position && a.velocity == b.velocity
        && a.acceleration == b.acceleration;
}

static bool same_input(const ruckig_c_input& a, const ruckig_c_input& b) {
    return same_state(a.current, b.current) && same_state(a.target, b.target)
        && a.max_velocity == b.max_velocity && a.max_acceleration == b.max_acceleration
        && a.max_jerk == b.max_jerk && a.minimum_duration == b.minimum_duration;
}

extern "C" ruckig_c_context* ruckig_c_create(double period_s) {
    if (!std::isfinite(period_s) || period_s <= 0.0) return nullptr;
    void* memory = std::malloc(sizeof(ruckig_c_context));
    return memory ? new (memory) ruckig_c_context(period_s) : nullptr;
}

extern "C" void ruckig_c_destroy(ruckig_c_context* context) {
    if (!context) return;
    context->~ruckig_c_context();
    std::free(context);
}

extern "C" void ruckig_c_reset(ruckig_c_context* context) {
    if (context) context->valid = false;
}

extern "C" size_t ruckig_c_context_size(void) { return sizeof(ruckig_c_context); }

extern "C" int ruckig_c_update(ruckig_c_context* context,
                              const ruckig_c_input* input, ruckig_c_output* output) {
    if (!context) return RUCKIG_C_INVALID_INPUT;
    if (!input || !output || !finite_state(input->current) || !finite_state(input->target)
        || !std::isfinite(input->max_velocity) || input->max_velocity <= 0.0
        || !std::isfinite(input->max_acceleration) || input->max_acceleration <= 0.0
        || !std::isfinite(input->max_jerk) || input->max_jerk <= 0.0
        || !std::isfinite(input->minimum_duration) || input->minimum_duration < 0.0) {
        context->valid = false;
        return RUCKIG_C_INVALID_INPUT;
    }

    const bool replan = !context->valid || !same_input(*input, context->expected);
    if (replan) {
        context->valid = false;
        auto& in = context->input;
        in.current_position[0] = input->current.position;
        in.current_velocity[0] = input->current.velocity;
        in.current_acceleration[0] = input->current.acceleration;
        in.target_position[0] = input->target.position;
        in.target_velocity[0] = input->target.velocity;
        in.target_acceleration[0] = input->target.acceleration;
        in.max_velocity[0] = input->max_velocity;
        in.max_acceleration[0] = input->max_acceleration;
        in.max_jerk[0] = input->max_jerk;
        in.minimum_duration = input->minimum_duration > 0.0
            ? std::optional<double>(input->minimum_duration) : std::nullopt;
        const auto result = context->planner.calculate(in, context->trajectory);
        if (result < 0) return static_cast<int>(result);
        context->time = 0.0;
    }

    ruckig_c_output next {};
    next.duration = context->trajectory.get_duration();
    next.time = std::min(context->time + context->planner.delta_time, next.duration);
    next.new_calculation = replan;
    context->trajectory.at_time(next.time, next.state.position,
                               next.state.velocity, next.state.acceleration);
    if (!std::isfinite(next.duration) || !finite_state(next.state)) {
        context->valid = false;
        return RUCKIG_C_ERROR;
    }
    const bool finished = next.time >= next.duration;
    if (finished) next.state = input->target;
    context->expected = *input;
    context->expected.current = next.state;
    context->time = next.time;
    context->valid = true;
    *output = next;
    return finished ? RUCKIG_C_FINISHED : RUCKIG_C_WORKING;
}
