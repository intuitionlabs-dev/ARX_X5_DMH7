#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include <ruckig/utils.hpp>


namespace ruckig {

//! Calculates (pre- or post-) profile to get current or final state below the limits
class BrakeProfile {
    static constexpr float eps {4.0f * std::numeric_limits<float>::epsilon()};

    void acceleration_brake(float v0, float a0, float vMax, float vMin, float aMax, float aMin, float jMax);
    void velocity_brake(float v0, float a0, float vMax, float vMin, float aMax, float aMin, float jMax);

public:
    //! Overall duration
    float duration {0.0f};

    //! Profile information for a two-step profile
    std::array<float, 2> t, j, a, v, p;

    //! Calculate brake trajectory for third-order position interface
    void get_position_brake_trajectory(float v0, float a0, float vMax, float vMin, float aMax, float aMin, float jMax);

    //! Calculate brake trajectory for second-order position interface
    void get_second_order_position_brake_trajectory(float v0, float vMax, float vMin, float aMax, float aMin);

    //! Calculate brake trajectory for third-order velocity interface
    void get_velocity_brake_trajectory(float a0, float aMax, float aMin, float jMax);

    //! Calculate brake trajectory for second-order velocity interface
    void get_second_order_velocity_brake_trajectory();

    //! Finalize third-order braking by integrating along kinematic state
    void finalize(float& ps, float& vs, float& as) {
        if (t[0] <= 0.0f && t[1] <= 0.0f) {
            duration = 0.0f;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        a[0] = as;
        std::tie(ps, vs, as) = integrate(t[0], ps, vs, as, j[0]);

        if (t[1] > 0.0f) {
            duration += t[1];
            p[1] = ps;
            v[1] = vs;
            a[1] = as;
            std::tie(ps, vs, as) = integrate(t[1], ps, vs, as, j[1]);
        }
    }

    //! Finalize second-order braking by integrating along kinematic state
    void finalize_second_order(float& ps, float& vs, float& as) {
        if (t[0] <= 0.0f) {
            duration = 0.0f;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        std::tie(ps, vs, as) = integrate(t[0], ps, vs, a[0], 0.0f);
    }
};

} // namespace ruckig
