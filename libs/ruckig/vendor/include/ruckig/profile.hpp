#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>

#include <ruckig/brake.hpp>
#include <ruckig/roots.hpp>
#include <ruckig/utils.hpp>


namespace ruckig {

//! Information about the position extrema
struct Bound {
    //! The extreme position
    float min, max;

    //! Time when the positions are reached
    float t_min, t_max;
};


//! @brief A single-dof kinematic profile with position, velocity, acceleration and jerk
//!
//! The class members are only available in the Ruckig Community Version.
class Profile {
    // FP32 arithmetic: allow bounded rounding at limits and integrated endpoints.
    static float limit_epsilon(float upper, float lower) {
        return 8.0f * std::numeric_limits<float>::epsilon()
            * std::max({1.0f, std::abs(upper), std::abs(lower)});
    }

    static bool close_endpoint(const std::array<float, 8>& values, float target, float integration_scale = 0.0f) {
        float scale = std::max({1.0f, std::abs(target), integration_scale});
        for (float value: values) scale = std::max(scale, std::abs(value));
        return std::abs(values.back() - target)
            <= 32.0f * std::numeric_limits<float>::epsilon() * scale;
    }

    // Include terms before cancellation, e.g. t*a + t*t*j/2 at reversal.
    float velocity_integration_scale() const {
        float scale = 0.0f;
        for (size_t i = 0; i < 7; ++i)
            scale += std::abs(t[i]) * (std::abs(a[i]) + std::abs(t[i]*j[i])/2);
        return scale;
    }

    float position_integration_scale() const {
        float scale = 0.0f;
        for (size_t i = 0; i < 7; ++i)
            scale += std::abs(t[i]) * (std::abs(v[i]) + std::abs(t[i])
                * (std::abs(a[i])/2 + std::abs(t[i]*j[i])/6));
        return scale;
    }

    constexpr static float t_max {1e12f};

public:
    std::array<float, 7> t, t_sum, j;
    std::array<float, 8> a, v, p;

    //! Brake sub-profiles
    BrakeProfile brake, accel;

    //! Target (final) kinematic state
    float pf, vf, af;

    enum class ReachedLimits { ACC0_ACC1_VEL, VEL, ACC0, ACC1, ACC0_ACC1, ACC0_VEL, ACC1_VEL, NONE } limits;
    enum class Direction { UP, DOWN } direction;
    enum class ControlSigns { UDDU, UDUD } control_signs;


    // For third-order velocity interface
    template<ControlSigns control_signs, ReachedLimits limits>
    bool check_for_velocity(float jf, float aMax, float aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (size_t i = 0; i < 6; ++i) {
            if (t[i+1] < 0) {
                return false;
            }

            t_sum[i+1] = t_sum[i] + t[i+1];
        }

        if constexpr (limits == ReachedLimits::ACC0) {
            if (t[1] <= 0.0f) {
                return false;
            }
        }

        if (t_sum.back() > t_max) { // For numerical reasons, is that needed?
            return false;
        }

        if constexpr (control_signs == ControlSigns::UDDU) {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? -jf : 0), 0, (t[6] > 0 ? jf : 0)};
        } else {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? jf : 0), 0, (t[6] > 0 ? -jf : 0)};
        }

        for (size_t i = 0; i < 7; ++i) {
            a[i+1] = a[i] + t[i] * j[i];
            v[i+1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i+1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));
        }

        this->control_signs = control_signs;
        this->limits = limits;

        direction = (aMax > 0) ? Profile::Direction::UP : Profile::Direction::DOWN;
        const float aUppLim = (direction == Profile::Direction::UP ? aMax : aMin) + limit_epsilon(aMax, aMin);
        const float aLowLim = (direction == Profile::Direction::UP ? aMin : aMax) - limit_epsilon(aMax, aMin);

        // Velocity limit can be broken in the beginning if both initial velocity and acceleration are too high
        // std::cout << std::setprecision(15) << "target: " << std::abs(p.back() - pf) << " " << std::abs(v.back() - vf) << " " << std::abs(a.back() - af) << " T: " << t_sum.back() << " " << to_string() << std::endl;
        return close_endpoint(v, vf, velocity_integration_scale()) && close_endpoint(a, af)
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim;
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_velocity_with_timing(float, float jf, float aMax, float aMin) {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        return check_for_velocity<control_signs, limits>(jf, aMax, aMin); // && (std::abs(t_sum.back() - tf) < t_precision);
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_velocity_with_timing(float tf, float jf, float aMax, float aMin, float jMax) {
        return (std::abs(jf) < std::abs(jMax) + limit_epsilon(jMax, -jMax)) && check_for_velocity_with_timing<control_signs, limits>(tf, jf, aMax, aMin);
    }

    inline void set_boundary_for_velocity(float p0_new, float v0_new, float a0_new, float vf_new, float af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
    }


    // For second-order velocity interface
    template<ControlSigns control_signs, ReachedLimits limits>
    bool check_for_second_order_velocity(float aUp) {
        // ReachedLimits::ACC0
        if (t[1] < 0.0f) {
            return false;
        }

        t_sum = {0, t[1], t[1], t[1], t[1], t[1], t[1]};
        if (t_sum.back() > t_max) { // For numerical reasons, is that needed?
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        a = {0, (t[1] > 0) ? aUp : 0, 0, 0, 0, 0, 0, af};
        for (size_t i = 0; i < 7; ++i) {
            v[i+1] = v[i] + t[i] * a[i];
            p[i+1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs;
        this->limits = limits;

        direction = (aUp > 0) ? Profile::Direction::UP : Profile::Direction::DOWN;

        // Velocity limit can be broken in the beginning if both initial velocity and acceleration are too high
        // std::cout << std::setprecision(15) << "target: " << std::abs(p.back() - pf) << " " << std::abs(v.back() - vf) << " " << std::abs(a.back() - af) << " T: " << t_sum.back() << " " << to_string() << std::endl;
        return close_endpoint(v, vf, velocity_integration_scale());
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_second_order_velocity_with_timing(float, float aUp) {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        return check_for_second_order_velocity<control_signs, limits>(aUp); // && (std::abs(t_sum.back() - tf) < t_precision);
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_second_order_velocity_with_timing(float tf, float aUp, float aMax, float aMin) {
        return (aMin - limit_epsilon(aMax, aMin) < aUp) && (aUp < aMax + limit_epsilon(aMax, aMin)) && check_for_second_order_velocity_with_timing<control_signs, limits>(tf, aUp);
    }


    // For third-order position interface
    template<ControlSigns control_signs, ReachedLimits limits, bool set_limits = false>
    bool check(float jf, float vMax, float vMin, float aMax, float aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (size_t i = 0; i < 6; ++i) {
            if (t[i+1] < 0) {
                return false;
            }

            t_sum[i+1] = t_sum[i] + t[i+1];
        }

        if constexpr (limits == ReachedLimits::ACC0_ACC1_VEL || limits == ReachedLimits::ACC0_VEL || limits == ReachedLimits::ACC1_VEL || limits == ReachedLimits::VEL) {
            if (t[3] <= 0.0f) {
                return false;
            }
        }

        if constexpr (limits == ReachedLimits::ACC0 || limits == ReachedLimits::ACC0_ACC1) {
            if (t[1] <= 0.0f) {
                return false;
            }
        }

        if constexpr (limits == ReachedLimits::ACC1 || limits == ReachedLimits::ACC0_ACC1) {
            if (t[5] <= 0.0f) {
                return false;
            }
        }

        if (t_sum.back() > t_max) { // For numerical reasons, is that needed?
            return false;
        }

        if constexpr (control_signs == ControlSigns::UDDU) {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? -jf : 0), 0, (t[6] > 0 ? jf : 0)};
        } else {
            j = {(t[0] > 0 ? jf : 0), 0, (t[2] > 0 ? -jf : 0), 0, (t[4] > 0 ? jf : 0), 0, (t[6] > 0 ? -jf : 0)};
        }

        direction = (vMax > 0) ? Profile::Direction::UP : Profile::Direction::DOWN;
        const float vUppLim = (direction == Profile::Direction::UP ? vMax : vMin) + limit_epsilon(vMax, vMin);
        const float vLowLim = (direction == Profile::Direction::UP ? vMin : vMax) - limit_epsilon(vMax, vMin);

        for (size_t i = 0; i < 7; ++i) {
            a[i+1] = a[i] + t[i] * j[i];
            v[i+1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i+1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));

            if constexpr (limits == ReachedLimits::ACC0_ACC1_VEL || limits == ReachedLimits::ACC0_ACC1 || limits == ReachedLimits::ACC0_VEL || limits == ReachedLimits::ACC1_VEL || limits == ReachedLimits::VEL) {
                if (i == 2) {
                    a[3] = 0.0f;
                }
            }

            if constexpr (set_limits) {
                if constexpr (limits == ReachedLimits::ACC1) {
                    if (i == 2) {
                        a[3] = aMin;
                    }
                }

                if constexpr (limits == ReachedLimits::ACC0_ACC1) {
                    if (i == 0) {
                        a[1] = aMax;
                    }

                    if (i == 4) {
                        a[5] = aMin;
                    }
                }
            }

            if (i > 1 && a[i+1] * a[i] < -std::numeric_limits<float>::epsilon()) {
                const float v_a_zero = v[i] - (a[i] * a[i]) / (2 * j[i]);
                if (v_a_zero > vUppLim || v_a_zero < vLowLim) {
                    return false;
                }
            }
        }

        this->control_signs = control_signs;
        this->limits = limits;

        const float aUppLim = (direction == Profile::Direction::UP ? aMax : aMin) + limit_epsilon(aMax, aMin);
        const float aLowLim = (direction == Profile::Direction::UP ? aMin : aMax) - limit_epsilon(aMax, aMin);

        // Velocity limit can be broken in the beginning if both initial velocity and acceleration are too high
        // std::cout << std::setprecision(16) << "target: " << std::abs(p.back() - pf) << " " << std::abs(v.back() - vf) << " " << std::abs(a.back() - af) << " T: " << t_sum.back() << " " << to_string() << std::endl;
        return close_endpoint(p, pf, position_integration_scale()) && close_endpoint(v, vf, velocity_integration_scale()) && close_endpoint(a, af)
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim
            && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_with_timing(float, float jf, float vMax, float vMin, float aMax, float aMin) {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        return check<control_signs, limits>(jf, vMax, vMin, aMax, aMin); // && (std::abs(t_sum.back() - tf) < t_precision);
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_with_timing(float tf, float jf, float vMax, float vMin, float aMax, float aMin, float jMax) {
        return (std::abs(jf) < std::abs(jMax) + limit_epsilon(jMax, -jMax)) && check_with_timing<control_signs, limits>(tf, jf, vMax, vMin, aMax, aMin);
    }

    inline void set_boundary(const Profile& profile) {
        a[0] = profile.a[0];
        v[0] = profile.v[0];
        p[0] = profile.p[0];
        af = profile.af;
        vf = profile.vf;
        pf = profile.pf;
        brake = profile.brake;
        accel = profile.accel;
    }

    inline void set_boundary(float p0_new, float v0_new, float a0_new, float pf_new, float vf_new, float af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
        pf = pf_new;
    }


    // For second-order position interface
    template<ControlSigns control_signs, ReachedLimits limits>
    bool check_for_second_order(float aUp, float aDown, float vMax, float vMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (size_t i = 0; i < 6; ++i) {
            if (t[i+1] < 0) {
                return false;
            }

            t_sum[i+1] = t_sum[i] + t[i+1];
        }

        if (t_sum.back() > t_max) { // For numerical reasons, is that needed?
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        if constexpr (control_signs == ControlSigns::UDDU) {
            a = {(t[0] > 0 ? aUp : 0), 0, (t[2] > 0 ? aDown : 0), 0, (t[4] > 0 ? aDown : 0), 0, (t[6] > 0 ? aUp : 0), af};
        } else {
            a = {(t[0] > 0 ? aUp : 0), 0, (t[2] > 0 ? aDown : 0), 0, (t[4] > 0 ? aUp : 0), 0, (t[6] > 0 ? aDown : 0), af};
        }

        direction = (vMax > 0) ? Profile::Direction::UP : Profile::Direction::DOWN;
        const float vUppLim = (direction == Profile::Direction::UP ? vMax : vMin) + limit_epsilon(vMax, vMin);
        const float vLowLim = (direction == Profile::Direction::UP ? vMin : vMax) - limit_epsilon(vMax, vMin);

        for (size_t i = 0; i < 7; ++i) {
            v[i+1] = v[i] + t[i] * a[i];
            p[i+1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs;
        this->limits = limits;

        // Velocity limit can be broken in the beginning if both initial velocity and acceleration are too high
        // std::cout << std::setprecision(16) << "target: " << std::abs(p.back() - pf) << " " << std::abs(v.back() - vf) << " " << std::abs(a.back() - af) << " T: " << t_sum.back() << " " << to_string() << std::endl;
        return close_endpoint(p, pf, position_integration_scale()) && close_endpoint(v, vf, velocity_integration_scale())
            && v[2] <= vUppLim && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[2] >= vLowLim && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_second_order_with_timing(float, float aUp, float aDown, float vMax, float vMin) {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        return check_for_second_order<control_signs, limits>(aUp, aDown, vMax, vMin); // && (std::abs(t_sum.back() - tf) < t_precision);
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_second_order_with_timing(float tf, float aUp, float aDown, float vMax, float vMin, float aMax, float aMin) {
        return (aMin - limit_epsilon(aMax, aMin) < aUp) && (aUp < aMax + limit_epsilon(aMax, aMin)) && (aMin - limit_epsilon(aMax, aMin) < aDown) && (aDown < aMax + limit_epsilon(aMax, aMin)) && check_for_second_order_with_timing<control_signs, limits>(tf, aUp, aDown, vMax, vMin);
    }


    // For first-order position interface
    template<ControlSigns control_signs, ReachedLimits limits>
    bool check_for_first_order(float vUp) {
        // ReachedLimits::VEL
        if (t[3] < 0.0f) {
            return false;
        }

        t_sum = {0, 0, 0, t[3], t[3], t[3], t[3]};
        if (t_sum.back() > t_max) { // For numerical reasons, is that needed?
            return false;
        }

        j = {0, 0, 0, 0, 0, 0, 0};
        a = {0, 0, 0, 0, 0, 0, 0, af};
        v = {0, 0, 0, t[3] > 0 ? vUp : 0, 0, 0, 0, vf};
        for (size_t i = 0; i < 7; ++i) {
            p[i+1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this->control_signs = control_signs;
        this->limits = limits;

        direction = (vUp > 0) ? Profile::Direction::UP : Profile::Direction::DOWN;

        return close_endpoint(p, pf, position_integration_scale());
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_first_order_with_timing(float, float vUp) {
        // Time doesn't need to be checked as every profile has a: tf - ... equation
        return check_for_first_order<control_signs, limits>(vUp); // && (std::abs(t_sum.back() - tf) < t_precision);
    }

    template<ControlSigns control_signs, ReachedLimits limits>
    inline bool check_for_first_order_with_timing(float tf, float vUp, float vMax, float vMin) {
        return (vMin - limit_epsilon(vMax, vMin) < vUp) && (vUp < vMax + limit_epsilon(vMax, vMin)) && check_for_first_order_with_timing<control_signs, limits>(tf, vUp);
    }



    // Secondary features
    static void check_position_extremum(float t_ext, float t_sum, float t, float p, float v, float a, float j, Bound& ext) {
        if (0 < t_ext && t_ext < t) {
            float p_ext, a_ext;
            std::tie(p_ext, std::ignore, a_ext) = integrate(t_ext, p, v, a, j);
            if (a_ext > 0 && p_ext < ext.min) {
                ext.min = p_ext;
                ext.t_min = t_sum + t_ext;
            } else if (a_ext < 0 && p_ext > ext.max) {
                ext.max = p_ext;
                ext.t_max = t_sum + t_ext;
            }
        }
    }

    static void check_step_for_position_extremum(float t_sum, float t, float p, float v, float a, float j, Bound& ext) {
        if (p < ext.min) {
            ext.min = p;
            ext.t_min = t_sum;
        }
        if (p > ext.max) {
            ext.max = p;
            ext.t_max = t_sum;
        }

        if (j != 0) {
            const float D = a * a - 2 * j * v;
            if (std::abs(D) < std::numeric_limits<float>::epsilon()) {
                check_position_extremum(-a / j, t_sum, t, p, v, a, j, ext);

            } else if (D > 0.0f) {
                const float D_sqrt = std::sqrt(D);
                check_position_extremum((-a - D_sqrt) / j, t_sum, t, p, v, a, j, ext);
                check_position_extremum((-a + D_sqrt) / j, t_sum, t, p, v, a, j, ext);
            }
        }
    }

    Bound get_position_extrema() const {
        Bound extrema;
        extrema.min = std::numeric_limits<float>::infinity();
        extrema.max = -std::numeric_limits<float>::infinity();

        if (brake.duration > 0.0f) {
            if (brake.t[0] > 0.0f) {
                check_step_for_position_extremum(0.0f, brake.t[0], brake.p[0], brake.v[0], brake.a[0], brake.j[0], extrema);

                if (brake.t[1] > 0.0f) {
                    check_step_for_position_extremum(brake.t[0], brake.t[1], brake.p[1], brake.v[1], brake.a[1], brake.j[1], extrema);
                }
            }
        }

        float t_current_sum {0.0f};
        for (size_t i = 0; i < 7; ++i) {
            if (i > 0) {
                t_current_sum = t_sum[i - 1];
            }
            check_step_for_position_extremum(t_current_sum + brake.duration, t[i], p[i], v[i], a[i], j[i], extrema);
        }

        if (pf < extrema.min) {
            extrema.min = pf;
            extrema.t_min = t_sum.back() + brake.duration;
        }
        if (pf > extrema.max) {
            extrema.max = pf;
            extrema.t_max = t_sum.back() + brake.duration;
        }

        return extrema;
    }

    bool get_first_state_at_position(float pt, float& time, float time_after=0.0f) const {
        float t_cum = 0.0f;

        for (size_t i = 0; i < 7; ++i) {
            if (t[i] == 0.0f) {
                continue;
            }

            if (std::abs(p[i] - pt) < FLT_EPSILON && t_cum >= time_after) {
                time = t_cum;
                return true;
            }

            for (const float _t: roots::solve_cubic(j[i]/6, a[i]/2, v[i], p[i] - pt)) {
                if (0 < _t && time_after - t_cum <= _t && _t <= t[i]) {
                    time = _t + t_cum;
                    return true;
                }
            }

            t_cum += t[i];
        }

        if ((t[6] > 0.0f || t_sum.back() == 0.0f) && std::abs(pf - pt) < 1e-9f && t_sum.back() >= time_after) {
            time = t_sum.back();
            return true;
        }

        return false;
    }

    std::string to_string() const {
        std::string result;
        switch (direction) {
            case Direction::UP: result += "UP_"; break;
            case Direction::DOWN: result += "DOWN_"; break;
        }
        switch (limits) {
            case ReachedLimits::ACC0_ACC1_VEL: result += "ACC0_ACC1_VEL"; break;
            case ReachedLimits::VEL: result += "VEL"; break;
            case ReachedLimits::ACC0: result += "ACC0"; break;
            case ReachedLimits::ACC1: result += "ACC1"; break;
            case ReachedLimits::ACC0_ACC1: result += "ACC0_ACC1"; break;
            case ReachedLimits::ACC0_VEL: result += "ACC0_VEL"; break;
            case ReachedLimits::ACC1_VEL: result += "ACC1_VEL"; break;
            case ReachedLimits::NONE: result += "NONE"; break;
        }
        switch (control_signs) {
            case ControlSigns::UDDU: result += "_UDDU"; break;
            case ControlSigns::UDUD: result += "_UDUD"; break;
        }
        return result;
    }
};

} // namespace ruckig
