#include <ruckig/block.hpp>
#include <ruckig/velocity.hpp>
#include <ruckig/profile.hpp>
#include <ruckig/roots.hpp>


namespace ruckig {

VelocityThirdOrderStep2::VelocityThirdOrderStep2(float tf, float v0, float a0, float vf, float af, float aMax, float aMin, float jMax): a0(a0), tf(tf), af(af), _aMax(aMax), _aMin(aMin), _jMax(jMax) {
    vd = vf - v0;
    ad = af - a0;
}

bool VelocityThirdOrderStep2::time_acc0(Profile& profile, float aMax, float aMin, float jMax) {
    // UD Solution 1/2
    {
        const float h1 = std::sqrt((-ad*ad + 2*jMax*((a0 + af)*tf - 2*vd))/(jMax*jMax) + tf*tf);

        profile.t[0] = ad/(2*jMax) + (tf - h1)/2;
        profile.t[1] = h1;
        profile.t[2] = tf - (profile.t[0] + h1);
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UU Solution
    {
        const float h1 = (-ad + jMax*tf);

        profile.t[0] = -ad*ad/(2*jMax*h1) + (vd - a0*tf)/h1;
        profile.t[1] = -ad/jMax + tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = tf - (profile.t[0] + profile.t[1]);

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UU Solution - 2 step
    {
        profile.t[0] = 0;
        profile.t[1] = -ad/jMax + tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = ad/jMax;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::ACC0>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    return false;
}

bool VelocityThirdOrderStep2::time_none(Profile& profile, float aMax, float aMin, float jMax) {
    if (std::abs(a0) < FLT_EPSILON && std::abs(af) < FLT_EPSILON && std::abs(vd) < FLT_EPSILON) {
        profile.t[0] = 0;
        profile.t[1] = tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::NONE>(tf, jMax, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    // UD Solution 1/2
    {
        const float h1 = 2*(af*tf - vd);

        profile.t[0] = h1/ad;
        profile.t[1] = tf - profile.t[0];
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        const float jf = ad*ad/h1;

        if (std::abs(jf) < std::abs(jMax) + 1e-12f && profile.check_for_velocity_with_timing<ControlSigns::UDDU, ReachedLimits::NONE>(tf, jf, aMax, aMin)) {
            profile.pf = profile.p.back();
            return true;
        }
    }

    return false;
}

bool VelocityThirdOrderStep2::get_profile(Profile& profile) {
    // Test all cases to get ones that match
    // However we should guess which one is correct and try them first...
    if (vd > 0) {
        return check_all(profile, _aMax, _aMin, _jMax) || check_all(profile, _aMin, _aMax, -_jMax);
    }

    return check_all(profile, _aMin, _aMax, -_jMax) || check_all(profile, _aMax, _aMin, _jMax);
}

} // namespace ruckig
