#include <ruckig/block.hpp>
#include <ruckig/position.hpp>


namespace ruckig {

PositionSecondOrderStep1::PositionSecondOrderStep1(float p0, float v0, float pf, float vf, float vMax, float vMin, float aMax, float aMin): v0(v0), vf(vf), _vMax(vMax), _vMin(vMin), _aMax(aMax), _aMin(aMin) {
    pd = pf - p0;
}

void PositionSecondOrderStep1::time_acc0(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, bool) const {
    profile->t[0] = (-v0 + vMax)/aMax;
    profile->t[1] = (aMin*v0*v0 - aMax*vf*vf)/(2*aMax*aMin*vMax) + vMax*(aMax - aMin)/(2*aMax*aMin) + pd/vMax;
    profile->t[2] = (vf - vMax)/aMin;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::ACC0>(aMax, aMin, vMax, vMin)) {
        add_profile(profile);
    }
}

void PositionSecondOrderStep1::time_none(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, bool return_after_found) const {
    float h1 = (aMax*vf*vf - aMin*v0*v0 - 2*aMax*aMin*pd)/(aMax - aMin);
    if (h1 >= 0.0f) {
        h1 = std::sqrt(h1);

        // Solution 1
        {
            profile->t[0] = -(v0 + h1)/aMax;
            profile->t[1] = 0;
            profile->t[2] = (vf + h1)/aMin;
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(aMax, aMin, vMax, vMin)) {
                add_profile(profile);
                if (return_after_found) {
                    return;
                }
            }
        }

        // Solution 2
        {
            profile->t[0] = (-v0 + h1)/aMax;
            profile->t[1] = 0;
            profile->t[2] = (vf - h1)/aMin;
            profile->t[3] = 0;
            profile->t[4] = 0;
            profile->t[5] = 0;
            profile->t[6] = 0;

            if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(aMax, aMin, vMax, vMin)) {
                add_profile(profile);
            }
        }
    }
}

bool PositionSecondOrderStep1::time_all_single_step(Profile* profile, float vMax, float vMin, float aMax, float aMin) const {
    if (std::abs(vf - v0) > FLT_EPSILON) {
        return false;
    }

    profile->t[0] = 0;
    profile->t[1] = 0;
    profile->t[2] = 0;
    profile->t[3] = 0;
    profile->t[4] = 0;
    profile->t[5] = 0;
    profile->t[6] = 0;

    if (std::abs(v0) > FLT_EPSILON) {
        profile->t[3] = pd / v0;
        if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(0.0f, 0.0f, vMax, vMin)) {
            return true;
        }

    } else if (std::abs(pd) < FLT_EPSILON) {
        if (profile->check_for_second_order<ControlSigns::UDDU, ReachedLimits::NONE>(0.0f, 0.0f, vMax, vMin)) {
            return true;
        }
    }

    return false;
}


bool PositionSecondOrderStep1::get_profile(const Profile& input, Block& block) {
    // Zero-limits special case
    if (_vMax == 0.0f && _vMin == 0.0f) {
        auto& p = block.p_min;
        p.set_boundary(input);

        if (time_all_single_step(&p, _vMax, _vMin, _aMax, _aMin)) {
            block.t_min = p.t_sum.back() + p.brake.duration + p.accel.duration;
            if (std::abs(v0) > FLT_EPSILON) {
                block.a = Block::Interval(block.t_min, std::numeric_limits<float>::infinity());
            }
            return true;
        }
        return false;
    }

    const ProfileIter start = valid_profiles.begin();
    ProfileIter profile = start;
    profile->set_boundary(input);

    if (std::abs(vf) < FLT_EPSILON) {
        // There is no blocked interval when vf==0, so return after first found profile
        const float vMax = (pd >= 0) ? _vMax : _vMin;
        const float vMin = (pd >= 0) ? _vMin : _vMax;
        const float aMax = (pd >= 0) ? _aMax : _aMin;
        const float aMin = (pd >= 0) ? _aMin : _aMax;

        time_none(profile, vMax, vMin, aMax, aMin, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, vMax, vMin, aMax, aMin, true);
        if (profile > start) { goto return_block; }

        time_none(profile, vMin, vMax, aMin, aMax, true);
        if (profile > start) { goto return_block; }
        time_acc0(profile, vMin, vMax, aMin, aMax, true);

    } else {
        time_none(profile, _vMax, _vMin, _aMax, _aMin, false);
        time_none(profile, _vMin, _vMax, _aMin, _aMax, false);
        time_acc0(profile, _vMax, _vMin, _aMax, _aMin, false);
        time_acc0(profile, _vMin, _vMax, _aMin, _aMax, false);
    }

return_block:
    return Block::calculate_block(block, valid_profiles, std::distance(start, profile));
}

} // namespace ruckig
