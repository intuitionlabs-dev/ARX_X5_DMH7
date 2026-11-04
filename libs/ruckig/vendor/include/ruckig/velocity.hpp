#pragma once

#include <array>
#include <optional>


namespace ruckig {

//! Mathematical equations for Step 1 in third-order velocity interface: Extremal profiles
class VelocityThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float a0, af;
    const float _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    float vd;

    // Max 3 valid profiles
    using ProfileIter = std::array<Profile, 3>::iterator;
    std::array<Profile, 3> valid_profiles;

    void time_acc0(ProfileIter& profile, float aMax, float aMin, float jMax, bool return_after_found) const;
    void time_none(ProfileIter& profile, float aMax, float aMin, float jMax, bool return_after_found) const;

    // Only for zero-limits case
    bool time_all_single_step(Profile* profile, float aMax, float aMin, float jMax) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit VelocityThirdOrderStep1(float v0, float a0, float vf, float af, float aMax, float aMin, float jMax);

    bool get_profile(const Profile& input, Block& block);
};


//! Mathematical equations for Step 2 in third-order velocity interface: Time synchronization
class VelocityThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float a0, tf, af;
    const float _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    float vd, ad;

    bool time_acc0(Profile& profile, float aMax, float aMin, float jMax);
    bool time_none(Profile& profile, float aMax, float aMin, float jMax);

    inline bool check_all(Profile& profile, float aMax, float aMin, float jMax) {
        return time_acc0(profile, aMax, aMin, jMax) || time_none(profile, aMax, aMin, jMax);
    }

public:
    explicit VelocityThirdOrderStep2(float tf, float v0, float a0, float vf, float af, float aMax, float aMin, float jMax);

    bool get_profile(Profile& profile);
};


//! Mathematical equations for Step 1 in second-order velocity interface: Extremal profiles
class VelocitySecondOrderStep1 {
    const float _aMax, _aMin;
    float vd; // Pre-calculated expressions

public:
    explicit VelocitySecondOrderStep1(float v0, float vf, float aMax, float aMin);

    bool get_profile(const Profile& input, Block& block);
};


//! Mathematical equations for Step 2 in second-order velocity interface: Time synchronization
class VelocitySecondOrderStep2 {
    const float tf, _aMax, _aMin;
    float vd; // Pre-calculated expressions

public:
    explicit VelocitySecondOrderStep2(float tf, float v0, float vf, float aMax, float aMin);

    bool get_profile(Profile& profile);
};

} // namespace ruckig
