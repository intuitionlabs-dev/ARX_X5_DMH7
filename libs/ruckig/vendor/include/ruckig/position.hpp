#pragma once

#include <array>
#include <optional>


namespace ruckig {



//! Mathematical equations for Step 1 in third-order position interface: Extremal profiles
class PositionThirdOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float v0, a0;
    const float vf, af;
    const float _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    float pd;
    float v0_v0, vf_vf;
    float a0_a0, a0_p3, a0_p4;
    float af_af, af_p3, af_p4;
    float jMax_jMax;

    // Max 5 valid profiles + 1 spare for numerical issues
    using ProfileIter = std::array<Profile, 6>::iterator;
    std::array<Profile, 6> valid_profiles;

    void time_all_vel(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax, bool return_after_found) const;
    void time_acc0_acc1(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax, bool return_after_found) const;
    void time_all_none_acc0_acc1(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax, bool return_after_found) const;

    // Only for numerical issues, always return_after_found
    void time_acc1_vel_two_step(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax) const;
    void time_acc0_two_step(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax) const;
    void time_vel_two_step(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax) const;
    void time_none_two_step(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, float jMax) const;

    // Only for zero-limits case
    bool time_all_single_step(Profile* profile, float vMax, float vMin, float aMax, float aMin, float jMax) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionThirdOrderStep1(float p0, float v0, float a0, float pf, float vf, float af, float vMax, float vMin, float aMax, float aMin, float jMax);

    bool get_profile(const Profile& input, Block& block);
};


//! Mathematical equations for Step 2 in third-order position interface: Time synchronization
class PositionThirdOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float v0, a0;
    const float tf, vf, af;
    const float _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    float pd;
    float tf_tf, tf_p3, tf_p4;
    float vd, vd_vd;
    float ad, ad_ad;
    float v0_v0, vf_vf;
    float a0_a0, a0_p3, a0_p4, a0_p5, a0_p6;
    float af_af, af_p3, af_p4, af_p5, af_p6;
    float jMax_jMax;
    float g1, g2;

    bool time_acc0_acc1_vel(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_acc1_vel(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_acc0_vel(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_vel(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_acc0_acc1(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_acc1(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_acc0(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_none(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);
    bool time_none_smooth(Profile& profile, float vMax, float vMin, float aMax, float aMin, float jMax);

public:
    bool minimize_jerk {false};

    explicit PositionThirdOrderStep2(float tf, float p0, float v0, float a0, float pf, float vf, float af, float vMax, float vMin, float aMax, float aMin, float jMax);

    bool get_profile(Profile& profile);
};


//! Mathematical equations for Step 1 in second-order position interface: Extremal profiles
class PositionSecondOrderStep1 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float v0, vf;
    const float _vMax, _vMin, _aMax, _aMin;

    // Pre-calculated expressions
    float pd;

    // Max 3 valid profiles
    using ProfileIter = std::array<Profile, 3>::iterator;
    std::array<Profile, 3> valid_profiles;

    void time_acc0(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, bool return_after_found) const;
    void time_none(ProfileIter& profile, float vMax, float vMin, float aMax, float aMin, bool return_after_found) const;

    // Only for zero-limits case
    bool time_all_single_step(Profile* profile, float vMax, float vMin, float aMax, float aMin) const;

    inline void add_profile(ProfileIter& profile) const {
        const auto prev_profile = profile;
        ++profile;
        profile->set_boundary(*prev_profile);
    }

public:
    explicit PositionSecondOrderStep1(float p0, float v0, float pf, float vf, float vMax, float vMin, float aMax, float aMin);

    bool get_profile(const Profile& input, Block& block);
};


//! Mathematical equations for Step 2 in second-order position interface: Time synchronization
class PositionSecondOrderStep2 {
    using ReachedLimits = Profile::ReachedLimits;
    using ControlSigns = Profile::ControlSigns;

    const float v0, tf, vf;
    const float _vMax, _vMin, _aMax, _aMin;

    // Pre-calculated expressions
    float pd, vd;

    bool time_acc0(Profile& profile, float vMax, float vMin, float aMax, float aMin);
    bool time_none(Profile& profile, float vMax, float vMin, float aMax, float aMin);

    inline bool check_all(Profile& profile, float vMax, float vMin, float aMax, float aMin) {
        return time_acc0(profile, vMax, vMin, aMax, aMin) || time_none(profile, vMax, vMin, aMax, aMin);
    }

public:
    explicit PositionSecondOrderStep2(float tf, float p0, float v0, float pf, float vf, float vMax, float vMin, float aMax, float aMin);

    bool get_profile(Profile& profile);
};


//! Mathematical equations for Step 1 in first-order position interface: Extremal profiles
class PositionFirstOrderStep1 {
    const float _vMax, _vMin;
    float pd; // Pre-calculated expressions

public:
    explicit PositionFirstOrderStep1(float p0, float pf, float vMax, float vMin);

    bool get_profile(const Profile& input, Block& block);
};


//! Mathematical equations for Step 2 in first-order position interface: Time synchronization
class PositionFirstOrderStep2 {
    const float tf, _vMax, _vMin;
    float pd; // Pre-calculated expressions

public:
    explicit PositionFirstOrderStep2(float tf, float p0, float pf, float vMax, float vMin);

    bool get_profile(Profile& profile);
};

} // namespace ruckig
