#include "segment.hpp"

namespace KDL {
    Segment::Segment(const std::string& _name, const Joint& _joint, const Frame& _f_tip, const RigidBodyInertia& _I):
        name(_name),
        joint(_joint),I(_I),
        f_tip(_joint.pose(0).Inverse() * _f_tip)
    {
    }

    Segment::Segment(const Joint& _joint, const Frame& _f_tip, const RigidBodyInertia& _I):
        name("NoName"),
        joint(_joint),I(_I),
        f_tip(_joint.pose(0).Inverse() * _f_tip)
    {
    }

    Segment::Segment(const Segment& in):
        name(in.name),joint(in.joint),I(in.I),
        f_tip(in.f_tip)
    {
    }

    Segment& Segment::operator=(const Segment& arg)
    {
        name=arg.name;
        joint=arg.joint;
        I=arg.I;
        f_tip=arg.f_tip;
        return *this;
    }

    Segment::~Segment()
    {
    }

    Frame Segment::pose(const double& q)const
    {
        return joint.pose(q)*f_tip;
    }

    Twist Segment::twist(const double& q, const double& qdot)const
    {
        return joint.twist(qdot).RefPoint(joint.pose(q).M * f_tip.p);
    }

    void Segment::setFrameToTip(const Frame& f_tip_new)
    {
        f_tip = joint.pose(0).Inverse() * f_tip_new;
    }
}
