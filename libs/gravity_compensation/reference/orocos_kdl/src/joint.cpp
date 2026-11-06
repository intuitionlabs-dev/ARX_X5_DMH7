#include "joint.hpp"

namespace KDL {
    Joint::Joint(const std::string& _name, const JointType& _type, const double& _scale, const double& _offset,
                 const double& _inertia, const double& _damping, const double& _stiffness):
      name(_name),type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness)
    {
      if (type == RotAxis || type == TransAxis) throw joint_type_ex;
      q_previous = 0;
    }

    Joint::Joint(const JointType& _type, const double& _scale, const double& _offset,
                 const double& _inertia, const double& _damping, const double& _stiffness):
      name("NoName"),type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness)
    {
      if (type == RotAxis || type == TransAxis) throw joint_type_ex;
      q_previous = 0;
    }

    Joint::Joint(const std::string& _name, const Vector& _origin, const Vector& _axis, const JointType& _type, const double& _scale,
                 const double& _offset, const double& _inertia, const double& _damping, const double& _stiffness):
      name(_name), type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness)
      , axis(_axis / _axis.Norm()), origin(_origin)
    {
      if (type != RotAxis && type != TransAxis) throw joint_type_ex;

      joint_pose.p = origin;
      joint_pose.M = Rotation::Rot2(axis, offset);
      q_previous = 0;
    }

    Joint::Joint(const Vector& _origin, const Vector& _axis, const JointType& _type, const double& _scale,
                 const double& _offset, const double& _inertia, const double& _damping, const double& _stiffness):
          name("NoName"), type(_type),scale(_scale),offset(_offset),inertia(_inertia),damping(_damping),stiffness(_stiffness),
          axis(_axis / _axis.Norm()),origin(_origin)
    {
      if (type != RotAxis && type != TransAxis) throw joint_type_ex;

      joint_pose.p = origin;
      joint_pose.M = Rotation::Rot2(axis, offset);
      q_previous = 0;
    }

    Joint::~Joint()
    {
    }

    Frame Joint::pose(const double& q)const
    {
        switch(type){
        case RotAxis:

            if (q != q_previous){
                q_previous = q;
                joint_pose.M = Rotation::Rot2(axis, scale*q+offset);
            }
            return joint_pose;
        case RotX:
            return Frame(Rotation::RotX(scale*q+offset));
        case RotY:
            return  Frame(Rotation::RotY(scale*q+offset));
        case RotZ:
            return  Frame(Rotation::RotZ(scale*q+offset));
        case TransAxis:
            return Frame(origin + (axis * (scale*q+offset)));
        case TransX:
            return  Frame(Vector(scale*q+offset,0.0,0.0));
        case TransY:
            return Frame(Vector(0.0,scale*q+offset,0.0));
        case TransZ:
            return Frame(Vector(0.0,0.0,scale*q+offset));
        case Fixed:
            return Frame::Identity();
        }
        return Frame::Identity();
    }

    Twist Joint::twist(const double& qdot)const
    {
        switch(type){
        case RotAxis:
            return Twist(Vector(0,0,0), axis * ( scale * qdot));
        case RotX:
            return Twist(Vector(0.0,0.0,0.0),Vector(scale*qdot,0.0,0.0));
        case RotY:
            return Twist(Vector(0.0,0.0,0.0),Vector(0.0,scale*qdot,0.0));
        case RotZ:
            return Twist(Vector(0.0,0.0,0.0),Vector(0.0,0.0,scale*qdot));
        case TransAxis:
            return Twist(axis * (scale * qdot), Vector(0,0,0));
        case TransX:
            return Twist(Vector(scale*qdot,0.0,0.0),Vector(0.0,0.0,0.0));
        case TransY:
            return Twist(Vector(0.0,scale*qdot,0.0),Vector(0.0,0.0,0.0));
        case TransZ:
            return Twist(Vector(0.0,0.0,scale*qdot),Vector(0.0,0.0,0.0));
        case Fixed:
            return Twist::Zero();
        }
        return Twist::Zero();
    }

  Vector Joint::JointAxis() const
  {
    switch(type)
      {
      case RotAxis:
        return axis;
      case RotX:
        return Vector(1.,0.,0.);
      case RotY:
        return Vector(0.,1.,0.);
      case RotZ:
        return Vector(0.,0.,1.);
      case TransAxis:
        return axis;
      case TransX:
        return Vector(1.,0.,0.);
      case TransY:
        return Vector(0.,1.,0.);
      case TransZ:
        return Vector(0.,0.,1.);
      case Fixed:
        return Vector::Zero();
      }
    return Vector::Zero();
  }

  Vector Joint::JointOrigin() const
  {
    return origin;
  }
}
