#ifndef KDL_ROTATIONALINERTIA_HPP
#define KDL_ROTATIONALINERTIA_HPP

#include "frames.hpp"

namespace KDL
{
    class RigidBodyInertia;

	class RotationalInertia{
    public:

        explicit RotationalInertia(double Ixx=0,double Iyy=0,double Izz=0,double Ixy=0,double Ixz=0,double Iyz=0);

        static inline RotationalInertia Zero(){
            return RotationalInertia(0,0,0,0,0,0);
        };

        friend RotationalInertia operator*(double a, const RotationalInertia& I);
        friend RotationalInertia operator+(const RotationalInertia& Ia, const RotationalInertia& Ib);

        KDL::Vector operator*(const KDL::Vector& omega) const;

        ~RotationalInertia();

        friend class RigidBodyInertia;

        friend RigidBodyInertia operator*(double a,const RigidBodyInertia& I);

        friend RigidBodyInertia operator+(const RigidBodyInertia& Ia,const RigidBodyInertia& Ib);

        friend Wrench operator*(const RigidBodyInertia& I,const Twist& t);
        /* 按 T_a_b 将转动惯量从坐标系 b 变换到 a。 */
        friend RigidBodyInertia operator*(const Frame& T,const RigidBodyInertia& I);

        friend RigidBodyInertia operator*(const Rotation& R,const RigidBodyInertia& I);

        double data[9];
	};

    RotationalInertia operator*(double a, const RotationalInertia& I);
    RotationalInertia operator+(const RotationalInertia& Ia, const RotationalInertia& Ib);
}

#endif
