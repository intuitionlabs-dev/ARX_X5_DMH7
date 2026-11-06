#ifndef KDL_RIGIDBODYINERTIA_HPP
#define KDL_RIGIDBODYINERTIA_HPP

#include "frames.hpp"

#include "rotationalinertia.hpp"

namespace KDL {
    /* 刚体空间惯量由质量、参考点和坐标基共同定义。 */
    class RigidBodyInertia{
    public:

        /* 输入质量、参考点到质心的向量及质心转动惯量。 */
        explicit RigidBodyInertia(double m=0, const Vector& oc=Vector::Zero(), const RotationalInertia& Ic=RotationalInertia::Zero());

        static inline RigidBodyInertia Zero(){
            return RigidBodyInertia(0.0,Vector::Zero(),RotationalInertia::Zero());
        };

        ~RigidBodyInertia(){};

        friend RigidBodyInertia operator*(double a,const RigidBodyInertia& I);
        friend RigidBodyInertia operator+(const RigidBodyInertia& Ia,const RigidBodyInertia& Ib);
        friend Wrench operator*(const RigidBodyInertia& I,const Twist& t);
        friend RigidBodyInertia operator*(const Frame& T,const RigidBodyInertia& I);
        friend RigidBodyInertia operator*(const Rotation& R,const RigidBodyInertia& I);

        /* 移动参考点，v 为当前坐标系中从旧点到新点的向量。 */
        RigidBodyInertia RefPoint(const Vector& p);

        double getMass() const{
            return m;
        };

        Vector getCOG() const{
            if(m==0) return Vector::Zero();
            else return h/m;
        };

        RotationalInertia getRotationalInertia() const{
            return I;
        };

    private:
        RigidBodyInertia(double m,const Vector& h,const RotationalInertia& I,bool mhi);
        double m;
        Vector h;
        RotationalInertia I;

        friend class ArticulatedBodyInertia;
    };

    RigidBodyInertia operator*(double a,const RigidBodyInertia& I);

    RigidBodyInertia operator+(const RigidBodyInertia& Ia,const RigidBodyInertia& Ib);

    Wrench operator*(const RigidBodyInertia& I,const Twist& t);

    /* 按 T_a_b 将惯量从坐标系 b 变换到 a。 */
    RigidBodyInertia operator*(const Frame& T,const RigidBodyInertia& I);

    RigidBodyInertia operator*(const Rotation& R,const RigidBodyInertia& I);
}

#endif
