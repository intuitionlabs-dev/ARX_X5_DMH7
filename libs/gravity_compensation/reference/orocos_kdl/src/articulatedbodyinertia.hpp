#ifndef KDL_ARTICULATEDBODYINERTIA_HPP
#define KDL_ARTICULATEDBODYINERTIA_HPP

#include "frames.hpp"

#include "rotationalinertia.hpp"
#include "rigidbodyinertia.hpp"

#include <Eigen/Core>

namespace KDL {
    /* 空间惯量相对于给定参考点和坐标基定义，参考点可与坐标原点不同。 */
    class ArticulatedBodyInertia{
    public:

        ArticulatedBodyInertia(){
            *this=ArticulatedBodyInertia::Zero();
        }

        ArticulatedBodyInertia(const RigidBodyInertia& rbi);

        /* 由质量、参考点到质心的向量及质心转动惯量构造空间惯量。 */
        explicit ArticulatedBodyInertia(double m, const Vector& oc=Vector::Zero(), const RotationalInertia& Ic=RotationalInertia::Zero());

        static inline ArticulatedBodyInertia Zero(){
            return ArticulatedBodyInertia(Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero());
        };

        ~ArticulatedBodyInertia(){};

        friend ArticulatedBodyInertia operator*(double a,const ArticulatedBodyInertia& I);
        friend ArticulatedBodyInertia operator+(const ArticulatedBodyInertia& Ia,const ArticulatedBodyInertia& Ib);
        friend ArticulatedBodyInertia operator+(const ArticulatedBodyInertia& Ia,const RigidBodyInertia& Ib);
        friend ArticulatedBodyInertia operator-(const ArticulatedBodyInertia& Ia,const ArticulatedBodyInertia& Ib);
        friend ArticulatedBodyInertia operator-(const ArticulatedBodyInertia& Ia,const RigidBodyInertia& Ib);
        friend Wrench operator*(const ArticulatedBodyInertia& I,const Twist& t);
        friend ArticulatedBodyInertia operator*(const Frame& T,const ArticulatedBodyInertia& I);
        friend ArticulatedBodyInertia operator*(const Rotation& R,const ArticulatedBodyInertia& I);

        /* 将参考点移动 v，v 为当前坐标系中从旧点到新点的向量。 */
        ArticulatedBodyInertia RefPoint(const Vector& p);

        ArticulatedBodyInertia(const Eigen::Matrix3d& M,const Eigen::Matrix3d& H,const Eigen::Matrix3d& I);

        Eigen::Matrix3d M;
        Eigen::Matrix3d H;
        Eigen::Matrix3d I;
    };

    ArticulatedBodyInertia operator*(double a,const ArticulatedBodyInertia& I);

    ArticulatedBodyInertia operator+(const ArticulatedBodyInertia& Ia,const ArticulatedBodyInertia& Ib);
    ArticulatedBodyInertia operator+(const ArticulatedBodyInertia& Ia,const RigidBodyInertia& Ib);
    ArticulatedBodyInertia operator-(const ArticulatedBodyInertia& Ia,const ArticulatedBodyInertia& Ib);
    ArticulatedBodyInertia operator-(const ArticulatedBodyInertia& Ia,const RigidBodyInertia& Ib);

    Wrench operator*(const ArticulatedBodyInertia& I,const Twist& t);

    /* 按 T_a_b 将惯量从坐标系 b 变换到 a。 */
    ArticulatedBodyInertia operator*(const Frame& T,const ArticulatedBodyInertia& I);

    ArticulatedBodyInertia operator*(const Rotation& R,const ArticulatedBodyInertia& I);
}
#endif
