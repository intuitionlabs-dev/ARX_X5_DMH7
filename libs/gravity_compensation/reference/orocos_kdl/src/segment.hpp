#ifndef KDL_SEGMENT_HPP
#define KDL_SEGMENT_HPP

#include "frames.hpp"
#include "rigidbodyinertia.hpp"
#include "joint.hpp"
#include <vector>

namespace KDL {
    class Segment {
        friend class Chain;
    private:
        std::string name;
        Joint joint;
        RigidBodyInertia I;
        Frame f_tip;

    public:

        explicit Segment(const std::string& name, const Joint& joint=Joint(Joint::Fixed), const Frame& f_tip=Frame::Identity(),const RigidBodyInertia& I = RigidBodyInertia::Zero());

        explicit Segment(const Joint& joint=Joint(Joint::Fixed), const Frame& f_tip=Frame::Identity(),const RigidBodyInertia& I = RigidBodyInertia::Zero());
        Segment(const Segment& in);
        Segment& operator=(const Segment& arg);

        virtual ~Segment();

        Frame pose(const double& q)const;
        /* 返回连杆末端参考点的空间速度，分量以连杆基座坐标系表示。 */
        Twist twist(const double& q,const double& qdot)const;

        const std::string& getName()const
        {
            return name;
        }

        const Joint& getJoint()const
        {
            return joint;
        }

        const RigidBodyInertia& getInertia()const
        {
            return I;
        }

        void setInertia(const RigidBodyInertia& Iin)
        {
            this->I=Iin;
        }

        Frame getFrameToTip()const
        {
            return joint.pose(0)*f_tip;
        }

        void setFrameToTip(const Frame& f_tip_new);
    };
}

#endif
