#ifndef KDL_JOINT_HPP
#define KDL_JOINT_HPP

#include "frames.hpp"
#include <string>
#include <exception>

namespace KDL {
    class Joint {
    public:
        typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,Fixed,None=Fixed} JointType;

        explicit Joint(const std::string& name, const JointType& type=Fixed,const double& scale=1,const double& offset=0,
              const double& inertia=0,const double& damping=0,const double& stiffness=0);

        explicit Joint(const JointType& type=Fixed,const double& scale=1,const double& offset=0,
               const double& inertia=0,const double& damping=0,const double& stiffness=0);

        Joint(const std::string& name, const Vector& _origin, const Vector& _axis, const JointType& type, const double& _scale=1, const double& _offset=0,
	      const double& _inertia=0, const double& _damping=0, const double& _stiffness=0);

        Joint(const Vector& _origin, const Vector& _axis, const JointType& type, const double& _scale=1, const double& _offset=0,
	      const double& _inertia=0, const double& _damping=0, const double& _stiffness=0);

        Frame pose(const double& q)const;

        Twist twist(const double& qdot)const;

        Vector JointAxis() const;

        Vector JointOrigin() const;

          const std::string& getName()const
          {
              return name;
          }

        const JointType& getType() const
        {
            return type;
        };

        const std::string getTypeName() const
        {
            switch (type) {
	    case RotAxis:
	        return "RotAxis";
            case TransAxis:
	        return "TransAxis";
	    case RotX:
                return "RotX";
            case RotY:
                return "RotY";
            case RotZ:
                return "RotZ";
            case TransX:
                return "TransX";
            case TransY:
                return "TransY";
            case TransZ:
                return "TransZ";
            case Fixed:
                return "Fixed";
            default:
                return "Fixed";
            }
        };

        const double& getInertia() const
        {
            return inertia;
        };

        const double& getDamping() const
        {
            return damping;
        };

        const double& getStiffness() const
        {
            return stiffness;
        };

        virtual ~Joint();

    private:
        std::string name;
        Joint::JointType type;
        double scale;
        double offset;
        double inertia;
        double damping;
        double stiffness;

        Vector axis, origin;
        mutable Frame  joint_pose;
        mutable double q_previous;

      class joint_type_exception: public std::exception{
	virtual const char* what() const throw(){
	  return "Joint Type excption";}
      } joint_type_ex;
    };
}

#endif
