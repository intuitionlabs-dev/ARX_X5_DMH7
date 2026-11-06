#include "rotationalinertia.hpp"
#include <Eigen/Core>

namespace KDL
{
	RotationalInertia::RotationalInertia(double Ixx,double Iyy,double Izz,double Ixy,double Ixz,double Iyz)
	{
        data[0]=Ixx;
        data[1]=data[3]=Ixy;
        data[2]=data[6]=Ixz;
        data[4]=Iyy;
        data[5]=data[7]=Iyz;
        data[8]=Izz;
	}

	RotationalInertia::~RotationalInertia()
	{
	}

	Vector RotationalInertia::operator*(const Vector& omega) const {
        Vector result;
        Eigen::Map<Eigen::Vector3d>(result.data) = Eigen::Map<const Eigen::Matrix3d>(this->data) * Eigen::Map<const Eigen::Vector3d>(omega.data);
        return result;
 	}

    RotationalInertia operator*(double a, const RotationalInertia& I){
        RotationalInertia result;
        Eigen::Map<Eigen::Matrix3d>(result.data) = a * Eigen::Map<const Eigen::Matrix3d>(I.data);
        return result;
    }

    RotationalInertia operator+(const RotationalInertia& Ia, const RotationalInertia& Ib){
        RotationalInertia result;
        Eigen::Map<Eigen::Matrix3d>(result.data) = Eigen::Map<const Eigen::Matrix3d>(Ia.data) + Eigen::Map<const Eigen::Matrix3d>(Ib.data);
        return result;
    }
}
