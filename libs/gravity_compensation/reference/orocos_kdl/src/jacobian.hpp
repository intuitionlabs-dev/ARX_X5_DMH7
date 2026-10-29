#ifndef KDL_JACOBIAN_HPP
#define KDL_JACOBIAN_HPP

#include "frames.hpp"
#include <Eigen/Core>

namespace KDL
{
    class Jacobian;
    bool Equal(const Jacobian& a,const Jacobian& b,double eps=epsilon);
    void SetToZero(Jacobian& jac);

    class Jacobian
    {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix<double,6,Eigen::Dynamic> data;
        Jacobian();
        explicit Jacobian(unsigned int nr_of_columns);
        Jacobian(const Jacobian& arg);

        void resize(unsigned int newNrOfColumns);

        Jacobian& operator=(const Jacobian& arg);

        bool operator ==(const Jacobian& arg)const;
        bool operator !=(const Jacobian& arg)const;

        friend bool Equal(const Jacobian& a,const Jacobian& b,double eps);

        ~Jacobian();

        double operator()(unsigned int i,unsigned int j)const;
        double& operator()(unsigned int i,unsigned int j);
        unsigned int rows()const;
        unsigned int columns()const;

        friend void SetToZero(Jacobian& jac);

        friend bool changeRefPoint(const Jacobian& src1, const Vector& base_AB, Jacobian& dest);
        friend bool changeBase(const Jacobian& src1, const Rotation& rot, Jacobian& dest);
        friend bool changeRefFrame(const Jacobian& src1,const Frame& frame, Jacobian& dest);

        Twist getColumn(unsigned int i) const;
        void setColumn(unsigned int i,const Twist& t);

        void changeRefPoint(const Vector& base_AB);
        void changeBase(const Rotation& rot);
        void changeRefFrame(const Frame& frame);
    };

    bool changeRefPoint(const Jacobian& src1, const Vector& base_AB, Jacobian& dest);
    bool changeBase(const Jacobian& src1, const Rotation& rot, Jacobian& dest);
    bool changeRefFrame(const Jacobian& src1,const Frame& frame, Jacobian& dest);
}

#endif
