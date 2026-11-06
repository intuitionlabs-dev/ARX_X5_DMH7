#ifndef KDL_JNTARRAY_HPP
#define KDL_JNTARRAY_HPP

#include "frames.hpp"
#include "jacobian.hpp"

#include <Eigen/Core>

namespace KDL
{
    /* 保存关节向量，默认构造为空；二元运算的输入维度须一致。 */

    class JntArray
    {
    public:
        Eigen::VectorXd data;

        /* 默认构造为零尺寸，访问元素前须 resize。 */
        JntArray();

        explicit JntArray(unsigned int size);

        JntArray(const JntArray& arg);
        ~JntArray();
        /* 调整尺寸时保留已有元素，扩展部分置零；可能分配内存。 */
        void resize(unsigned int newSize);

        JntArray& operator = ( const JntArray& arg);

        double operator()(unsigned int i,unsigned int j=0)const;

        double& operator()(unsigned int i,unsigned int j=0);

        unsigned int rows()const;

        unsigned int columns()const;

        friend void Add(const JntArray& src1,const JntArray& src2,JntArray& dest);
        friend void Subtract(const JntArray& src1,const JntArray& src2,JntArray& dest);
        friend void Multiply(const JntArray& src,const double& factor,JntArray& dest);
        friend void Divide(const JntArray& src,const double& factor,JntArray& dest);
        friend void MultiplyJacobian(const Jacobian& jac, const JntArray& src, Twist& dest);
        friend void SetToZero(JntArray& array);
        friend bool Equal(const JntArray& src1,const JntArray& src2,double eps);

        friend bool operator==(const JntArray& src1,const JntArray& src2);
        };

    bool operator==(const JntArray& src1,const JntArray& src2);

    void Add(const JntArray& src1,const JntArray& src2,JntArray& dest);

    void Subtract(const JntArray& src1,const JntArray& src2,JntArray& dest);

    void Multiply(const JntArray& src,const double& factor,JntArray& dest);

    void Divide(const JntArray& src,const double& factor,JntArray& dest);

    void MultiplyJacobian(const Jacobian& jac, const JntArray& src, Twist& dest);

    void SetToZero(JntArray& array);

    bool Equal(const JntArray& src1,const JntArray& src2,double eps=epsilon);
}

#endif
