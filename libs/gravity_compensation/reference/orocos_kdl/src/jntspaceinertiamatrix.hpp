#ifndef KDL_JNTSPACEINERTIAMATRIX_HPP
#define KDL_JNTSPACEINERTIAMATRIX_HPP

#include "frames.hpp"
#include "jacobian.hpp"
#include "jntarray.hpp"

#include <Eigen/Core>

namespace KDL
{
    /* 保存关节空间惯量方阵，默认构造为空；运算的输入维度须匹配。 */

    class JntSpaceInertiaMatrix
    {
    public:
        Eigen::MatrixXd data;

        /* 默认构造为零尺寸，访问元素前须 resize。 */
        JntSpaceInertiaMatrix();

        explicit JntSpaceInertiaMatrix(int size);

        JntSpaceInertiaMatrix(const JntSpaceInertiaMatrix& arg);
        ~JntSpaceInertiaMatrix();
        /* 调整方阵尺寸，元素由调用方初始化；可能分配内存。 */
        void resize(unsigned int newSize);

        JntSpaceInertiaMatrix& operator = ( const JntSpaceInertiaMatrix& arg);

        double operator()(unsigned int i,unsigned int j)const;

        double& operator()(unsigned int i,unsigned int j);

        unsigned int rows()const;

        unsigned int columns()const;

        friend void Add(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest);
        friend void Subtract(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest);
        friend void Multiply(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest);
        friend void Divide(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest);
        friend void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest);
        friend void SetToZero(JntSpaceInertiaMatrix& matrix);
        friend bool Equal(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,double eps);
        friend bool operator==(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2);
    };

    bool operator==(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2);

    void Add(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest);

    void Subtract(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest);

    void Multiply(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest);

    void Divide(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest);

    void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest);

    void SetToZero(JntSpaceInertiaMatrix& matrix);

    bool Equal(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,double eps=epsilon);

    bool operator==(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2);
}

#endif
