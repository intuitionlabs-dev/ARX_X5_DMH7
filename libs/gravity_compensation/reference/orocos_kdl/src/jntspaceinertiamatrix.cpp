#include "jntspaceinertiamatrix.hpp"

namespace KDL
{
    JntSpaceInertiaMatrix::JntSpaceInertiaMatrix()
    {
    }

    JntSpaceInertiaMatrix::JntSpaceInertiaMatrix(int _size):
        data(_size, _size)
    {
        data.setZero();
    }

    JntSpaceInertiaMatrix::JntSpaceInertiaMatrix(const JntSpaceInertiaMatrix& arg):
        data(arg.data)
    {
    }

    JntSpaceInertiaMatrix& JntSpaceInertiaMatrix::operator = (const JntSpaceInertiaMatrix& arg)
    {
        data=arg.data;
        return *this;
    }

    JntSpaceInertiaMatrix::~JntSpaceInertiaMatrix()
    {
    }

    void JntSpaceInertiaMatrix::resize(unsigned int newSize)
    {
        data.resize(newSize,newSize);
    }

    double JntSpaceInertiaMatrix::operator()(unsigned int i,unsigned int j)const
    {
        return data(i, j);
    }

    double& JntSpaceInertiaMatrix::operator()(unsigned int i,unsigned int j)
    {
        return data(i, j);
    }

    unsigned int JntSpaceInertiaMatrix::rows()const
    {
        return data.rows();
    }

    unsigned int JntSpaceInertiaMatrix::columns()const
    {
        return data.cols();
    }

    void Add(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest)
    {
        dest.data=src1.data+src2.data;
    }

    void Subtract(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2,JntSpaceInertiaMatrix& dest)
    {
        dest.data=src1.data-src2.data;
    }

    void Multiply(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest)
    {
        dest.data=factor*src.data;
    }

    void Divide(const JntSpaceInertiaMatrix& src,const double& factor,JntSpaceInertiaMatrix& dest)
    {
        dest.data=src.data/factor;
    }

    void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest)
    {
        dest.data=src.data.lazyProduct(vec.data);
    }

    void SetToZero(JntSpaceInertiaMatrix& mat)
    {
        mat.data.setZero();
    }

    bool Equal(const JntSpaceInertiaMatrix& src1, const JntSpaceInertiaMatrix& src2,double eps)
    {
        if(src1.rows()!=src2.rows()||src1.columns()!=src2.columns())
            return false;
        return (src1.data-src2.data).isZero(eps);
    }

    bool operator==(const JntSpaceInertiaMatrix& src1,const JntSpaceInertiaMatrix& src2){return Equal(src1,src2);}
}
