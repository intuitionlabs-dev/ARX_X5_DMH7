/* Frame_A_B 将 B 中的坐标表示变换到 A；速度和力同时依赖坐标基与参考点。 */
#ifndef KDL_FRAMES_H
#define KDL_FRAMES_H

#include "utilities/kdl-config.h"
#include "utilities/utility.h"

namespace KDL {
class Vector;
class Rotation;
class Frame;
class Wrench;
class Twist;
class Vector2;
class Rotation2;
class Frame2;

inline bool Equal(const Vector& a,const Vector& b,double eps=epsilon);
inline bool Equal(const Frame& a,const Frame& b,double eps=epsilon);
inline bool Equal(const Twist& a,const Twist& b,double eps=epsilon);
inline bool Equal(const Wrench& a,const Wrench& b,double eps=epsilon);
inline bool Equal(const Vector2& a,const Vector2& b,double eps=epsilon);
inline bool Equal(const Rotation2& a,const Rotation2& b,double eps=epsilon);
inline bool Equal(const Frame2& a,const Frame2& b,double eps=epsilon);

/* 向量乘法返回叉积，dot 返回点积。 */
class Vector
{
public:
    double data[3];

     inline Vector() {data[0]=data[1]=data[2] = 0.0;}

     inline Vector(double x,double y, double z);

     inline Vector(const Vector& arg);

     inline Vector& operator = ( const Vector& arg);

     inline double operator()(int index) const;

     inline double& operator() (int index);

     double operator[] ( int index ) const
       {
	 return this->operator() ( index );
       }

     double& operator[] ( int index )
       {
	 return this->operator() ( index );
       }

     inline double x() const;
     inline double y() const;
     inline double z() const;
     inline void x(double);
     inline void y(double);
     inline void z(double);

     inline void ReverseSign();

     inline Vector& operator-=(const Vector& arg);

     inline Vector& operator +=(const Vector& arg);

     inline friend Vector operator*(const Vector& lhs,double rhs);

     inline friend Vector operator*(double lhs,const Vector& rhs);

     inline friend Vector operator/(const Vector& lhs,double rhs);
     inline friend Vector operator+(const Vector& lhs,const Vector& rhs);
     inline friend Vector operator-(const Vector& lhs,const Vector& rhs);
     inline friend Vector operator*(const Vector& lhs,const Vector& rhs);
     inline friend Vector operator-(const Vector& arg);
     inline friend double dot(const Vector& lhs,const Vector& rhs);

     inline friend void SetToZero(Vector& v);

     inline static Vector Zero();

     double Normalize(double eps=epsilon);

     double Norm(double eps=epsilon) const;

     inline void Set2DXY(const Vector2& v);

     inline void Set2DYZ(const Vector2& v);

     inline void Set2DZX(const Vector2& v);

     inline void Set2DPlane(const Frame& F_someframe_XY,const Vector2& v_XY);

     inline friend bool Equal(const Vector& a,const Vector& b,double eps);

     inline friend bool operator==(const Vector& a,const Vector& b);

     inline friend bool operator!=(const Vector& a,const Vector& b);

     friend class Rotation;
     friend class Frame;
};

class Rotation
{
public:
    double data[9];

    inline Rotation() {
		*this = Rotation::Identity();
	}
    inline Rotation(double Xx,double Yx,double Zx,
                double Xy,double Yy,double Zy,
                double Xz,double Yz,double Zz);
    inline Rotation(const Vector& x,const Vector& y,const Vector& z);

    inline Rotation(const Rotation& arg);

     inline Rotation& operator=(const Rotation& arg);

     inline Vector operator*(const Vector& v) const;

     /* 行列索引均为 0～2，调试构建检查索引边界。 */
     inline double& operator()(int i,int j);

     /* 行列索引均为 0～2，调试构建检查索引边界。 */
     inline double operator() (int i,int j) const;

     friend Rotation operator *(const Rotation& lhs,const Rotation& rhs);

     inline void SetInverse();

     inline Rotation Inverse() const;

     inline Vector Inverse(const Vector& v) const;

     inline Wrench Inverse(const Wrench& arg) const;

     inline Twist Inverse(const Twist& arg) const;

     inline static Rotation Identity();

    inline static Rotation RotX(double angle);

    inline static Rotation RotY(double angle);

    inline static Rotation RotZ(double angle);

    inline void DoRotX(double angle);

    inline void DoRotY(double angle);

    inline void DoRotZ(double angle);

    static Rotation Rot(const Vector& rotvec,double angle);

    /* 旋转轴输入须为单位向量。 */
    static Rotation Rot2(const Vector& rotvec,double angle);

    Vector GetRot() const;

	double GetRotAngle(Vector& axis,double eps=epsilon) const;

/* ZYZ 欧拉角依次绕当前 Z、新 Y 和新 Z 轴旋转。 */
    static Rotation EulerZYZ(double Alfa,double Beta,double Gamma);

	/* ZYZ 输出 beta∈[0,π]；beta 为 0 或 π 时选择 gamma=0。 */
    void GetEulerZYZ(double& alpha,double& beta,double& gamma) const;

    static Rotation Quaternion(double x,double y,double z, double w);

    void GetQuaternion(double& x,double& y,double& z, double& w) const;

    static Rotation RPY(double roll,double pitch,double yaw);

/* RPY 绕固定 X、Y、Z 轴旋转，pitch∈[-π/2,π/2]，roll、yaw∈[-π,π]。 */
    void GetRPY(double& roll,double& pitch,double& yaw) const;

    /* ZYX 欧拉角依次绕当前 Z、新 Y 和新 X 轴旋转。 */
    inline static Rotation EulerZYX(double Alfa,double Beta,double Gamma) {
        return RPY(Gamma,Beta,Alfa);
    }

    /* ZYX 输出 beta∈[-π/2,π/2]，奇异点选择 gamma=0。 */
    inline void GetEulerZYX(double& Alfa,double& Beta,double& Gamma) const {
        GetRPY(Gamma,Beta,Alfa);
    }

     inline Twist operator * (const Twist& arg) const;

     inline Wrench operator * (const Wrench& arg) const;

     inline Vector UnitX() const {
         return Vector(data[0],data[3],data[6]);
     }

     inline void UnitX(const Vector& X) {
        data[0] = X(0);
        data[3] = X(1);
        data[6] = X(2);
     }

     inline Vector UnitY() const {
         return Vector(data[1],data[4],data[7]);
     }

     inline void UnitY(const Vector& X) {
        data[1] = X(0);
        data[4] = X(1);
        data[7] = X(2);
     }

     inline Vector UnitZ() const {
         return Vector(data[2],data[5],data[8]);
     }

     inline void UnitZ(const Vector& X) {
        data[2] = X(0);
        data[5] = X(1);
        data[8] = X(2);
     }

     friend bool Equal(const Rotation& a,const Rotation& b,double eps);

     friend bool operator==(const Rotation& a,const Rotation& b);

     friend bool operator!=(const Rotation& a,const Rotation& b);

     friend class Frame;
};
    bool operator==(const Rotation& a,const Rotation& b);
    bool Equal(const Rotation& a,const Rotation& b,double eps=epsilon);

class Frame {
public:
    Vector p;
    Rotation M;

public:

     inline Frame(const Rotation& R,const Vector& V);

     explicit inline Frame(const Vector& V);

     explicit inline Frame(const Rotation& R);

     inline Frame() {}

     inline Frame(const Frame& arg);

     void Make4x4(double* d);

     /* 行列索引均为 0～3，调试构建检查索引边界。 */
     inline double operator()(int i,int j);

     /* 行列索引均为 0～3，调试构建检查索引边界。 */
     inline double operator() (int i,int j) const;

     inline Frame Inverse() const;

     inline Vector Inverse(const Vector& arg) const;

     inline Wrench Inverse(const Wrench& arg) const;

     inline Twist  Inverse(const Twist& arg) const;

     inline Frame& operator = (const Frame& arg);

     inline Vector operator * (const Vector& arg) const;

     inline Wrench operator * (const Wrench& arg) const;

     inline Twist operator * (const Twist& arg) const;

     inline friend Frame operator *(const Frame& lhs,const Frame& rhs);

     inline static Frame Identity();

     inline void Integrate(const Twist& t_this,double frequency);

     static Frame DH_Craig1989(double a,double alpha,double d,double theta);

     static Frame DH(double a,double alpha,double d,double theta);

     inline friend bool Equal(const Frame& a,const Frame& b,double eps);

     inline friend bool operator==(const Frame& a,const Frame& b);

     inline friend bool operator!=(const Frame& a,const Frame& b);
};

/* 索引 0～2 为线速度，3～5 为角速度。 */
class Twist {
public:
    Vector vel;
    Vector rot;
public:

    Twist():vel(),rot() {};

    Twist(const Vector& _vel,const Vector& _rot):vel(_vel),rot(_rot) {};

    inline Twist& operator-=(const Twist& arg);
    inline Twist& operator+=(const Twist& arg);

    inline double& operator()(int i);

    inline double operator()(int i) const;

     double operator[] ( int index ) const
       {
	 return this->operator() ( index );
       }

     double& operator[] ( int index )
       {
	 return this->operator() ( index );
       }

     inline friend Twist operator*(const Twist& lhs,double rhs);
     inline friend Twist operator*(double lhs,const Twist& rhs);
     inline friend Twist operator/(const Twist& lhs,double rhs);
     inline friend Twist operator+(const Twist& lhs,const Twist& rhs);
     inline friend Twist operator-(const Twist& lhs,const Twist& rhs);
     inline friend Twist operator-(const Twist& arg);
     inline friend double dot(const Twist& lhs,const Wrench& rhs);
     inline friend double dot(const Wrench& rhs,const Twist& lhs);
     inline friend void SetToZero(Twist& v);

    inline friend Twist operator*(const Twist& lhs,const Twist& rhs);

    inline friend Wrench operator*(const Twist& lhs,const Wrench& rhs);

     static inline Twist Zero();

     inline void ReverseSign();

     /* 移动速度参考点，参数向量在当前坐标系中表示。 */

     inline Twist RefPoint(const Vector& v_base_AB) const;

     inline friend bool Equal(const Twist& a,const Twist& b,double eps);

     inline friend bool operator==(const Twist& a,const Twist& b);

     inline friend bool operator!=(const Twist& a,const Twist& b);

    friend class Rotation;
    friend class Frame;
};

/* 索引 0～2 为力，3～5 为力矩。 */
class Wrench
{
public:
    Vector force;
    Vector torque;
public:

    Wrench():force(),torque() {};
    Wrench(const Vector& _force,const Vector& _torque):force(_force),torque(_torque) {};

     inline Wrench& operator-=(const Wrench& arg);
     inline Wrench& operator+=(const Wrench& arg);

     inline double& operator()(int i);

     inline double operator()(int i) const;

     double operator[] ( int index ) const
       {
	 return this->operator() ( index );
       }

     double& operator[] ( int index )
       {
	 return this->operator() ( index );
       }

     inline friend Wrench operator*(const Wrench& lhs,double rhs);

     inline friend Wrench operator*(double lhs,const Wrench& rhs);

     inline friend Wrench operator/(const Wrench& lhs,double rhs);

     inline friend Wrench operator+(const Wrench& lhs,const Wrench& rhs);
     inline friend Wrench operator-(const Wrench& lhs,const Wrench& rhs);

     inline friend Wrench operator-(const Wrench& arg);

     inline friend void SetToZero(Wrench& v);

     static inline Wrench Zero();

     inline void ReverseSign();

     /* 移动力参考点，参数向量在当前坐标系中表示。 */

     inline Wrench RefPoint(const Vector& v_base_AB) const;

     inline friend bool Equal(const Wrench& a,const Wrench& b,double eps);

     inline friend bool operator==(const Wrench& a,const Wrench& b);

     inline friend bool operator!=(const Wrench& a,const Wrench& b);

    friend class Rotation;
    friend class Frame;
};

class Vector2
{
    double data[2];
public:

     Vector2() {data[0]=data[1] = 0.0;}
     inline Vector2(double x,double y);
     inline Vector2(const Vector2& arg);

     inline Vector2& operator = ( const Vector2& arg);

     inline double operator()(int index) const;

     inline double& operator() (int index);

	double operator[] ( int index ) const
	{
		return this->operator() ( index );
	}

	double& operator[] ( int index )
	{
		return this->operator() ( index );
	}

	inline double x() const;
	inline double y() const;
	inline void x(double);
	inline void y(double);

     inline void ReverseSign();
     inline Vector2& operator-=(const Vector2& arg);
     inline Vector2& operator +=(const Vector2& arg);

     inline friend Vector2 operator*(const Vector2& lhs,double rhs);
     inline friend Vector2 operator*(double lhs,const Vector2& rhs);
     inline friend Vector2 operator/(const Vector2& lhs,double rhs);
     inline friend Vector2 operator+(const Vector2& lhs,const Vector2& rhs);
     inline friend Vector2 operator-(const Vector2& lhs,const Vector2& rhs);
     inline friend Vector2 operator*(const Vector2& lhs,const Vector2& rhs);
     inline friend Vector2 operator-(const Vector2& arg);
     inline friend void SetToZero(Vector2& v);

     inline static Vector2 Zero();

     double Normalize(double eps=epsilon);

     double Norm(double eps=epsilon) const;

     inline void Set3DXY(const Vector& v);

     inline void Set3DYZ(const Vector& v);

     inline void Set3DZX(const Vector& v);

     inline void Set3DPlane(const Frame& F_someframe_XY,const Vector& v_someframe);

     inline friend bool Equal(const Vector2& a,const Vector2& b,double eps);

	inline friend bool operator==(const Vector2& a,const Vector2& b);

	inline friend bool operator!=(const Vector2& a,const Vector2& b);

    friend class Rotation2;
};

class Rotation2
{
    double s,c;

public:

    Rotation2() {c=1.0;s=0.0;}

    explicit Rotation2(double angle_rad):s(sin(angle_rad)),c(cos(angle_rad)) {}

    Rotation2(double ca,double sa):s(sa),c(ca){}

    Rotation2(const Rotation2& arg);

     inline Rotation2& operator=(const Rotation2& arg);
     inline Vector2 operator*(const Vector2& v) const;
     /* 行列索引均为 0～1，调试构建检查索引边界。 */
     inline double operator() (int i,int j) const;

     inline friend Rotation2 operator *(const Rotation2& lhs,const Rotation2& rhs);

     inline void SetInverse();
     inline Rotation2 Inverse() const;
     inline Vector2 Inverse(const Vector2& v) const;

     inline void SetIdentity();
     inline static Rotation2 Identity();

     inline void SetRot(double angle);

     inline static Rotation2 Rot(double angle);

     inline double GetRot() const;

     inline friend bool Equal(const Rotation2& a,const Rotation2& b,double eps);
};

class Frame2
 {
public:
    Vector2 p;
    Rotation2 M;

public:

     inline Frame2(const Rotation2& R,const Vector2& V);
     explicit inline Frame2(const Vector2& V);
     explicit inline Frame2(const Rotation2& R);
     inline Frame2(void);
     inline Frame2(const Frame2& arg);
     inline void Make4x4(double* d);

     /* 行列索引均为 0～2，调试构建检查索引边界。 */
     inline double operator()(int i,int j);

     /* 行列索引均为 0～3，调试构建检查索引边界。 */
     inline double operator() (int i,int j) const;

     inline void SetInverse();
     inline Frame2 Inverse() const;
     inline Vector2 Inverse(const Vector2& arg) const;
     inline Frame2& operator = (const Frame2& arg);
     inline Vector2 operator * (const Vector2& arg) const;
     inline friend Frame2 operator *(const Frame2& lhs,const Frame2& rhs);
     inline void SetIdentity();
     inline void Integrate(const Twist& t_this,double frequency);
     inline static Frame2 Identity() {
        Frame2 tmp;
        tmp.SetIdentity();
        return tmp;
     }
     inline friend bool Equal(const Frame2& a,const Frame2& b,double eps);
};

IMETHOD Vector diff(const Vector& p_w_a,const Vector& p_w_b,double dt=1);

/* 返回从 b1 到 b2 的旋转轴乘以旋转角，并以坐标系 a 表示，旋转角范围为 [0,π]。 */
IMETHOD Vector diff(const Rotation& R_a_b1,const Rotation& R_a_b2,double dt=1);

/* 返回从 F_a_b1 到 F_a_b2 的位姿差，以坐标系 a 表示。 */
IMETHOD Twist diff(const Frame& F_a_b1,const Frame& F_a_b2,double dt=1);

IMETHOD Twist diff(const Twist& a,const Twist& b,double dt=1);

IMETHOD Wrench diff(const Wrench& W_a_p1,const Wrench& W_a_p2,double dt=1);

IMETHOD Vector addDelta(const Vector& p_w_a,const Vector& p_w_da,double dt=1);

IMETHOD Rotation addDelta(const Rotation& R_w_a,const Vector& da_w,double dt=1);

IMETHOD Frame addDelta(const Frame& F_w_a,const Twist& da_w,double dt=1);

IMETHOD Twist addDelta(const Twist& a,const Twist&da,double dt=1);

IMETHOD Wrench addDelta(const Wrench& a,const Wrench&da,double dt=1);

#ifdef KDL_INLINE
#include "frames.inl"
#endif
}

#endif
