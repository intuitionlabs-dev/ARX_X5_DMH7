#ifndef KDL_UTILITY_H
#define KDL_UTILITY_H

#include "kdl-config.h"
#include <cstdlib>
#include <cassert>
#include <cmath>

#ifdef KDL_INLINE
    #ifdef _MSC_VER

        #define IMETHOD __forceinline
    #else

        #define IMETHOD inline
    #endif
#else
    #define IMETHOD
#endif

/* 启用索引边界检查时使用 assert，行为受 NDEBUG 控制。 */

#ifdef KDL_INDEX_CHECK
    #define FRAMES_CHECKI(a) assert(a)
#else
    #define FRAMES_CHECKI(a)
#endif

namespace KDL {
#ifdef __GNUC__

    using ::sin;
    using ::cos;
    using ::exp;
    using ::log;
    using ::sin;
    using ::cos;
    using ::tan;
    using ::sinh;
    using ::cosh;
    using ::pow;
    using ::sqrt;
    using ::atan;
    using ::hypot;
    using ::asin;
    using ::acos;
    using ::tanh;
    using ::atan2;
#endif
#ifndef __GNUC__

    #pragma warning (disable:4786)

    inline double sin(double a) {
        return ::sin(a);
    }

    inline double cos(double a) {
        return ::cos(a);
    }
    inline double exp(double a) {
        return ::exp(a);
    }
    inline double log(double a) {
        return ::log(a);
    }
    inline double tan(double a) {
        return ::tan(a);
    }
    inline double cosh(double a) {
        return ::cosh(a);
    }
    inline double sinh(double a) {
        return ::sinh(a);
    }
    inline double sqrt(double a) {
        return ::sqrt(a);
    }
    inline double atan(double a) {
        return ::atan(a);
    }
    inline double acos(double a) {
        return ::acos(a);
    }
    inline double asin(double a) {
        return ::asin(a);
    }
    inline double tanh(double a) {
        return ::tanh(a);
    }
    inline double pow(double a,double b) {
        return ::pow(a,b);
    }
    inline double atan2(double a,double b) {
        return ::atan2(a,b);
    }
#endif

#if (__cplusplus > 199711L)
using std::isnan;
#endif

template <class T>
class TI
{
    public:
        typedef const T& Arg;
};

template <>
class TI<double> {
public:
    typedef double Arg;
};

template <>
class TI<int> {
public:
    typedef int Arg;
};

extern int          STREAMBUFFERSIZE;

extern int          MAXLENFILENAME;

extern const double PI;

extern const double PI_2;

extern const double PI_4;

extern const double deg2rad;

extern const double rad2deg;

/* Equal 比较使用的默认精度为 1e-7。 */
extern double     epsilon;

extern int          VSIZE;

#ifndef _MFC_VER
#undef max
inline double max(double a,double b) {
    if (b<a)
        return a;
    else
        return b;
}

#undef min
inline double min(double a,double b) {
    if (b<a)
        return b;
    else
        return a;
}
#endif

#ifdef _MSC_VER

    #define INLINE __forceinline

#else
    #define INLINE inline
#endif

inline double LinComb(double alfa,double a,
        double beta,double b ) {
            return alfa*a+beta*b;
}

inline void LinCombR(double alfa,double a,
        double beta,double b,double& result ) {
            result=alfa*a+beta*b;
         }

inline void SetToZero(double& arg) {
    arg=0;
}

inline void SetToIdentity(double& arg) {
    arg=1;
}

inline double sign(double arg) {
    return (arg<0)?(-1):(1);
}

inline double sqr(double arg) { return arg*arg;}
inline double Norm(double arg) {
    return fabs(  (double)arg );
}

#if defined __WIN32__ && !defined __GNUC__
inline double hypot(double y,double x) { return ::_hypot(y,x);}
inline double abs(double x) { return ::fabs(x);}
#endif

inline bool Equal(double a,double b,double eps=epsilon)
{
    double tmp=(a-b);
    return ((eps>tmp)&& (tmp>-eps) );
}

inline void random(double& a) {
	a = 1.98*rand()/(double)RAND_MAX -0.99;
}

inline void posrandom(double& a) {
	a = 0.001+0.99*rand()/(double)RAND_MAX;
}

inline double diff(double a,double b,double dt) {
	return (b-a)/dt;
}

inline double addDelta(double a,double da,double dt) {
	return a+da*dt;
}
}

#endif
