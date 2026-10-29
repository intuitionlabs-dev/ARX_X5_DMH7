#ifndef FRAMES_IO_H
#define FRAMES_IO_H

#include "utilities/utility_io.h"
#include "frames.hpp"
#include "jntarray.hpp"
#include "jacobian.hpp"

namespace KDL {
    std::ostream& operator << (std::ostream& os,const Vector& v);
    std::ostream& operator << (std::ostream& os,const Rotation& R);
    std::ostream& operator << (std::ostream& os,const Frame& T);
    std::ostream& operator << (std::ostream& os,const Twist& T);
    std::ostream& operator << (std::ostream& os,const Wrench& T);
    std::ostream& operator << (std::ostream& os,const Vector2& v);
    std::ostream& operator << (std::ostream& os,const Rotation2& R);
    std::ostream& operator << (std::ostream& os,const Frame2& T);

    std::istream& operator >> (std::istream& is,Vector& v);
    std::istream& operator >> (std::istream& is,Rotation& R);
    std::istream& operator >> (std::istream& is,Frame& T);
    std::istream& operator >> (std::istream& os,Twist& T);
    std::istream& operator >> (std::istream& os,Wrench& T);
    std::istream& operator >> (std::istream& is,Vector2& v);
    std::istream& operator >> (std::istream& is,Rotation2& R);
    std::istream& operator >> (std::istream& is,Frame2& T);
}

#endif
