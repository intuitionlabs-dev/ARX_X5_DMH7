#ifndef KDL_UTILITY_IO_H_84822
#define KDL_UTILITY_IO_H_84822

#include <iostream>
#include <iomanip>
#include <fstream>

namespace KDL {
void _check_istream(std::istream& is);

void Eat(std::istream& is, int delim );

void Eat(std::istream& is,const char* descript);

void EatWord(std::istream& is,const char* delim,char* storage,int maxsize);

void EatEnd( std::istream& is, int delim );
}

#endif
