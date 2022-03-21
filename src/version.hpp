#ifndef VERSION_H
#define VERSION_H


#include <stdio.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 1
#define REVISION 0
#define STRINGIFY(x) #x
#define VERSION_STR(A,B,C) STRINGIFY(A) "." STRINGIFY(B) "."  STRINGIFY(C)


static std::string version_string = VERSION_STR(VERSION_MAJOR, VERSION_MINOR, REVISION);

#endif // VERSION_H