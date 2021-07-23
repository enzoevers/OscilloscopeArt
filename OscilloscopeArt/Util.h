#ifndef UTIL_H
#define UTIL_H

//--------------------
// System libraries
//--------------------
#include <math.h>

//--------------------
// Defines and macros
//--------------------
#define SET |=
#define UNSET &= ~
#define ARRAYLENGTH(x)  (sizeof(x) / sizeof((x)[0]))
#define degToRadFactor M_PI / 180

#endif /* UTIL_H */
