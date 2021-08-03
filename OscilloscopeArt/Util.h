#ifndef UTIL_H
#define UTIL_H

//--------------------
// System libraries
//--------------------
#include <math.h>
#include <stdint.h>

//--------------------
// Defines and macros
//--------------------
#define SET |=
#define UNSET &= ~
#define ARRAYLENGTH(x)  (sizeof(x) / sizeof((x)[0]))
#define degToRadFactor M_PI / 180

//--------------------
// Functions
//--------------------
uint32_t lcm(uint32_t n1, uint32_t n2);
uint32_t gcd(uint32_t n1, uint32_t n2);

#endif /* UTIL_H */
