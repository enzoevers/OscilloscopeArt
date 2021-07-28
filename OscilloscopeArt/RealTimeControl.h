#ifndef REAL_TIME_CONTROL_H
#define REAL_TIME_CONTROL_H

//--------------------
// System libraries
//--------------------
#include <stdint.h>

//--------------------
// Custom code
//--------------------
#include "CoordinateVector.h"

//--------------------
// Variables/constants
//--------------------
extern const uint16_t FS;
extern const uint16_t bufferSize;

//--------------------
// Function declerations
//--------------------
void setupTimer();
bool addSample(vector2 newSample);

#endif /* REAL_TIME_CONTROL_H */
