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
// Internal clock frequency is 16MHz
// Desired sampling frequency is 44.1KHz
//
// 44,100/16,000,000 = 0.00275625
// 1/0.00275625 ~= 362.81179 ~= 363 cycles
// 16,000,000/363 ~= 44,077Hz = 44.077KHz
// So counting 363 cycles from the internal clock results in 44.077KHz
// which is almost the desired 44.1KHz.
#define FS 44077
#define RT_bufferSize 128


//--------------------
// Function declerations
//--------------------
void setupTimer();
void clearOutputBuffer();
bool addSample(vector2 newSample);

void disableOutput();
void enableOutput();

#endif /* REAL_TIME_CONTROL_H */
