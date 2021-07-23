#ifndef DRAW_IO_CONTROL_H
#define DRAW_IO_CONTROL_H

//--------------------
// System libraries
//--------------------
#include <Arduino.h>
#include <stdint.h>

//--------------------
// Custom code
//--------------------
#include "CoordinateVector.h"

#define Px PORTB
#define Py PORTD
#define tapsX 6
#define tapsY 6
#define tapsCountX (0x01 << tapsX)
#define tapsCountY (0x01 << tapsY)
#define maskPortY 0xFC // 0b 1111 1100
#define maskPortX 0x3F // 0b 0011 1111

//--------------------
// Function declerations
//--------------------
void setupIO();
void drawXY(vector2 coordinateSample);

#endif /* DRAW_IO_CONTROL_H */
