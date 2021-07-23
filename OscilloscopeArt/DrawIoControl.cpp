#include "DrawIoControl.h"

//--------------------
// Custom code
//--------------------
#include "Util.h"

//--------------------
// Function implementations
//--------------------
void setupIO()
{
  DDRD SET maskPortY; // 0b 1111 1100
  PORTD UNSET maskPortY;

  DDRB SET maskPortX; // 0b 0011 1111
  PORTB UNSET maskPortX;
}

void drawXY(vector2 coordinateSample)
{
  uint8_t xInt = coordinateSample.x + (tapsCountX/2);
  uint8_t yInt = tapsCountY - coordinateSample.y - (tapsCountY/2); // On the oscilloscope Y starts at the bottom.
  // So to make top-left (0,0) start from the maximum.

  Px = xInt;
  Py = (yInt << 2);
}
