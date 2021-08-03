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

bool on = true;
void drawXY(vector2 coordinateSample)
{
  //uint8_t xInt = coordinateSample.x * (tapsCountX - 1);
  //uint8_t yInt = (tapsCountY - 1) * (1 - coordinateSample.y); // On the oscilloscope Y starts at the bottom.
  // So to make top-left (0,0) start from the maximum.

  Px = (uint8_t)coordinateSample.x;
  Py = ((uint8_t)coordinateSample.y << 2);

/*
  if (on)
  {
    PORTB SET (0x1 << 5);
    on = false;
  }
  else
  {
    PORTB UNSET (0x1 << 5);
    on = true;
  }
  */
}

vector2 scaleToCoordinates(vector2 scaler)
{
  uint8_t xInt = scaler.x * (tapsCountX - 1);
  uint8_t yInt = (tapsCountY - 1) * (1 - scaler.y); // On the oscilloscope Y starts at the bottom.
  // So to make top-left (0,0) start from the maximum.

  return vector2{xInt, yInt};
}
