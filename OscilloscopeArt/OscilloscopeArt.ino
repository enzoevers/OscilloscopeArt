#include <limits.h>
// https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
#define Px PORTB
#define Py PORTD

#define SET |=
#define UNSET &= ~
#define ARRAYLENGTH(x)  (sizeof(x) / sizeof((x)[0]))

const uint8_t tapsX = 6;
const uint8_t tapsY = 6;

const uint8_t tapsCountX = 0b1 << tapsX;
const uint8_t tapsCountY = 0b1 << tapsY;

const uint8_t maskPortY = 0xFC;
const uint8_t maskPortX = 0x3F;

const float degToRadFactor = M_PI / 180;

long int startMilis = 0;

uint8_t reverse(uint8_t b);


typedef struct
{
  uint32_t frequency;
  float amplitude;
  float phase;
} ToneData;

ToneData tone220hz = ToneData{220, 0.2, 0};
ToneData tone440hz = ToneData{440, 0.3, 0};
ToneData tone550hz = ToneData{550, 0.5, 0};
ToneData tone800hz = ToneData{800, 0.5, 0};
ToneData tone1000hz = ToneData{1000, 0.7, 0};
ToneData tone1hz = ToneData{1, 0.7, 0};

void drawXY(float x, float y);
float interpolate(float x1, float y1, float x2, float y2, float x);
void testSquare();
void circleStatic();
void circleRotateInX();
void circleRotateInY();
void circleRotateInXY();
void squareOfLength(float sideLength);
void pulsingSquare();
float pitchCos(uint32_t frequency, int32_t degree, float amplitude, float phase);
void playTones(ToneData* tones, uint8_t toneCount, float duration_s);

// https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
// https://nl.mathworks.com/help/aerotbx/ug/quatrotate.html
// https://eater.net/quaternions

template <typename T>
struct vector2
{
  T x;
  T y;
};

template <typename T>
struct vector3
{
  T x;
  T y;
  T z;
};

template <typename T>
struct Quaternion
{
  T r;
  T i;
  T j;
  T k;
};

float dot(vector3<float> v1, vector3<float> v2);
vector3<float> cross(vector3<float> v1, vector3<float> v2);
vector3<float> scale(float s, vector3<float> v);
vector3<float> add(vector3<float> v1, vector3<float> v2);
vector3<int8_t> rotateVector(vector3<int8_t> vIn, vector3<int8_t> vRot, float angleDeg);
vector2<int8_t> project2dTo3d(vector3<int8_t>, uint8_t distFromScreen);

// Internal clock frequency is 16MHz
// Desired sampling frequency is 44.1KHz
//
// 44,100/16,000,000 = 0.00275625
// 1/0.00275625 ~= 362.81179 ~= 363 cycles
// 16,000,000/363 ~= 44,077Hz = 44.077KHz
// So counting 363 cycles from the internal clock results in 44.077KHz
// which is almost the desired 44.1KHz.
const uint16_t fs = 44077;

volatile int testValue = 1;

ISR(TIMER1_COMPA_vect)
{
  PINB SET 0x20;
  PINB SET 0x20;
}

void setup()
{
  //--------------------
  // Audio sample timer
  //--------------------
  cli(); // Disable interrupts

  // Set Timer 1 to mode 4
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B SET (1<<WGM12); // Use mode 4 
  TCCR1B SET (1<<CS10); // Use the internal clock as the counter for the clock
  OCR1A = 363; // See the calculations above the declaration of the 'fs' constant
  TCNT1  = 0x00; // Reset the timer counter 
  TIMSK1 = 0;
  TIMSK1 SET (1<<OCIE1A); // Enable overflow interrupt
  
  sei(); // Enable interrupts

  //--------------------
  // I/O
  //--------------------
  DDRD SET maskPortY; // 0b 1111 1100
  PORTD UNSET maskPortY;

  DDRB SET maskPortX; // 0b 0011 1111
  PORTB UNSET maskPortX;





  /*Serial.begin(9600);
  vector3<int8_t> rotatedVector = rotateVector({1, 0, 1}, {0, 0, 1}, 90);
  Serial.println();
  Serial.println(rotatedVector.x);
  Serial.println(rotatedVector.y);
  Serial.println(rotatedVector.z);
*/
}

void loop()
{/*
  for (uint8_t i = 0; i < 3; i++)
  {
    pulsingSquare();
  }

  for (uint8_t i = 0; i < 1; i++)
  {
    circleRotateInX();
    circleRotateInY();
  }


  for (uint32_t p = 50; p < 10000; p += 50)
  {
    ToneData tonep = ToneData{p, 0.3, 0};
    ToneData tones[] = {tonep};
    playTones(tones, ARRAYLENGTH(tones), 0.001);
  }

  ToneData tones2[] = {tone1000hz};
  playTones(tones2, ARRAYLENGTH(tones2), 0.1);
  */
}

void testSquare()
{
  for (uint8_t y = 0; y < tapsCountY; y++)
  {
    for (uint8_t x = 0; x < tapsCountX; x++)
    {
      drawXY((float)x / tapsCountX, (float)y / tapsCountY);
      delay(1);
    }
  }
}

void circleStatic()
{
  for (float d = 0; d < 360; d++)
  {
    drawXY(0.5 + (0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin(d * degToRadFactor)));
    delay(1);
  }
}

void circleRotateInX()
{
  for (float r = 0; r < 180; r += 3)
  {
    for (float d = 0; d < 360; d += 6)
    {
      drawXY(0.5 + (0.45 * cos(d * degToRadFactor)), 0.5 + (cos(r * degToRadFactor) * 0.4 * sin(d * degToRadFactor)));
    }
    //delayMicroseconds(1);
  }
}

void circleRotateInY()
{
  for (float r = 0; r < 180; r += 3)
  {
    for (float d = 0; d < 360; d += 6)
    {
      drawXY(0.5 + (cos(r * degToRadFactor) * 0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin(d * degToRadFactor)));
    }
    //delayMicroseconds(1);
  }
}

void circleRotateInXY()
{
  for (float r = 0; r < 180; r += 3)
  {
    for (float d = 0; d < 360; d += 4)
    {
      drawXY(0.5 + (0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin((d + r)*degToRadFactor)));
    }
    delayMicroseconds(10);
  }
}

void squareOfLength(float sideLength)
{
  float c[][2] = {
    {0.5 - (sideLength / 2), 0.5 - (sideLength / 2)}, {0.5 + (sideLength / 2), 0.5 - (sideLength / 2)},
    {0.5 - (sideLength / 2), 0.5 + (sideLength / 2)}, {0.5 + (sideLength / 2), 0.5 + (sideLength / 2)}
  };

  uint8_t interpolationCount = 31;

  drawXY(c[0][0], c[0][1]);
  float stepSize = (c[1][0] - c[0][0]) / (interpolationCount + 1);
  float newX = c[0][0] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newX += stepSize;
    drawXY(newX, interpolate(c[0][0], c[0][1], c[1][0], c[1][1], newX));
  }

  drawXY(c[1][0], c[1][1]);
  stepSize = (c[3][1] - c[1][1]) / (interpolationCount + 1);
  float newY = c[1][1] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newY += stepSize;
    drawXY(interpolate(c[1][1], c[1][0], c[3][1], c[3][0], newY), newY);
  }

  drawXY(c[3][0], c[3][1]);
  stepSize = (c[2][0] - c[3][0]) / (interpolationCount + 1);
  newX = c[3][0] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newX += stepSize;
    drawXY(newX, interpolate(c[3][0], c[3][1], c[2][0], c[2][1], newX));
  }

  drawXY(c[2][0], c[2][1]);
  stepSize = (c[0][1] - c[2][1]) / (interpolationCount + 1);
  newY = c[2][1] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newY += stepSize;
    drawXY(interpolate(c[2][1], c[2][0], c[0][1], c[0][0], newY), newY);
  }
}

void pulsingSquare()
{
  for (int8_t i = 0; i <= 6; i++)
  {
    squareOfLength(i / 10.0);
    delay(50);
  }

  delay(30);

  for (int8_t i = 6; i >= 0; i--)
  {
    squareOfLength(i / 10.0);
    delay(50);
  }

  delay(20);
}

float pitchCos(uint32_t frequency, int32_t degree, float amplitude, float phase)
{
  return 0.5 + (0.5 * (amplitude * cos(frequency * (((((float)degree / 1000) / frequency) - phase) * degToRadFactor))));
}

void playTones(ToneData* tones, uint8_t toneCount, float duration_s)
{
  static const uint32_t fs = 44100;

  float combinedTone = 0;

  const uint32_t sampleCount = fs * duration_s;

  for (uint32_t s = 0; s < sampleCount; s++)
  {
    combinedTone = 0;
    for (uint8_t i = 0; i < toneCount; i++)
    {
      combinedTone += 0.5 + (0.5 * tones[i].amplitude * cos((2.0 * M_PI * ((float)tones[i].frequency / fs) * s) - (tones[i].phase * degToRadFactor)));
    }
    drawXY(combinedTone, combinedTone);
  }
}

// Return the y value for position x interpolated between (x1,y1) and (x2,y2)
float interpolate(float x1, float y1, float x2, float y2, float x)
{
  if (x1 == x2)
  {
    return 0;
  }

  float slope = (y2 - y1) / (x2 - x1);
  return y1 + slope * (x - x1);
}

void drawXY(float x, float y)
{
  uint8_t xInt = x * tapsCountX;
  uint8_t yInt = tapsCountY * (1 - y); // On the oscilloscope Y starts at the bottom.
  // So to make top-left (0,0) start from the maximum.

  Px = xInt;
  Py = (yInt << 2);
}

//https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte/61109975
uint8_t reverse(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}


float dot(vector3<float> v1, vector3<float> v2)
{
  return (
           v1.x * v2.x +
           v1.y * v2.y +
           v1.z * v2.z
         );
}

vector3<float> cross(vector3<float> v1, vector3<float> v2)
{
  return
  {
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.z,
  };
}

vector3<float> scale(float s, vector3<float> v)
{
  return
  {
    s * v.z,
    s * v.x,
    s * v.y,
  };
}

vector3<float> add(vector3<float> v1, vector3<float> v2)
{
  return
  {
    v1.x + v2.x,
    v1.y + v2.y,
    v1.z + v2.z
  };
}

vector3<int8_t> rotateVector(vector3<int8_t> vIn, vector3<int8_t> vRot, float angleDeg)
{
  const float halfAngle = angleDeg / 2;
  const float cosHalfAngle = cos(halfAngle * degToRadFactor);
  const float sinHalfAngle = sin(halfAngle * degToRadFactor);

  vector3<float> v =
  {
    vIn.x,
    vIn.y,
    vIn.z
  };

  vector3<float> rotationVector =
  {
    sinHalfAngle * vRot.x,
    sinHalfAngle * vRot.y,
    sinHalfAngle * vRot.z
  };

  Serial.println(v.x);
  Serial.println(v.y);
  Serial.println(v.z);
  Serial.println();
  Serial.println(rotationVector.x);
  Serial.println(rotationVector.y);
  Serial.println(rotationVector.z);

  vector3<float> vprime = add(
                            add(
                              scale(2.0 * dot(rotationVector, v), rotationVector), scale((cosHalfAngle * cosHalfAngle - dot(rotationVector, rotationVector)), v)
                            )
                            , scale(2.0 * cosHalfAngle, cross(rotationVector, v))
                          );

  vector3<int8_t> rotatedVector;
  if (vprime.x > SCHAR_MAX)
  {
    rotatedVector.x = SCHAR_MAX;
  }
  else if (vprime.x < SCHAR_MIN)
  {
    rotatedVector.x = SCHAR_MIN;
  }
  else
  {
    rotatedVector.x = vprime.x;
  }

  if (vprime.y > SCHAR_MAX)
  {
    rotatedVector.y = SCHAR_MAX;
  }
  else if (vprime.y < SCHAR_MIN)
  {
    rotatedVector.y = SCHAR_MIN;
  }
  else
  {
    rotatedVector.y = vprime.y;
  }

  if (vprime.z > SCHAR_MAX)
  {
    rotatedVector.z = SCHAR_MAX;
  }
  else if (vprime.z < SCHAR_MIN)
  {
    rotatedVector.z = SCHAR_MIN;
  }
  else
  {
    rotatedVector.z = vprime.z;
  }
}

/*
  vector2_int8 project2dTo3d(&vector3_int8, int8_t distFromScreen)
  {
  static const vector3_int8 camera = {0, 0, -distFromScreen};
  }
*/
