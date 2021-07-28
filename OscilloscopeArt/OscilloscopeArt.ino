// https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf

//--------------------
// System libraries
//--------------------
#include <limits.h>

//--------------------
// Custom code
//--------------------
#include "Util.h"
#include "DrawIoControl.h"
#include "RealTimeControl.h"
#include "CoordinateVector.h"

//--------------------
// Variables/constants
//--------------------
long int startMilis = 0;


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


float interpolate(float x1, float y1, float x2, float y2, float x);

void testSquare();
void circleStatic();
void circleRotateInX();
void circleRotateInY();
void circleRotateInXY();
void squareOfLength(float sideLength);
void pulsingSquare();
float pitchCos(uint32_t frequency, int32_t degree, float amplitude, float phase);
bool playTones(uint32_t fs, ToneData* tones, uint8_t toneCount, float duration_s);


// https://nl.mathworks.com/help/aerotbx/ug/quatrotate.html
// https://eater.net/quaternions

float dot(vector3 v1, vector3 v2);
vector3 cross(vector3 v1, vector3 v2);
vector3 scale(float s, vector3 v);
vector3 add(vector3 v1, vector3 v2);
vector3 rotateVector(vector3 vIn, vector3 vRot, float angleDeg);
vector2 project2dTo3d(vector3, float distFromScreen);

vector2 oneSecondGeneralBuffer[128];

void setup()
{
  setupTimer();
  setupIO();
  
  Serial.begin(115200);
  vector3 rotatedVector = rotateVector({1, 0, 1}, {0, 0, 1}, 90);
  Serial.println();
  Serial.println(rotatedVector.x);
  Serial.println(rotatedVector.y);
  Serial.println(rotatedVector.z);
}

void loop()
{ /*
    for (int y = 0; y < 100; y++)
    {
     for (int x = 0; x < 100; )
     {
       vector2 myVector =
       {
         (float)x / 100.0,
         (float)y / 100.0
       };

       bool ret = addSample(myVector);

       if (ret)
       {
         x++;
       }

       delay(10);
     }
    }
  */
  /*
    for (uint8_t i = 0; i < 3; i++)
    {
    pulsingSquare();
    }

    for (uint8_t i = 0; i < 1; i++)
    {
    circleRotateInX();
    circleRotateInY();
    }
  */
  /*
        for (uint32_t p = 50; p < 10000; p += 50)
        {
         ToneData tonep = ToneData{p, 0.3, 0};
         ToneData tones[] = {tonep};
         playTones(tones, ARRAYLENGTH(tones), 0.001);
        }
  */
  ToneData tones2[] = {tone1000hz};
  playTones(FS, tones2, ARRAYLENGTH(tones2), 0.002);
}

void testSquare()
{
  for (uint8_t y = 0; y < tapsCountY; y++)
  {
    for (uint8_t x = 0; x < tapsCountX; x++)
    {
      addSample(vector2{(float)x / tapsCountX, (float)y / tapsCountY});
      delay(10);
    }
  }
}

void circleStatic()
{
  for (float d = 0; d < 360; d++)
  {
    addSample(vector2{0.5 + (0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin(d * degToRadFactor))});
    delay(1);
  }
}

void circleRotateInX()
{
  for (float r = 0; r < 180; r += 3)
  {
    for (float d = 0; d < 360; d += 6)
    {
      addSample(vector2{0.5 + (0.45 * cos(d * degToRadFactor)), 0.5 + (cos(r * degToRadFactor) * 0.4 * sin(d * degToRadFactor))});
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
      addSample(vector2{0.5 + (cos(r * degToRadFactor) * 0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin(d * degToRadFactor))});
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
      addSample(vector2{0.5 + (0.4 * cos(d * degToRadFactor)), 0.5 + (0.4 * sin((d + r)*degToRadFactor))});
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

  addSample(vector2{c[0][0], c[0][1]});
  float stepSize = (c[1][0] - c[0][0]) / (interpolationCount + 1);
  float newX = c[0][0] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newX += stepSize;
    addSample(vector2{newX, interpolate(c[0][0], c[0][1], c[1][0], c[1][1], newX)});
  }

  addSample(vector2{c[1][0], c[1][1]});
  stepSize = (c[3][1] - c[1][1]) / (interpolationCount + 1);
  float newY = c[1][1] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newY += stepSize;
    addSample(vector2{interpolate(c[1][1], c[1][0], c[3][1], c[3][0], newY), newY});
  }

  addSample(vector2{c[3][0], c[3][1]});
  stepSize = (c[2][0] - c[3][0]) / (interpolationCount + 1);
  newX = c[3][0] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newX += stepSize;
    addSample(vector2{newX, interpolate(c[3][0], c[3][1], c[2][0], c[2][1], newX)});
  }

  addSample(vector2{c[2][0], c[2][1]});
  stepSize = (c[0][1] - c[2][1]) / (interpolationCount + 1);
  newY = c[2][1] + stepSize;
  for (uint8_t p = 0; p < interpolationCount; p++)
  {
    newY += stepSize;
    addSample(vector2{interpolate(c[2][1], c[2][0], c[0][1], c[0][0], newY), newY});
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

bool playTones(uint32_t fs, ToneData* tones, uint8_t toneCount, float duration_s)
{
  if (duration_s*fs > 128)
  {
    return false;
  }

  float combinedTone = 0;

  const uint32_t sampleCount = fs * duration_s;

  for (uint32_t s = 0; s < sampleCount; s++)
  {
    combinedTone = 0;
    for (uint8_t i = 0; i < toneCount; i++)
    {
      combinedTone += 0.5 + (0.5 * tones[i].amplitude * cos((2.0 * M_PI * ((float)tones[i].frequency / fs) * s) - (tones[i].phase * degToRadFactor)));
    }

    if (combinedTone > 1)
    {
      combinedTone = 1;
    }
    else if (combinedTone < 0)
    {
      combinedTone = 0;
    }
    
    //addSample(vector2{combinedTone, combinedTone});
    oneSecondGeneralBuffer[s] = {combinedTone, combinedTone};
  }

  Serial.println(sampleCount);

  for (uint32_t i = 0; i < sampleCount; i++)
  {
    bool ret = addSample(oneSecondGeneralBuffer[i]);
    while (!ret) 
    {
      ret = addSample(oneSecondGeneralBuffer[i]);
    }
  }

  return true;
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





float dot(vector3 v1, vector3 v2)
{
  return (
           v1.x * v2.x +
           v1.y * v2.y +
           v1.z * v2.z
         );
}

vector3 cross(vector3 v1, vector3 v2)
{
  return
  {
    v1.y * v2.z - v1.z * v2.y,
    v1.z * v2.x - v1.x * v2.z,
    v1.x * v2.y - v1.y * v2.z,
  };
}

vector3 scale(float s, vector3 v)
{
  return
  {
    s * v.x,
    s * v.y,
    s * v.z,
  };
}

vector3 add(vector3 v1, vector3 v2)
{
  return
  {
    v1.x + v2.x,
    v1.y + v2.y,
    v1.z + v2.z
  };
}

vector3 rotateVector(vector3 vIn, vector3 vRot, float angleDeg)
{
  // https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
  // Do the math
  // u is the vector about rotation with xyz of the quaternion and v the vector to rotate
  //vprime = 2.0f * dot(u, v) * u
  //      + (s*s - dot(u, u)) * v
  //      + 2.0f * s * cross(u, v);

  const float halfAngle = angleDeg / 2;
  const float cosHalfAngle = cos(halfAngle * degToRadFactor);
  const float sinHalfAngle = sin(halfAngle * degToRadFactor);

  vector3 rotationVector =
  {
    sinHalfAngle * vRot.x,
    sinHalfAngle * vRot.y,
    sinHalfAngle * vRot.z
  };

  Serial.println(vIn.x);
  Serial.println(vIn.y);
  Serial.println(vIn.z);
  Serial.println();
  Serial.println(rotationVector.x);
  Serial.println(rotationVector.y);
  Serial.println(rotationVector.z);

  float two_dot_rv = 2.0 * dot(rotationVector, vIn);
  Serial.print("two_dot_rv: ");
  Serial.println(two_dot_rv);
  vector3 scale_r = scale(two_dot_rv, rotationVector);
  Serial.print("scale_r: ");
  Serial.println(scale_r.x);
  Serial.println(scale_r.y);
  Serial.println(scale_r.z);

  float square_s_min_norm_r = cosHalfAngle * cosHalfAngle - dot(rotationVector, rotationVector);
  Serial.print("square_s_min_norm_r: ");
  Serial.println(square_s_min_norm_r);
  vector3 scale_v = scale(square_s_min_norm_r, vIn);
  Serial.print("scale_v: ");
  Serial.println(scale_v.x);
  Serial.println(scale_v.y);
  Serial.println(scale_v.z);

  vector3 sum_scaledr_scaledv = add(scale_r, scale_v);
  Serial.print("sum_scaledr_scaledv: ");
  Serial.println(sum_scaledr_scaledv.x);
  Serial.println(sum_scaledr_scaledv.y);
  Serial.println(sum_scaledr_scaledv.z);

  vector3 cross_rv = cross(rotationVector, vIn);
  Serial.print("cross_rv: ");
  Serial.println(cross_rv.x);
  Serial.println(cross_rv.y);
  Serial.println(cross_rv.z);
  vector3 scaled_cross_rv = scale(2.0 * cosHalfAngle, cross_rv);
  Serial.print("scaled_cross_rv: ");
  Serial.println(scaled_cross_rv.x);
  Serial.println(scaled_cross_rv.y);
  Serial.println(scaled_cross_rv.z);

  vector3 sum_scaledr_scaledv_scaledCrossrv = add(sum_scaledr_scaledv, scaled_cross_rv);

  return sum_scaledr_scaledv_scaledCrossrv;
}

/*
  vector2_int8 project2dTo3d(&vector3_int8, int8_t distFromScreen)
  {
  static const vector3_int8 camera = {0, 0, -distFromScreen};
  }
*/

//https://stackoverflow.com/questions/2602823/in-c-c-whats-the-simplest-way-to-reverse-the-order-of-bits-in-a-byte/61109975
/*
  uint8_t reverse(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
  }
*/
