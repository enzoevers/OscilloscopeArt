#include "RealTimeControl.h"

//--------------------
// System libraries
//--------------------
#include <Arduino.h>
#include <stdint.h>

//--------------------
// Custom code
//--------------------
#include "Util.h"
#include "DrawIoControl.h"
#include "CircularBuffer.h"

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
//const uint16_t FS = 44077;

//const uint16_t bufferSize = 64;
volatile circularBuffer<vector2> outputBuffer(RT_bufferSize);

//--------------------
// ISR
//--------------------
ISR(TIMER1_COMPA_vect)
{
  vector2 curCoor;
  bool ret = outputBuffer.dequeue(curCoor);
  if (ret)
  {
    drawXY(curCoor);
  }
}

//--------------------
// Function implementations
//--------------------
void setupTimer()
{
  cli(); // Disable interrupts

  // Set Timer 1 to mode 4
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B SET (1 << WGM12); // Use mode 4
  TCCR1B SET (1 << CS10); // Use the internal clock as the counter for the clock
  OCR1A = 363; // See the calculations above the declaration of the 'fs' constant
  TCNT1  = 0x00; // Reset the timer counter
  TIMSK1 = 0;
  TIMSK1 SET (1 << OCIE1A); // Enable overflow interrupt

  sei(); // Enable interrupts
}

void clearOutputBuffer()
{
  outputBuffer.clearBuffer();
}

bool addSample(vector2 newSample)
{
  bool ret = outputBuffer.enqueue(newSample);
  return ret;
}

void disableOutput()
{
  cli(); // Disable interrupts
  TIMSK1 UNSET (1 << OCIE1A); // Disaable overflow interrupt
  sei(); // Enable interrupts
}

void enableOutput()
{
  //cli(); // Disable interrupts
  //TIMSK1 SET (1 << OCIE1A); // Enable overflow interrupt
  //sei(); // Enable interrupts
  setupTimer();
}
