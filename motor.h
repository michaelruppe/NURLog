/*
Morse.h - Library for flashing Morse code.
Created by David A. Mellis, November 2, 2007.
Released into the public domain.
*/
#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

// Data structure for holding motor parameters
struct Motor
{
  int rpm;
  int current;
  int voltage;
  uint8_t throttle;
  uint8_t tempMotor;
  uint8_t tempController;
};



#endif
