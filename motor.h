/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class Motor
{
  public:
    Motor();
    int rpm;
    int current;
    int voltage;
  private:
    int dummy;
};

#endif
