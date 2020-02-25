// ToDo: Look into compressing functionality using function templates

#ifndef DATA_PACKAGING_H
#define DATA_PACKAGING_H

#include "Arduino.h"
#include <stdint.h>

/* For a Teensy 3.6:
char are 1 byte(s)
shorts are 2 byte(s)
Ints are 4 byte(s)
long are 4 byte(s)

floats are 4 byte(s)
double are 8 byte(s)

word are 4 byte(s)
*/





// Pack the binary data for a float variable into a buffer for a bitstream
void packFloat(float var, uint8_t buf[sizeof(float)], uint8_t index);
float unPackFloat(uint8_t buf[sizeof(float)], uint8_t index);
void packInt(int var, uint8_t buf[sizeof(int)], uint8_t index);
int unPackInt(uint8_t buf[sizeof(int)], uint8_t index);
void packUint(int var, uint8_t buf[sizeof(unsigned int)], uint8_t index);
unsigned int unPackUint(uint8_t buf[sizeof(unsigned int)], uint8_t index);
void packByte(int var, uint8_t buf[sizeof(char)], uint8_t index);
unsigned int unPackByte(uint8_t buf[sizeof(uint8_t)], uint8_t index);


#endif
