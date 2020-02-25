#ifndef DATA_PACKAGING_H
#define DATA_PACKAGING_H

#include "Arduino.h"
#include <stdint.h>

// Pack the binary data for a float variable into a buffer for a bitstream
void packFloat(float var, uint8_t buf[sizeof(float)], uint8_t index);
void packInt(int var, uint8_t buf[sizeof(float)], uint8_t index);
void packUint(int var, uint8_t buf[sizeof(unsigned int)], uint8_t index);
float unPackFloat(uint8_t buf[sizeof(float)], int index);
int unPackInt(uint8_t buf[sizeof(int)], int index);
unsigned int unPackUint(uint8_t buf[sizeof(unsigned int)], int index);

#endif
