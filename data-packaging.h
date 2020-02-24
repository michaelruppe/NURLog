#ifndef DATA_PACKAGING_H
#define DATA_PACKAGING_H

#include <stdint.h>
#include "Arduino.h"

// Pack the binary data for a float variable into a buffer for a bitstream
void packFloat(float var, uint8_t buf[sizeof(float)], uint8_t index);

// UnPack the binary data for a float variable from a buffer
float unPackFloat(uint8_t buf[sizeof(float)], int index);


#endif
