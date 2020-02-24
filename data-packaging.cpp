#include "data-packaging.h"

// Pack the binary data for a float variable into a buffer for a bitstream
void packFloat(float var, uint8_t buf[sizeof(float)], uint8_t index) {
  memcpy(buf+index, &var, sizeof(float));

}

// UnPack the binary data for a float variable from a buffer
float unPackFloat(uint8_t buf[sizeof(float)], int index) {
  float temp;
  memcpy(&temp, buf + index, sizeof(float));
  return temp;
}
