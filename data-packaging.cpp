#include "data-packaging.h"

// Pack the binary data for a float variable into a buffer for a bitstream
void packFloat(float var, uint8_t buf[], uint8_t index) {
  memcpy(buf+index, &var, sizeof(float));
}
// UnPack the binary data for a float variable from a buffer
float unPackFloat(uint8_t buf[], uint8_t index) {
  float temp;
  memcpy(&temp, buf + index, sizeof(float));
  return temp;
}


// Pack the binary data for a int variable into a buffer for a bitstream
void packInt(int var, uint8_t buf[], uint8_t index) {
  memcpy(buf+index, &var, sizeof(int));
}
// UnPack the binary data for a int variable from a buffer
int unPackInt(uint8_t buf[sizeof(int)], uint8_t index) {
  int temp;
  memcpy(&temp, buf + index, sizeof(int));
  return temp;
}


// Pack the binary data for a unsigned int variable into a buffer for a bitstream
void packUint(int var, uint8_t buf[], uint8_t index) {
  memcpy(buf+index, &var, sizeof(unsigned int));
}
// UnPack the binary data for a unsigned int variable from a buffer
unsigned int unPackUint(uint8_t buf[], uint8_t index) {
  unsigned int temp;
  memcpy(&temp, buf + index, sizeof(unsigned int));
  return temp;
}


// Pack the binary data for a unsigned int variable into a buffer for a bitstream
void packByte(int var, uint8_t buf[], uint8_t index) {
  memcpy(buf+index, &var, sizeof(uint8_t));
}
// UnPack the binary data for a unsigned int variable from a buffer
unsigned int unPackByte(uint8_t buf[], uint8_t index) {
  uint8_t temp;
  memcpy(&temp, buf + index, sizeof(uint8_t));
  return temp;
}
