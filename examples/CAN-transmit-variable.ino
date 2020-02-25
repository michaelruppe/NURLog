//*****************************************************************************
// Transmit variables over CAN
// Pack the raw variable data into a CAN message, transmit it, then unpack it
// on the receiving end. This requires CAN0 & CAN1 to be looped together.
//
// You will have to copy this code to a new project and include the relevant
// libraries.
//
//
// Michael Ruppe, Jan-2020
// www.github.com/michaelruppe
//
//
//*****************************************************************************

#include <FlexCAN.h>
#include "data-packaging.h"

static const int CAN_BAUD = 25000;
static const uint8_t hex[17] = "0123456789abcdef";


char buff[50];

static CAN_message_t  msg;


void setup() {
  delay(250);
  Serial.begin(115200);
  Serial.println("Start: Transmit variables over CAN");
  Can0.begin(CAN_BAUD);
  Can1.begin(CAN_BAUD);

  msg.ext = 0;
  msg.id = 0x100;
  msg.len = 8;
  msg.buf[0] = 0x00;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;


  // Test all variable types
  msg.id = 0x100;
  packByte(0x55, msg.buf, 0);   // packs byte[0]
  packShort(-256, msg.buf, 1);  // packs byte[1-2]
  packUShort(256, msg.buf, 3);  // packs byte[3-4]
  hexDump(8, msg.buf);
  Can1.write(msg);

  msg.id = 0x101;
  packInt(-4321, msg.buf, 0); // packs byte[0-3]
  packUInt(4321, msg.buf, 4); // packs byte[4-7]
  hexDump(8, msg.buf);
  Can1.write(msg);

  msg.id = 0x102;
  packFloat(1.23456789, msg.buf, 0); // packs byte[0-3]
  packFloat(3.145, msg.buf, 4); // packs byte[0-3]
  hexDump(8, msg.buf);
  Can1.write(msg);

}



void loop() {

  CAN_message_t inMsg;
  while (Can0.available())
  {
    Serial.println("CAN0 available");
    Can0.read(inMsg);
    Serial.print("Receive: "); hexDump(8, inMsg.buf);
    if (inMsg.id == 0x100){ // [Byte, Short, UShort]
      uint8_t a = unPackByte(inMsg.buf,0);
      short b = unPackShort(inMsg.buf,1);
      unsigned short c = unPackUShort(inMsg.buf,3);
      Serial.print(a); Serial.print(" "); Serial.print(b); Serial.print(" "); Serial.println(c);
    }
    if (inMsg.id == 0x101){ // [int, Uint]
      int d = unPackInt(inMsg.buf,0);
      unsigned int e = unPackUInt(inMsg.buf,4);
      Serial.print(d); Serial.print(" "); Serial.println(e);
    }
    if (inMsg.id == 0x102){ // [float, float]
      float f = unPackFloat(inMsg.buf,0);
      float g = unPackFloat(inMsg.buf,4);
      Serial.print(f); Serial.print(" "); Serial.println(g);
    }
  }

  delay(100);

}

// Dump the contents of a buffer to serial.
void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
    Serial.print(" ");
  }
  Serial.write('\r');
  Serial.write('\n');
}
