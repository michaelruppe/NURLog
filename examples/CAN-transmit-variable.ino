//*****************************************************************************
// Transmit two variables in an 8-byte CAN frame.
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
  Serial.print("Before : "); hexDump(8, msg.buf);

  // Pack an Int (Teensy3.6: 4bytes) and a float (Teensy3.6: 4bytes) into an
  // 8-byte CAN message buffer.
  int testInt = 1234;
  float testFloat = 3.415;
  packInt(testInt, msg.buf, 0); // packs byte[0-3]
  packFloat(testFloat, msg.buf, 4); // packs byte[4-7]
  Serial.print("After  : "); hexDump(8, msg.buf);
  Serial.println("Transmitting CAN1");
  Can1.write(msg);

}



void loop() {

  CAN_message_t inMsg;
  while (Can0.available())
  {
    Serial.println("CAN0 available");
    Can0.read(inMsg);
    Serial.print("Receive: "); hexDump(8, inMsg.buf);
    float a = unPackFloat(inMsg.buf, 4);
    int b = unPackInt(inMsg.buf, 0);

    Serial.print(a); Serial.print(" ");
    Serial.print(b); Serial.print(" ");

    Serial.println();
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
