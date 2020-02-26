//*****************************************************************************
// NURLog
// Michael Ruppe, Jan-2020
// www.github.com/michaelruppe
//
// FlexCAN: the vanilla library, and an additional file in the library: kinetis_flexcan.h
//
//
//
//
//*****************************************************************************

#include <SD.h>
// #include <TimeLib.h>          //RTC
#include <FlexCAN.h>          //CAN
#include <kinetis_flexcan.h>  //Additional CAN library to allow for extended ID's for filtering


#include "logger.h"
#include "kelly.h"
#include "motor.h"
#include "data-packaging.h"

static const int CAN_BAUD = 50000;
static const uint8_t hex[17] = "0123456789abcdef";

File logFile;

char buff[50];

static CAN_message_t inMsgR, inMsgL, msg;
Motor motorL, motorR;

void setup() {

  delay(300);
  Serial.begin(115200);
  Serial.println(F("Begin."));


  initialiseSD(logFile);

  // if (!SD.begin(BUILTIN_SDCARD)) {
  //   Serial.println(F("SD Card initialization failed!"));
  // } else {
  //   Serial.println(F("SD Card initialization ."));
  //   createLogFile(logFile);
  // }

  // CAN
  Can0.begin(CAN_BAUD);
  Can1.begin(CAN_BAUD);



}



void loop() {

  CAN_message_t inMsg;
  while (Can0.available())
  {
    Serial.println("available");
    Can0.read(inMsg);
    Serial.print("Afterr: "); hexDump(8, inMsg.buf); Serial.println("");
    float a = unPackFloat(inMsg.buf, 0);
    // int b = unPackInt(inMsg.buf, 0);
    // unsigned int c = unPackUint(inMsg.buf, 6);
    Serial.print("Receive: ");
    Serial.print(a); Serial.print(" ");
    // Serial.print(b); Serial.print(" ");
    // Serial.print(c);
    Serial.println();
  }
  // msg.buf[0]++;
  // Can1.write(msg);
  // msg.buf[0]++;
  // Can1.write(msg);
  // msg.buf[0]++;
  // Can1.write(msg);
  // msg.buf[0]++;
  // Can1.write(msg);
  // msg.buf[0]++;
  // Can1.write(msg);
  delay(1000);

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

// void RTC_INIT(void)
// {
//   //Initialise RTC
//   setSyncProvider(getTeensy3Time);
// }
//
// time_t getTeensy3Time()
// {
//   return Teensy3Clock.get();
// }
