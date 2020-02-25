#include <SD.h>
// #include <TimeLib.h>          //RTC
#include <FlexCAN.h>          //CAN
// #include <kinetis_flexcan.h>  //Additional CAN library to allow for extended ID's for filtering


#include "logger.h"
#include "kelly.h"
#include "motor.h"
#include "data-packaging.h"

#define CAN_BAUD 50000
static uint8_t hex[17] = "0123456789abcdef";

File logFile;

char buff[50];

static CAN_message_t inMsgR, inMsgL, msg;
Motor motorL, motorR;

void setup() {

  delay(300);
  Serial.begin(115200);
  Serial.println(F("Begin."));
  // put your setup code here, to run once:

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD Card initialization failed!"));
  } else {
    Serial.println(F("SD Card initialization ."));
    createLogFile(logFile);
  }


  // CAN
  Can0.begin();
  Can1.begin();
  //Create a CAN filter: allow certain IDs
  // CAN_filter_t allPassFilter;
  // allPassFilter.id=0;   //ID = 0 defaults to allow everything in. Set to a value to allow only that ID through
  // allPassFilter.ext=1;  //ext = 1 allows 29-bit ID's through, ext=0 allows 11-bit ID's through
  // allPassFilter.rtr=0;  //rtr = 0 (unsure as of yet)
  // //Apply the above filter to all the 16 mailboxes to begin with -> Will be refined later
  // for (uint8_t MailboxNum = 0; MailboxNum < 16;MailboxNum++)
  // {
  //   Can0.setFilter(allPassFilter,MailboxNum);
  //   Can1.setFilter(allPassFilter,MailboxNum);
  // }

  msg.ext = 0;
  msg.id = 0x100;
  msg.len = 8;
  msg.buf[0] = 0x9d;
  msg.buf[1] = 0x06;
  msg.buf[2] = 0x9e;
  msg.buf[3] = 0x3f;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  // hexDump(8, msg.buf);
  Serial.println("Start:");
  // Can1.write(msg);


  // TEST converting float to byte-buffer for transmission.
    // float x = 1.23457678;
    // float a = 3.145;
    // packFloat(x, msg.buf, 4);
    // packFloat(a, msg.buf, 0);
    // hexDump(8, msg.buf);
    // float y = unPackFloat(msg.buf, 4);
    // float z = unPackFloat(msg.buf, 0);

    // Serial.println(y,10);
    // Serial.println(z,10);


}



void loop() {

  CAN_message_t inMsg;
  while (Can0.available())
  {
    Serial.println("available");
    Can0.read(inMsg);
    Serial.print("CAN bus 0: "); hexDump(8, inMsg.buf);
  }

  msg.buf[0]++;
  Can1.write(msg);
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

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
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
