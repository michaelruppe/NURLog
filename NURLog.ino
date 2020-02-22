#include <SD.h>
#include <TimeLib.h>          //RTC
#include <FlexCAN.h>          //CAN
#include <kinetis_flexcan.h>  //Additional CAN library to allow for extended ID's for filtering


#include "logger.h"
#include "kelly.h"
#include "motor.h"


File logFile;

char buff[50];

CAN_message_t inMsgR, inMsgL;
Motor motorL, motorR;

void setup() {

  delay(3000);
  Serial.begin(115200);
  Serial.println(F("Begin."));
  // put your setup code here, to run once:

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD Card initialization failed!"));
  } else {
    Serial.println(F("SD Card initialization done."));
  }

  createLogFile(logFile);



}

void loop() {
  // put your main code here, to run repeatedly:

}


void RTC_INIT(void)
{
  //Initialise RTC
  setSyncProvider(getTeensy3Time);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
