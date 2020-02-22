#include <SD.h>
#include <TimeLib.h>          //RTC


#include "logger.h"
#include "kelly.h"
#include "motor.h"


File logFile;
String file_name;

char buff[50];

void setup() {
  delay(4000);
  Serial.begin(115200);
  Serial.println(F("Begin."));
  // put your setup code here, to run once:

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD Card initialization failed!"));
  } else {
    Serial.println(F("SD Card initialization done."));
  }

  file_name=generateFilename();
  strcat(buff,file_name.c_str());
  Serial.print("access file: ");
  Serial.println(buff);
  logFile = SD.open(buff, FILE_WRITE);  // 8.3 file format name eg: 12345678.txt. Directories do not count towards name length.

  if(logFile)
  {
    Serial.println("success");
    logFile.println("hello world");
    logFile.close();
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}


// String SD_new_file_str(void)
// {
//   String new_file="";
//   // new_file+= year();
//   // new_file+= "-";
//   // new_file+= month();
//   // new_file+= "-";
//   // new_file+= day();
//   // new_file+= "/";
//   new_file+=hour();
//   new_file+="_";
//   new_file+=minute();
//   new_file+="_";
//   new_file+=second();
//   new_file+=".txt";
//   return new_file;
// }

void RTC_INIT(void)
{
  //Initialise RTC
  setSyncProvider(getTeensy3Time);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}
