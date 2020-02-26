#include "logger.h"
#include <TimeLib.h>          //RTC

void initialiseSD(File &file){
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(F("SD Card initialization failed!"));
  } else {
    Serial.println(F("SD Card initialization ."));
    createLogFile(file);
  }
}

int createLogFile(File &file) {
  String file_name=generateFilename();
  Serial.print("access file: ");
  Serial.println(file_name.c_str());
  file = SD.open(file_name.c_str(), FILE_WRITE);  // 8.3 file format name eg: 12345678.txt. Directories do not count towards name length.
  delay(20);
  if(file)
  {
    Serial.println("success");
    file.println("hello world");
    file.close();
    return 1;
  } else {
    return 0;
  }
}

// Generate a filename based off the current time.
String generateFilename(void)
{
  // Make a directory with today's date
  String directory="";
  directory+=year();
  directory+="-";
  directory+=month();
  directory+="-";
  directory+=day();
  SD.mkdir(directory.c_str());

  // Make a timestamped file, in today's directory
  String new_file = directory + "/";
  new_file+=hour();
  new_file+="_";
  new_file+=minute();
  new_file+="_";
  new_file+=second();
  new_file+=".txt";
  return new_file;
}
