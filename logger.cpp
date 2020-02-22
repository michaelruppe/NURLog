#include "logger.h"
#include <TimeLib.h>          //RTC


int createLogFile(File &file) {
  char buff[50];
  String file_name=generateFilename();
  strcat(buff,file_name.c_str());
  Serial.print("access file: ");
  Serial.println(buff);
  file = SD.open(buff, FILE_WRITE);  // 8.3 file format name eg: 12345678.txt. Directories do not count towards name length.

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
  String new_file="";
  // new_file+= year();
  // new_file+= "-";
  // new_file+= month();
  // new_file+= "-";
  // new_file+= day();
  // new_file+= "/";
  new_file+=hour();
  new_file+="_";
  new_file+=minute();
  new_file+="_";
  new_file+=second();
  new_file+=".txt";
  return new_file;
}
