#include "logger.h"
#include <TimeLib.h>          //RTC


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
