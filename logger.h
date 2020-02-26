#ifndef LOGGER_H
#define LOGGER_H

#include "Arduino.h"
#include <SD.h>

void initialiseSD(File &file);
String generateFilename(void);

int createLogFile(File &file);

#endif
