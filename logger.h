#ifndef LOGGER_H
#define LOGGER_H

#include "Arduino.h"
#include <SD.h>

String generateFilename(void);

int createLogFile(File &file);

#endif
