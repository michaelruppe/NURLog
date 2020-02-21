#include "kelly.h"

#include <FlexCAN.h>          //CAN
#include <kinetis_flexcan.h>  //Additional CAN library to allow for extended ID's for filtering


int kelly_getRPM(CAN_message_t &message) {
  return 1;
}
