#include "kelly.h"


int kelly_getRPM(CAN_message_t &message) {
  // return message.buf[KMC_RPM_ADDL]+256*message.buf[KMC_RPM_ADDU];
  return message.buf[KMC_RPM_ADDU] << 8 | message.buf[KMC_RPM_ADDL];
}

int kelly_getCurrent(CAN_message_t &message) {
  return (message.buf[KMC_CUR_ADDL]+256*message.buf[KMC_CUR_ADDU])/10;
}

int kelly_getVoltage(CAN_message_t &message) {
  return (message.buf[KMC_VLT_ADDL]+256*message.buf[KMC_VLT_ADDU])/10;
}

uint8_t kelly_getThrottle(CAN_message_t &message){
  return (message.buf[KMC_THR_ADD]/255.0)*100;
}

uint8_t kelly_getMotorTemp(CAN_message_t &message){
  return (message.buf[KMC_TEMP_MOTOR_ADD] - KMC_OFS_MOT_TEMP);
}

uint8_t kelly_getControllerTemp(CAN_message_t &message){
  return (message.buf[KMC_TEMP_CONT_ADD] - KMC_OFS_DRV_TEMP);
}
