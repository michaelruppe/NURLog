// This file defines constants for the Kelly Motor Controllers (KMC)

#ifndef KELLY_H
#define KELLY_H

#include <stdint.h>

// int testFunction(void);
// int kelly_getRPM(CAN_message_t &message);
// int kelly_getCurrent(CAN_message_t &message);
// int kelly_getVoltage(CAN_message_t &message);
// uint8_t kelly_getThrottle(CAN_message_t &message);
// uint8_t kelly_getMotorTemp(CAN_message_t &message);
// uint8_t kelly_getControllerTemp(CAN_message_t &message);

const unsigned long KMC_MSG_ID_1 = 0x0CF11E05;
const unsigned long KMC_MSG_ID_2 = 0x0CF11F05;

// Byte-addresses of parameters within a CAN message
// ADDL => Address, lower-byte
// ADDU => Address, upper-byte

// RPM
const uint8_t KMC_RPM_ADDL = 0;
const uint8_t KMC_RPM_ADDU = 1;
// Current
const uint8_t KMC_CUR_ADDL = 2;
const uint8_t KMC_CUR_ADDU = 3;
// Voltage
const uint8_t KMC_VLT_ADDL = 4;
const uint8_t KMC_VLT_ADDU = 5;

// Throttle
const uint8_t KMC_THR_ADD  = 0;

// Temp, Motor
const uint8_t KMC_TEMP_MOTOR_ADD  = 2;

// Temp, Driver
const uint8_t KMC_TEMP_CONT_ADD  = 1;



const uint8_t KMC_OFS_MOT_TEMP = 40;
const uint8_t KMC_OFS_DRV_TEMP = 40;
// int kelly_getRPM(CAN_message_t &message);

#endif
