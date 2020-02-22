/************************************
 *  Hardware Definitions for NURLog *
 ************************************/

 
#define GPS_TX       0     //Set pin 0 as the GPS TX Signal
#define GPS_RX       1     //Set pin 1 as the GPS RX Signal
#define COOLING_SIG  2     //Set pin 2 as the signal input from the Cooling -> Digital
#define BRAKE_SIG    5     //Set pin 5 as the signal input from the brake pedal -> Digital
#define RTD_SIG      6     //Set pin 6 as the signal input from the RTD State -> Digital
#define PDOC_SIG     7     //Set pin 7 as the signal input from the PDOC -> Digital
#define BMS_SIG      8     //Set pin 8 as the signal input from the BMS -> Digital
#define BSPD_SIG     9     //Set pin 9 as the signal input from the BSPD -> Digital
#define IMD_SIG      10    //Set pin 10 as the signal input from the IMD -> Digital
#define TEENSY_LED   13    //Set pin 13 as the onboard Teensy 3.6 LED, as per hardware
#define POWER_SIG    A0    //Set pin 14 (A0) as the signal input from the 12V Battery Power pedal -> Analog
#define GPS_FIX__SIG 15    //Set pin 15 as the signal input from the GPS Fix State -> Digital (PWM)
#define IMD_PWM__SIG 16    //Set pin 16 as the signal input from the IMD Data Pin -> Digital (PWM)
#define APS_OOB__SIG 17    //Set pin 17 as the signal input from the APPS Out Of Bounds status -> Digital
#define POS_AIR_SIG  21    //Set pin 21 as the signal input from the Positive AIR State -> Digital
#define NEG_AIR_SIG  22    //Set pin 22 as the signal input from the Negative AIR State -> Digital
#define THR_SIG      A9    //Set pin 23 (A9) as the signal input from the Raw Throttle -> Analog
#define DIS1_DIO_PIN 24    //Set pin 24 as clock pin for the speedo display
#define DIS1_CLK_PIN 25    //Set pin 25 as data pin for the speedo display
#define DIS1_STB_PIN 26    //Set pin 26 as reset pin for the speedo display
#define DIS2_DIO_PIN 28    //Set pin 28 as clock pin for the Motor Temp display
#define DIS2_CLK_PIN 29    //Set pin 29 as data pin for the Motor Temp display
#define DIS2_STB_PIN 30    //Set pin 30 as reset pin for the Motor Temp display
#define GPIO1_SIG    35    //Set pin 35 as the signal input from the GPIO Pin 1 -> Digital
#define APS_DIS_SIG  39    //Set pin 39 as the signal input from the Sensor Disagree Check on the APPS -> Digital
#define LATCH_THR    36    //Set pin 36 as the signal input for the state of the latched throttle check of APPS -> Digital
#define LIN_RES_SIG1 A10   //Set pin A10 as the signal input from the Linear Resistive Sensor 1 -> Analog
#define LIN_RES_SIG2 A11   //Set pin A11 as the signal input from the Linear Resistive Sensor 2 -> Analog
#define DAC_OUT_1    A21   //Set pin A21 as the signal output from the onboard DAC 1 -> Analog
#define DAC_OUT_2    A22   //Set pin A22 as the signal output from the onboard DAC 2 -> Analog
