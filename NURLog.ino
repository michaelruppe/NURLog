/*
 *  ******************************************
 *  Project:  Datalogger for F-SAE EV-One NU Racing
 *  Function: Bring in raw data from the motor
 *            controllers and BMS through CAN as well
 *            as various signals from the
 *            around the car to be logged onto an SD card
 *            and display certain parameters
 *            on the dash
 *   Hardware: - Teesny 3.6
 *             - 2 x TM1638 Display boards
 *             - CCU v3 Breakout Board (See team documents and wiki for KiCad files and pinout)
 *             - Adafruit Ultimate GPS
 *             - 32GB micro SD card
 *             - EV.One and all of its goodies
 *   Authors:
 *            Nic Rodgers - c3206083
 *            Michael Ruppe github.com/michaelruppe
 *
 *
 *   Date:  26-08-19 v1 Rodgers
 *          XX-XX-XX v2 Ruppe
 *
 *  ******************************************
 */

 /***********************************
 *  Libraries                       *
 ***********************************/
#include <SD.h>               //SD Card
#include <SPI.h>              //SPI Communication
#include <TimeLib.h>          //RTC
#include <TM1638.h>           //Displays
#include <binary.h>           //Required for LED's on Displays
#include <FlexCAN.h>          //CAN
#include <kinetis_flexcan.h>  //Additional CAN library to allow for extended ID's for filtering
#include <Wire.h>             //I2C Communication
#include <Adafruit_GPS.h>     //GPS Module
#include <EEPROM.h>

/***********************************
*  Project Files                       *
***********************************/
#include "NURLog_GPIO.h"
#include "GPS.h"
#include "kelly.h"


const int chipSelect = BUILTIN_SDCARD; //This is referencing the onboard SD card Chip Select


/***********************************
 *  Define all modules             *
 ***********************************/
//Display Modules
TM1638 module1(DIS1_DIO_PIN, DIS1_CLK_PIN, DIS1_STB_PIN);
TM1638 module2(DIS2_DIO_PIN, DIS2_CLK_PIN, DIS2_STB_PIN);

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

//Define the GPS to be on Serial Port 1
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

/***********************************
 * Variables                       *
 ***********************************/
File myFile;
int voltageR, rpmR,currentR,throttleR,tempMotR,tempDrvR,voltageL,rpmL,currentL,throttleL,tempMotL,CTMP_L, min_cell_volt_IDX, min_cell_temp, max_cell_temp, max_cell_temp_IDX;
float max_cell = 4.1;
float min_cell = 2.9;
float rad = 0.22;     //Wheel Radius
float min_cell_volt,max_cell_volt,shunt_volt;
long shunt_cur;
int cell_num   = 42;
#define TIME_HEADER  "T"          // Header tag for serial time sync message
#define CAN_BAUD 250000           // Constant that is used for CAN Baud Rate -> Matches that of the Kelly Cont.
#define pi 3.14159265359          // PI = 3.14159265359
CAN_message_t inMsgR,inMsgL;      // Input Message Structure that is saved. Inludes all data, ID's speeds etc.
String file_name,Var_Names,Units,min_vals,max_vals;
char buff[50],buff2[50];
bool SD_flag = 0,display_flag = 0,low_cell_err,high_cell_err,low_cell_temp_err,high_cell_temp_err,low_shunt_volt_err,high_shunt_volt_err;
char c;
int GPS_FIX;
float Lat,Longi,GPS_Vel;
float input_volt,input_throttle,tot_dist;
/************************************
 * interupts                        *
 ************************************/
 IntervalTimer disp_Timer;
 IntervalTimer SD_Timer;

/***************************************************
 *  SETUP                                          *
 **************************************************/
void setup()
{
  delay(100);
 // Open serial communications and wait for port to open:
  Serial.begin(115200);
  //Initialising Utilised Modules
  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD Card initialization failed!"));
  }
  Serial.println(F("SD Card initialization done."));
  RTC_INIT();
  GPS_Init();
  GPIO_Init();

  //Initialising CAN
  Can0.begin(CAN_BAUD);
  Can1.begin(CAN_BAUD);

  //Create a CAN filter: allow certain IDs
  CAN_filter_t allPassFilter;
  allPassFilter.id=0;   //ID = 0 defaults to allow everything in. Set to a value to allow only that ID through
  allPassFilter.ext=1;  //ext = 1 allows 29-bit ID's through, ext=0 allows 11-bit ID's through
  allPassFilter.rtr=0;  //rtr = 0 (unsure as of yet)
  //Apply the above filter to all the 16 mailboxes to begin with -> Will be refined later
  for (uint8_t MailboxNum = 0; MailboxNum < 16;MailboxNum++)
  {
    Can0.setFilter(allPassFilter,MailboxNum);
    Can1.setFilter(allPassFilter,MailboxNum);
  }
  module2.setDisplayToString("EU.1");
  module1.setDisplayToString("EU.1");
  delay(12000);
  module1.clearDisplay();
  module2.clearDisplay();

  disp_Timer.begin(disp_flag_toggle,100000);
  SD_Timer.begin(SD_Timer_toggle,100000); //10000 micro seconds

  file_name=SD_new_file_str();
  strcat(buff,file_name.c_str());

  myFile = SD.open(buff, FILE_WRITE);  //MAX NEW FILE LENGTH IS 12 Characters -> can only have 8 cahracters then ".txt"
  Var_Names="Right RPM,Right Current,Right Voltage,Right Throttle,Right Motor Temp,Right Controller Temp,Left RPM,Left Current,Left Voltage,Left Throttle,Left Motor Temp,Left Controller Temp,PDOC_status,BMS_status,BSPD_status,IMD_status,RTC_hour,RTC_min,RTC_sec,min cell volt,max cell volt,min cell volt index,min cell temp,max cell temp,max cell temp index,shunt voltage,shunt current,low_cell_voltage_err,high_cell_voltage_err,low_cell_temp_err,high_cell_temp_err,low_shunt_volt_err,high_shunt_volt_err,Lat,Long,GPS Velocity,UTC,date,GPS angle,Right Velocity,Left Velocity,Input Voltage,Raw Throttle Input,Out of Bounds APPS Check,Disagreeing Sensors APPS Check,Trailbraking APPS Check,Negative AIR status,Cooling State,Brake State,Ready to Drive State,Running Total Distance,Total Current Draw, Total Power";
  Units="[RPM],[Amps],[Volts],[%],[deg C],[deg C],[RPM],[Amps],[Volts],[%],[deg C],[deg C],[bool],[bool],[bool],[bool],[hour],[min],[sec],[Volts],[Volts],[Cell no.],[deg C],[deg C],[Cell no.],[Volts],[milliAmps],[bool],[bool],[bool],[bool],[bool],[bool],[decimal_deg],[decimal_deg],[kph],[hh:mm:ss.sss],[YYMMDD],[deg],[kph],[kph],[V],[%],[bool],[bool],[bool],[bool],[bool],[bool],[bool],[m],[Amps],[kW]";
  min_vals="0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-365000,0,0,0,0,0,0,-33,151,0,0,191101,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
  max_vals="1200,180,200,100,160,70,1200,180,200,100,160,70,1,1,1,1,24,60,60,4.5,4.5,42,65,65,42,200,365000,1,1,1,1,1,1,-32,152,100,23:59:59.999,500101,360,100,100,16,100,1,1,1,1,1,1,1,25000,250,50";
  myFile.println(Var_Names);
  myFile.println(Units);
  myFile.println(min_vals);
  myFile.println(max_vals);
  myFile.println();
  myFile.close();
  Serial.println();
}

// SANDBOX ================================
int kelly_getRPM(CAN_message_t &message) {
  return message.buf[KMC_RPM_ADDL]+256*message.buf[KMC_RPM_ADDU];
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

// SANDBOX ^

/***************************************************
 *  Loop: Main Code that iterates through      *
 **************************************************/
void loop(void)
{
  //CAN
  while(Can0.available())
  {
    //Serial.println(inMsgR.id,HEX);
    for(int j=0;j<2;j++)
    {
      if(Can0.read(inMsgR))
      {
        //Right Motor Controllers Message 1
        if(inMsgR.id == KMC_MSG_ID_1)
        {
          //Serial.println("Entered RHS 0x0CF11E05");
          // rpmR=inMsgR.buf[0]+256*inMsgR.buf[1];
          rpmR = kelly_getRPM(inMsgR);
          // currentR=(inMsgR.buf[2]+256*inMsgR.buf[3])/10;
          currentR = kelly_getCurrent(inMsgR);
          // voltageR=(inMsgR.buf[4]+256*inMsgR.buf[5])/10;
          voltageR = kelly_getVoltage(inMsgR);
        }
        //Right Motor Controllers Message 2
        else if(inMsgR.id == KMC_MSG_ID_1)
        {
          //Serial.println("Entered RHS 0x0CF11F05");
          // throttleR = (inMsgR.buf[0]/255.0)*100; //Avoid integer division by dividing by 255.0
          throttleR = kelly_getThrottle(inMsgR);
          // tempMotR= (inMsgR.buf[2]-40);        //Note this says 30 deg. offset in datasheet, see sensor experiment. This is cofirmed to be a 40 deg. offset
          tempMotR = kelly_getMotorTemp(inMsgR);
          // tempDrvR= (inMsgR.buf[1]-40);        //Note this says 40 deg. offset in datasheet
          tempDrvR = kelly_getControllerTemp(inMsgR);
        }
        //BMS Baud Translation Board Output Message 1
        else if(inMsgR.id==0x0111110)
        {
          //Serial.println("Entered BMS 0x0111110");
          min_cell_volt=(inMsgR.buf[0]+inMsgR.buf[1]*256.0)/1000;
          max_cell_volt=(inMsgR.buf[2]+inMsgR.buf[3]*256.0)/1000;
          min_cell_volt_IDX=inMsgR.buf[4];
          min_cell_temp=inMsgR.buf[5]-40;
          max_cell_temp=inMsgR.buf[6]-40;
          max_cell_temp_IDX=inMsgR.buf[7];
        }
        //BMS Baud Translation Board Output Message 2
        else if(inMsgR.id==0x0111120)
        {
          shunt_volt=(inMsgR.buf[0]+inMsgR.buf[1]*256.0)/100;
          for(int i = 2; i < 6; i++)
          //Serial.println("Entered BMS 0x0111120");
          {
             shunt_cur = (shunt_cur << 8) | inMsgR.buf[i];
          }
          low_cell_err=bitRead(inMsgR.buf[6],0);
          high_cell_err=bitRead(inMsgR.buf[6],1);
          low_cell_temp_err=bitRead(inMsgR.buf[6],2);
          high_cell_temp_err=bitRead(inMsgR.buf[6],3);
          low_shunt_volt_err=bitRead(inMsgR.buf[7],0);
          high_shunt_volt_err=bitRead(inMsgR.buf[7],1);
        }
        else Serial.println("Message ID incorrect");
      }
    }
  }
  while(Can1.available())
  {
    //Serial.println(inMsgL.id,HEX);
    for(int j=0;j<2;j++)
    {
      if(Can1.read(inMsgL))
      {
        //Left Motor Controllers Message 1
        if(inMsgL.id == KMC_MSG_ID_1)
        {
          //Serial.println("Entered LHS 0x0CF11E05");
          rpmL=inMsgL.buf[0]+256*inMsgL.buf[1];
          currentL=(inMsgL.buf[2]+256*inMsgL.buf[3])/10;
          voltageL=(inMsgL.buf[4]+256*inMsgL.buf[5])/10;
        }
        //Left Motor Controllers Message 2
        else if(inMsgL.id == KMC_MSG_ID_2)
        {
          //Serial.println("Entered LHS 0x0CF11F05");
          throttleL = (inMsgL.buf[0]/255.0)*100; //Avoid integer division by dividing by 255.0
          tempMotL= (inMsgL.buf[2]-40);        //Note this says 30 deg. offset in datasheet, see sensor experiment. This is cofirmed to be a 40 deg. offset
          CTMP_L= (inMsgL.buf[1]-40);        //Note this says 40 deg. offset in datasheet
        }
        //BMS Baud Translation Board Output Message 1
        else if(inMsgL.id==0x0111110)
        {
          //Serial.println("Entered BMS 0x0111110");
          min_cell_volt=(inMsgL.buf[0]+inMsgL.buf[1]*256.0)/1000;
          max_cell_volt=(inMsgL.buf[2]+inMsgL.buf[3]*256.0)/1000;
          min_cell_volt_IDX=inMsgL.buf[4];
          min_cell_temp=inMsgL.buf[5]-40;
          max_cell_temp=inMsgL.buf[6]-40;
          max_cell_temp_IDX=inMsgL.buf[7];
        }
        //BMS Baud Translation Board Output Message 2
        else if(inMsgL.id==0x0111120)
        {
          //Serial.println("Entered BMS 0x0111120");
          shunt_volt=(inMsgL.buf[0]+inMsgL.buf[1]*256.0)/100;
          for(int i = 2; i < 6; i++)
          {
             shunt_cur = (shunt_cur << 8) | inMsgR.buf[i];
          }
          low_cell_err=bitRead(inMsgL.buf[6],0);
          high_cell_err=bitRead(inMsgL.buf[6],1);
          low_cell_temp_err=bitRead(inMsgL.buf[6],2);
          high_cell_temp_err=bitRead(inMsgL.buf[6],3);
          low_shunt_volt_err=bitRead(inMsgL.buf[7],0);
          high_shunt_volt_err=bitRead(inMsgL.buf[7],1);
        }
        else Serial.println("Message ID incorrect");
      }
    }
  }

  //********************GPS Conversions********************************************************//
  GPS_FIX=GPS_Task();
  Lat=Pos_Conv(GPS.latitude,GPS.lat);
  Longi=Pos_Conv(GPS.longitude,GPS.lon);
  GPS_Vel=GPS.speed*1.852;

  //********************ALL SD Prints********************************************************//
  //print to SD card on the timer trigger
  if(SD_flag && GPS_FIX>=5)
  {
    //Read all Digital/Analog Pins
    tot_dist=(GPS_Vel/3.6)*.1+tot_dist;           //This is the trapazoidal rule: distance travelled = velocity(in m/s)*timestep+running velocity
    input_volt=PWR_CHECK();
    print_SD(rpmR,currentR,voltageR,throttleR,tempMotR,tempDrvR,rpmL,currentL,voltageL,throttleL,tempMotL,CTMP_L,digitalRead(PDOC_SIG),digitalRead(BMS_SIG), digitalRead(BSPD_SIG), digitalRead(IMD_SIG),min_cell_volt, max_cell_volt, min_cell_volt_IDX, min_cell_temp, max_cell_temp, max_cell_temp_IDX, shunt_volt, shunt_cur, low_cell_err, high_cell_err, low_cell_temp_err, high_cell_temp_err, low_shunt_volt_err, high_shunt_volt_err,Lat,Longi,GPS_Vel,GPS.hour,GPS.minute,GPS.seconds,GPS.milliseconds,GPS.day,GPS.month,GPS.year,GPS.angle,input_volt,thr_check(),digitalRead(APS_OOB__SIG),digitalRead(APS_DIS_SIG),digitalRead(LATCH_THR),digitalRead(NEG_AIR_SIG),digitalRead(COOLING_SIG),digitalRead(BRAKE_SIG),RTD_check(),tot_dist);
    SD_flag = 0;
  }

  //********************ALL PRINTS TO SCREENS********************************************************//
  //update the displays on the timer trigger
  if(display_flag)
  {
    // Battery Display
    disp_batt_LEDs(voltageR);
    // Hard Fault Checks
    HF_LEDs(digitalRead(PDOC_SIG),digitalRead(BMS_SIG), digitalRead(BSPD_SIG), digitalRead(IMD_SIG),low_shunt_volt_err,low_cell_err,digitalRead(LATCH_THR),GPS_FIX);
    // Motor Temperature Display
    disp_mot_temps(tempMotR,tempMotL);
    // Tractive System Voltage Display
    disp_voltages(min_cell_volt,voltageR);
    display_flag=0;
  }
}

/*
 *************************************************************************************
 * Interrupt Function - Runs every 150ms to toggle a flag to reprint to the displays *
 * ***********************************************************************************
*/
void disp_flag_toggle(void)
{
    display_flag = 1;
}

void SD_Timer_toggle(void)
{
    SD_flag = 1;
}

/********************************************************************
 *                      INPUT PINS ANALYSIS                         *
 * ******************************************************************
 */

float PWR_CHECK(void)
{
  int inVol;
  float pwr;
  inVol=analogRead(POWER_SIG);
  pwr=(inVol/1024.0)*(3.3*630/100);           //This multiplier has come from trial and error
  return(pwr);
}

float thr_check(void)
{
  int dig_thr;
  float raw_thr;
  dig_thr=analogRead(THR_SIG);
  raw_thr=(((dig_thr/1024.0)*(9.9/2))/5)*100;           //This multiplier has come from 3.3V rails through a voltage divider down from 5 -> therefore: (100k+200k/200k)*3.3
  return(raw_thr);
}

bool RTD_check(void)
{
  //This ensures that the RTD is high (on) only when the shutdown power is active, and also the RTD signal is low (already activated)
  if(digitalRead(NEG_AIR_SIG) && !digitalRead(RTD_SIG))
  {
    return 1;
  }
  else return 0;
}

/*
 *******************************************************************
 * All SD Card Code - Initialisation, display SD card type etc.
 * *****************************************************************
*/

String SD_new_file_str(void)
{
  String new_file="";
  new_file="";
  new_file+=hour();
  new_file+="_";
  new_file+=minute();
  new_file+="_";
  new_file+=second();;
  new_file+=".txt";
  return new_file;
}

/*
 *******************************************************************
 * All GPS Code - Initialisation, Polling etc.
 * *****************************************************************
*/
void GPS_Init(void)
{
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

int GPS_Task(void)
{
  c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return 0; // we can fail to parse a sentence in which case we should just wait for another
  }
  return GPS.satellites;
}

float Pos_Conv(float inLong,char dir)
{
  int deg;
  float minu,out;

  deg=(int)(inLong/100);
  minu=inLong-deg*100;
  out=deg+minu/60;

  if(dir=='S'||dir=='W')
  {
    out=out*-1;
  }
  return out;
}

/*
 *******************************************************************
 * All RTC Code - Initialisation, setting time, displaying time etc.
 * *****************************************************************
*/
void RTC_INIT(void)
{
  //Initialise RTC
  setSyncProvider(getTeensy3Time);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*
 *******************************************************************
 * All Display Code - Different Displays parts etc.
 * *****************************************************************
 *
*/

/*
 * This function will set the right Display to show both Motor Temps
 * With the left Motor Temp on the left side of the display, and the
 * right temp on the right side
 */
void disp_mot_temps(int tempR, int tempL)
{
  String Left = "";
  String Right = "";
  String dis = "";

  if(tempL<0)
  {
      Left += " NC ";
  }
  else if(tempL<10)
  {
      Left += "   ";
      Left += tempL;
  }
  else if((tempL<100))
  {
      Left += "  ";
      Left += tempL;
  }
  else if((tempL<200))
  {
      Left += " ";
      Left += tempL;
  }
  else
  {
      Left += " ERR";
  }

  if(tempR<0)
  {
      Right += " NC ";
  }
  else if(tempR<10)
  {
      Right += "   ";
      Right += tempR;
  }
  else if((tempR<100))
  {
      Right += "  ";
      Right += tempR;
  }
  else if((tempR<200))
  {
      Right += " ";
      Right += tempR;
  }
  else
  {
    Right += " ERR";
  }

  dis += Left;
  dis += Right;

  module1.setDisplayToString(dis);

}

void disp_voltages(float min_cell_vol, int pack_vol)
{
  String Left = "";
  String Right = "";
  String dis = "";
  int min_cell_vol_int;
  min_cell_vol_int=(int)(min_cell_vol*100);
  if(min_cell_vol_int<1)
  {
      Left += " NC ";
  }
  else if(min_cell_vol_int<10)
  {
      Left += "  ";
      Left += min_cell_vol;
  }
  else if((min_cell_vol_int<100))
  {
      Left += " ";
      Left += min_cell_vol;
  }
  else if((min_cell_vol_int<1000))
  {
      Left += "";
      Left += min_cell_vol;
  }
  else
  {
      Left += " ERR";
  }
  if(pack_vol<2)
  {
      Right += " NC ";
  }
  else if(pack_vol<10)
  {
      Right += "   ";
      Right += pack_vol;
  }
  else if((pack_vol<100))
  {
      Right += "  ";
      Right += pack_vol;
  }
  else if((pack_vol<1000))
  {
      Right += " ";
      Right += pack_vol;
  }
  else
  {
    Right += " ERR";
  }

  dis += Left;
  dis += Right;

  module2.setDisplayToString(dis);

}

/*
 * This function will set the LED's on the dash displays to show the
 * Current battery % as a function of
 */
void disp_batt_LEDs(int volts)
{
  float batt_perc=((float(volts)-min_cell*42)/(max_cell*42-min_cell*42))*100;
  if(batt_perc<(0*(100/8)))
  {
    module1.setLED(TM1638_COLOR_NONE,0);
    module1.setLED(TM1638_COLOR_NONE,1);
    module1.setLED(TM1638_COLOR_NONE,2);
    module1.setLED(TM1638_COLOR_NONE,3);
    module1.setLED(TM1638_COLOR_NONE,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);

  }
  else if(batt_perc<(1*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_NONE,1);
    module1.setLED(TM1638_COLOR_NONE,2);
    module1.setLED(TM1638_COLOR_NONE,3);
    module1.setLED(TM1638_COLOR_NONE,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(2*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_NONE,2);
    module1.setLED(TM1638_COLOR_NONE,3);
    module1.setLED(TM1638_COLOR_NONE,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(3*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_NONE,3);
    module1.setLED(TM1638_COLOR_NONE,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(4*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_RED,3);
    module1.setLED(TM1638_COLOR_NONE,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(5*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_RED,3);
    module1.setLED(TM1638_COLOR_RED,4);
    module1.setLED(TM1638_COLOR_NONE,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(6*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_RED,3);
    module1.setLED(TM1638_COLOR_RED,4);
    module1.setLED(TM1638_COLOR_RED,5);
    module1.setLED(TM1638_COLOR_NONE,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else if(batt_perc<(7*(100/8)))
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_RED,3);
    module1.setLED(TM1638_COLOR_RED,4);
    module1.setLED(TM1638_COLOR_RED,5);
    module1.setLED(TM1638_COLOR_RED,6);
    module1.setLED(TM1638_COLOR_NONE,7);
  }
  else
  {
    module1.setLED(TM1638_COLOR_RED,0);
    module1.setLED(TM1638_COLOR_RED,1);
    module1.setLED(TM1638_COLOR_RED,2);
    module1.setLED(TM1638_COLOR_RED,3);
    module1.setLED(TM1638_COLOR_RED,4);
    module1.setLED(TM1638_COLOR_RED,5);
    module1.setLED(TM1638_COLOR_RED,6);
    module1.setLED(TM1638_COLOR_RED,7);
  }
}

/*
 * This function will set the LED's on the dash displays to show various params.
 * Module 1: Batt percentage All lights = full; down to no lights = empty
 * Module 2: HF Checks: PDOC=1/2; BMS=3/4; BSPD=5/6; IMD=7/8
 */
void HF_LEDs(int PDOC,int BMS, int BSPD, int IMD,int LOW_PACK, int LOW_CELL, int LATCHED_THR, int GPS_SAT)
{
  //Latched Until Powered off again
  if(PDOC==0)
  {
    module2.setLED(TM1638_COLOR_RED,0);
  }
  if(BMS==0)
  {
    module2.setLED(TM1638_COLOR_RED,1);
  }
  if(BSPD==0)
  {
    module2.setLED(TM1638_COLOR_RED,2);
  }
  if(IMD==0)
  {
    module2.setLED(TM1638_COLOR_RED,3);
  }
  if(LOW_PACK==1)
  {
    module2.setLED(TM1638_COLOR_RED,4);
  }
  if(LOW_CELL==1)
  {
    module2.setLED(TM1638_COLOR_RED,5);
  }

  //Resettable Pins
  if(LATCHED_THR==0)
  {
    module2.setLED(TM1638_COLOR_RED,6);
  }
  else
  {
    module2.setLED(TM1638_COLOR_NONE,6);
  }
  if(GPS_SAT<5)
  {
    module2.setLED(TM1638_COLOR_RED,7);
  }
  else
  {
    module2.setLED(TM1638_COLOR_NONE,7);
  }
}

/*
***********************************************************
*               SD CARD SAVING FUNCTION                   *
*(All params are localised using the _C (for card)        *
*            prefix, gross I know -> sorry)               *
***********************************************************
 */
void print_SD(int rpmR_C, int currentR_C, int voltageR_C, int throttleR_C, int tempMotR_C, int tempDrvR_C, int rpmL_C, int currentL_C, int voltageL_C, int throttleL_C, int tempMotL_C, int CTMP_L_C, bool PDOC_C, bool BMS_C, bool BSPD_C, bool IMD_C, float min_cell_volt_C, float max_cell_volt_C, int min_cell_volt_IDX_C, int min_cell_temp_C, int max_cell_temp_C, int max_cell_temp_IDX_C, float shunt_volt_C, long shunt_cur_C, bool  low_cell_err_C, bool  high_cell_err_C, bool  low_cell_temp_err_C, bool high_cell_temp_err_C, bool low_shunt_volt_err_C, bool high_shunt_volt_err_C,float Lat_C,float Longi_C,float GPS_Vel_C,int hour_C,int minute_C,int seconds_C,int milliseconds_C,int GPS_day_C,int GPS_month_C,int GPS_year_C,float GPS_angle_C,float input_volt_C,float input_throttle_C,bool APS_OOB__SIG_C,bool APS_DIS_SIG_C,bool LATCH_THR_C,bool NEG_AIR_SIG_C,bool COOLING_SIG_C,bool BRAKE_SIG_C,bool RTD_SIG_C,float running_distance_C)
{
  String log_data="";
  log_data+=rpmR_C;
  log_data+=",";
  log_data+=currentR_C;
  log_data+=",";
  log_data+=voltageR_C;
  log_data+=",";
  log_data+=throttleR_C;
  log_data+=",";
  log_data+=tempMotR_C;
  log_data+=",";
  log_data+=tempDrvR_C;
  log_data+=",";
  log_data+=rpmL_C;
  log_data+=",";
  log_data+=currentL_C;
  log_data+=",";
  log_data+=voltageL_C;
  log_data+=",";
  log_data+=throttleL_C;
  log_data+=",";
  log_data+=tempMotL_C;
  log_data+=",";
  log_data+=CTMP_L_C;
  log_data+=",";
  log_data+=PDOC_C;
  log_data+=",";
  log_data+=BMS_C;
  log_data+=",";
  log_data+=BSPD_C;
  log_data+=",";
  log_data+=IMD_C;
  log_data+=",";
  log_data+=hour();
  log_data+=",";
  log_data+=minute();
  log_data+=",";
  log_data+=second();
  log_data+=",";
  log_data+=min_cell_volt_C;
  log_data+=",";
  log_data+=max_cell_volt_C;
  log_data+=",";
  log_data+=min_cell_volt_IDX_C;
  log_data+=",";
  log_data+=min_cell_temp_C;
  log_data+=",";
  log_data+=max_cell_temp_C;
  log_data+=",";
  log_data+=max_cell_temp_IDX_C;
  log_data+=",";
  log_data+=shunt_volt_C;
  log_data+=",";
  log_data+=shunt_cur_C;
  log_data+=",";
  log_data+=low_cell_err_C;
  log_data+=",";
  log_data+=high_cell_err_C;
  log_data+=",";
  log_data+=low_cell_temp_err_C;
  log_data+=",";
  log_data+=high_cell_temp_err_C;
  log_data+=",";
  log_data+=low_shunt_volt_err_C;
  log_data+=",";
  log_data+=high_shunt_volt_err_C;
  log_data+=",";
  log_data+=String(Lat_C,6);
  log_data+=",";
  log_data+=String(Longi_C,6);
  log_data+=",";
  log_data+=String(GPS_Vel_C,2);
  log_data+=",";
  if(hour_C<10)
  {
    log_data+="0";
    log_data+=hour_C;
    log_data+=":";
  }
  else
  {
    log_data+=hour_C;
    log_data+=":";
  }
  if(minute_C<10)
  {
    log_data+="0";
    log_data+=minute_C;
    log_data+=":";
  }
  else
  {
    log_data+=minute_C;
    log_data+=":";
  }
  if(seconds_C<10)
  {
    log_data+="0";
    log_data+=seconds_C;
    log_data+=":";
  }
  else
  {
    log_data+=seconds_C;
    log_data+=":";
  }
  if(milliseconds_C<10)
  {
    log_data+="00";
    log_data+=milliseconds_C;
    log_data+=",";
  }
  else if(milliseconds_C<100)
  {
    log_data+="0";
    log_data+=milliseconds_C;
    log_data+=",";
  }
  else
  {
    log_data+=milliseconds_C;
    log_data+=",";
  }
  if(GPS_year_C<10)
  {
    log_data+="0";
    log_data+=GPS_year_C;
  }
  else
  {
    log_data+=GPS_year_C;
  }
  if(GPS_month_C<10)
  {
    log_data+="0";
    log_data+=GPS_month_C;
  }
  else
  {
    log_data+=GPS_month_C;
  }
  if(GPS_day_C<10)
  {
    log_data+="0";
    log_data+=GPS_day_C;
  }
  else
  {
    log_data+=GPS_day_C;
  }
  log_data+=",";
  log_data+=GPS_angle_C;
  log_data+=",";
  log_data+=rpmR_C*(2*pi*rad*3.6/60)*2;    //Extra *2 multiplier is because for some reason the controllers output half RPM?
  log_data+=",";
  log_data+=rpmL_C*(2*pi*rad*3.6/60)*2;    //Extra *2 multiplier is because for some reason the controllers output half RPM?
  log_data+=",";
  log_data+=input_volt_C;
  log_data+=",";
  log_data+=input_throttle_C;
  log_data+=",";
  log_data+=APS_OOB__SIG_C;
  log_data+=",";
  log_data+=APS_DIS_SIG_C;
  log_data+=",";
  log_data+=LATCH_THR_C;
  log_data+=",";
  log_data+=NEG_AIR_SIG_C;
  log_data+=",";
  log_data+=COOLING_SIG_C;
  log_data+=",";
  log_data+=BRAKE_SIG_C;
  log_data+=",";
  log_data+=RTD_SIG_C;
  log_data+=",";
  log_data+=running_distance_C;
  log_data+=",";
  log_data+=currentR_C+currentL_C;    //Total Current Draw
  log_data+=",";
  log_data+=((currentR_C+currentL_C)*shunt_volt_C)/1000;  //Total Power Draw

  myFile = SD.open(buff, FILE_WRITE);  //MAX NEW FILE LENGTH IS 12 Characters -> can only have 8 characters then ".txt"
  if(myFile)
  {
    myFile.println(log_data);
    myFile.close();
  }
}


void GPIO_Init(void) {
  //Digital Input Reads
  pinMode(COOLING_SIG,INPUT);
  pinMode(BRAKE_SIG,INPUT);
  pinMode(RTD_SIG,INPUT);
  pinMode(GPS_FIX__SIG,INPUT);
  pinMode(IMD_PWM__SIG,INPUT);
  pinMode(POS_AIR_SIG,INPUT);
  pinMode(NEG_AIR_SIG,INPUT);
  pinMode(THR_SIG,INPUT);
  pinMode(LATCH_THR,INPUT);
  pinMode(APS_DIS_SIG,INPUT);
  pinMode(APS_OOB__SIG,INPUT);
  //Analog Input Reads
  pinMode(POWER_SIG,INPUT);
  pinMode(THR_SIG,INPUT);
  //Hard Fault Error inputs
  pinMode(PDOC_SIG,INPUT);
  pinMode(BSPD_SIG,INPUT);
  pinMode(IMD_SIG,INPUT);
  pinMode(BMS_SIG,INPUT);
  //Turning LED on whenever the Teensy has power
  pinMode(TEENSY_LED,OUTPUT);
  digitalWrite(TEENSY_LED,HIGH);
}
