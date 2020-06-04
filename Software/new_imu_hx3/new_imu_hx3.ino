/**
  **********************************************************************
  * File:    new_imu_hx3.ino
  * Author:   Gabriel Delgado
  * Created on: 26/05/2020
  * 
  *   Teensy 3.6
  *   LCD SSD1322 NHD 2.8"
  *   IMU - xsens 
  *   Bluetooth 4.0 (CM-10)
  *   SparkFun Battery Babysitter - LiPo Battery Manager
  *   Force Sensor K3D60a 500N
  *   3 HX711 24bits (x,y,z - sensor force)
  *   PushButton
  *   
  * 
  * G-NEC   -   CAR    -    CSIC 
  * 
  **********************************************************************
*/

//ADC 24 bits HX711
#include "HX711-multi.h"

//IMU XSENS
#include <XSens.h>
#include <Wire.h>
//quaternions
float q[4];



//LCD
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);

//SD


//BLUETOOTH


//PUSH BUTTON



//        RTC       // Agregar bateria de 3v CR2032 
#include <TimeLib.h>


//      BATTERY
#include <SparkFunBQ27441.h>
// Set BATTERY_CAPACITY
const unsigned int BATTERY_CAPACITY = 2000; //  2000mAh battery


//      SENSOR DE FUERZA       //
// Pins to the load cell amp
#define CLK A0      // clock pin to the load cell amp
#define DOUT1 A1    // data pin to the first lca
#define DOUT2 A2    // data pin to the second lca
#define DOUT3 A3    // data pin to the third lca

#define BOOT_MESSAGE "Force Sensor OK"

#define TARE_TIMEOUT_SECONDS 4

byte DOUTS[3] = {DOUT1, DOUT2, DOUT3};

#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)

long int results[CHANNEL_COUNT];

HX711MULTI scales(CHANNEL_COUNT, DOUTS, CLK);

//******************************************************************//


//*********************** IMU XSENS *****************************************//
XSens xsens(0x6B); //0x1d
//************************************************************************//





//----------------------------------------------------------------------------------------------//

void setupBQ27441(void)
{
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
  // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
  }
  Serial.println("Connected to BQ27441!");
  
  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);
}

//-------------------------------------------------------------------------------------------------//


void setup() {
  setSyncProvider(getTeensy3Time);

  Serial.begin(115200);
  Serial.println(BOOT_MESSAGE);
  Serial.flush();
  pinMode(11,OUTPUT);
  
//************XSens***********//
  Wire.begin();
  xsens.begin();
//***************************//


//*************BLUETOOTH ************// RX1:0  TX1:1
  Serial1.begin(9600);
//**********************************//



//***************** RTC ******************//


 //Serial.begin(115200);
 // while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
//**********************************************//


//*************** Force Sensor *****************//

  tare();


//*********************************************//

//******************* LCD ***************************//
 u8g2.begin();
 u8g2.clearBuffer();
 drawbb();
 //u8g2.drawXBMP( 0, 0, 246, 64, bbalance);
 u8g2.sendBuffer();

 delay(3000);
 
 u8g2.clearBuffer();
 draw_screen_ini();
 //u8g2.drawXBMP( 0, 0, 256, 64, screen_ini);
 u8g2.sendBuffer();
 delay(1000);
//****************************************************//

}


//--------------------------------------------------------------------------------------------------------------//

void tare() {
  bool tareSuccessful = false;

  unsigned long tareStartTime = millis();
  while (!tareSuccessful && millis()<(tareStartTime+TARE_TIMEOUT_SECONDS*1000)) {
    tareSuccessful = scales.tare(20,10000);  //reject 'tare' if still ringing
  }
}


//--------------------------------------------------------------------------------------------------------------//

void sendRawData() {
  scales.read(results);
  for (int i=0; i<scales.get_count(); ++i) {;
    Serial.print( -results[i]);  
    Serial.print( (i!=scales.get_count()-1)?"\t":"\n");
  }  
  //delay(10);
}


//---------------------------------------------------------------------------------------------------------------//

void loop() {
  
  sendRawData(); //this is for sending raw data, for where everything else is done in processing

  //on serial data (any data) re-tare
  if (Serial.available()>0) {
    while (Serial.available()) {
      Serial.read();
    }
    tare();
  }

  
//******************************** IMU ******************************************************************************//
  
  xsens.updateMeasures();


  //Serial.print("Outputting Orientation Data :  ");
  for(int i = 0 ; i < 4; ++i){
    //Serial.print(xsens.getQuat()[i]); //xsens.getQuat()[i]
    q[i]= (xsens.getQuat()[i]);
    //Serial.print(q[i]);
    //Serial.print(" ");
  }



 
  //Serial.println(" ");

  
 /* Serial.print("Outputting Acceleration Data :  ");
  for(int i = 0 ; i < 3; ++i){
    Serial.print(xsens.getAccel()[i]);
    Serial.print(" ");
  }
  //Serial.println(" ");

  
  Serial.print("Outputting Rate Of Turn Data :  ");
  for(int i = 0 ; i < 3; ++i){
    Serial.print(xsens.getRot()[i]);
    Serial.print(" ");
  }
  //Serial.println(" ");
  
  Serial.print("Outputting Yaw :  ");
  Serial.println(xsens.getHeadingYaw());

 // delay(10);

*/



  // Euler Angles.

  float yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  float pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  float roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
/*
  Serial.print("Yaw: ");
  Serial.println(yaw);
  Serial.print("Pitch: ");
  Serial.println(pitch);
  Serial.print("Roll: ");
  Serial.println(roll);*/
  
//*********************************************************************************************//


//**************************  BATTERY  ******************************************************************//
  unsigned int soc = lipo.soc();  // Read state-of-charge (%)
  /*Serial.print("LIPO:  ");
  Serial.println(soc);*/
//*****************************************************************************************//


//***********************  LCD  **********************************//

  u8g2.clearBuffer();   
  u8g2.setFont(u8g2_font_6x10_tn);
  u8g2.setCursor(45, 39);
  u8g2.print(roll);
  u8g2.setCursor(45, 50);
  u8g2.print(pitch);
  u8g2.setCursor(45, 61);
  u8g2.print(yaw);
  u8g2.updateDisplayArea(5,4,10,4);
 // u8g2.sendBuffer(); 

  //u8g2.clearBuffer();   
  u8g2.setFont(u8g2_font_6x10_tn);
  u8g2.setCursor(173, 39);
  u8g2.print(-results[2]);
  u8g2.setCursor(173, 50);
  u8g2.print(-results[1]);
  u8g2.setCursor(173, 61);
  u8g2.print(-results[0]);
 // u8g2.drawUTF8(1,12,"Gabriel Delgado");
  u8g2.updateDisplayArea(20,4,10,4);

  //delay(50);

 
//************************************************************/

  
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
unsigned long pctime = 0L;
const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
