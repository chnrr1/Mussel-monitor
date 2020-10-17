

/*30/08/2020 Hao Shen
 * This project is to calculate the heart rate and valve movement of mussel Mytilus galloprovoncialis.
 * Manual setting of RTC not working.
 * I/O pins can be power or GND for IR. Time sharing is a must.

*/
#include "arduinoFFT.h"
#include <Wire.h>
#include <DS1307.h>
#include <SPI.h>
#include <SD.h>

arduinoFFT FFT = arduinoFFT(); // Create FFT object

const uint8_t samples = 128; // For FFT. MUST ALWAYS be a power of 2
const double samplingFrequency = 4; // Hz, must be less than 10000 due to ADC.
// heartbeat varies between 5 per min and 40 per min. so 128/4 = 32s is ok.

unsigned long sampling_period_us = round(1000000*(1/samplingFrequency));//data type must be big enough
unsigned long microseconds;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

 char analog[16] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};//8 for IR, 8 for Hall
const uint8_t power[8] = {22,26,30,34,38,42,46,2};//power for IR and Hall
const uint8_t GND[24] = {23,24,25,27,28,29,31,32,33,35,36,37,39,40,41,43,44,45,47,48,49,3,4,5};//ground for sensors. The order does not matter.

int rtc[7];
byte rr[7];

const int CS = 53;//CS pin for SD
const uint8_t RTCpower = 12; //recommend battery
const uint8_t SDbegin = 13; //showing SD working
//------------------------------------------------setup------------------------------------------------
void setup(){
//------------------------------------------------snesors----------------------------------------------  
  for (uint8_t i = 0; i<16; i++){
    pinMode(analog[i], INPUT);
  } 
  for (uint8_t i = 0; i<8; i++){//all power and GND start low. GND will always be low.
    pinMode(power[i],OUTPUT);
    digitalWrite(power[i],LOW);
  }
  for (uint8_t i = 0; i<24; i++){
    pinMode(GND[i],OUTPUT);
    digitalWrite(GND[i],LOW);
  }
 
//-----------------------------------------------RTC-------------------------------------------------   
  pinMode(RTCpower, OUTPUT);
  digitalWrite(RTCpower,HIGH);
  RTC.stop();
  RTC.set(DS1307_SEC,0); // set time
  RTC.set(DS1307_MIN,0);
  RTC.set(DS1307_HR,0);
  RTC.set(DS1307_DOW,3);
  RTC.set(DS1307_DATE,16);
  RTC.set(DS1307_MTH,9);
  RTC.set(DS1307_YR,20); 
  RTC.start();
  RTC.SetOutput(DS1307_SQW8KHZ); 
//----------------------------------------------SD--------------------------------------------------
  pinMode (SDbegin, OUTPUT);
  digitalWrite(SDbegin, LOW);
  if (!SD.begin(CS)) {
    
    while (1);
  }
  //Serial.println("card initialized.");
  digitalWrite(SDbegin, HIGH);
  delay(500);
  digitalWrite(SDbegin, LOW);
  delay(500);
}

//-------------------------------------------loop--------------------------------------------

void loop(){
  
  String dataString = "";// make a string for assembling the data to log
  
//----------------------------------------IR function----------------------------------------  
 
  for (uint8_t i = 0; i<8; i++){ // IR A0-A7
    digitalWrite(power[i], HIGH);
    delay(10);
    int y = IR(i);
    //Serial.print(y);
    //Serial.print(", ");
    //digitalWrite(power[i],LOW);//this is a must, or the board is damaged.
    dataString += (String(y)+",");
  }
  delay(10);
//---------------------------------------Hall function--------------------------------------

  for (uint8_t i = 0; i<8; i++){ // Hall A8-A15
    digitalWrite(power[i],HIGH);
    delay(10);
    
    //oversampling method
    uint32_t extraBits = 0;
    for(int j = 0; j < 255; j++){
    extraBits = extraBits + analogRead(analog[i+8]);
    }
    uint32_t reading =(extraBits >> 4);

    //Serial.print(reading);
    //Serial.print(", ");
    //digitalWrite(power[i],LOW);
    dataString += (String(reading)+",");
  }
  delay(10);
//---------------------------------------RTC function--------------------------------------
  RTC.get(rtc,true);
  for(int i=0; i<7; i++){
    //Serial.print(rtc[i]);
    //Serial.print(", ");
    dataString += (String(rtc[i])+",");
  }

  //if (Serial.available() > 6) {
    //setTime();//It's not working correctly and I don't know why.
  //} 
//---------------------------------------SD function---------------------------------------  

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.  
  char fileName[13];
  int year = rtc[6];
  int month = rtc[5];
  int day = rtc[4];
  sprintf(fileName, "%4d%02d%02d.csv",year, month,day);//Dynamic file name
  File dataFile = SD.open(fileName, FILE_WRITE);

  if (dataFile){ // if the file is available, write to it:
    dataFile.println(dataString);
    dataFile.close();   
    //Serial.print(dataString);// print to the serial port too:
    digitalWrite(SDbegin, HIGH);
    delay(500);
    digitalWrite(SDbegin, LOW);
    delay(10);
  }else {
    //Serial.print("error opening datalog.txt");
  }
  //Serial.println();
  delay(50); 
}

//--------------------The functions of IR and RTC------------------------


uint16_t IR (uint8_t Order){
  
  double vReal[samples];
  double vImag[samples];
    
  microseconds = micros();  
  for(int i = 0; i < samples; i++){    
    while (micros() - microseconds < sampling_period_us);// wait for the right time to take the sample
    microseconds += sampling_period_us;  // update as per the period   
   
    //oversampling method. A sensor already provides noise so no extra noise is needed.
    //the range of data is between 0 and 16384. The signal resolution is theoratically x16 higher.
    uint32_t extraBits = 0;
    for(int j = 0; j < 255; j++){
    extraBits = extraBits + analogRead(analog[Order]);
    }
    uint32_t reading =(extraBits >> 4);
    vReal[i] = reading;
    vImag[i] = 0;       
  }
  
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);    // Weigh data
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // Compute FFT
  FFT.ComplexToMagnitude(vReal, vImag, samples); // Compute magnitudes 
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency); // Compute largest peak
  int y = round((60/(1/x))); // result; 
   
  return(y);
}

char BCD2DEC(char var){/* RTC set time */
  if (var>9){
     var=(var>>4)*10+(var&0x0f);
  }
  return var;
}

/*void setTime(){
  for(int i=0;i<7;i++){
    rr[i]=BCD2DEC(Serial.read());
  }
  Serial.print("SET TIME:");
  RTC.stop();
  RTC.set(DS1307_SEC,rr[6]);
  RTC.set(DS1307_MIN,rr[5]);
  RTC.set(DS1307_HR,rr[4]);
  RTC.set(DS1307_DOW,rr[3]);
  RTC.set(DS1307_DATE,rr[2]);
  RTC.set(DS1307_MTH,rr[1]);
  RTC.set(DS1307_YR,rr[0]);
  RTC.start();
}*/
