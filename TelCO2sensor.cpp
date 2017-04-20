/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "TelCO2sensor.h"
#include <Wire.h>

TelClaireCO2::TelClaireCO2(int deviceAddress)
{
  
  _deviceAddress = deviceAddress;
}

void TelClaireCO2::calibration()
{
  Wire.beginTransmission(_deviceAddress);
  Wire.write(0x05); Wire.write(0x03); Wire.write(0xEC); Wire.write(0xFF); Wire.write(0x00);
  // end transmission
  Wire.endTransmission();
}

unsigned int TelClaireCO2::status()
{
  char statusResponse[4] = {0,0,0,0} ;
  int status = 0;
  //for (char i=0;i<100;i++)
  //{
  Wire.beginTransmission(_deviceAddress);
  Wire.write(0x04); Wire.write(0x13); Wire.write(0x8A); Wire.write(0x00);  Wire.write(0x01);
  // end transmission
  Wire.endTransmission();
  delay(100);
  //Serial.println("calibration in progress ....");
  Wire.requestFrom(_deviceAddress, 4);    // request 4 bytes from slave device
  statusResponse[0]=Wire.read();  //function code
  statusResponse[1]=Wire.read();  // Byte count
  statusResponse[2]=Wire.read();  // status(MSB)
  statusResponse[3]=Wire.read();  // status(LSB)
  status = ((statusResponse[2] & 0xFF) <<8 ) | (statusResponse[3] & 0xFF);  
  return status & 0x0000FFFF;
 }

 int TelClaireCO2::readCO2()
{ 
  int data [4];
  int CO2ppmValue;
  
  // start I2C
  Wire.beginTransmission(_deviceAddress);
  Wire.write(0x04); Wire.write(0x13); Wire.write(0x8B); Wire.write(0x00); Wire.write(0x01);
  // end transmission
  Wire.endTransmission();
  // read report of current gas measurement in ppm
  delay(2000);
  Wire.requestFrom(_deviceAddress, 4);    // request 4 bytes from slave device
  data[0] = Wire.read(); //MSB
  data[1] = Wire.read();
  data[2] = Wire.read();
  data[3] = Wire.read(); //LSB
    Serial.print("Func code: "); Serial.print(data[0],HEX);
    Serial.print(" byte count: "); Serial.println(data[1],HEX);
    Serial.print("MSB: 0x");  Serial.print(data[2],HEX); Serial.print("  ");
    Serial.print("LSB: 0x");  Serial.print(data[3],HEX); Serial.print("  ");
   CO2ppmValue = (((data[2] & 0x3F ) *256) + data[3]);
   return CO2ppmValue;
}


