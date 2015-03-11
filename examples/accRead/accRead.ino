/* Wiring with GY-291: 

   UNO         GY-291
   13(SCK)     SCL
   12(MISO)    SDO
   11(MOSI)    SDA
   xx(CS)      CS
   
   MEGA        GY-291
   52(SCK)     SCL
   50(MISO)    SDO
   51(MOSI)    SDA
   xx(CS)      CS
*/

#include <Wire.h>
#include <SPI.h>
#include "ADXL345.h"
#define HEADER_H (85 & 0xff)
#define HEADER_L (170 & 0xff)


ADXL345 myAcc = ADXL345(_SPI);
int16_t ax, ay, az;
void setup()
{
  myAcc.setCS(49);
  Serial.begin(9600);
  if(!myAcc.begin())
  {
    Serial.println("ADXL345 begin failed!");
    while(1);
  }
  Serial.println("ADXL345 begin success!");
  
}

void loop()
{
  ax = myAcc.getX();
  ay = myAcc.getY();
  az = myAcc.getZ();
  
  Serial.write(HEADER_H);
  Serial.write(HEADER_L);
  Serial.write((ax >> 8) & 0xff);
  Serial.write(ax & 0xff);
  Serial.write((ay >> 8) & 0xff);
  Serial.write(ay & 0xff);
  Serial.write((az >> 8) & 0xff);
  Serial.write(az & 0xff);

//  Serial.print(ax);
//  Serial.print("\t\t");
//  Serial.print(ay);
//  Serial.print("\t\t");
//  Serial.println(az);
//  
//  delay(1);
}
