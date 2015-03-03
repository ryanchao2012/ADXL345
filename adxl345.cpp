/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
    
    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <SPI.h>
#include <Wire.h>

#include "adxl345.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline uint8_t adxl345::i2cread(void) 
{
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
inline void adxl345::i2cwrite(uint8_t x) 
{
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}


/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void adxl345::writeRegister(uint8_t reg, uint8_t value) 
{
  if (_i2c) 
  {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite((uint8_t)reg);
    i2cwrite((uint8_t)(value));
    Wire.endTransmission();
  } 
  else 
  {
    digitalWrite(_cs, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(_cs, HIGH);
  }
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8_t adxl345::readRegister(uint8_t reg) 
{
  if (_i2c) 
  {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    return (i2cread());
  } 
  else 
  {
    reg |= 0x80; // read byte
    digitalWrite(_cs, LOW);
    SPI.transfer(reg);
    uint8_t reply = SPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    return reply;
  }  
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16_t adxl345::read16(uint8_t reg) 
{
  if (_i2c) 
  {
    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite(reg);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 2);
    return (uint16_t)(i2cread() | (i2cread() << 8));  
  } 
  else 
  {
    reg |= 0x80 | 0x40; // read byte | multibyte
    digitalWrite(_cs, LOW);
    SPI.transfer(reg);
    uint16_t reply = SPI.transfer(0x00)  | (SPI.transfer(0x00) << 8);
    digitalWrite(_cs, HIGH);
    return reply;
  }    
}

/**************************************************************************/
/*! 
    @brief  Read the device ID (can be used to check connection)
*/
/**************************************************************************/
uint8_t adxl345::getDeviceID(void) 
{
  // Check device ID register
  return readRegister(ADXL345_REG_DEVID);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent X axis value
*/
/**************************************************************************/
int16_t adxl345::getX(void) 
{
  return read16(ADXL345_REG_DATAX0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Y axis value
*/
/**************************************************************************/
int16_t adxl345::getY(void) 
{
  return read16(ADXL345_REG_DATAY0);
}

/**************************************************************************/
/*! 
    @brief  Gets the most recent Z axis value
*/
/**************************************************************************/
int16_t adxl345::getZ(void) 
{
  return read16(ADXL345_REG_DATAZ0);
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class with default setting
*/
/**************************************************************************/
adxl345::adxl345(commMode_t mode) 
{
  _i2c = (mode == _I2C) ? true : false;

  if(!_i2c)
  { 
    _spi_mode = SPI_MODE3;
    _cs = 10;
  }

  _range = ADXL345_RANGE_2_G;

  _dataRate = ADXL345_DATARATE_3200_HZ;

}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL345 class with custom setting
*/
/**************************************************************************/


/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
bool adxl345::begin() 
{ 
  if (_i2c)
    Wire.begin();
  else 
  {
    SPI.begin();
    SPI.setDataMode(_spi_mode);
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
  }

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5)
  {
    /* No ADXL345 detected ... return false */
    return false;
  }
  
  // Configure register settings in ADXL345
  setDataRate(_dataRate);
  setRange(_range);
  setOffset();

  // Enable measurements
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);  
    
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
void adxl345::setRange(range_t range)
{
  writeRegister(ADXL345_REG_POWER_CTL, 0x00);
  uint8_t format = 0x00;

  /* Update the data rate */
  format |= range;
  
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  
  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_DATA_FORMAT, format);
  
  /* Keep track of the current range (to avoid readbacks) */
  _range = range;
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);
}

/**************************************************************************/
/*!
    @brief  Gets the g range for the accelerometer
*/
/**************************************************************************/
range_t adxl345::getRange(void)
{
  /* Read the data format register to preserve bits */
  return (range_t)(readRegister(ADXL345_REG_DATA_FORMAT) & 0x03);
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
void adxl345::setDataRate(dataRate_t dataRate)
{
  writeRegister(ADXL345_REG_POWER_CTL, 0x00);
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  uint8_t format = 0x00;

  /* Update the data rate */
  format |= dataRate;

  /* Write the register back to the IC */
  writeRegister(ADXL345_REG_BW_RATE, format);

  /* Keep track of the current dataRate (to avoid readbacks) */
  _dataRate = dataRate;
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);
}

/**************************************************************************/
/*!
    @brief  Gets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
dataRate_t adxl345::getDataRate(void)
{
  return (dataRate_t)(readRegister(ADXL345_REG_BW_RATE) & 0x0F);
}

/**************************************************************************/
/*!
    @brief  Sets the offset adjustments in the ADXL345
*/
/**************************************************************************/
void adxl345::setOffset(void)
{
  writeRegister(ADXL345_REG_POWER_CTL, 0x00);
  writeRegister(ADXL345_REG_OFSX, _ofsx);
  writeRegister(ADXL345_REG_OFSY, _ofsy);
  writeRegister(ADXL345_REG_OFSZ, _ofsz);
  writeRegister(ADXL345_REG_POWER_CTL, 0x08);
}

/**************************************************************************/
/*!
    @brief  Sets the CS pin(serial port enable line)
*/
/**************************************************************************/
void adxl345::setCS(uint8_t pinCS)
{
  this -> _cs = pinCS;
}
