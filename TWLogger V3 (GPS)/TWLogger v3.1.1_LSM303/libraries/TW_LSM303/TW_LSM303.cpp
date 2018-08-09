/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <TW_LSM303.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool TW_LSM303::begin()
{
  Wire.begin();
  /*
(1E3)16 = (0001 1110 0011)2
(0A2B)16 = (0000 1010 0010 1011)2
(7E0C)16 = (0111 1110 0000 1100)2

Related converters: Binary To Hex Converter
Hexadecimal to Binary Conversion Chart
Hexadecimal Binary
0 0000
1 0001
2 0010
3 0011
4 0100
5 0101
6 0110
7 0111
8 1000
9 1001
A 1010
B 1011
C 1100
D 1101
E 1110
F 1111
*/
  // Enable the accelerometer
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57); // 100Hz ODR | Enable XYZ (0101 0111)
  // Set ACC scale to 8G, continuous update, High Resolution Enabled
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x28); // (0010 1000)

  // Enable Temp Sensor and set Mag ODR to 75Hz
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, 0x98); // (1001 1000)
  // Set Magnetometer Gain to +- 4  gauss Fullscale Tablt 75 in datasheet
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, 0x80); // (1000 0000)
  // Enable the magnetometer continuous data update
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00); 


  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void TW_LSM303::readAccel() {
  // Read the accelerometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_ACCEL);
  Wire.write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_ACCEL, (byte)6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  // KTOWN: 12-bit values are left-aligned, no shift needed
  // accelData.x = (xlo | (xhi << 8)) >> 4;
  // accelData.y = (ylo | (yhi << 8)) >> 4;
  // accelData.z = (zlo | (zhi << 8)) >> 4;
  accelData.x = (int16_t)((xhi << 8) | xlo);
  accelData.y = (int16_t)((yhi << 8) | ylo);
  accelData.z = (int16_t)((zhi << 8) | zlo);
}
void TW_LSM303::readMag() {
  // Read the magnetometer
  Wire.beginTransmission((byte)LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom((byte)LSM303_ADDRESS_MAG, (byte)6);
  
  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Note high before low (different than accel)  
  uint8_t xhi = Wire.read();
  uint8_t xlo = Wire.read();
  uint8_t zhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t ylo = Wire.read();
  
  
  // Shift values to create properly formed integer (low byte first)
  magData.x = (int16_t)(xhi << 8 | xlo); //(xlo | (xhi << 8));
  magData.y = (int16_t)(yhi << 8 | ylo); //(ylo | (yhi << 8));
  magData.z = (int16_t)(zhi << 8 | zlo); //(zlo | (zhi << 8));  

}
void TW_LSM303::readTemp() {

  uint8_t valueL;
  uint8_t valueH;

  valueL = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_TEMP_OUT_L_M);
  valueH = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_TEMP_OUT_H_M);
  temperature = (int16_t)(( valueH << 8 | valueL ) >> 4 ); // Should this be an unsingned int?
}

void TW_LSM303::setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void TW_LSM303::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte TW_LSM303::read8(byte address, byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}
