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

/***************************************************************************
  Tapered Wings Logger Updates to Adafruit Library
  Version 2 (4/8/2020)
  James Fahlbusch
  Stanford University

  Notes:
  Added functionality to change sensor settings (ODR, scale and gain)
  Added meters per second^2 calculation
***************************************************************************/ 
#include <TW_LSM303v2.h>


static float _lsm303Accel_MG_LSB     = 0.004F;   // 1, 2, 4 or 12 mg per lsb
static float _lsm303Mag_Gauss_LSB_XY = 450.0F;  // Varies with gain
static float _lsm303Mag_Gauss_LSB_Z  = 400.0F;   // Varies with gain


/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
bool TW_LSM303v2::begin()
{
  Wire.begin();
/*
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
  // Set Magnetometer Gain to +- 4  gauss Fullscale Table 75 in datasheet
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, 0x80); // (1000 0000)
  // Enable the magnetometer continuous data update
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00); 

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
// This function reads the raw ACC data, which is stored as an integer
void TW_LSM303v2::readAccel() {
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
  // Note: bit shifting to the right has the effect of dividing by 16
  // A mg/LSB correction must also be made (depending on the scale):
  // +-2g: Raw * .001, +-4g: Raw * .002, +-8g: Raw * .004, +-16g: Raw * .012

  //accelData.x = (int16_t)((xhi << 8) | xlo);
  //accelData.y = (int16_t)((yhi << 8) | ylo);
  //accelData.z = (int16_t)((zhi << 8) | zlo); 
  accelData.x = (int16_t)(xlo | (xhi << 8)) >> 4;
  accelData.y = (int16_t)(ylo | (yhi << 8)) >> 4;
  accelData.z = (int16_t)(zlo | (zhi << 8)) >> 4;
}

void TW_LSM303v2::readAccel_ms2() {
  // Read the accelerometer (raw Values)
  //readAccel();

  accelData_ms2.x = (float)accelData.x * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  accelData_ms2.y = (float)accelData.y * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  accelData_ms2.z = (float)accelData.z * _lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
}

void TW_LSM303v2::readMag() {
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

void TW_LSM303v2::readMag_mT() {
  // Read the magnetometer (raw Values)
  //readMag();
  magData_mT.x = (float)magData.x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  magData_mT.y = (float)magData.y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  magData_mT.z = (float)magData.z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
}




void TW_LSM303v2::readTemp() {

  uint8_t valueL;
  uint16_t valueH;

  valueL = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_TEMP_OUT_L_M);
  valueH = read8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_TEMP_OUT_H_M);
  temperature = (((int16_t)( valueH << 8 | valueL )) >> 4 );
}

bool TW_LSM303v2::setAccScale(lsm303AccScale scale)
{
 // Set ACC scale, continuous update, High Resolution Enabled
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG4_A, (byte)scale); 
  return true;
}
bool TW_LSM303v2::setMagGain(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);
  magGain = gain;

  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      _lsm303Mag_Gauss_LSB_XY = 1100;
      _lsm303Mag_Gauss_LSB_Z  = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      _lsm303Mag_Gauss_LSB_XY = 855;
      _lsm303Mag_Gauss_LSB_Z  = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      _lsm303Mag_Gauss_LSB_XY = 670;
      _lsm303Mag_Gauss_LSB_Z  = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      _lsm303Mag_Gauss_LSB_XY = 450;
      _lsm303Mag_Gauss_LSB_Z  = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      _lsm303Mag_Gauss_LSB_XY = 400;
      _lsm303Mag_Gauss_LSB_Z  = 355;
      break;
    case LSM303_MAGGAIN_5_6:
      _lsm303Mag_Gauss_LSB_XY = 330;
      _lsm303Mag_Gauss_LSB_Z  = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      _lsm303Mag_Gauss_LSB_XY = 230;
      _lsm303Mag_Gauss_LSB_Z  = 205;
      break;
  }
  return true;
}
bool TW_LSM303v2::setAccODR(lsm303AccODR accODR)
{
  write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, (byte)accODR);
  return true;
}
bool TW_LSM303v2::setMagODR(lsm303MagODR magODR)
{
  // Enable Temp Sensor and set Mag ODR
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRA_REG_M, (byte)magODR);
  return true;
}


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void TW_LSM303v2::write8(byte address, byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte TW_LSM303v2::read8(byte address, byte reg)
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
