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
  Adjusted ACC mg per LSB values based on the datasheet 
***************************************************************************/ 

#ifndef __LSM303_H__
#define __LSM303_H__

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x
#define LSM303_ID                     (0b11010100)

// Linear Acceleration: mg per LSB
#define LSM303_ACCEL_MG_LSB_2G (0.001F)
#define LSM303_ACCEL_MG_LSB_4G (0.002F)
#define LSM303_ACCEL_MG_LSB_8G (0.004F)
#define LSM303_ACCEL_MG_LSB_16G (0.012F) 

// Magnetic Field Strength: gauss range
#define LSM303_MAG_MGAUSS_2GAUSS      (0.08F)
#define LSM303_MAG_MGAUSS_4GAUSS      (0.16F)
#define LSM303_MAG_MGAUSS_8GAUSS      (0.32F)
#define LSM303_MAG_MGAUSS_12GAUSS     (0.48F)

// Gravity Standards
#define SENSORS_GRAVITY_EARTH (9.80665F) /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON (1.6F)      /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN (275.0F)     /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD (SENSORS_GRAVITY_EARTH)

// Magnetic Field Standards
#define SENSORS_MAGFIELD_EARTH_MAX (60.0F) /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN (30.0F) /**< Minimum magnetic field on Earth's surface */
#define SENSORS_GAUSS_TO_MICROTESLA (100)  /**< Gauss to micro-Tesla multiplier */

// Temperature: LSB per degree celsius
#define LSM303_TEMP_LSB_DEGREE_CELSIUS    (8)  // 1°C = 8, 25° = 200, etc.


class TW_LSM303v2
{
  public:
    typedef enum
    {                                                     // DEFAULT    TYPE
      LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
      LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
      LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
      LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
      LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
      LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
      LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
      LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
      LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
      LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
      LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
      LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
      LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
      LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
      LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
      LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
      LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
      LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
      LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
      LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
      LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
      LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
      LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
      LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
      LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
      LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
    } lsm303AccelRegisters_t;

	typedef enum
    {
      LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
      LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
      LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
      LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
      LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
      LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
      LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
      LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
      LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
      LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } lsm303MagRegisters_t;

  typedef enum
  { // Note High resolution is enabled for all (#### 1000)
    LSM303_ACCSCALE_2G                        = 0x08,  // +/- 2 g
    LSM303_ACCSCALE_4G                        = 0x18,  // +/- 4 g
    LSM303_ACCSCALE_8G                        = 0x28,  // +/- 8 g
    LSM303_ACCSCALE_16G                       = 0x38,  // +/- 16 g
  } lsm303AccScale;  

  typedef enum
  {
    LSM303_ACCODR_1HZ                         = 0x17,  //   1 Hz
    LSM303_ACCODR_10HZ                        = 0x27,  //  10 Hz
    LSM303_ACCODR_25HZ                        = 0x37,  //  25 Hz
    LSM303_ACCODR_50HZ                        = 0x47,  //  50 Hz
    LSM303_ACCODR_100HZ                       = 0x57,  // 100 Hz
    LSM303_ACCODR_200HZ                       = 0x67,  // 200 Hz
  } lsm303AccODR;

	typedef enum
	{
	  LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
	  LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
	  LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
	  LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
	  LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
	  LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
	  LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
	} lsm303MagGain;
  
  typedef enum
  {
    LSM303_MAGODR_1_5HZ                       = 0x84,  // 1.5 Hz
    LSM303_MAGODR_7_5HZ                       = 0x8C,  // 7.5 Hz
    LSM303_MAGODR_15HZ                        = 0x90,  //  15 Hz
    LSM303_MAGODR_30HZ                        = 0x94,  //  30 Hz
    LSM303_MAGODR_75HZ                        = 0x98,  //  75 Hz
  } lsm303MagODR;
/*=========================================================================
    MAGNETOMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
      LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
      LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
      LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
      LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
      LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
      LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
      LSM303_MAGRATE_220                        = 0x07   // 200 Hz
    } lsm303MagRate;
/*=========================================================================*/


    typedef struct lsm303AccelData_s
    { // Return integer raw values
      int16_t x; //float x;
      int16_t y; //float y;
      int16_t z; //float z;
    } lsm303AccelData;

    typedef struct lsm303AccelData_ms2_s
    { // Return float values in m/s^2
      float x;
      float y;
      float z;
    } lsm303AccelData_ms2;
	
	typedef struct lsm303MagData_s
	{ // Return integer raw values
      int16_t x; //float x;
      int16_t y; //float y;
      int16_t z; //float z;
	} lsm303MagData;

  typedef struct lsm303MagData_mT_s
  { // Return float values in micro Teslas
      float x;
      float y;
      float z;
  } lsm303MagData_mT;

  bool begin(void);
  void readAccel(void);
  void readAccel_ms2(void);
  void readMag(void);
  void readMag_mT(void);
  void readTemp(void);
  bool setAccScale(lsm303AccScale scale);
	bool setMagGain(lsm303MagGain gain);
  bool setAccODR(lsm303AccODR accODR);
  bool setMagODR(lsm303MagODR magODR);

  lsm303AccelData accelData;          // Last read accelerometer data will be available here (Raw)
  lsm303AccelData_ms2 accelData_ms2;  // Last read accelerometer data will be available here (ms^2) 
  lsm303MagData magData;              // Last read magnetometer data will be available here (Raw)
  lsm303MagData magData_mT;           // Last read magnetometer data will be available here (microTeslas)
  int16_t temperature;                // Last read temperature data will be available here (Raw)
  lsm303MagGain magGain;
  
  void write8(byte address, byte reg, byte value);
  byte read8(byte address, byte reg);

  private:
};

#endif
