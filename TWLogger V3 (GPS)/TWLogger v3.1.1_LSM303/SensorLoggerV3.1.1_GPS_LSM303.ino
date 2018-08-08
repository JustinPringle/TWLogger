/*
 Author: James Fahlbusch
 
 Low Power Inertial Movement Datalogger for Feather M0 Adalogger 
 Version 3.1.1
 Samples Temp, Accel, Mag, and GPS
 Logs to CSV, flushing data after SamplesPerCycle samples
 Internal RTC used to timestamp sensor data
 Utilizes ReTick to Save Power (Sleeps during Idle time)
 Restarts logging if reset (time sync is lost)
 
 RED LED blinks once per minute for a low consumption pulse
 
 Connections (USES I2C and UART)
 Note: M0 to LSm303|UltimateGPS
 ===========
 Connect Pin 24 to SCL
 Connect Pin 22 to SDOG and SDOXM (MISO)
 Connect Pin 23 to SDA (MOSI)
 Connect Pin 10 to CSXM
 Connect Pin 09 to CSG
 Connect 3V to VIN
 Connect GND to GND

 Version Notes:
 Added Logfile Timestamps 1/15/17
 Improved Serial Output 1/18/17
 Added clock sync and Sensor Standby 1/21/17
 Changed Acc ODR to 50HZ 1/25/17 
 Implemented ReTick to sleep during unutilized CPU time 1/27/17
 Added Serial Menus and Updated Alarm and Timing 1/29/17
 Added reset timeout for tag restart 11/3/17
 Added GPS functionality 6/4/18 
 Added GPS control Options 6/29/18

 Power Consumption: 
 500mAh Battery - 27hours with Gyro, 39hours without Gyro
*/

// ToDo~!!
// Sensitivity testing of variables like timeout, number of samples to collect, MS smart delay
// Reduce the wait time for time check
// add 2 digit hour

#include <RTCZero.h> 
//#include <SPI.h>
#include <SdFat.h>
SdFat SD;
#include <Wire.h>
#include <Adafruit_Sensor.h>     //General sensor library for Adafruit
#include <TW_LSM303.h> //library for Accel Mag Sensor LSM 303
#include <TinyGPS.h>
//Libraries for ReTick
#include "Arduino.h"
#include "retick.h"


//////////////// Key Settings /////////////////////////////////
#define vers "Version 3.1.1"
//#define ECHO_TO_SERIAL // Allows serial output if uncommented
//#define GPSECHO  true // Echo the GPS data to the Serial console
//#define Gyro_On // Allows Gyro output if uncommented

// Number of samples to buffer before uSD card flush is called. 
#define SamplesPerCycle 2000
// Number of lines of data per file
// Should be divisible by SamplesPerCycle value
#define SamplesPerFile 500000
// Retick Cycle Rate to adjust sampling rate will inplement as user selectable
// 40ms = 25Hz
// 20ms = 50Hz
// 10ms = 100Hz
#define retickRate 20
#define Serial_Timeout  300000 // Wait time in menu before starting tag (5 min)
// GPS Settings
#define GPS_TIMEOUT         60 // Seconds to wait for a fix to be logged before sleeping
#define GPS_SAMPLING_RATE  120 // Seconds of GPS sleep between data acquisition
#define GPS_DELAY_READ       8 // Seconds to wait before trying to log - allows for a fix
#define LOOP_MAX_VAL         1 // Number of GPS reads before sleeping
#define SMART_DELAY_MS      10 // Number of milliseconds to spend encoding GPS data between other instructions

//////////////// GPS Seentences ///////////////////////////////
// GPS Setup Sentences
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 
///////////////////////////////////////////////////////////////


/////////////// Global Objects ////////////////////

#define GPSSerial Serial1
// Connect to the GPS on the hardware port
TinyGPS GPS;
bool gpsStandby = false;
unsigned int gpscount = 0;
bool endGPSRead = false;
bool logGPS = false; 
// Seconds of GPS sleep between data acquisition
unsigned short gpsRate = GPS_SAMPLING_RATE; // Default Value
 // Seconds to wait for a fix to be logged before sleeping
unsigned short gpsTimeOut = GPS_TIMEOUT; // Default Value
// Seconds to wait before trying to log - allows for a fix
unsigned short gpsDelay = GPS_DELAY_READ;   // Default Value
// Number of GPS reads before sleeping
byte loopMaxGPS = LOOP_MAX_VAL; // Default Value
// Number of milliseconds to spend encoding GPS data between other instructions
byte smartDelayMS = SMART_DELAY_MS;  // Default Value
unsigned int  loopCounter  = 0;        // Loop counter
unsigned int readCount = 0;
unsigned int hzGPSLog = 0;
char          inByte       = 0;        // incoming serial byte
String        reading      = "";       // String to hold current reading being created
//Intialize GPS variables
unsigned short sentences = 0, failed = 0;
long lat, lon, altitude;
unsigned long fixage, age, dateUTC, timeUTC, chars = 0;
unsigned long course, speed, sats, hdop;


TW_LSM303 lsm = TW_LSM303(); // Initialize the IMU

#define cardSelect 4  // Set the pin used for uSD
#define RED 13        // Red LED on Pin #13
#define GREEN 8       // Green LED on Pin #8
#define VBATPIN A7    // Battery Voltage on Pin A7
#define gpsEnable 12  // GPS Enable Pin

// re-defines USB serial from M0 chip so it appears as regular serial
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

//not sure if this is necessary since you can set the clock using a gps
char s_month[5];
int tmonth, tday, tyear, thour, tminute, tsecond; 
static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec"; 

/* Change these values to set the current initial time */
byte hours = 0;
byte minutes = 0;
byte seconds = 0;
/* Change these values to set the current initial date */
byte day = 1;
byte month = 1;
byte year = 17; 
uint32_t tmstmp = 0; // to store Milliseconds (from millis())    
//Used for a time delay, Sets arduino into standby for x minutes
//Could be a local variable instead of global
unsigned short delayStart = 0;
// Variable used to keep track of changes in seconds, minutes, hours, days, months
// Limits the number of calls to RTC
unsigned int timekeeper = 0;
// Variable to store the tag number
byte tag = 0;

RTCZero rtc;          // Create RTC object
File logfile;         // Create file object
File settingsFile;    // Create file object
char filename[15];    // Array for file name data logged to named in setup 
char settingsFilename[15];    // Array for file name of Settings File to log settings of Deployment
unsigned int CurrentCycleCount;  // Num of smaples in current cycle, before uSD flush call
unsigned int CurrentFileCount;   // Num of samples in current file
bool start = false;
char tzOffset[] = "+00";    // Array to store Timezone Offset, stored in info file

//////////////    Setup   ///////////////////
void setup() {  
  strcpy(filename, "T0D0101-00.CSV");   // Template for log file name, characters 8 & 9 get set automatically later
  strcpy(settingsFilename, "T0S0101-00.TXT");   // Template for settings file name, characters 8 & 9 will be used if other deployments are on the same card.
  //enable the GPS
  pinMode(gpsEnable, OUTPUT);
  digitalWrite(gpsEnable, HIGH);

  Serial.begin(115200);
  tmstmp = millis(); //to store a timeout value
  while (!Serial) { // Wait for Serial monitor to open
    if( millis() - tmstmp > Serial_Timeout ) {
       // timed out, restart tag using default values
       start = true;
       tmstmp = 0;
       break;  
    }
  }
  
  //This could be a part of the menu and could give the option to turn it on or leave it off
  GPSSerial.begin(9600); // For Tiny GPS
  GPSSerial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(500);
  GPSSerial.println(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(500);
  GPSSerial.println(PMTK_API_SET_FIX_CTL_1HZ);
  delay(500);

  GPSSerial.println(PMTK_SET_BAUD_57600);
  delay(500);
  GPSSerial.end();
  delay(500);
  GPSSerial.begin(57600);
  delay(500);
  GPSSerial.println(PGCMD_NOANTENNA); 
  delay(500);

  if(!start) {
    // This code sets the clock using serial time.
    // __DATE__ is a C++ preprocessor string with the current date in it.
    // It will look something like 'Mar  13  2016'.
    // So we need to pull those values out and convert the month string to a number.
    sscanf(__DATE__, "%s %d %d", s_month, &tday, &tyear);
    // Similarly, __TIME__ will look something like '09:34:17' so get those numbers.
    sscanf(__TIME__, "%d:%d:%d", &thour, &tminute, &tsecond);
    // Find the position of this month's string inside month_names, do a little
    // pointer subtraction arithmetic to get the offset, and divide the
    // result by 3 since the month names are 3 chars long.
    tmonth = (strstr(month_names, s_month) - month_names) / 3;
    month = tmonth + 1;  // The RTC library expects months to be 1 - 12.
    day = tday;
    year = tyear - 2000; // The RTC library expects years to be from 2000.
    hours = thour;
    minutes = tminute;
    seconds = tsecond;  
    // This is ~30 seconds off due to the time it takes to compile the
    // .ino file and upload the app
  }
  rtc.begin(); // initialize RTC
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);
  updateFileNames();
  
  while(!start){
    // Print the control menu:
    printMenu();
    // Then wait for any serial data to come in:
    while (!Serial.available());   
    // Once serial data is received, call parseMenu to act on it:
    parseMenu(Serial.read());   
  }

  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println(F("Found LSM303"));
  /* Display some basic information on this sensor */
  displaySensorDetails(); //add critical settings to be displayed as well
  /* Setup the sensor gain and integration time */
  configureSensor();
  delay(500);

    //Set Alarm
  if(delayStart > 0) {
    
    Serial.println("Logging will begin at:");  
    print2digits(rtc.getAlarmHours());
    Serial.print(":");
    print2digits(rtc.getAlarmMinutes());
    Serial.print(":");
    print2digits(rtc.getAlarmSeconds());
    Serial.print(" on ");
    // Print date...
    print2digits(rtc.getAlarmMonth());
    Serial.print("/");
    print2digits(rtc.getAlarmDay());
    Serial.print("/");
    print2digits(rtc.getAlarmYear());
    Serial.println();
    Serial.println("It is now safe to unplug the unit");
    Serial.end();
    delay(1000);
    rtc.standbyMode();
    rtc.disableAlarm(); 
    rtc.detachInterrupt();
    rtc.updateAlarms();
    delay(5000);
  }
  else {
    Serial.println("Logging will begin immediately"); 
    Serial.println("It is now safe to unplug the unit");
    Serial.end();
  }
  delay(500);
  
  #ifdef ECHO_TO_SERIAL
    while (! Serial); // Wait until Serial is ready
    Serial.begin(115200);
    Serial.println("\r\nTapered Wings Logger");
  #endif
  #ifdef GPSECHO
    while (! Serial); // Wait until Serial is ready
    Serial.begin(115200);
    Serial.println("\r\nTapered Wings Logger");
  #endif
  delay(500);
  CreateSettingsFile();  
  //start with the current time
  updateTime();
  CreateFile(); 
  reading      = ""; // reset reading
}  

/////////////////////   Loop    //////////////////////
void loop() {
  /*  Each run-through of tick will be 20ms (50Hz)
   *  If it takes less than 20ms, the CPU will sleep
   *  If it takes more, it immediately goes into next 
   *  tick once the previous finishes
   */
  retick(retickRate);
}

/////////////////////   ReTick    //////////////////////
void tick() {
  //GPS Sampling Rate
  if(gpscount >= gpsTimeOut && !gpsStandby){
    #ifdef GPSECHO
      Serial.println("sleeping");
      Serial.print(hours);
      Serial.print(":");
      if(minutes < 10)
        Serial.print('0'); // add leading zero for formatting
      Serial.print(minutes);
      Serial.print(":");
      if(seconds < 10)
        Serial.print('0'); // add leading zero for formatting
      Serial.print(seconds); Serial.println();
    #endif
    //Disable the GPS
    digitalWrite(12, LOW);
    gpsStandby = true;
    //gpsStandby = GPS.standby();
    gpscount = 0; // reset counter
  }
  else if (gpscount>=gpsRate && gpsStandby) {
    #ifdef GPSECHO
      Serial.println("waking up");
      Serial.print(hours);
      Serial.print(":");
      if(minutes < 10)
        Serial.print('0'); // add leading zero for formatting
      Serial.print(minutes);
      Serial.print(":");
      if(seconds < 10)
        Serial.print('0'); // add leading zero for formatting
      Serial.print(seconds); Serial.println();
    #endif
    //enable the GPS
    digitalWrite(12, HIGH);
    gpsStandby = false;
    //gpsStandby = GPS.wakeup();
    gpscount = 0; // reset counter
  }
  
  CurrentCycleCount += 1;       //  Increment samples in current uSD flush cycle
  CurrentFileCount += 1;        //  Increment samples in current file
  WriteToSD();                  // Output to uSD card stream, will not actually be written due to buffer/page size
  // Code to increment files limiting number of lines in each hence size, close the open file first.
  if( CurrentFileCount >= SamplesPerFile ) {
    if (logfile.isOpen()) {
      logfile.close();
    }
    CreateFile();
    //reset both counters
    CurrentFileCount = 0;
    CurrentCycleCount = 0;
    #ifdef ECHO_TO_SERIAL
      Serial.println("New log file created: ");
      Serial.println(filename); 
    #endif
  }
  //  Code to limit the number of power hungry writes to the uSD
  //  Don't sync too often - requires 2048 bytes of I/O to SD card. 512 bytes of I/O if using Fat16 library
  //  But this code forces it to sync at a fixd interval, i.e. once per hour etc depending on what is set
  if( CurrentCycleCount >= SamplesPerCycle ) { 
    logfile.flush(); // this takes about 16 milliseconds and hungrily uses power
    CurrentCycleCount = 0;
  }
}

///////////////   Functions   //////////////////

// Create new file on uSD incrementing file name as required
//Purpose of the file is to capture sensor settings, sampling rate, associated log files and battery status
void CreateSettingsFile(void)
{  
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    #ifdef ECHO_TO_SERIAL
      Serial.println("Card init. failed! or Card not present");
    #endif
    error(2);     // Two red flashes means no card or card init failed.
  }
  //increments the value of the ## in filename (Max is 99)
  for (uint8_t i = 0; i < 100; i++) {
    settingsFilename[8] = '0' + i/10;
    settingsFilename[9] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(settingsFilename)) {
      break;
    }
  }  

  settingsFile = SD.open(settingsFilename, FILE_WRITE);
  uint16_t C_year = year+2000;
  // set creation date time
  settingsFile.timestamp(T_CREATE, C_year, month, day, hours, minutes, seconds);
  settingsFile.timestamp(T_WRITE, C_year, month, day, hours, minutes, seconds);
  settingsFile.timestamp(T_ACCESS, C_year, month, day, hours, minutes, seconds);
   
  writeDeploymentDetails();
  if( ! settingsFile ) {
   #ifdef ECHO_TO_SERIAL    
    Serial.print("Couldnt create "); 
    Serial.println(settingsFilename); 
   #endif 
    error(3);
  }
  #ifdef ECHO_TO_SERIAL
    Serial.print("Writing to "); 
    Serial.println(settingsFilename);
  #endif
  settingsFile.flush(); //changed from close

}

// Create new file on uSD incrementing file name as required
void CreateFile()
{
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    #ifdef ECHO_TO_SERIAL
      Serial.println("Card init. failed! or Card not present");
    #endif
    error(2);     // Two red flashes means no card or card init failed.
  }
  //increments the value of the ## in filename (Max is 99)
  for (uint8_t i = 0; i < 100; i++) {
    filename[8] = '0' + i/10;
    filename[9] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }  
  //log the newly created file name to the settings file
  settingsFile = SD.open(settingsFilename, FILE_WRITE); 
  settingsFile.println(filename); 
  settingsFile.flush();
  //is this necessary
  delay(2); 
  logfile = SD.open(filename, FILE_WRITE);
  // set creation date time
  uint16_t C_year = year+2000;
  logfile.timestamp(T_CREATE, C_year, month, day, hours, minutes, seconds);
  logfile.timestamp(T_WRITE, C_year, month, day, hours, minutes, seconds);
  logfile.timestamp(T_ACCESS, C_year, month, day, hours, minutes, seconds);
  writeHeader();
  if( ! logfile ) {
   #ifdef ECHO_TO_SERIAL    
    Serial.print("Couldnt create "); 
    Serial.println(filename); 
   #endif 
    error(3);
  }
}

// Write data header to file of uSD.
void writeHeader() {
  #ifdef Gyro_On
  logfile.println("Date(MM/DD/YYYY),Time(hh:mm:ss),Timestamp(Ms),Temp(Raw),ACCELX,ACCELY,ACCELZ,MAGX,MAGY,MAGZ,GYRX,GYRY,GYRZ,Sats,HDOP,Latitude,Longitude,FixAge,DateUTC,TimeUTC,DateAge,Altitude,Course,Speed");
  #else
  logfile.println("Date(MM/DD/YYYY),Time(hh:mm:ss),Timestamp(Ms),Temp(Raw),ACCELX,ACCELY,ACCELZ,MAGX,MAGY,MAGZ,Sats,HDOP,Latitude,Longitude,FixAge,DateUTC,TimeUTC,DateAge,Altitude,Course,Speed");
  #endif
}

// Print data and time followed by battery voltage to SD card
void WriteToSD() {
  /* Read all the sensors. */
  /*   These are Raw values, Calculated values can be obtained by:
   *   event->acceleration.x = accelData.x * _accel_mg_lsb;
   *   event->acceleration.x /= 1000;
   *   event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
   */
  lsm.readAccel();
  /*   These are Raw values, Calculated values can be obtained by:
   *   event->magnetic.x = magData.x * _mag_mgauss_lsb;
   *   event->magnetic.x /= 1000; 
   */
  lsm.readMag();
 
  #ifdef Gyro_On  
  /*   These are Raw values, Calculated values can be obtained by:
   *   event->gyro.x = gyroData.x * _gyro_dps_digit;
   */ 
    lsm.readGyro();
  #endif
  //Get a time stamp for the data
  tmstmp = millis();    
  timekeeper = rtc.getSeconds();
  if(seconds != timekeeper) {
    gpscount += 1; //increment the count at 1Hz
    seconds = timekeeper;
    // NOTE: Temp values are raw and uncalibrated
    //temperature = 21.0 + (float)temperature/8; (21.0 is a guess)
    //this is sampling at same rate as magnetometerr (make sure it is continuous)
    lsm.readTemp(); //read the temp sensor at 1Hz
    if(seconds == 0){ 
      minutes = rtc.getMinutes();
      //pulse once per minute
      digitalWrite(RED, HIGH); 
      if(minutes == 0){
        hours = rtc.getHours();
        if(hours == 0){
          day = rtc.getDay();
          if(day == 1){
            month = rtc.getMonth();
            if(month == 1){
              year = rtc.getYear();
            }
          }
        } 
      }
      digitalWrite(RED, LOW);
    }
  }
  
  if(!gpsStandby && gpscount > gpsDelay){ // wait whole seconds before logging  
    reading = ""; // reset reading
    smartdelay(smartDelayMS); // Do - While Loop to read and encode GPS
    #ifdef GPSECHO 
        GPS.stats(&chars, &sentences, &failed);
        Serial.println(chars);
        Serial.println(sentences);
        Serial.println(failed);
    #endif
    GPS.get_position(&lat, &lon, &fixage);  // lat/long in MILLIONTHs of a degree and age of fix in milliseconds    
    sats = GPS.satellites(); //make sure valid satellites before logging
    
    // ensure that there are valid satellites, 1 hz logging and a recent position (within 10 Seconds)
    if(sats != 255 && hzGPSLog != gpscount && fixage < 10000){
        logGPS = true; // starts as true, if invalid is found, set to false
        log_int(sats, TinyGPS::GPS_INVALID_SATELLITES, 5);
        log_int(GPS.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
        log_int(lat, TinyGPS::GPS_INVALID_ANGLE, 10);
        log_int(lon, TinyGPS::GPS_INVALID_ANGLE, 11);
        log_int(fixage, TinyGPS::GPS_INVALID_AGE, 5);
        log_date(GPS);
        log_int(GPS.altitude(), TinyGPS::GPS_INVALID_ALTITUDE, 7);
        log_int(GPS.course(), TinyGPS::GPS_INVALID_ANGLE, 7);
        log_int(GPS.speed(), TinyGPS::GPS_INVALID_SPEED, 6);     
    }
      
  }
  //RTC Uses DD MM YYYY HH MM SS, so epoch results in extra math
 // Formatting for file output mm/dd/yyyy, hh:mm:ss 
  if(month < 10)
    logfile.print('0'); // add leading zero for formatting
  logfile.print(month);
  logfile.print("/");
  logfile.print(day);
   logfile.print("/20");
  logfile.print(year);
  logfile.print(",");
  if(hours < 10)
    logfile.print('0'); // add leading zero for formatting
  logfile.print(hours);
  logfile.print(":");
  if(minutes < 10)
    logfile.print('0'); // add leading zero for formatting
  logfile.print(minutes);
  logfile.print(":");
  if(seconds < 10)
    logfile.print('0'); // add leading zero for formatting
  logfile.print(seconds); logfile.print(",");
  // sensor timestamp (divide by 1000, remainder is the thousandths of a second)
  logfile.print(tmstmp); logfile.print(",");
  // print Temperature data to the log file:
  logfile.print(lsm.temperature); logfile.print(",");
 
  // print accelleration data to the log file: 
  logfile.print(lsm.accelData.x); logfile.print(",");
  logfile.print(lsm.accelData.y); logfile.print(",");
  logfile.print(lsm.accelData.z); logfile.print(","); 
  // print magnetometer data to the log file:
  logfile.print(lsm.magData.x); logfile.print(",");
  logfile.print(lsm.magData.y); logfile.print(",");
  #ifdef Gyro_On
    logfile.print(lsm.magData.z); logfile.print(",");
    // print gyroscopic data to the log file:  
    logfile.print(lsm.gyroData.x); logfile.print(",");
    logfile.print(lsm.gyroData.y); logfile.print(",");
    logfile.print(lsm.gyroData.z); 
  #else
    logfile.print(lsm.magData.z); 
  #endif
  // if there is a reading, append it
  if(logGPS){
    logfile.println(reading);
    #ifdef ECHO_TO_SERIAL
      Serial.println(reading);
    #endif
    #ifdef GPSECHO
      Serial.println(reading);
    #endif
    logGPS = false; //reset
    reading = ""; // reset reading
    readCount++;  // increment the count
    if(readCount == loopMaxGPS){  // Stop logging and put GPS to Sleep
      gpscount = gpsTimeOut;       // Put the GPS to sleep
      readCount = 0;                // Reset the count of GPS log events
      hzGPSLog = 0;                 // Reset the 1 hz counter
    } else {                        // Continue logging at 1 Hz
      hzGPSLog = gpscount;
    }
    
  }
  // if no reading, move on to next line and compensate commas for no GPS data
  else logfile.println(",,,,,,,,,,,");

  #ifdef ECHO_TO_SERIAL
    SerialOutput();    // Only logs to serial if ECHO_TO_SERIAL is uncommented at start of code
  #endif
}


// Debbugging output of time/date and battery voltage
void SerialOutput() {
  if(month < 10)
    Serial.print('0'); // add leading zero for formatting
  Serial.print(month);
  Serial.print("/");
  Serial.print(day);
   Serial.print("/20");
  Serial.print(year);
  Serial.print(" ");
  Serial.print(hours);
  Serial.print(":");
  if(minutes < 10)
    Serial.print('0'); // add leading zero for formatting
  Serial.print(minutes);
  Serial.print(":");
  if(seconds < 10)
    Serial.print('0'); // add leading zero for formatting
  Serial.print(seconds);
  // sensor timestamp (divide by 1000, remainder is the thousandths of a second)
  Serial.print("     "); 
  Serial.println(tmstmp%1000);
  Serial.print("TempRaw: "); Serial.print(lsm.temperature); Serial.print(" ");
  Serial.print("Temp Adjusted: "); Serial.print(21.0 + (float)lsm.temperature/8); Serial.println(" *C");
  Serial.print("AX: "); Serial.print(lsm.accelData.x); Serial.print(" ");
  Serial.print(" AY: "); Serial.print(lsm.accelData.y);       Serial.print(" ");
  Serial.print(" AZ: "); Serial.print(lsm.accelData.z);     Serial.println("  \tRAW Values");
  Serial.print("MX: "); Serial.print(lsm.magData.x); Serial.print(" ");
  Serial.print(" MY: "); Serial.print(lsm.magData.y);       Serial.print(" ");
  Serial.print(" MZ: "); Serial.print(lsm.magData.z);     Serial.println("  \tRAW Values");
  #ifdef Gyro_On
  Serial.print("GX: "); Serial.print(lsm.gyroData.x); Serial.print(" ");
  Serial.print(" GY: "); Serial.print(lsm.gyroData.y);       Serial.print(" ");
  Serial.print(" GZ: "); Serial.print(lsm.gyroData.z);     Serial.println("  \tRAW Values");
  #endif
}

//Probably not necessary since noone will see the error blinks
// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  //Display sensor Details such as sensitivity
  /*sensor_t accel, mag, gyro, temp;
  lsm.getSensor(&accel, &mag, &gyro, &temp);

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  #ifdef Gyro_On 
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  #endif
  */
}

/**************************************************************************/
/*
    Writes to a TXT file pertinent information about the settings of the 
    Deloyment.
*/
/**************************************************************************/
void writeDeploymentDetails(void)
{
  //Start with Date and Time
  settingsFile.println("Tapered Wings Logger");
  settingsFile.println(vers);
  settingsFile.println();
  settingsFile.print("Tag Number: ");
  settingsFile.println(tag);
  settingsFile.println();
  settingsFile.print("Initiation Timestamp: ");
  settingsFile.print(month);
  settingsFile.print("/");
  settingsFile.print(day);
  settingsFile.print("/");
  settingsFile.print("20");
  settingsFile.print(year);
  settingsFile.print(" ");
  settingsFile.print(hours);
  settingsFile.print(":");
  if(minutes < 10)
    settingsFile.print('0');      // add leading zero for formatting
  settingsFile.print(minutes);
  settingsFile.print(":");
  if(seconds < 10)
    settingsFile.print('0');      // add leading zero for formatting
  settingsFile.println(seconds);
  settingsFile.print("Timezone Offset: ");
  settingsFile.println(tzOffset);
  settingsFile.println(); 
  //Add sampling Rate
  settingsFile.println(F("----------Key Settings----------"));
  settingsFile.print("Number of Samples before Flush() is called: "); // at an if to display the proper sampling rates
  settingsFile.println(SamplesPerCycle);
  settingsFile.print("Number of Samples per CSV file: "); // at an if to display the proper sampling rates
  settingsFile.println(SamplesPerFile);
  settingsFile.println("Sampling Rate: 50Hz");
  settingsFile.println();
  settingsFile.println("GPS Settings:");
  settingsFile.print("GPS Sampling Rate (seconds): "); settingsFile.println(gpsRate);
  settingsFile.print("GPS Timeout (seconds): "); settingsFile.println(gpsTimeOut);
  settingsFile.print("GPS Read Delay (seconds): "); settingsFile.println(gpsDelay);
  settingsFile.print("GPS Samples per logging interval: "); settingsFile.println(loopMaxGPS);
  settingsFile.print("GPS Smart Delay (milliseconds): "); settingsFile.println(smartDelayMS);
  settingsFile.println();
  settingsFile.println();
  settingsFile.println(F("Associated Log Files:"));

}


/**************************************************************************/
/*
    Configures the gain and integration time for the LSM9DS0
*/
/**************************************************************************/
void configureSensor(void)
{
  /*
  // Change ODR accelerometer continous Now done in LSM9DS0.cpp
  //lsm.write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG1_XM, 0x57); // 50hz XYZ
  //lsm.write8(XMTYPE, LSM9DS0_REGISTER_CTRL_REG5_XM, 0b01110000); //turns off temp sensor

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope (Degrees per second)
  #ifdef Gyro_On 
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
  #endif
  */
}

void printMenu()
{
  Serial.println();
  Serial.println();         
  Serial.println(F("////////////////////////////////////////////"));
  Serial.println(F("//       Tapered Wings Logger             //"));
  Serial.println(F("////////////////////////////////////////////"));
  Serial.print("             ");
  Serial.println(vers);
  Serial.print("              ");
  print2digits(rtc.getMonth());
  Serial.print("/");
  print2digits(rtc.getDay());
  Serial.print("/20");
  print2digits(rtc.getYear());
  Serial.println();
  Serial.println();
  Serial.println(F("1) Enter Tag Number"));
  Serial.println(F("2) Set Time (GMT)"));
  Serial.println(F("3) Display Time"));
  Serial.println(F("4) Set Start Delay"));
  Serial.println(F("5) Set Date (only if different from above)"));
  Serial.println(F("6) Set Local Timezone Offset"));
  Serial.println(F("7) Check GPS Fix"));
  Serial.println(F("8) Set GPS Parameters"));
  Serial.println(F("9) Start Logging"));
  Serial.println(); //could add display tag details and eventually add ability to update ODR
}

void parseMenu(char c)
{
  switch (c)
  {
    case '1':
        setTagNum();      
      break;
    case '2':
        setRTCTime();      
      break;
    case '3':
        displayRTCTime();   
      break;
    case '4':
        setDelayStart();
      break;
    case '5':
        setRTCDate();
      break;
    case '6':
        setTZ();
      break;
    case '7':
         getFix();
      break;
    case '8':
         setGPS();
      break;
    case '9':
        //Set Flags to start logging
        start = true;
      break;
  }
}


/**************************************************************************/
/*
    Helper function to prompt for a value, and return
    it if it's within a valid range.
*/
/**************************************************************************/
byte prompt(String ask, int mini, int maxi)
{
  Serial.print(ask + "? ");
  while (!Serial.available()) ; // Wait for numbers to come in
  byte rsp = Serial.parseInt();
  if ((rsp >= mini) && (rsp <= maxi))
  {
    Serial.println(rsp);
    return rsp;
  }
  else
  {
    Serial.println("Invalid.");
    return 111;
  }
}
/**************************************************************************/
/*
  This function allows the user to sync the time to a GPS
*/
/**************************************************************************/
void updateTime(void)
{
    seconds = rtc.getSeconds();
    minutes = rtc.getMinutes();
    hours = rtc.getHours();
}
/**************************************************************************/
/*
  This function allows the user to Enter a tag Number
  It also sets the Intialization File name
*/
/**************************************************************************/
void setTagNum(void)
{
  bool tagSet = false;
  while(!tagSet){
    Serial.println("Set the Tag Number:");
    byte rsp = prompt("Tag (1-9)", 1, 9);
    if(rsp >= 1 && rsp < 10){
    tag = rsp;
    settingsFilename[1] = '0' + tag;
    filename[1] = '0' + tag;
    tagSet = true;
    Serial.println();
    Serial.print("Tag Number: ");
    Serial.println(tag);
    delay(1000);
    }
  }
}

/**************************************************************************/
/*
  This function allows the user to Enter a Timezone offset
  The value is stored in txOffset
*/
/**************************************************************************/
void setTZ(void)
{
  bool tzSet = false;
  while(!tzSet){
    Serial.println("Set the Timezone offset:");
    byte rsp = prompt("Plus or Minus from GMT (Falklands are Minus)? 1 (+), 2 (-)", 1, 2);
    if(rsp >= 1 && rsp < 3){
      if(rsp == 1){
        tzOffset[0] = '+';
      } else {
        tzOffset[0] = '-';
      }
      rsp = prompt("Number of hours? (0-12)", 0, 12);
      if(rsp >= 0 && rsp < 13){
        if(rsp < 10){
          tzOffset[1] = '0';
          tzOffset[2] = '0' + rsp;
        } else{
          tzOffset[1] = '1';
          tzOffset[2] = '0' + rsp%10;
        }    
      }
    tzSet = true;
    Serial.println();
    Serial.print("Timzone Offset: ");
    Serial.print(tzOffset);
    Serial.println();
    delay(1000);
    }
  }
}

/**************************************************************************/
/*
  This function allows the user to set the Date
*/
/**************************************************************************/
void setRTCDate(void)
{
    Serial.println("Set the Date:");
    year = prompt("Year (YY)", 0, 99); // Get the year
    month = prompt("Month", 1, 12); // Get the month
    day = prompt("Day", 1, 31); // Get the day
    rtc.setDate(day, month, year);
    updateFileNames();
    Serial.println(); 
}

/**************************************************************************/
/*
  This function allows the user to sync the time to a GPS
*/
/**************************************************************************/
void setRTCTime(void)
{
  //Create a loop here to allow user to resync clock
  bool timeSet = false;
  while(!timeSet){
    Serial.println("Set the Tag's Clock. Sync with a gps");
    hours = prompt("Hour (0-23)", 0, 23); // Get the hour
    minutes = prompt("Minute", 0, 59); // Get the minute
    seconds = prompt("Second", 0, 59); // Get the second
    Serial.println("Press any key to begin (spacebar, then enter)");
    while (!Serial.available()) ; // Wait for keypress to start clock 
    rtc.begin();    // Start the RTC in 24hr mode
    rtc.setTime(hours, minutes, seconds); // Then set the time
    rtc.setDate(day, month, year); // And the date
    Serial.parseInt();
    Serial.println("Time Set:");
    print2digits(rtc.getHours());
    Serial.print(":");
    print2digits(rtc.getMinutes());
    Serial.print(":");
    print2digits(rtc.getSeconds());
    Serial.println();
    //Add clock display and allow user to reset
    byte i2 = 0;
    while(i2 < 6){
      timekeeper = rtc.getSeconds();
      if(seconds != timekeeper) {
        seconds = timekeeper;
        Serial.println(seconds);
        i2++;
      }
    }
    Serial.println("Are you happy with the time sync?");
    delay(500); 
    i2 = prompt("0 = No, 1 = Yes", 0, 1);  
    Serial.println();
    if(i2 != 0){
      timeSet = true;
    }  
  }
}

/**************************************************************************/
/*
  This function displays the clock to confirm sync with external source
*/
/**************************************************************************/
void displayRTCTime(void)
{
    Serial.println();
    //Add clock display and allow user to reset
    Serial.println("Press letter key then enter to exit");
    while (!Serial.available()){
      timekeeper = rtc.getSeconds();
      if(seconds != timekeeper) {
        seconds = timekeeper;
        // Print the time
        print2digits(rtc.getHours());
        Serial.print(":");
        print2digits(rtc.getMinutes());
        Serial.print(":");
        print2digits(seconds);
        Serial.println();   
      }
    }
    Serial.parseInt();
    Serial.println();
    Serial.println(); 
}


/**************************************************************************/
/*
 * This function allows the user to set a start delay to the tag
 * Currently only minutes are implemented
*/
/**************************************************************************/
void setDelayStart(void)
{ 
    Serial.println(); 
    Serial.println("Logging Delay Disabled");
    Serial.println(); 
    delay(2000); 
  /* Logging delay does not work with Retick
  Serial.println("Enter a time delay in minutes (must be less than 360):");
  delayStart = prompt("Delay Minutes (0-360)", 0, 360); // Get the delayStart
  if(delayStart > 0) {
    //start with the current time
    updateTime();
    uint8_t delayHrs = hours + (delayStart/60);
    uint8_t delayMin = minutes + (delayStart%60);
    byte delayDAY = day;
    byte delayMON = month;
    byte delayYR = year;
    if (delayMin >= 60){
      delayHrs++;
      delayMin = delayMin%60;
    }
    if (delayHrs >= 24){
      delayHrs = delayHrs%24;
      Serial.println("Please confirm the date the tag will start:");
      print2digits(delayHrs);
      Serial.print(":");
      print2digits(delayMin);
      Serial.println(":00 on"); 
      delayDAY = prompt("Day (DD)", 1, 31); // Get the day
      delayMON = prompt("Month (MM)", 1, 12); // Get the month    
      delayYR = prompt("Year (YY)", 0, 99); // Get the year 
    }
    rtc.setAlarmTime(delayHrs, delayMin, 0);
    rtc.setAlarmDate(delayDAY, delayMON, delayYR); 
    rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS); // Once, on a specific date and a specific time
    Serial.println(); 
    Serial.println("Logging will begin at:"); 
    // ...print time
    print2digits(rtc.getAlarmHours());
    Serial.print(":");
    print2digits(rtc.getAlarmMinutes());
    Serial.print(":");
    print2digits(rtc.getAlarmSeconds());
    Serial.print(" on "); 
    // Print date...
    print2digits(rtc.getAlarmMonth());
    Serial.print("/");
    print2digits(rtc.getAlarmDay());
    Serial.print("/");
    print2digits(rtc.getAlarmYear());
    Serial.println();
    Serial.println("You must select Start Logging to initialize the tag.");
    delay(5000);
  }else {
    rtc.disableAlarm();
    Serial.println(); 
    Serial.println("Logging Delay Disabled");
    Serial.println(); 
    delay(5000); 
  }
  */
}


/**************************************************************************/
/*
 * This function allows the user to set GPS sampling parameters
*/
/**************************************************************************/
void setGPS(void)
{ 
    byte i3 = 0;
    Serial.println("Current GPS Settings:");
    Serial.print("GPS Sampling Rate (seconds): "); Serial.println(gpsRate);
    Serial.print("GPS Timeout (seconds): "); Serial.println(gpsTimeOut);
    Serial.print("GPS Read Delay (seconds): "); Serial.println(gpsDelay);
    Serial.print("GPS Samples per logging interval: "); Serial.println(loopMaxGPS);
    Serial.print("GPS Smart Delay (milliseconds): "); Serial.println(smartDelayMS);
    
    Serial.println("Would you like to make changes?");
    delay(500); 
    i3 = prompt("0 = No, 1 = Yes", 0, 1);  
    Serial.println();
    if(i3 != 0){  
      gpsRate = prompt("GPS Sampling Rate(seconds) - default is 120", 1, 600);       // Seconds of GPS sleep between data acquisition
      gpsTimeOut = prompt("GPS Timeout(seconds) - default is 60", 20, 90);           // Seconds to wait for a fix to be logged before sleeping
      gpsDelay = prompt("GPS Read Delay(seconds) - default is 8", 1, 30);            // Seconds to wait before trying to log - allows for a fix
      loopMaxGPS = prompt("GPS Samples per logging interval - default is 1", 1, 30); // Number of GPS reads before sleeping
      smartDelayMS = prompt("GPS Smart Delay(milliseconds) - default is 10", 1, 20); // Number of milliseconds to spend encoding GPS data between other instructions
    }
}


/**************************************************************************/
/*
  This function displays the GPS Fix status
*/
/**************************************************************************/
static void getFix(void)
{
    float flat, flon;
    unsigned long age, date, time, chars = 0;
    unsigned short sentences = 0, failed = 0;
    static const double PG_LAT = 36.6177, PG_LON = -121.9166;
    Serial.println();
    Serial.println("Press any key then enter to exit");
    delay(1000);
    Serial.println();
    Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
    Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  - to Pacific Grove -  RX    RX        Fail");
    Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");
    
    while (!Serial.available()){
      
      print_int(GPS.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
      print_int(GPS.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
      GPS.f_get_position(&flat, &flon, &age);
      print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
      print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
      print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
      print_date(GPS);
      print_float(GPS.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
      print_float(GPS.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
      print_float(GPS.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
      print_str(GPS.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(GPS.f_course()), 6);
      print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, PG_LAT, PG_LON) / 1000, 0xFFFFFFFF, 9);
      print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, PG_LAT, PG_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
      print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, PG_LAT, PG_LON)), 6);

      GPS.stats(&chars, &sentences, &failed);
      print_int(chars, 0xFFFFFFFF, 6);
      print_int(sentences, 0xFFFFFFFF, 10);
      print_int(failed, 0xFFFFFFFF, 9);
      Serial.println();
      smartdelay(1000);
      
    }
    Serial.println();
    Serial.println(); 
}

/**************************************************************************/
/*
  This function updates Filenames when a change in Date is made
*/
/**************************************************************************/
void updateFileNames(void){
  // Set MMDD on the init File
  if(month >= 10){
    filename[3] = '0' + month/10;
    filename[4] = '0' + month%10;
    settingsFilename[3] = '0'+ month/10;
    settingsFilename[4] = '0' + month%10; 
  }
  else{
    filename[3] = '0';
    filename[4] = '0' + month;
    settingsFilename[3] = '0';
    settingsFilename[4] = '0' + month; 
  }
  if(day >= 10){
    filename[5] = '0' + day/10;
    filename[6] = '0' + day%10;
    settingsFilename[5] = '0' + day/10;
    settingsFilename[6] = '0' + day%10; 
  }
  else{
    filename[5] = '0';
    filename[6] = '0' + day;
    settingsFilename[5] = '0';
    settingsFilename[6] = '0' + day; 
  }
}


/**************************************************************************/
/*
  This function prints 2-Digit numbers to serial
*/
/**************************************************************************/
static void print2digits(byte number) {
  if (number < 10) {
    Serial.print("0"); // print a 0 before if the number is < than 10
  }
  Serial.print(number);
}

/**************************************************************************/
/*
  This function Feeds the GPS library data to parse
*/
/**************************************************************************/

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      GPS.encode(Serial1.read());
  } while (millis() - start < ms);
}

///////////////////////////////////////////
/////  GPS Print to Serial Functions  /////
static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  GPS.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}

//////////////////////////////////////
///// GPS Logging Functions /////////

static void log_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid) {
    logGPS = false;
  }
  else {
    sprintf(sz, ",%ld", val);
    reading += sz;
  }
  smartdelay(0);
}

static void log_date(TinyGPS &gps)
{
  int yearGPS;
  byte monthGPS, dayGPS, hourGPS, minuteGPS, secondGPS, hundredthsGPS;
  unsigned long age;
  GPS.crack_datetime(&yearGPS, &monthGPS, &dayGPS, &hourGPS, &minuteGPS, &secondGPS, &hundredthsGPS, &age);
  if (age == TinyGPS::GPS_INVALID_AGE) {
    logGPS=false;
  }
  else
  {
    char sz[32];
    sprintf(sz, ",%02d/%02d/%02d,%02d:%02d:%02d,%d",
        monthGPS, dayGPS, yearGPS, hourGPS, minuteGPS, secondGPS, age);
    reading += sz; 
  }

  smartdelay(0);
}

