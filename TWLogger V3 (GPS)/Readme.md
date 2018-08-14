## Low Power Inertial Movement Datalogger for Feather M0 Adalogger 
### Version 3 (GPS)
 - TWLogger3.X_LSM303 version samples Temp, Accel, Mag, GPS

### Summary:
 - Logs to CSV, flushing data after MinutesPerCycle minutes
 - Internal RTC used to timestamp sensor data
 - Utilizes ReTick to ensure regular sampling intervals and save power (Sleeps during Idle time)
 - Restarts logging if reset (time sync is lost)
 - RED LED blinks once per minute for a low consumption pulse
 - User Defined GPS Sampling Parameters
