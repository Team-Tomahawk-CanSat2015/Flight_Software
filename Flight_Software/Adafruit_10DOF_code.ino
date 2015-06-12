//THIS CODE IS FOR THE ADAFRUIT 10 DOF SENSOR------
  //CHECK BUTTOM FOR FUNCTIONS
  //#include <Wire.h> 
  #include <Adafruit_Sensor.h>
  #include <Adafruit_LSM303_U.h>              
  #include <Adafruit_BMP085_U.h>
  #include <Adafruit_L3GD20_U.h>
  #include <Adafruit_10DOF.h>
  
  sensors_event_t accel_event;        
  sensors_event_t mag_event;       
  sensors_event_t gyro_event;          
  sensors_event_t bmp_event;   
  sensors_vec_t   orientation;
  
  Adafruit_10DOF                dof   = Adafruit_10DOF();
  Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
  Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
  Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
  
  Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //Gyro deceleration
  
  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;  /* Update this with the correct SLP for accurate altitude measurements */
  

  
  
  
  
  //***********************************************************************************************************************************
  //***********************************************************************************************************************************
  //***********************************************************************************************************************************
  //***********************************************************************************************************************************
  //***********************************************************************************************************************************
  //ADD YOUR FUNCTIONS HERE--ADD YOUR FUNCTIONS HERE--ADD YOUR FUNCTIONS HERE
  /*--DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS----DO NOT CROSS--*/
  void  initilize_Adafruit_10_DOF_Sensors()
  {
    gyro.enableAutoRange(true);           //Enable auto range gyro
    if(!accel.begin())
    {Serial.println("ERROR Accelerometer"); while(1);}
    if(!mag.begin())
    {Serial.println("ERROR Magnetometer"); while(1);}
    if(!bmp.begin())
    { Serial.println("ERROR Pressure");while(1);}
    if(!gyro.begin())
    { Serial.println("ERROR Gyroscope");while(1);}
  }
  void adafruit_function (float *y_alpha, float *x_alpha, float *z_alpha, float *z_rollrate, float *Altitude, float *Temperature)
  { 
    
    accel.getEvent(&accel_event);                                        
    gyro.getEvent(&gyro_event);
    
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {/* 'orientation' should have valid .roll and .pitch fields *///Serial.print(F("Roll: "));  Serial.print(orientation.roll);Serial.print(orientation.pitch);
     *y_alpha = orientation.pitch;
      *x_alpha = orientation.roll;}
        mag.getEvent(&mag_event);
    if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
    {  *z_alpha = orientation.heading;}
        bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      float Temp;
       bmp.getTemperature(&Temp);
       *Temperature = Temp;
         float seaLevelPressure = 1013; //units = hpa
       *Altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, Temp); 
       
       //z_rollrate = gyro_event.gyro.z; //rad/s
       *z_rollrate = (gyro_event.gyro.z) * (180/3.142); //deg/s
       if (mock)
       {
         unsigned long m_time = a_time - initialize_time;
         if (m_time <= 60)
           *Altitude = 70; //pre-flight
         else if (m_time<=4*60)
           *Altitude = -700.0/(8100.0) *(float)(m_time-60)*(m_time-4*60)+70; //Flight
         else
           *Altitude = 70; //buzzer/post-flight
       }
    }
  }

