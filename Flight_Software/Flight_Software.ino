/*
* ---Team Tomahalk Payload Flight Software---
* File contains the core flight software loop
*/
#include <Servo.h>
#include <Wire.h>

#define RocketBurn_time      2   //sec //From manual
#define RocketDelay_time      9    //sec //From Manual
#define PayloadDeployDelay_time  5    //sec //Estimate
#define WireBurn_time         4   //sec //Estimate
#define altCalibrationDuration 5
#define descentRateSamplingPause 200

//define pins
#define servoOnePin 9
#define servoTwoPin 11
#define memResetBtnPin 8
#define voltageMeasurementPin A0
#define buzzerPin 6

/**
* Flight Software state variable:
*  0 - initialize
*  1 - Launch Wait
*  2 - Ascent
*  3 - Rocket Deployment / Stabilization
*  4 - Seperation
*  5 - Descent (Main Payload Action Stage)
*  6 - Landed
**/
byte state;

// Time variables
unsigned int initialize_time;
unsigned int liftoff_time;
unsigned int a_time;  //corresponds to actual time in seconds from midnight
unsigned int prev_Time;

const char trasmitionDelim = ',';

unsigned int packet_count;
unsigned int ground_alt;
float  init_Heading;

byte sensor_size = 10;
float sensor_data[10];

//used for descent rate calculation
//stores last 5 altitudes measured with timestamp //TODO: to figure alt and descent rate
float alt_buffer[5];
unsigned long alt_buffer_time[5];

Servo servo1, servo2;

void setup()
{
  Serial.begin(9600);

  //setup for Adafruit 10DoF IMU
  Wire.begin();
  initilize_Adafruit_10_DOF_Sensors();  //Enable adafruit sensors;

  //setup GPS
  setupGPS();

  //Configure servo pins
  servo1.attach (servoOnePin);
  servo2.attach (servoTwoPin);

  if (digitalRead(memResetBtnPin) == HIGH)
    ClearMemory();

  boot();
 
}

/**
* Main Software Loop:
* 1. Collect data from sensors
* 2. Preform State-specific functions (actions and transitions check)
* 3. Save State to memory
* 4. Transmit data
**/
void loop()
{
  if (digitalRead(memResetBtnPin) == HIGH)
  {
    ClearMemory();
    boot();
  }

  //1. Collect data from sensors and fill Sensor_Data array
  Collect_Sensor_Data();
  
  //2. Preform State-specific functions
  if (sensor_data[3]<990)
  {
    switch (state)
    {
      case 0:
        initialize();
        break;
      case 1:
        launch_wait();
        break;
      case 2:
        ascent();
        break;
      case 3:
        rocketDeployment_Stabilization();
        break;
      case 4:
        seperation();
        break;
      case 5:
        descent();
        break;
      case 6:
        landed();
        break;
      default:
        boot();
    }
  }

  //3. Save State to memory
  saveState();
  
  //4. Transmit data
  if (a_time - prev_Time > 0)
  {
    unsigned int missionTime = a_time-initialize_time;
    transmitData(&missionTime);
    prev_Time = a_time;
  }
}

/**
* Pulls data from sensors to fill the flight software's sensor_data float array
*
* Layout:
* array pos. - value (units-accuracy)
* [0] - temp (celcius-1)
* [1] - latitude
* [2] - longitude
* [3] - altitude (m-0.1)
* [4] - descent rate (m/s - 0.1)
* [5] - voltage (volts-0.05)
* [6] - x axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [7] - y axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [8] - z axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [9]- z_axis roll Rate (deg/s)
*
**/
void Collect_Sensor_Data()
{
  adafruit_function (&sensor_data[7], &sensor_data[6], &sensor_data[8], &sensor_data[9], 0, &sensor_data[0]); //(&y_alpha, &x_alpha, &z_alpha, &z_rollrate, 0, &temp)
  getGPSdata (&sensor_data[1], &sensor_data[2], &sensor_data[3],&a_time); //(&latitude, &longitude, &alt,&time)
  readVoltage(&sensor_data[5]);
  calculate_descentRate(&(sensor_data[3]),&(sensor_data[4]));
  sensor_data[3] -= ground_alt;
  
}

void readVoltage(float* voltage)
{
  *voltage =  analogRead(voltageMeasurementPin) * 2.0 * (5.0 / 1023.0);
}

/**
* function uses the new altitude data, stores it in the alt_buffer array
* and then calculates an average descent rate based on the previous 5 altitudes
* returns float value of calculated average descent rate
**/
void calculate_descentRate(float *new_alt,float *descentRate) //TODO: to figure alt and descent rate stuffs
{
  if (millis()-alt_buffer_time[0]>descentRateSamplingPause)
  {
      //shift alt_buffer and alt_buffer_time array elements
      for (byte i = 4; i > 0; i--)
      {
        alt_buffer[i] = alt_buffer[i - 1];
        alt_buffer_time[i] = alt_buffer_time[i - 1];
      }
      //add new elements
      alt_buffer[0] = *new_alt;
      alt_buffer_time[0] = millis();
    
      //calculate average of the average descent rates between each altitude step ie. 5->4, 4->3, 3->2, 2->1
      float sum_average_descent_rate_step = 0;
      byte numberOfDeltas = 0;
    
      for (byte i = 0; i < 5; i++)
      {
        float altNewer = alt_buffer_time[i];
        float altOlder = alt_buffer_time[i+1];
         if ((alt_buffer_time[i] - alt_buffer_time[i+1])>0  && altOlder<990 && altNewer <990)
         {
           sum_average_descent_rate_step += (altOlder - altNewer)*1000.0 / (alt_buffer_time[i] - alt_buffer_time[i+1]);
           numberOfDeltas ++;
         }
      }
      if (numberOfDeltas != 0) 
        *descentRate =  sum_average_descent_rate_step / numberOfDeltas;
  }
}



/**
*
* Transmission format:
* Transmission 1: 1,2,3,45,6,123,55,3,22,454
* Transmission 2: 33,11,244,55,22,44,222,44
* ie. ',' delimintes new value, '\n' deliminates new transmission
**/
void transmitData (unsigned int *missionTime)
{
  //transmit mission time in seconds
  Serial.print(++ packet_count);// Amount of data sent;
  Serial.print(trasmitionDelim);
  Serial.print(*missionTime);
  Serial.print(trasmitionDelim);
  Serial.print(state);

  //transmit sensor data
  for (int i = 0; i < sensor_size; i++)
  {
    Serial.print(trasmitionDelim);
    if (i == 1 || i == 2) // GPS Lat and Longitude
    {
      Serial.print(sensor_data[i], 4);
    }
    else
    {
      Serial.print(sensor_data[i], 1);
    }
  }

  //end transmition
  Serial.println();
}
