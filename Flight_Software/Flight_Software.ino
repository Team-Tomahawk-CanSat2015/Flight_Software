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

//define pins
#define servoOnePin 9
#define servoTwoPin 11
#define memResetBtnPin 8
#define voltageMeasurementPin A0

//---------------
//Time globals
 int a_time;  //corresponds to actual time in seconds from midnight
 int m_time; //corresponds to missiontime or time since we started the mission which we could initilaze


/**
* Flight Software state variable:
*  0 - Uninizialized
*  1 - Launch Wait
*  2 - Ascent
*  3 - Rocket Deployment / Stabilization
*  4 - Seperation
*  5 - Descent (Main Payload Action Stage)
*  6 - Landed
**/
byte state = 0;

// Transmission variables
const short transmitInterval = 1000;
unsigned long previousTransmitTime = 0;
unsigned long currentMillis;
const char trasmitionDelim = ',';

unsigned int packet_count = 0, liftoff_time;
unsigned short  init_Heading, ground_alt;


byte sensor_size = 11;
float sensor_data[11];

//used for descent rate calculation
//stores last 5 altitudes measured with timestamp
float alt_buffer[5] = {0,0,0,0,0};
unsigned long alt_buffer_time[5]= {0,0,0,0,0};

Servo servo1, servo2;

void setup()
{
  packet_count = 0;
  Serial.begin(9600);

  //setup for Adafruit 10DoF IMU
  Wire.begin();
  initilize_Adafruit_10_DOF_Sensors();  //Enable adafruit sensors;

  //setup GPS
  setupGPS();
  ground_alt = 0; //GROUND ALTITUDE IN METERS

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
    ClearMemory();

  //1. Collect data from sensors and fill Sensor_Data array
  Collect_Sensor_Data();

  //2. Preform State-specific functions
  switch (state)
  {
    case 1:
      launch_wait();
    case 2:
      ascent();
    case 3:
      rocketDeployment_Stabilization();
    case 4:
      seperation();
    case 5:
      descent();
    case 6:
      landed();
    default:
      boot();
      ;
  }

  //3. Save State to memory
  saveState();

  //4. Transmit data
  currentMillis = millis();
  if (currentMillis - previousTransmitTime >= transmitInterval)
  {
    transmitData(&currentMillis);
    //Calibrate time to transmit next interval step
    previousTransmitTime = currentMillis - currentMillis % transmitInterval;
  }
}

/**
* Pulls data from sensors to fill the flight software's sensor_data float array
*
* Layout:
* array pos. - value (units-accuracy)
* [0] - altitude (m-0.1)
* [1] - temp (celcius-1)
* [2] - voltage (volts-0.05)
* [3] - x axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [4] - y axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [5] - z axis angle, "alpha" (degrees)  //Look at IMU for axis referencing
* [6] - descent rate (m/s - 0.1)
* [7] - latitude
* [8] - longitude
* [9]- z_axis roll Rate (deg/s)
*
**/
void Collect_Sensor_Data()//TODO when more sure:: remove local float variables a mem-hole
{
  //local memory hole (52 bytes)
  float alt;
  float temp; //IMU
  float x_alpha;  //IMU, Angular position relative to Adafruit x Axis
  float y_alpha; //IMU, Angular position relative to Adafruit y Axis
  float z_alpha; //IMU, Heading
  float z_rollrate; //IMU, rollrate relative to Adafruit z Axis
  float descentRate; //calculate based on previous alts
  float latitude; //GPS
  float longitude; //GPS

  adafruit_function (&y_alpha, &x_alpha, &z_alpha, &z_rollrate, 0, &temp);
  getGPSdata (&latitude, &longitude, &alt);

  descentRate = calculate_descentRate(&alt, millis());

  sensor_data[0] = alt;
  sensor_data[1] = m_time;
  sensor_data[2] = temp;
  sensor_data[3] = readVoltage;
  sensor_data[4] = x_alpha;
  sensor_data[5] = y_alpha;
  sensor_data[6] = z_alpha;
  sensor_data[7] = descentRate;
  sensor_data[8] = latitude;
  sensor_data[9] = longitude;
  sensor_data[10] = z_rollrate;
}

float readVoltage()
{
  return analogRead(voltageMeasurementPin) * 2.0 * (5.0 / 1023.0);
}

/**
* function uses the new altitude data, stores it in the alt_buffer array
* and then calculates an average descent rate based on the previous 5 altitudes
* returns float value of calculated average descent rate
**/
float calculate_descentRate(float *new_alt, unsigned long new_alt_timestamp)
{
  //shift alt_buffer and alt_buffer_time array elements
  for (byte i = 4; i > 0; i--)
  {
    alt_buffer[i] = alt_buffer[i - 1];
    alt_buffer_time[i] = alt_buffer_time[i - 1];
  }
  //add new elements
  alt_buffer[0] = *new_alt;
  alt_buffer_time[0] = new_alt_timestamp;

  //calculate average of the average descent rates between each altitude step ie. 5->4, 4->3, 3->2, 2->1
  float sum_average_descent_rate_step = 0;

  for (byte i = 4; i > 0; i--)
  {
    sum_average_descent_rate_step += (alt_buffer[i] - alt_buffer[i - 1]) / (alt_buffer_time[i - 1] - alt_buffer_time[i]);
  }
  return sum_average_descent_rate_step / 4.0;
}



/**
*
* Transmission format:
* Transmission 1: 1,2,3,45,6,123,55,3,22,454
* Transmission 2: 33,11,244,55,22,44,222,44
* ie. ',' delimintes new value, '\n' deliminates new transmission
**/
void transmitData (unsigned long *currentMillis)
{
  //transmit mission time in seconds
  Serial.print(++ packet_count);// Amount of data sent;
  Serial.print(trasmitionDelim);
  Serial.print(*currentMillis / 1000.0, 2);
  Serial.print(trasmitionDelim);
  Serial.print(state);

  //transmit sensor data
  for (int i = 0; i < sensor_size; i++)
  {
    Serial.print(trasmitionDelim);
    if (i == 7 || i == 8) // GPS Lat and Longitude
    {
      Serial.print(sensor_data[i], 5);
    }
    else
    {
      Serial.print(sensor_data[i], 1);
    }
  }

  //end transmition
  Serial.println();
}
