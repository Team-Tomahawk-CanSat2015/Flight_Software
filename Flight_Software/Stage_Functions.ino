/*********************************************************************************************************/
/* Stage transition and task Alogorithms/
**********Sensor data reference************
sensor_data[0] = alt;
sensor_data[1] = extTemp;
sensor_data[2] = inTemp;
sensor_data[3] = voltage;
sensor_data[4] = x_alpha;
sensor_data[5] = y_alpha;
sensor_data[6] = z_alpha;
sensor_data[7] = descentRate;
sensor_data[8] = latitude;
sensor_data[9] = longitude;
sensor_data[10] = z_rollrate;
/*****************Relevant Times reference ***************
RocketBurn_time = 1.8s;
Delay_time = 9 sec;
SatDeployDelay = 2 sec;
Nichromeburn_time = 3 sec:
/*********************************************************************************************************/


void launch_wait() {
  
  /********FUNCTION task*********/
   //Reset stuff here
  
  /********Transition Check*********/
  if (sensor_data[0] > (ground_alt + 3) && sensor_data[7] != 0) { 
       state = 2;
       liftoff_time = a_time; //Register time of liftoff
  }
}

void ascent() {
    /********FUNCTION task*********/
   //NO function task for ascent

   /********Transition Check*********/
  if ( (a_time - liftoff_time) >  ( RocketBurn_time + RocketDelay_time )  ) { //I guess this is where we need a RTC however i used packet count for now.
                                                                             //our GPS had an RTC ANS I WILL probably activate that
       state = 3;
  }
}

  void rocketDeployment_Stabilization() { //CANSAT stabilization/deployment
    /********FUNCTION task*********/
   //NO function task for ascent

   /********Transition Check*********/
  if ( (a_time - liftoff_time) >  (RocketBurn_time + RocketDelay_time + PayloadDeployDelay_time)\
         || sensor_data[0]  <=  400  ) {  // if (9 +2 + 2) seconds has passed (9 sec delay + 1.8 sec burn + 4 sec to stabilize) 
       state = 4;
  }
}

  void seperation() {
      /********FUNCTION task*********/
   digitalWrite(5, HIGH);  //Nichrome BURN BBAABYY!!!!!!!!!!!!!

   /********Transition Check*********/
  if ( (a_time - liftoff_time) >  (RocketBurn_time + RocketDelay_time + PayloadDeployDelay_time + WireBurn_time)) {
       state = 5;
       init_Heading = sensor_data[6]; //initialize heading for fin stabilization
  }
}

  void descent() {
   /********FUNCTION task*********/
   stabilize(init_Heading, sensor_data[7]);  //Fins Activate !!!!!!!!!!!!!

   /********Transition Check*********/
  if (sensor_data[0] < (ground_alt + 3) ) {
       state = 6;
  }
  
  
  }
  void landed() {
   /********FUNCTION task*********/
   //Buzzer function here  
   
   /********Transition Check*********/
   //This is terminal stage!! Recover Cansat   
  
  }
