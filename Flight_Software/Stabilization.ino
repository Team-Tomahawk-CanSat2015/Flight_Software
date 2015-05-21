  /*********************************************************************************************************/
 /* Stabilizatin Algortithm Call this function in the stablilization stage*/
 /*********************************************************************************************************/
  void stabilize (float init_Heading, float descentRate){ //input is heading and descent rate
       
   //dECELARATIONS  and initilizations
    float headingscaled, ok_offset, fin_angle, deg_rad, gain, y_alpha, x_alpha, z_alpha, z_rollrate, Altitude, Temperature, pos, maxpos, rad_deg;
      bool more_than;
      gain = 0.6;
      ok_offset = 5;
      deg_rad = 3.14159/180;
      rad_deg = 180/3.142;
      maxpos = 45;
      
      adafruit_function (&y_alpha, &x_alpha, &z_alpha, &z_rollrate, &Altitude, &Temperature); //Just used to get the IMU DATA
  
      
      if ( sin(z_alpha) >= sin (init_Heading - ok_offset) && sin(z_alpha) <= sin (init_Heading + ok_offset ) ){ //If CANSAT is close in Okay zone
      // sideNote: Okay zone is simply when the cansat is +/- 'ok_offset'(5 degrees) from initial heading
      servo1.write(90);
      servo2.write(90);
      }
      
      else{   //if CANSAT is not in okay zone 
      fin_angle = ((sinh ((z_rollrate*deg_rad)/descentRate))/ deg_rad) *gain; //sTILL WORKING on this equation, it may
      
      if (fin_angle > maxpos)
      more_than = true;
      else
      more_than = false;
      
      switch (more_than){
            case (true):
            if (z_rollrate <= 0)  //based on the direction of spin f the cansat * I (Tayo) will still change this if its oposite
            maxpos = -maxpos;    
            
                 servo1.write(90 + maxpos);
                 servo2.write(90 - maxpos);
                 break;
                 
            case (false):
                 servo1.write(90 + fin_angle);
                 servo2.write(90 - fin_angle);
                 break;
      }
      }
      
      }
