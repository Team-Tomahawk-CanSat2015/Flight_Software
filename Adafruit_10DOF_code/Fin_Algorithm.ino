  void stabilize (float init_Heading, float descentRate){
       
      float headingscaled, ok_offset, fin_angle, deg_rad, gain, y_alpha, x_alpha, z_alpha, z_rollrate, Altitude, Temperature, pos, maxpos, rad_deg;
      bool more_than;
      adafruit_function (&y_alpha, &x_alpha, &z_alpha, &z_rollrate, &Altitude, &Temperature);
      gain = 0.6;
      ok_offset = 90;
      deg_rad = 3.14159/180;
      rad_deg = 180/3.142;
      maxpos = 45;
  
  
       //If CANSAT is close in Okay zone
      if ( sin(z_alpha) >= sin (init_Heading - ok_offset) && sin(z_alpha) <= sin (init_Heading + ok_offset ) )
      { 
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
            if (z_rollrate <= 0)
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
