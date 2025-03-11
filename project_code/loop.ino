/* main high-level control loop */
void loop() {
  // don't use the delay() function as it blocks code and prevents
  // "back" "front" etc. from updating correctly in the ISR

  // using Serial.print() functions slows the code down and can reduce control performance

  // BNO055 IMU yaw angle (compass heading) is stored in variable: yaw_angle
  // values read from left and right Hall sensors are stored in variables: hall_L, hall_R
  // distance in cm to target from ultrasonic sensor is stored in variable: distance
  // estimated distance travelled along ground in terms of left and right encoder pulses are stored
  // in variables: ldistance, rdistance

  currentMillis = millis();                   // time since Arduino turned on (ms)
  currentMillis = currentMillis - startTime;  // want time since setup() has finished
  float error = 10;
  //float ROT;
  float desired_heading;


  if (currentMillis <= 4000)  // wait 4s before moving for robot to gain its balance on the ground
  {
    front = 0;
    back = 0;
    turnl = 0;
    turnr = 0;
    spinl = 0;
    spinr = 0;
    turnoutput = 0;
    flag_wait = true;
  } else {
    flag_wait = false;  // flag to indicate it is ok to start moving
    digitalWrite(RPin, HIGH);
    digitalWrite(GPin, LOW);
    digitalWrite(BPin, HIGH);  // turn on Green LED
  }
  if (flag_wait == false)  // ok to start doing something
  {
    // to validate pitch angle estimates from Kalman filter, take average of 25 Kalman filter estimates
    // *************************************************************************************************
    // *** ensure pwm motor commands at end of Balance.cpp are commented out so motor does not turn ****
    // *************************************************************************************************
    const float normal_value = 112;
    const float bound = 10;

    while (true) {
      //long int startMillis = millis(); // time since Arduino turned on (ms)
      //while(millis() < startMillis + 100)
      //{
      // wait for .1 second
      //}
      go();
      if (hall_R > normal_value + bound || hall_R < normal_value - bound) {
        stop();
        desired_heading = yaw_angle;
        unsigned long time_begin = millis();
        while (time_begin + 1000 < millis()) {
          back = 25;
          digitalWrite(RPin, LOW);
          digitalWrite(GPin, HIGH);
          digitalWrite(BPin, LOW);
        }
        stop();
      }
      if (hall_L > normal_value + bound || hall_L < normal_value - bound) {
        stop();
        desired_heading = yaw_angle;
        unsigned long time_begin = millis();
        while (time_begin + 1000 < millis()) {
          back = 25;
          digitalWrite(RPin, LOW);
          digitalWrite(GPin, LOW);
          digitalWrite(BPin, HIGH);
        }
        stop();
      }
      while (error != 0) {
        error = yaw_correction(desired_heading, yaw_angle);
        if (error > 0) {
          turn_right(error);
        } else {
          turn_left(error);
        }
      }
    }
  }
}
/*
        startMillis = millis();
        flag_buzzer = true;
        while(millis() < startMillis + 5000)
          {
            
          }
        flag_buzzer = true;
        desired_heading = yaw_angle + 180;
        while(error != 0)
        {
         error = yaw_correction(desired_heading, yaw_angle);
         if(error>0)
         {
           turn_right(error);
         }
         else
         {
           turn_left(error);
         }
        }
        stop();
        flag_buzzer = true;
        startMillis = millis();

        while(millis() < startMillis + 5000)
        {
            
        }
        desired_heading = yaw_angle - 180;
        error = 10;
        while(error != 0)
          {
            error = yaw_correction(desired_heading, yaw_angle);
            if(error>0)
            {
              turn_right(error);
            }
            else
            {
              turn_left(error);
            }
          }
        stop();    
        flag_buzzer=true;    

      Serial.print("Pitch angle= ");
      Serial.print(kalmanfilter.angle,1); Serial.print(" deg   ");
      Serial.print("Pitch angular rate= ");
      Serial.print(kalmanfilter.angle_dot,2); Serial.println(" deg/s");

      // Pathfinding Algorithm
      desired_heading = yaw_angle;
      while(true)
      {
        desired_heading = pathfinding(hall_L, hall_R);
        error = yaw_correction(desired_heading, yaw_angle);
        if(error>0)
        {
          turn_right(error);
        }
        else
        {
          turn_left(error);
        }
      }

    }
    
  }

}
*\