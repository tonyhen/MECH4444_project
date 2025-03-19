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
  float ROT;
  float desired_heading;
  float right_old = 112;
  float left_old = 112;
  bool left_flag = 0;
  bool right_flag = 0;
  unsigned long right_time;
  unsigned long left_time;
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
    while (true) {
      long int startMillis = millis();  // time since Arduino turned on (ms)
      while (millis() < startMillis + 100) {
        // wait for .1 second
      }
      const int normal = 112;
      const int threshold = 10;
      const int upper = normal + threshold;
      const int lower = normal - threshold;
      const int turn_time = 10;
      startTime = millis();
      while (true) 
      {
        digitalWrite(RPin,LOW);
        digitalWrite(BPin,LOW);
        digitalWrite(GPin,LOW);
        float abs_error_left = abs(hall_L - normal);
        float abs_error_right = abs(hall_R - normal);
        

        while ((upper > hall_L) && (upper > hall_R) && (hall_R > lower) && (hall_L > lower)) 
        {
          front = 10;
        }
        stop();
        if (hall_L > upper || hall_L < lower) 
        {
          left_time = millis();
          left_flag = 1;
          digitalWrite(RPin,LOW);
          digitalWrite(BPin,HIGH);
          digitalWrite(GPin,LOW);
          startTime = millis();
          while (millis() < startTime + turn_time) 
          {
            turn_left(1);
          }
          stop();
        }
        else 
        {
          left_flag = 0;
        }
        if (hall_R > upper || hall_R < lower) 
        {
          right_time = millis();
          right_flag = 1;
          digitalWrite(RPin,HIGH);
          digitalWrite(BPin,LOW);
          digitalWrite(GPin,LOW);
          startTime = millis();
          while (millis() < startTime + turn_time) 
          {
            turn_right(1);
          }
          stop();
        }
        else {
        right_flag = 0;
        }

        if((left_flag == 1) && (right_flag == 1))
        {
          flag_buzzer = true;
          if (right_time > left_time)
          {
            stop();
            startTime = millis();
            while(millis() < startTime + 1500)
            {
              spinr = 1;
            }
            stop();
          }
          if (right_time < left_time)
          {
            stop();
            startTime = millis();
            while(millis() < startTime + 1500)
            {
              spinl = 1;
            }
            stop();
          }
        }
        //ultrasonic distance measurements
        int desired_distance = 10; //desired distance in cm
        float actual_distance = ultrasonic_dist(distance,5);   //get the average of distance measurements
        float dist_error = actual_distance - desired_distance; //calculate the error between the  distances
        //while the distance error is in between 1 cm
        while(abs(dist_error) < 1)
        {
            stop();
        }

      }
    }
  }
}
