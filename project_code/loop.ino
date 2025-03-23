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
  float ROT; // Variable to hold the amount of rotation in degrees
  float desired_heading; // Variable to hold the desired heading in degrees
  bool left_flag = 0; //Flag for error correcting spin function left wheel
  bool right_flag = 0; // Flag for error correcting spin function right wheel
  unsigned long right_time; // variable for when a wheel detects the strip
  unsigned long left_time; // Variable for when the left wheel detects the strip
  const int desired_distance = 10; //desired distance in cm
  bool at_distance = 0;


  // Existing Code to initialize balancing and gain balance provided by Dr. Bauer

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

  // End of Dr.Bauer's initialization and balance code

  {
    while (true) {
      long int startMillis = millis();  // time since Arduino turned on (ms)
      while (millis() < startMillis + 100)
      {
        // wait for .1 second
      }
      const int normal = 112; // Nominal hall effect sensor values
      const int threshold = 10; // Deadzone to determine when a wheel intersects magnetic strip
      const int upper = normal + threshold; // Upper threshold for magnetic strip detection
      const int lower = normal - threshold; // Lower threshold for magnetic strip detection
      const int turn_time = 10; // Variale for how long a turn should be completed after detection
      startTime = millis();
      while (true)
      {
        // Turn LEDs WHITE if magnetic strip not detected
        digitalWrite(RPin,LOW);
        digitalWrite(BPin,LOW);
        digitalWrite(GPin,LOW);

        // Checks that both wheels are not detecting and drives forward
        while ((upper > hall_L) && (upper > hall_R) && (hall_R > lower) && (hall_L > lower)) 
        {
          front = 10; // Drive forward at speed 10
          if (abs(ultrasonic_dist(distance,5) - desired_distance) < 1) // Check if for wall object
          {
            stop(); // Stop the vehicle at that spot
            bool at_distance = 1;
          }
        }


        stop(); // If magnetic strip detected stop robot


        if (hall_L > upper || hall_L < lower) // If left side detects strip turn left
        {
          left_time = millis(); // Stores time of detection
          left_flag = 1; // Stores that the strip is detected

          // Turns LED Blue after detecting left strip
          digitalWrite(RPin,LOW);
          digitalWrite(BPin,HIGH);
          digitalWrite(GPin,LOW);
          startTime = millis(); // Stores time for turn time
          while (millis() < startTime + turn_time) //turns for a set amount of time
          {
            turn_left(); // Turn left function
          }
          stop(); //Stops after turn function complete
        }
        else 
        {
          left_flag = 0;
        }


        if (hall_R > upper || hall_R < lower)  //If right side detects strip turn right
        {
          right_time = millis(); // stores time of detection
          right_flag = 1; // Stores that the strip is detected

          // Set LED Red if right strip detected
          digitalWrite(RPin,HIGH);
          digitalWrite(BPin,LOW);
          digitalWrite(GPin,LOW);

          startTime = millis(); //Stores time for turn time
          while (millis() < startTime + turn_time) //turns for a set amount of time
          {
            turn_right(); // Turn right function
          }
          stop(); // Stop after turn time complete
        }
        else {
        right_flag = 0;
        }


        // following logic is for when both wheels detect the strip meaning the robot is orthogonal to the strip length

        if((left_flag == 1) && (right_flag == 1))
        {
          flag_buzzer = true; // Ring buzzer to show that logic is active

          if (right_time > left_time) // Checks which wheel hit it first to determine turn direction
          {
            stop(); // Stops motion
            startTime = millis();
            while(millis() < startTime + 1500) // Spin Right for 1.5 seconds
            {
              spinr = 1;
            }
            stop(); // Stop
          }
          if (right_time < left_time) // Checks which wheel hit it first to determine turn direction
          {
            stop();
            startTime = millis();
            while(millis() < startTime + 1500) // Spin Left for 1.5 seconds
            {
              spinl = 1;
            }
            stop(); // Stop
          }
        }




        //ultrasonic distance measurements

        Serial.println(distance);
        //while the distance error is in between 1 cm
        while(abs(ultrasonic_dist(distance,5) - desired_distance) < 1)
        {
            stop();
        }

      }
    }
  }
}
