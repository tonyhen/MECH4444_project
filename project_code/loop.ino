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
  float error = 30;
  float ROT; // Variable to hold the amount of rotation in degrees
  float desired_heading; // Variable to hold the desired heading in degrees
  bool left_flag = 0; //Flag for error correcting spin function left wheel
  bool right_flag = 0; // Flag for error correcting spin function right wheel
  const int desired_distance = 10; //desired distance in cm
  bool at_distance = 0; //Flag to enable the distance keeping routine

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



  {
    while (true) {
      long int startMillis = millis();  // time since Arduino turned on (ms)
      while (millis() < startMillis + 100)
      {
        // wait for .1 second
      }

  //####################################################################################################################
  // End of Dr.Bauer's initialization and balance code



      const int normal = 112; // Nominal hall effect sensor values
      const int threshold = 8; // Deadzone to determine when a wheel intersects magnetic strip
      const int upper = normal + threshold; // Upper threshold for magnetic strip detection
      const int lower = normal - threshold; // Lower threshold for magnetic strip detection
      const int turn_time = 10; // Variale for how long a turn should be completed after detection

      while (true)
      {
        // Turn LEDs WHITE if magnetic strip not detected
        digitalWrite(RPin,LOW);
        digitalWrite(BPin,LOW);
        digitalWrite(GPin,LOW);

        // Checks that both wheels are not detecting and drives forward
        while ((upper > hall_L) && (upper > hall_R) && (hall_R > lower) && (hall_L > lower) && (at_distance == 0))
        {
          front = 18; // Drive forward at speed 10

          // Debugging Code
          //Serial.println(abs(distance - desired_distance));
          // Serial.print("Left Hall: ");
          // Serial.println(hall_L);
          // Serial.print("Right Hall: ");
          // Serial.println(hall_R);

          if (abs(distance - desired_distance) < 8) // Check for wall object
          {
            stop(); // Stop the vehicle at that spot
            at_distance = 1; // Flagged to start position keeping
            flag_buzzer = 1; // Notify User

            // Change LED colour
            digitalWrite(RPin,HIGH);
            digitalWrite(BPin,HIGH);
            digitalWrite(GPin,HIGH);
            break;
          }
        }

        stop(); // If magnetic strip detected stop robot

        if (hall_L > upper || hall_L < lower) // If left side detects strip turn left
        {
          left_flag = 1; // Stores that the strip is detected
          // Serial.print("Left Time: ");
          // Serial.println(left_time);

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
          left_flag = 0; // Wheel is not detecting the strip
        }

        if (hall_R > upper || hall_R < lower)  //If right side detects strip turn right
        {
          right_flag = 1; // Stores that the strip is detected
          // Serial.print("Right Time: ");
          // Serial.println(right_time);

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
        else
        {
        right_flag = 0; // Wheel is not detecting the strip
        }




      // following logic is for when both wheels detect the strip meaning the robot is orthogonal to the strip length
      // uses the sign of the gyroscope to determine the direction to spin
      if(left_flag == 1 && right_flag == 1) // Both wheels flagged and in
       {
        flag_buzzer = true; // announce that spin is active

        if(gz < 0) // if gyro is negative spin left
        {
          stop(); // Stops motion
          startTime = millis();
          while(millis() < startTime + 1500) // Spin Right for 1.5 seconds
          {
            spinl = 1;
            //change leds for left turn
            digitalWrite(RPin,LOW);
            digitalWrite(GPin,HIGH);
            digitalWrite(BPin,HIGH);
          }
          stop(); // Stop
        }


        else if(gz > 0) // if gyro is positive spin right
        {
          stop(); // Stops motion
          startTime = millis();
          while(millis() < startTime + 1500) // Spin Right for 1.5 seconds
          {
            spinr = 1;

            // change leds for right turn
            //Serial.println("Turning Right"); // debugging printing
            digitalWrite(RPin,HIGH);
            digitalWrite(GPin,HIGH);
            digitalWrite(BPin,LOW);
          }
          stop(); // Stop
        }
       }
        // Reset flags and times 
        left_flag = 0;
        right_flag = 0;

        desired_heading = yaw_angle; // Desired heading to keep when distance keeping at wall
        //ultrasonic distance measurements
        while (at_distance == 1) // Flag to start distance keeping
        {
        error = yaw_correction(desired_heading, yaw_angle); //correct heading for end of scope

        if (error < 0) // if error is negative turn left
        {
          turn_left();
        }

        else if (error > 0) // if error is positive turn right
        {
          turn_right();
        }

        else // if no error stop
        {
          stop();
        }

        if (distance - desired_distance > 1) // If too far move forward
        {
            digitalWrite(GPin,LOW); // change colour for too far away
            back = 0;
            front = 5; // move forward
        }

        else if(distance - desired_distance < -1) // IF too close reverse
        {
            digitalWrite(BPin,LOW); // change led colour for too close
            front = 0;
            back = 5; // backup
        }
        else
        {
          stop(); // if in threshold stop
        }

        }
      }
    }
  }
}
