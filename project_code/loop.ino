/* main high-level control loop */

/* Code by Team 6/7 of Mech4444.
By Mackenzie Hallet, Arshvir Singh, Ashley Leonard, Gordon Henderson

This code will be the primary drive loop for the self balancing robot to follow the magnetic strip track
and will also stop at 10cm from a wall.

Referenced functions can also be found in the functions.h file

Changes to the code outside of this file and function file:

- Added a wheel_bias to the balance.cpp pwm2 calculation
- Added wheel_bias variable to project_code.ino
- PID turn integral constrain was increased to +- 2000
*/




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

  // End of Dr.Bauer's initialization and balance code



  {
    while (true) {
      long int startMillis = millis();  // time since Arduino turned on (ms)
      while (millis() < startMillis + 100)
      {
        // wait for .1 second
      }
      const int normal = 112; // Nominal hall effect sensor values
      const int threshold = 8; // Deadzone to determine when a wheel intersects magnetic strip
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
        while ((upper > hall_L) && (upper > hall_R) && (hall_R > lower) && (hall_L > lower) && (at_distance == 0))
        {
          front = 20; // Drive forward at speed 10
          Serial.println(abs(distance));
          // Serial.print("Left Hall: ");
          // Serial.println(hall_L);
          // Serial.print("Right Hall: ");
          // Serial.println(hall_R);

          if (abs(distance - desired_distance) < 5) // Check if for wall object
          {
            stop(); // Stop the vehicle at that spot
            at_distance = 1; // Change flag to distance keeping
            flag_buzzer = 1; // Ring Buzzer 
            digitalWrite(RPin,HIGH);
            digitalWrite(BPin,HIGH);
            digitalWrite(GPin,HIGH);
            break; // Break out of while loop
          }
        }

        stop(); // If magnetic strip detected stop robot

        if (hall_L > upper || hall_L < lower) // If left side detects strip turn left
        {
           // Stores time of detection
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
          left_flag = 0; // changes flag to not detecting strip
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
        right_flag = 0; // changes flag to not detecting strip
        }

      //Serial.println(distance);
      
      // following logic is for when both wheels detect the strip meaning the robot is orthogonal to the strip length
      if(left_flag == 1 && right_flag == 1)
       {
        flag_buzzer = true; // Ring buzzer

        if(gz < 0) // Checks if the gyroscope is negative 
        {
          stop(); // Stops motion
          startTime = millis();
          while(millis() < startTime + 1500) // Spin Right for 1.5 seconds
          {
            spinl = 1;
            //Serial.println("Turning Right");
            digitalWrite(RPin,LOW);
            digitalWrite(GPin,HIGH);
            digitalWrite(BPin,HIGH);
          }
          stop(); // Stop
        }


        else if(gz > 0) // Check if gyroscope is positive 
        {
          stop(); // Stops motion
          startTime = millis();
          while(millis() < startTime + 1500) // Spin Right for 1.5 seconds
          {
            spinr = 1;
            //Serial.println("Turning Right");
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

        // Check if the robot is at the object 
        if (abs(distance - desired_distance) < 5) // Check if for wall object
        {
          stop(); // Stop the vehicle at that spot
          at_distance = 1; // Changes to distance keeping behavior
          //flag_buzzer = 1;

          // Turn off 
          digitalWrite(RPin,HIGH);
          digitalWrite(BPin,HIGH);
          digitalWrite(GPin,HIGH);
        }

        desired_heading = yaw_angle; // Desired heading to keep when distance keeping at wall
        //ultrasonic distance measurements
        while (at_distance == 1) // Flag to start distance keeping
        {
        
        //while the distance error is in between 1 cm

        error = yaw_correction(desired_heading, yaw_angle);

        if (error < 0)
        {
          turn_left();
        }

        else if (error > 0)
        {
          turn_right();
        }

        else
        {
          stop();
        }

        if (distance - desired_distance > 1) // If too far move forward
        //if (ultrasonic_dist(distance,5) - desired_distance > 1) // If too far move forward
        {
            digitalWrite(GPin,LOW);
            back = 0;
            front = 5;
        }

        else if(distance - desired_distance < -1) // IF too close reverse
        {
            digitalWrite(BPin,LOW);
            front = 0;
            back = 5;
        }
        else
        {
          stop();
        }

        }
      }
    }
  }
}
