/* Interrupt timing 5ms timer interrupt */

void inter()
{
  sei();         // enable interupts                                
  countpulse();                                     // calculate accumulated pulses in 5ms
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     // I2C get MPU6050 six axis data ax ay az gx gy gz
  
  // calculate pitch angle using Kalman filter, 
  // roll angle using complementary filter, yaw rate
  kalmanfilter.calculate_angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle,K1);  

  //PD angle loop control
  balance_robot.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.angle_dot; 
  speedcc++;
  if (speedcc >= 8) // 5ms x 8 = 40ms into the speed loop control (25 times/sec)
  {
    Outputs = balance_robot.speedPiOut(kp_speed,ki_speed,front,back,setp0); // forward/backward PI speed controller
    speedcc = 0;
      
    // read from Hall effect sensors
    hall_L = analogRead(hall_L_pin);
    hall_R = analogRead(hall_R_pin);
    Serial.print(hall_R);
    Serial.print(" ");
    Serial.println(hall_L);
 
    // check if buzzer is require to make a short beep
    if (flag_buzzer == true)
    {
      digitalWrite(buzzerPin, HIGH);
      flag_buzzer = false;
    }
    else
    {
      digitalWrite(buzzerPin, LOW);
    }
  }

  turncount++;
  if (turncount > 2) // 10ms into the yaw turn control loop (100 times/sec)
  {                                
      // yaw turn controller
      turnoutput = balance_robot.turnSpin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);  
      turncount = 0; // reset turncount after 10ms
  } 
  
  // every 5ms determine motor outputs
  balance_robot.pwma(Outputs,turnoutput,kalmanfilter.angle,kalmanfilter.rollangle,turnl,turnr,spinl,spinr,
                     front,back,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB); // motor commands   

  detTime++;
  if(detTime>=50) // 5ms x 50 = 250ms, read distance, heading, and buzzer flag every 0.25s (4 times/sec)
  { 
    distance = Dist.getDistanceCentimeter(); // read distance in centimeters
    detTime = 0; // reset detTime
    
    // Get heading data from BNO055
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    yaw_angle = event.orientation.x;  // yaw angle in degrees
  }
}
