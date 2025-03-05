/* setup.ino is concatinated to the main file */
// initialization and calibration
void setup() 
{
  // TB6612FNGN drive module control signal initialization
  pinMode(IN1M, OUTPUT);// control the direction of the motor 1 (left motor): 
                        // In1M = 0, In2M = 1 (01) for moving forward rotation, (10) for moving backward rotation
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);// control the direction of the motor 2 (right motor):
                        // In3M = 0, In4M = 1 (01) for moving forward rotation, (10) for moving backward rotation
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);// left motor PWM
  pinMode(PWMB, OUTPUT);// right motor PWM

  //Initialize the motor drive module
  digitalWrite(IN1M, 0); 
  digitalWrite(IN2M, 1); // left  motor forward rotation
  digitalWrite(IN3M, 0); 
  digitalWrite(IN4M, 1); // right motor forward rotation
  analogWrite(PWMA, 0);  // PWM with 0 duty cycle - left  motor not rotating
  analogWrite(PWMB, 0);  // PWM with 0 duty cycle - right motor not rotating

  pinMode(PinA_left, INPUT);  // left encoder count pin
  pinMode(PinA_right, INPUT); // right encoder count pin

  pinMode(RPin, OUTPUT);      // set RPin to output mode (A0)
  pinMode(GPin, OUTPUT);      // set GPin to output mode (A1)
  pinMode(BPin, OUTPUT);      // set BPin to output mode (D8)
  pinMode(hall_R_pin, INPUT); // set RHS hall Pin to input mode (A2)
  pinMode(hall_L_pin, INPUT); // set LHS hall Pin to input mode (A3)

  digitalWrite(RPin,HIGH);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW); // turn on blue LED

  // Initialize distance ultrasonic sensor pins
  Dist.begin(echoPin,trigPin); // begin(int echoPin, int trigPin)
  
  // Initialize the I2C bus
  Wire.begin();  
  
  // Turn on the serial port and set the baud rate to 9600
  //Serial.begin(9600);
  Serial.begin(2000000);
  delay(250);
  Serial.println("Initialized the I2C bus...");
  
  // Initialize the MPU6050 IMU
  mpu.initialize();    
  delay(300);
  Serial.println("Initialized the MPU6050...");
  
  // check connection with BNO055 
  if(!bno.begin()) // default is NDOF mode
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  else
  {
    Serial.println("BNO055 connected successfully!");
  }
  delay(250);
  bno.setExtCrystalUse(true); // use external crystal as clock source
                              // likely for better timing accuracy

  // use buzzer to indicate start of magnetometer calibration on BNO055
  digitalWrite(buzzerPin, HIGH);
  delay(10);
  digitalWrite(buzzerPin, LOW);
  
  // calibrate BNO055 before setting up interrupts for balancing
  Serial.println("Calibrating magnetometer, rotate robot until LEDs turn off");
  bno.getCalibration(&sys, &gyro, &accel, &mag); // get calibration status for each sensor
  while(mag < 3) // magnetometers calibrated when mag = 3
  {
    bno.getCalibration(&sys, &gyro, &accel, &mag); // get calibration status for each sensor
    Serial.print(mag); Serial.println("\t");
    //Serial.print(gyro); Serial.print("\t");
    //Serial.print(accel); Serial.println("\t");
    delay(200); 
  }
  Serial.println("BNO055 Magnetometers calibrated for Compass Heading");
  digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,HIGH); // turn on RED LED
  delay(5000); // wait 5s for robot to be placed on ground before starting to balance
  Serial.println("Balancing starts now.");

  // set up interrupt service routines for encoder pulse counting
  attachInterrupt(0, Count_left, CHANGE); // count encoder pulse for left motor, interrupt "0" on Pin2
  attachPinChangeInterrupt(PinA_right, Count_right, CHANGE); // count encoder pulse for right motor
  
  // 5ms timer interrupt setting. Use Timer2. Note: Using timer2 will affect the PWM output of pin3 and pin11.
  MsTimer2::set(5, inter); // trigger the primary interrupt service routine called "inter" every 5ms
  MsTimer2::start();       // start this primary 5ms timer interrupt service routine
  startTime = millis();    // get reference program run start time
}
