/***********************************************************
Self-Balancing Robot Code Starting Point
R. Bauer
Feb 24, 2024
% This version added pin8 so all RGB colors are available on RGB LED
Adapted from AdeeptSelfBalancingRobotCode.ino 
https://www.adrive.com/public/97GXSs/Adeept_SelfBalancingRobotKit-V1.0.zip 
www.adeept.com, accessed November 17, 2022 

Operation: (you can connect to the Serial Monitor for more initialization details for debugging)
1. Carefully hold robot upright.
2. Turn robot on (move switch to the left at the back of the robot).
3. Wait for short beep from buzzer.
4. Calibrate BNO055 magnetometer by carefully rotating robot in the air until LEDs turn off.
5. Once LEDs turn off, you have 5 seconds to carefully place the robot on level ground (on floor, not desk)
   before it starts balancing.

Refer to the loop.ino code to determine what the robot will do once it is balancing
***********************************************************/
// libraries in local directory
#include "Balance.h"         // balance controller, adapted from original Adeept_Balance2WD Adeept Library
#include "KalmanFilter.h"    // Kalman filter to estimate angle, adapted from original Adeept_KalmanFilter 
                             // Adeept Library
#include "functions.h"
// need following libraries from original Adeept Library to be installed in /Documents/Arduino/libraries
#include "Adeept_Distance.h" // distance sensor functions
#include "PinChangeInt.h"    // PinChangeInt-master, allows pins other than 2 and 3 to initiate an interrupt 
                             // on Arduino UNO
#include "MsTimer2.h"        // allows user to run an interrupt function using Timer2 on Arduino with 1ms resolution
#include "I2Cdev.h"          // I2Cdev1, I2C read/write communication functions
#include "MPU6050_6Axis_MotionApps20.h" // read MPU6050 gyro and accelerometer data

// the following Arduino Library should already be available under menu: Sketch->Include Library
#include "Wire.h" // enables communication via I2C on Arduino

// need the following libraries to read from BNO055
#include <Adafruit_Sensor.h>  // Adafruit_Unified_Sensor Library driver
#include <Adafruit_BNO055.h>  // allows user to communicate with the BNO055 IMU
#include <utility/imumaths.h> // within the Adafruit_BNO055 directory to define matrix, vector and quaternion

#include "interrupts.h" // interrupt functions, from original code in AdeeptSelfBalancingRobotCode.ino

// Define Global Constants

// pin 0 and 1 used for serial communication for debugging

// motor encoder pulse counting
const byte PinA_left  = 2; //Interrupt "0" is on Pin 2 for UNO
const byte PinA_right = 4; //Interrupt "1" set to Pin 4 using PinChangeInt library

// ultrasonic distance measurement
const byte trigPin = 3; // trigger is on Pin 3
const byte echoPin = 5; // echo is on Pin 5

// TB6612FNG Motor Drive module control signal, https://www.sparkfun.com/products/14451
// STBY pin on TB6612FNG motor driver board is internally pulled high on 
// Jon MacDonald's shield so motor is always out of standby mode
const byte IN1M = 7;  // left  motor direction control pin
const byte IN2M = 6;  // left  motor direction control pin
                      // In1M = 0, In2M = 1 (01) for moving forward  rotation
                      // In1M = 1, In2M = 0 (10) for moving backward rotation
const byte IN3M = 13; // right motor direction control pin
const byte IN4M = 12; // right motor direction control pin
                      // In3M = 0, In4M = 1 (01) for moving forward  rotation
                      // In3M = 1, In4M = 0 (10) for moving backward rotation
const byte PWMA = 9;  // left  motor pwm command pin
const byte PWMB = 10; // right motor pwm command pin

const byte buzzerPin = 11;  // buzzer is on Pin 11

// analog pins, A4 and A5 are used by SCA and SCL to transfer data via I2C

const byte RPin = A0; // RGB LED Pin R (red wire),    HIGH = OFF, LOW = ON
const byte GPin = A1; // RGB LED Pin G (yellow wire), HIGH = OFF, LOW = ON
const byte BPin = 8;  // RGB LED Pin B (green wire),  HIGH = OFF, LOW = ON

const byte hall_L_pin = A2; // LHS Hall effect sensor pin
const byte hall_R_pin = A3; // RHS Hall effect sensor pin

// Instantiate Global Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // instantiate an MPU6050 object with the object name 
                                                 // mpu (Motion Processing Unit)
                                                 // I2C device id = 55, address is 0x28;
// read I2C from BNO055
// connections:
// BNO055    Arduino
// Vin    to 5V  RED
// GND    to GND BLACK
// SDA    to A4  YELLOW (Serial DAta)
// SCL    to A5  GREEN (Serial CLock)

MPU6050 mpu; // instantiate an MPU6050 object with the object name mpu (Motion Processing Unit)

Balance balance_robot;    // instantiate a balance object with the object name balance_robot
KalmanFilter kalmanfilter;// instantiate a KalmanFilter object with the object name kalmanfilter
Adeept_Distance Dist;     // instantiate a distance object with the object name Dist

// Define Global Variables

// variables used to store BNO055 calibration status
uint8_t sys = 0, gyro = 0, accel = 0, mag = 0; // or can use "byte" == unsigned integer of length 8 bits

// variables used for determining heading with BNO055
volatile float yaw_angle = 0.0; // yaw angle in degrees

int desired_distance = 0; // desired distance to travel

int16_t ax, ay, az, gx, gy, gz; // MPU6050 accelerometer and gyroscope readings, 16-bit integer

/***************** Hall sensor *******************/
volatile int hall_R = 0, hall_L = 0; // initialize RHS and LHS Hall effect sensor readings

bool flag_wait = true;    // wait at the beginning to allow robot to find its balance on level ground
volatile bool flag_buzzer = false; // enable buzzer to turn on/off (currently used within the 0.5 s update ISR)

float kp = 30,  kd = 0.58;                               // pitch angle PD control gains
float kp_turn = 28, kd_turn = 0.29;                      // yaw turn PD control gains
float kp_speed = 3.6, ki_speed = 0.1058;                 // forward/backward speed PD control gains 
float Outputs = 0;                                       // forward/backward speed PI controller output
float setp0 = 0;                                         // reference speed for balancing ( = 0 )

/********************Angle data*********************/
float K1 = 0.05;     // the weight of the accelerometer in data fusion using complementary filter for roll angle
float angle0 = 0.00; // mechanical balance angle (0 degrees)

/***************Kalman_Filter*********************/
float timeChange = 5;          // filter method sampling time interval in milliseconds
float dt = timeChange * 0.001; // the value of dt is the filter sampling time in seconds
float Q_angle = 0.2;  // process Gaussian noise covariance for angular rate plus 
                      // measurement Gaussian noise covariance for measured angular rate 
                      // from gyroscope
float Q_gyro = 1.0;   // process Gaussian noise covariance for angular rate bias
float R_angle = 0.5; // measurement Gaussian noise covariance for measured angle from accelerometer

/******************* Speed count ************/
uint8_t speedcc = 0;               // used to control how often the speed control loop is executed

// Use the volatile long type to ensure that the value is valid for external interrupt pulse count values used 
// in other functions
volatile int count_right = 0; // encoder count of right motor
volatile int count_left = 0;  // encoder count of left motor

/***********************Pulse calculation*****************************/
// right and left encoder counts used to estimate speed
int rpulse = 0; 
int lpulse = 0;

/********************Distance travelled along ground variables*********/
volatile int rdistance = 0; // used to measure distance travelled (right wheel)
volatile int ldistance = 0; // used to measure distance travelled (left wheel)

/********************Turn parameters for rotation**********************/
uint8_t turncount = 0;    // used to control how often the yaw turn control loop is executed
float  turnoutput = 0; // yaw turn controller output 

/****************Motion variables*******************/
volatile uint8_t front = 0; // forward variable  (faster forward  = larger positive value)
volatile uint8_t back  = 0; // backward variable (faster backward = larger positive value)
volatile bool    turnl = 0; // turn left variable  ( 1 = turn left,  0 = don't turn left)
volatile bool    turnr = 0; // turn right variable ( 1 = turn right, 0 = don't turn right)
volatile bool    spinl = 0; // spin left  variable (spin is just a faster turn, 1 = spin left,  0 = don't spin left)
volatile bool    spinr = 0; // spin right variable (spin is just a faster turn, 1 = spin right, 0 = don't spin right)

/***************Ultrasonic distance sensor******************/
int distance=100; // cm, set initial distance to be far so does not beep when checking 
                   // for object in front (distance < x)
uint8_t detTime=0;     // used to control how often the ultrasonic sensor is employed

/****************Time***************************************/
unsigned long currentMillis; // keep track of how long program has been running
unsigned long startTime;     // keep track of how long program has been running
int wheel_bias = 9; // Value to preferentially 



