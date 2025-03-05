/*
  Code adapted from:
  Adeept Balance 2WD library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
*/

#include "Balance.h"

// this function is called every 40ms and use Proportional Integral (PI)
// forward/backward speed control to create an output signal that controls the speed of the vehicle
// proportional gain kps acts on the speed error to try to regulate the speed to desired setpoint p0 of zero
// integral gain kis acts on the accumulated speed error which can be biased with forward/backward commands

float Balance::speedPiOut(float kps,float kis,int f,int b,float p0)
// INPUTS:
// kps = proportional control gain for forward/backward speed control
// kis = integral     control gain for forward/backward speed control
// f   = forward  speed command (faster forward  = larger positive value)
// b   = backward speed command (faster backward = larger positive value)
// p0  = reference speed for balancing ( = 0 )
// OUTPUTS: 
// output= forward/backward speed PI controller output
{
  float speeds = (pulseleft + pulseright);               // overall vehicle speed over 40ms 
                                                         // if excactly turning on itself then due to difference 
                                                         // in signs the added result would be zero
                                                         // if added result is negative then vehicle moving forward 
                                                         // (but could be also slightly turning)
                       
  pulseright = pulseleft = 0;                            // reset pulseright and pulseleft values

  speeds_filterold *= 0.7;                               // complementary filtering to estimate current speed with 
                                                         // weight of 0.7 on previous speed
  float speeds_filter = speeds_filterold + speeds * 0.3; // and 0.3 on current speed
  speeds_filterold = speeds_filter;

  positions += speeds_filter;  // accumulated speed used to accumulate the speed error in the integral controller

  // adds a bias to the accumulated speed which the integral controller will act on to bias the control output
  // to make the robot move forward or backward
  positions += f;    // positive f = move forward
  positions -= b;    // positive b = move backward

  positions = constrain(positions, -1000,1000);   // anti-integral saturation (prevents integral windup) 
                                                  // this value can be adjusted

  float output = kis * (p0 - positions) + kps * (p0 - speeds_filter); // PI forward/backward speed control
                                                                      // proportional kps acts on speed error, 
                                                                      // where desired speed is zero
                                                                      // integrator kis acts on accumulated error

  if(flag1==1) // motors have been commanded to stop so effectively reset the accumulated error in 
               // the integral controller
  {
    positions=0;
  }
  
  return output; // forward/backward speed PI controller output
}

//this function is called every 10ms to coordinate yaw turn/spin commands
float Balance::turnSpin(bool turnleftflag,bool turnrightflag,bool spinleftflag,
                        bool spinrightflag,float kpturn,float kdturn,float Gyroz)
// INPUTS:
// turnlefflag   = turn left variable  ( 1 = turn left,  0 = don't turn left)
// turnrightflag = turn right variable ( 1 = turn right, 0 = don't turn right)
// spinleftflag  = spin left  variable (spin is just a faster turn, 1 = spin left,  0 = don't spin left)
// spinrightflag = spin right variable (spin is just a faster turn, 1 = spin right, 0 = don't spin right)
// kpturn = yaw turn proportional control gain
// kdturn = yaw turn derivative   control gains
// Gyroz  = yaw turn angular rate error (Gyroz=0-gz) where gz is angular rate from gyro
// OUTPUTS: 
// turnoutput = return yaw rotation PD control output
{
  // initialize variables
  float turnspeed = 0.0;
  float rotationratio = 0.0;
  float turnoutput = 0.0;
  
  if (turnleftflag == 1 || turnrightflag == 1 || spinleftflag == 1 || spinrightflag == 1) // turn/spin command issued
  {
    turnspeed = ( pulseright + pulseleft);       // overall vehicle speed over 10ms, note if turning on itself 
                                                 // then turnspeed is zero
                                                 // this value is used to determine how quickly the car should 
                                                 // turn to avoid sharp turns when moving fast
    if (turnspeed < 0.0)       // current speed of the car
    {
      turnspeed = -turnspeed;  // absolute value of car's current speed
    }

    // use spin flag for fast turns, turn flag for slower turns
    if(turnleftflag==1||turnrightflag==1) // set maximum turning rate
    {
      // control maximum turn rate, can increase this if desired
      turnmax= 1.0;
      turnmin=-1.0;
    }

    // use spin flag for fast turns, turn flag for slower turns
    if(spinleftflag==1||spinrightflag==1) // spin rate is set higher than turn rate
    {
      // control maximum spin rate, can increase this if desired
      turnmax= 2.0;
      turnmin=-2.0;
    }
    rotationratio = 55.0 / turnspeed;     // according to the car speed set value of rotation ratio 
                                          // (ie if current speed is high, rotationratio is small
                                          // to avoid sharp turns at high speeds), 55 must be an 
                                          // emperically-determined value
    if (rotationratio < 0.5)rotationratio = 0.5; // minimum rotationratio (slowest turn since current speed 
                                                 // must be high)
    if (rotationratio > 5.0)rotationratio = 5.0; // maximum rotationratio (fastest turn since current speed 
                                                 // must be low)
  }
  else // no turn has been commanded
  {
    rotationratio = 0.5; // minimum rotationratio
    turnspeed = 0.0;
  }

  if (turnleftflag == 1 || spinleftflag == 1)       // according to the direction parameters superimposed
  {
    turnout += rotationratio; // turn/spin left, increase desired turn angle error as long as turn is commanded
  }
  else if (turnrightflag == 1 || spinrightflag == 1) // according to the direction parameters superimposed
  {
    turnout -= rotationratio; // turn/spin right, decrease desired turn angle error (increase in negative direction) 
                              // as long as turn is commanded
  }
  else
  { 
     turnout = 0.0; // reset turnout if not turning
  }

  if (turnout > turnmax) turnout = turnmax;        // most positive desired turn angle error value
  if (turnout < turnmin) turnout = turnmin;        // most negative desired turn angle error value


  // PD controller: kpturn acts on yaw turn angle error, kdturn acts on yaw turn angular 
  // rate error (Gyroz=0-gz) where gz is angular rate from gyro
  turnoutput = -turnout * kpturn - Gyroz * kdturn; // yaw rotation PD control
  return turnoutput; // return yaw rotation PD control output
}

// this function calculates and applies the resulting PWM motor commands to the motor taking into consideration 
// PI speedoutput control, PD rotation output control, and the pitch/roll angles (to shut off motors if angles 
// become too high)
void Balance::pwma(float speedoutput,float rotationoutput,float angle,float rollangle,
                   bool turnleftflag,bool turnrightflag,bool spinleftflag,bool spinrightflag,
                   int f,int b,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB)
// INPUTS:
// speedoutput = forward/backward speed PI control output
// rotationoutput = yaw rotation PD control output
// angle = pitch angle from Kalman filter
// rollangle = roll angle from complementary filter
// turnleftflag  =  turn left variable  ( 1 = turn left,  0 = don't turn left)
// turnrightflag = turn right variable ( 1 = turn right, 0 = don't turn right)
// spinleftflag  = spin left  variable (spin is just a faster turn, 1 = spin left,  0 = don't spin left)
// spinrightflag = spin right variable (spin is just a faster turn, 1 = spin right, 0 = don't spin right)
// f   = forward  speed command (faster forward  = larger positive value)
// b   = backward speed command (faster backward = larger positive value)
// Pin1, Pin2 = left  motor direction control pins
// Pin3, Pin4 = right motor direction control pins
// PinPWMA = left  motor pwm command pin
// PinPWMB = right motor pwm command pin
{
  // results of controllers are superimposed
  pwm1 = -angleoutput - speedoutput - rotationoutput; //Left motor PWM output value, rotation command opposite 
                                                      // to each motor
  pwm2 = -angleoutput - speedoutput + rotationoutput; //Right motor PWM output value

  // pwm amplitude limit
  const uint8_t pwm_max = 100;  // maximum value is 255 but avoid using too large a value to prevent damage to robot
  if (pwm1 > pwm_max) pwm1 = pwm_max;
  if (pwm1 < -pwm_max) pwm1 = -pwm_max;
  if (pwm2 > pwm_max) pwm2 = pwm_max;
  if (pwm2 < -pwm_max) pwm2 = -pwm_max;

  // the pitch angle is too large so stop the motor, (robot no longer balancing well).
  if (angle > 12.0 || angle < -12.0) // CHANGED from above
  {
    pwm1 = 0;
    pwm2 = 0;
    flag1=1; // stop motors
  }
  else
  {
    flag1 = 0; // don't stop motors
  }

  // if roll angle (along longitudinal axis of vehicle) is becoming large and we are not commanding a 
  // turn/spin/forward/backward movement
  if (rollangle > 20.0 || rollangle < -20.0)
  {
    pwm1 = 0;
    pwm2 = 0;
    flag1=1; // stop motors
  }
  else
  {
    flag1 = 0; // don't stop motors
  }

  
  // Comment out the remaining code below to prevent motors from turning
  // motor rotation for left motor
  /*
  if (pwm1 >= 0) // positive pwm1 means move left motor backward
  {
    digitalWrite(Pin1, 1);
    digitalWrite(Pin2, 0);      // motor driver rotates left motor backward
    analogWrite(PinPWMA, pwm1); // motor voltage command (always positive)
  }
  else // pwm1 < 0 // move left motor forward
  {
    digitalWrite(Pin1, 0);
    digitalWrite(Pin2, 1);       // motor driver rotates left motor forward
    analogWrite(PinPWMA, -pwm1); // motor voltage command (note -pwm1 adjust so the argument is always positive)
  }
  
  // motor rotation for right motor
  if (pwm2 >= 0) // positive pwm2 means move right motor backward
  {
    digitalWrite(Pin3, 1);
    digitalWrite(Pin4, 0);      // motor driver rotates right motor backward
    analogWrite(PinPWMB, pwm2); // motor voltage command (always positive)
  } 
  else // negative pwm2 means move right motor forward
  {
    digitalWrite(Pin3, 0);
    digitalWrite(Pin4, 1);       // motor driver rotates right motor forward
    analogWrite(PinPWMB, -pwm2); // motor voltage command (note -pwm2 adjust so the argument is always positive)
  }
  */
}
