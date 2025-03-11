/*
  Code adapted from:
  Adeept Balance 2WD library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
*/

#ifndef BALANCE_H_
#define BALANCE_H_
#include <Arduino.h>

class Balance
{
public:
  float speedPiOut(float kps,float kis,int f,int b,float p0);
  float turnSpin(bool turnleftflag,bool turnrightflag,bool spinleftflag,bool spinrightflag,float kpturn,
                 float kdturn,float Gyroz);
  void pwma(float speedoutput,float rotationoutput,float angle,float angle6,bool turnleftflag,bool turnrightflag,
            bool spinleftflag,bool spinrightflag, int f,int b,
            int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB);
	int pulseright = 0;
	int pulseleft = 0;
	float angleoutput=0, pwm1 = 0, pwm2 = 0;
  
private:
	float speeds_filterold;  // speed complementary filtering
	float positions;         // speed error
    float turnmax = 0;       // Rotate the output amplitude
	float turnmin = 0;       // Rotate the output amplitude
	float turnout = 0;       // yaw turn angle error
	bool  flag1 = 0;         // motor stop flag
};
#endif
