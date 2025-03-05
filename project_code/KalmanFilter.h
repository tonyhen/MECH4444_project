/*
  Code adapted from:
  Adeept Kalman Filter library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
*/

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_
#include <Arduino.h>

class KalmanFilter
{
public:
	void roll_angle(float angle_m, float gyro_m,float dt,float K1);
	void kalmanFilter(float yk, float wx,float dt,float Q_angle,float Q_gyro,float R_angle);
	void calculate_angle(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,
	                     float Q_angle,float Q_gyro,float R_angle,float K1);
  float Gyro_x,Gyro_y,Gyro_z;
  float angle;
  float angle_dot;                               
  float rollangle;
private:
  float theta_k_k_1, theta_k_k = 0.0;
	float beta_k_k_1,  beta_k_k  = 0.0;
  float y_tilda;
  float P_k_k_1[2][2] = {{ 1, 0 }, { 0, 1 }};
  float P_k_k[2][2]   = {{ 0, 0 }, { 0, 0 }};
  float S;
  float K[2] = {0,0};
};
#endif
