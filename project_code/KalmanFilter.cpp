/*
  Code adapted from:
  Adeept Kalman Filter library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
*/

#include "KalmanFilter.h"

///////////////////////////// Calculate Angles /////////////////////////////////
void KalmanFilter::calculate_angle(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
                                   float dt,float Q_angle,float Q_gyro, float R_angle,float K1)
// INPUTS:
// ax,ay,az = MPU6050 accelerometer x-, y- and z-axis readings
// gx,gy,gz = MPU6050 gyroscope x-, y- and z-axis readings
// dt = filter sampling time in seconds
// Q_angle = process Gaussian noise covariance for angular rate
// Q_gyro  = process Gaussian noise covariance for angular rate bias
// R_angle = measurement Gaussian noise covariance for measured angle from accelerometer
// K1 = weight of the accelerometer in data fusion using complementary filter for roll angle
{
  //Balance parameters
  float yk = atan2(ay , az) * 57.3; // use acelerometer data to estimate Pitch (Tilt) angle in degrees 
                                    // (positive if pitching forward)
  Gyro_x = (gx + 293 ) / 131;   // use gyro to estimate pitch angular velocity (deg/s) (positive if pitching forward)
                                // ran example in MPU6050 library called "MPU6050_raw" to see gyro x values 
                                // of -293 when stationary (requiring an offset of +293)
                                // 131 = sensitivity scale factor to convert raw gyro readings to deg/s
                                // (value found in datasheet:
                                // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) 
 
  kalmanFilter(yk, Gyro_x, dt, Q_angle, Q_gyro, R_angle); // Kalman filter to fuse accelerometer angle and 
                                                             // gyro angular rate to estimate Pitch angle (deg)

  // turning yaw rate from z-axis of gyro
  Gyro_z = -(gz-10) / 131;                  // yaw angular rate, gyro angular rate about the vertical z-axis (deg/s), 
                                            // subtracted 10 based on calibration experiment so reads 0 when stationary
                                            // negative because it is actually used as the 
                                            // angular rate error = desired-actual = 0 - gz = -gz
  
  //Roll angle about longitudinal y axis (should be zero if both wheels are at the same ground level)
  float angle_acc = atan2(ax, az) * 180 / PI; // use accelerometer data to estimate Roll angle in degrees 
                                              // (positive if rolling to the right)
  Gyro_y = -(gy+244) / 131.00;          // use gyro to estimate roll angular velocity in deg/s 
                                        // added 244 based on calibration experiment so reads 0 when stationary
                                        // (adjusted so that positive is rolling to the right)
  roll_angle(angle_acc, Gyro_y, dt, K1);// Complementary filter fusing accelerometer and gyro data
                                        // used to estimate "angle6" which is the roll rotation angle about 
                                        // longitudinal axis (positive if rolling to the right)
}

////////////////////////// roll angle complementary filter ////////////////////
void KalmanFilter::roll_angle(float angle_m, float gyro_m,float dt,float K1)
// INPUTS:
// angle_m = roll angle estimate from x- and z-axis of accelerometer
// gyro_m = roll angular velocity estimate from y-axis of gyro
// dt = filter sampling time in seconds
// K1 = weight of the accelerometer in data fusion using complementary filter for roll angle
{
  // complementary filter used to estimate roll angle (angle6) in deg based on a weighting of the roll angle 
  // estimated from accelerometer (angle_m) and the roll angle estimate based on gyroscope readings
  rollangle = K1 * angle_m + (1 - K1) * (rollangle + gyro_m * dt);
}

//////////////////////// pitch angle and pitch angular rate Kalman filter /////////////////////////
void KalmanFilter::kalmanFilter(float yk, float wx,float dt,float Q_angle,float Q_gyro,float R_angle)
// INPUTS:
// yk = pitch angle estimate from y- and z-axis of accelerometer
// wx = pitch angular velocity estimate from x-axis of gyro
// dt = filter sampling time in seconds
// Q_angle = Q_angle = process Gaussian noise covariance for angular rate plus 
                    // measurement Gaussian noise covariance for measured angular rate 
                    // from gyroscope
// Q_gyro  = process Gaussian noise covariance for angular rate bias
// R_angle = measurement Gaussian noise covariance for measured angle from accelerometer
{
  theta_k_k_1 = theta_k_k + (wx-beta_k_k)*dt;  // from Eq (4)  predicted state 
  beta_k_k_1 = beta_k_k; // from Eq (5) predicted state
  
  P_k_k_1[0][0] = P_k_k[0][0] + (Q_angle*dt + P_k_k[1][1]*dt - P_k_k[0][1] - P_k_k[1][0])*dt;
  P_k_k_1[0][1] = P_k_k[0][1] - P_k_k[1][1]*dt;
  P_k_k_1[1][0] = P_k_k[1][0] - P_k_k[1][1]*dt;
  P_k_k_1[1][1] = P_k_k[1][1] + Q_gyro*dt*dt;  // from Eq (7) predicted covariance

  y_tilda = yk-theta_k_k_1; // from Eq (7) innovation

  S = P_k_k_1[0][0] + R_angle;  // from Eq (8) innovation variance

  K[0] = P_k_k_1[0][0] / S;
  K[1] = P_k_k_1[1][0] / S;   // from Eq (9) Kalman gain

  theta_k_k = theta_k_k_1 + K[0] * y_tilda; 
  beta_k_k = beta_k_k_1 + K[1] * y_tilda;   // from Eq (10) updated state estimate (pitch angle and angular rate bias)

  P_k_k[0][0] = P_k_k_1[0][0] - K[0]*P_k_k_1[0][0];
  P_k_k[0][1] = P_k_k_1[0][1] - K[0]*P_k_k_1[0][1];
  P_k_k[1][0] = P_k_k_1[1][0] - K[1]*P_k_k_1[0][0];
  P_k_k[1][1] = P_k_k_1[1][1] - K[1]*P_k_k_1[0][1];  // from Eq (11) updated variance of estimate
  
  // need "angle" and "angle_dot" in other parts of code to balance
  angle = theta_k_k;  // best estimate of pitch angle
  angle_dot = wx - beta_k_k;  // from Eq (2) best estimate of pitch angular velocity
}
