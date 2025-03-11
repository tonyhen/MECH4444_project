#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Declare external global variables (since they are defined in another file)
extern volatile uint8_t front, back;
extern volatile bool turnl, turnr, spinl, spinr, spinl, spinr;

// Function prototypes
float yaw_correction(float reference, float yaw_angle);
void turn_left(bool *turnl, bool *turnr);
void turn_right(bool *turnl, bool *turnr);
float pathfinding(float hall_R,float hall_L);
void stop();  // Declare stop() before defining it

// Function to adjust yaw error
float yaw_correction(float desired_heading, float yaw_angle) {
    int error = desired_heading - yaw_angle;  // Adjust error based on reference
    if (error > 180) 
    {
        error -= 360;
    } 
    else if (error < -180) 
    {  
        error += 360;
    }

    return error;  // Return corrected yaw error
}


void turn_left(float error)
{
    if (abs(error) > 10)
    {
    spinr = 0;
    spinl = 1;
    }
    else
    {
    spinr = 0;
    spinl = 0;
    turnr = 0;
    turnl = 1;
    }
}


void turn_right(float error)
{
    if (abs(error) > 10)
    {
    spinr = 1;
    spinl = 0;
    }
    else
    {
    spinr = 0;
    spinl = 0;
    turnr = 1;
    turnl = 0;
    }
}

// Function to stop movement
void stop()
{
    front = 0;
    back = 0;
    turnl = 0;
    turnr = 0;
    spinl = 0;
    spinr = 0;
}

//new path finding code
float pathfinding(float hall_L,float hall_R)
{
    if (hall_R > 140 || hall_R < 50)
    {
        stop();
        desired_heading = yaw_angle;
        unsigned long time_begin = millis();
        while(time_begin + 1000 < mills())
        {
            back = 1;
            digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW);
        }
        stop();
    }
    else if (hall_L > 140 || hall_L < 50)
    {
        stop();
        desired_heading = yaw_angle;
        unsigned long time_begin = millis();
        while(time_begin + 1000 < millis())
        {
            back = 1;
            digitalWrite(RPin,LOW);digitalWrite(GPin,LOW);digitalWrite(BPin,HIGH);
        }
        stop();


    }
    return desired_heading;
}


// PID Pathfinding Function
/*
float pathfinding(float hall_L, float hall_R, float desired_heading)
{
  // PID Constants (Tune these experimentally)
  float Kp = 1.5;  // Proportional gain
  float Ki = 0.05; // Integral gain
  float Kd = 0.8;  // Derivative gain

  float previous_error = 0;
  float integral = 0;
  float target_value;
  float drive_error = (hall_R - hall_L);  // Difference between left and right sensors

    // Corner Detection: If both sensors deviate too much, adjust target dynamically
    if (hall_L < 80 || hall_L > 140 || hall_R < 80 || hall_R > 140)
    {
        // Increase or decrease target value based on trend
        target_value = (hall_L + hall_R) / 2;
    }

    integral += drive_error;  // Accumulate error
    float derivative = drive_error - previous_error;  // Rate of change of error

    float correction = (Kp * drive_error) + (Ki * integral) + (Kd * derivative);  // PID output

    desired_heading += correction;  // Adjust heading
    previous_error = drive_error;  // Store error for next loop

    return desired_heading;
}
*/
#endif