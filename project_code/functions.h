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
float ultrasonic_dist(int dist,int window_size);
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
    back = 2;
    }
    else
    {
    back = 0;
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
    back = 0;
    }
    else
    {
    back  = 0;
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

float ultrasonic_dist(int dist, int window_size)
{
    int distance_readings[window_size];
    float sum_dist = 0;
    for(int i=0;i<window_size;i++)
    {
        distance_readings[i] = dist;
        sum_dist += distance_readings[i];
    }
    float avg_dist = sum_dist/window_size;
    return avg_dist;
}


#endif