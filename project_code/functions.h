#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Declare external global variables (since they are defined in another file)
extern volatile uint8_t front, back;
extern volatile bool turnl, turnr, spinl, spinr;

// Function prototypes
float yaw_correction(float error, float reference);
void turn_left(bool *turnl, bool *turnr);
void turn_right(bool *turnl, bool *turnr);
void stop();  // Declare stop() before defining it

// Function to adjust yaw error
float yaw_correction(float error, float desired_heading) {
    error = error - desired_heading;  // Adjust error based on reference
    if (error > 180) {
        error -= 360;
    } 
    else if (error < -180) {  
        error += 360;
    }

    return error;  // Return corrected yaw error
}

// Function to turn left using pointers
void turn_left()
{
    turnr = 1;
    turnl = 0;
}

// Function to turn right using pointers
void turn_right()
{
    turnr = 0;
    turnl = 1;
}

// Function to stop movement
void stop() {
    front = 0;
    back = 0;
    turnl = 0;
    turnr = 0;
    spinl = 0;
    spinr = 0;
}

#endif