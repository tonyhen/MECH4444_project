#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Declare external global variables (since they are defined in another file)
extern volatile uint8_t front, back;
extern volatile bool turnl, turnr, spinl, spinr, spinl, spinr;

// Function prototypes
float yaw_correction(float reference, float yaw_angle); // Corrects for error outside of range
void turn_left(); // Declare a turn_left function
void turn_right(); //Declare a turn_right function
void stop();  // Declare stop


// Function to adjust yaw error to account for going through 0 or 360 degrees
float yaw_correction(float desired_heading, float yaw_angle) {
    int error = desired_heading - yaw_angle;  // Adjust error based on reference

    if (error > 180)  // If error is more than 180 subtract 360 to account for going out of range
    {
        error -= 360;
    } 
    else if (error < -180) // If error is more than -180 degrees add 360 degrees
    {  
        error += 360;
    }

    return error;  // Return corrected yaw error
}

// Left turn function, sets everything except the turn left flag to 0
void turn_left()
{
    front = 0;
    back = 0;
    spinr = 0;
    spinl = 0;
    turnr = 0;
    turnl = 1;
}

// Right turn function, sets everything except the turn right flag to 0
void turn_right()
{
    front = 0;
    back  = 0;
    spinr = 0;
    spinl = 0;
    turnr = 1;
    turnl = 0;
}
// Function to stop movement
void stop()
{
    front = 0; // Set front flag to 0
    back = 0; // Set back flag to 0
    turnl = 0; // Set left turn flag to 0
    turnr = 0; // Set right turn flag to 0
    spinl = 0; // Set spin left flag to 0
    spinr = 0; // Set spin right flag to 0
}

// Rolling average function for the ultrasonic distance
#endif