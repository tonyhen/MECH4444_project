#ifndef INTERRUPTS_H
#define INTERRUPTS_H
#include <Arduino.h>

extern volatile int count_right; // variables defined in AdeeptSelfBalancingRobotCode_rjb
extern volatile int count_left;  // variables defined in AdeeptSelfBalancingRobotCode_rjb

void Count_left();
void Count_right();

#endif
