#include "interrupts.h"

// Encoder Pulse Counting Interrupt Routines

/*Left encoder pulse called from interrupt*/
void Count_left() 
{
  count_left ++; // increment left encoder pulse counter
} 

/*Right encoder pulse called from interrupt*/
void Count_right() 
{
  count_right ++; // increment right encoder pulse counter
} 
