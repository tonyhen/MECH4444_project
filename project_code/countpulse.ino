/* Encoder pulse calculation runs every 5ms */

void countpulse()
{
  lpulse = count_left;  // left encoder count value after 5ms (used as a measure of current wheel rotational speed)
                        // which we will attach a sign to shortly to indicate current forward/backwards speed
  rpulse = count_right; // right encoder count value after 5ms (used as a measure of current wheel rotational speed)
                        // which we will attach a sign to shortly to indicate current forward/backwards speed
  
  // reset encoder count values
  count_left = 0;   
  count_right = 0;

  // pwm1 = left motor PWM (motor voltage) output value between -255 and +255 calculated in pwma function of 
  //        Balance2WD library
  // pwm2 = right motor PWM (motor voltage) output value between -255 and +255 calculated in pwma function of 
  //        Balance2WD library
  // if pwm1 or pwm2 is positive then motor drives associated wheel backwards
  // if pwm1 or pwm2 is negative then motor drives associated wheel forwards
  if ((balance_robot.pwm1 < 0) && (balance_robot.pwm2 < 0)) // moving straight forwards
  {
    rpulse = -rpulse;  // set current right motor speed to negative (moving forwards)
    lpulse = -lpulse;  // set current left motor speed to negative (moving forwards)
  }
  else if ((balance_robot.pwm1 > 0) && (balance_robot.pwm2 > 0)) // moving straight backwards
  {
    rpulse = rpulse; // set current right motor speed to positive
    lpulse = lpulse; // set current left motor speed to positive
  }
  else if ((balance_robot.pwm1 < 0) && (balance_robot.pwm2 > 0)) // left moving forward, right moving backwards, 
                                                                 // rotating to the right
  {
    rpulse =  rpulse; // set current right motor speed to positive
    lpulse = -lpulse; // set current left motor speed to negative
  }
  else if ((balance_robot.pwm1 > 0) && (balance_robot.pwm2 < 0)) // left moving backwards, right moving forward, 
                                                                 // rotating to the left
  {
    rpulse = -rpulse; // set current right motor speed to negative
    lpulse =  lpulse; // set current left motor speed to positive
  }
  
  // record accumulated pulses over this 5ms time window, these values accumulate 8 times before they are used
  // by speedPiOut() (since countpulse() is called every 5ms, while speedPiOut() is called every 8x5ms = 40ms
  balance_robot.pulseright += rpulse; // overall speed over 40ms, note pulseright is reset every 40ms by speedPiOut()
  balance_robot.pulseleft  += lpulse; // overall speed over 40ms, note pulseleft  is reset every 40ms by speedPiOut()

  // record accumulated pulses for measuring distance travelled along the ground 
  rdistance -= rpulse;  // rpulse is negative if moving forwards
  ldistance -= lpulse;  // lpulse is negative if moving forwards
}
