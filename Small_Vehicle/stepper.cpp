#include "Arduino.h"
#include "config.h"

int delay_sec = (int)(1000 / (steps_per_rev * split * rev_per_sec));

int isMaxHeight(long step_count)
{
    if(step_count != (MAX_REV * split *steps_per_rev))
    {
      return 0;
    }
    else
    {
      Serial.println("is Max Height.");
      step_count = MAX_REV * split *steps_per_rev;
      return 1;
    }
}

int isMinHeight(long step_count)
{
    if(step_count != 0)
    {
      return 0;
    }
    else
    {
      Serial.println("is Max Height.");
      step_count = 0;
      return 1;
    }
}

int isDropHeight(long step_count)
{
    if(step_count == (DROP_REV * split *steps_per_rev))
    {
      return 1;
    }
    else
    {
      return 0;
    }
}

void stepper_up(long step_count)
{
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);  
    step_count += 2; 
}  

void stepper_down(long step_count)
{
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);   
    step_count -= 2;
}  