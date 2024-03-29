#include "Arduino.h"
#include "config.h"

extern unsigned long step_count;
int delay_sec = 10;//(int)(1000 / (steps_per_rev * split * rev_per_sec));

// int isMaxHeight(unsigned long step_count)
// {
//     if(step_count != (MAX_REV * split *steps_per_rev))
//     {
//       return 0;
//     }
//     else
//     {
//       Serial.println("is Max Height.");
//       step_count = MAX_REV * split *steps_per_rev;
//       return 1;
//     }
// }

// int isMinHeight(unsigned long step_count)
// {
//     if(step_count != 0)
//     {
//       return 0;
//     }
//     else
//     {
//       Serial.println("is Max Height.");
//       step_count = 0;
//       return 1;
//     }
// }

// int isDropHeight(unsigned long step_count)
// {
//     if(step_count == (DROP_REV * split *steps_per_rev))
//     {
//       return 1;
//     }
//     else
//     {
//       return 0;
//     }
// }

void stepper_up()
{
    Serial.println("stepper_up");
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);  
}  

void stepper_stop()
{
  digitalWrite(stepPin, LOW);
}

void stepper_down()
{
    Serial.println("stepper_down");
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);   
}  