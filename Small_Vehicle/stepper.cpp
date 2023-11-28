#include "Arduino.h"
#include "config.h"

int delay_sec = (int)(1000 / (steps_per_rev * split * rev_per_sec));

void stepper_up()
{
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);   
}  

void stepper_down()
{
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);   
}  