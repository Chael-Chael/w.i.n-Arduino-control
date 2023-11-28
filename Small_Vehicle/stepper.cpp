#include "Arduino.h"
#include "config.h"

int delay_sec = (int)(1000 / (steps_per_rev * split * rev_per_sec));
int step_count;

int isMaxHeight(int step_count)
{
    if()
}

int isMinHeight(int step_count)
{

}

void stepper_up())
{
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);  
    step_count += 2; 
}  

void stepper_down(int step_count)
{
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec/2);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec/2);   
    step_count += 2;
}  