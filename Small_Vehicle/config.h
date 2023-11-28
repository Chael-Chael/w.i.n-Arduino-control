#include "Arduino.h"

//A4988 Pin
#define dirPin
#define stepPin
 
#define MS1
#define MS2
#define MS3

//step_motor Pin
#define stepsPerRotation 200
#define pulse 

//servo Pin
#define 


//conveyor Pin
#define motor_en
#define motor_in1
#define motor_in2


void stepper_up(int sp);
void stepper_down(int sp);
