#include "Arduino.h"

//A4988 Pin
#define dirPin
#define stepPin
 
#define MS1
#define MS2
#define MS3

//step_motor Pin
#define steps_per_rev 200
#define max_rev
#define split 1
#define  

//servo Pin
#define 


//conveyor Pin
#define motor_en
#define motor_in1
#define motor_in2

//extern int rev_count;
extern int delay_sec;

void stepper_up();
void stepper_down();
