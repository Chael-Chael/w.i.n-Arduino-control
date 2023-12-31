#include "Arduino.h"

//PS2 pin
#define PS2_DAT_PIN  A11
#define PS2_CMD_PIN  A10
#define PS2_SEL_PIN  A9
#define PS2_CLK_PIN  A8  

//A4988 Pin
#define dirPin 1
#define stepPin 2
 
#define MS1 1
#define MS2 1
#define MS3 1

//servo Pin
#define steerSig 1
#define gripperSig 1

//conveyor Pin
#define convey_en 1
#define convey_in1 1
#define convey_in2 1
#define CONVEY_SPEED 100
//step_motor
#define steps_per_rev 200
#define rev_per_sec 1
#define split 1

//car rotate speed(0 - 255)
#define RSPEED 200
#define LFSPEED 255
//brake acceleration
#define RANGE 10
//tweak delay between movements for better control
#define DELAY 50
//acceleration
#define PLUSACC 5
#define MINUSACC 5
//speed
#define MAX_SPEED 255
//axis threshold for accelerating, must be no more than 125
#define HOLD 100
#define pressures   false  //按键模式
#define rumble      false  //振动模式

//extern int rev_count;
extern int delay_sec;

void stepper_up();
void stepper_down();
void ps2_init();
