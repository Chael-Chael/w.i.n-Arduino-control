#include "Arduino.h"
#include <PS2X_lib.h> 

//PS2 pin
#define PS2_DAT_PIN  A8
#define PS2_CMD_PIN  A9
#define PS2_SEL_PIN  A10
#define PS2_CLK_PIN  A11

//A4988 Pin
#define dirPin 10
#define stepPin 11
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
#define MAX_REV 200
#define DROP_REV 50

//car rotate speed(0 - 255)
#define RSPEED 200
#define LFSPEED 255
//brake acceleration
#define RANGE 10
//tweak delay between movements for better control
#define DELAY 0
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
extern unsigned long step_count;
extern int error;
extern PS2X ps2x; 
extern byte type;
extern byte vibrate;

void stepper_stop();
void stepper_up(unsigned long step_count);
void stepper_down(unsigned long step_count);
void ps2_init();

int isMaxHeight(unsigned long step_count);
int isMinHeight(unsigned long step_count);
int isDropHeight(unsigned long step_count);
