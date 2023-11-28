#include <PS2X_lib.h>  //PS2的库文件，需先添加到Arduino IDE的库文件
#include <math.h>
#include <Servo.h>
#include "config.h"
//每次使用请把夹爪升到最低状态
//steer
Servo steer;
//gripper
Servo gripper;

PS2X ps2x;     

int error = 0;
byte type = 0;
byte vibrate = 0;

//Wheels
//int ACC = 0;
int sp = 0;//speed initialization
long step_count = MAX_REV * split *steps+per_rev;
unsigned long previousTime = 0;
int steerState = 0;
int stepperState = 0;
int gripState = 0;
int direction - 0;

// Motor L1 connections
int L1_en = 9;
int L1_in1 = 28;
int L1_in2 = 26;
// Motor R1 connections
int R1_en = 8;
int R1_in1 = 24;
int R1_in2 = 22;
//Motor L2 connections
int L2_en = 6;
int L2_in1 = 34;
int L2_in2 = 36;
//Motor R2 connections
int R2_en = 7;
int R2_in1 = 30;
int R2_in2 = 32;

//steerSp, must be no more than 90
int steerSp=10;
int steerDelay=250;
int gripSp=10;
int gripDelay=250;
int gripState=0;//0 = open, 1 = closed

/*小车运行状态枚举*/
enum {
  enSTOP = 1,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT,
  enRotateClock,
  enRotateAntiClock,
} enCarState;

int g_CarState = enSTOP;

void (* resetFunc) (void) = 0;

void setup() {
  //PS2X Setup
  ps2_init();

  //stepper setup
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  switch (split)
  {
    case 1:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case 2:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case 4:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case 8:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case 16:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, HIGH);
      break;
    default:
      break;
  }

  //steer setup
  steer.attach(steerSig);
  gripper.attach(gripperSig);

  //Motors setup
  pinMode(L1_in1, OUTPUT);
  pinMode(L1_in2, OUTPUT);
  pinMode(L1_en, OUTPUT);

  pinMode(L2_in1, OUTPUT);
  pinMode(L2_in2, OUTPUT);
  pinMode(L2_en, OUTPUT);

  pinMode(R1_in1, OUTPUT);
  pinMode(R1_in2, OUTPUT);
  pinMode(R1_en, OUTPUT);

  pinMode(R2_in1, OUTPUT);
  pinMode(R2_in2, OUTPUT);
  pinMode(R2_en, OUTPUT);

  pinMode(convey_en, OUTPUT);
  pinMode(convey_in1, OUTPUT);
  pinMode(convey_in2, OUTPUT);

	// Turn off motors - Initial state
	digitalWrite(L1_in1, LOW);
	digitalWrite(L1_in2, LOW);
	digitalWrite(R1_in1, LOW);
	digitalWrite(R2_in2, LOW);
  digitalWrite(L2_in1, LOW);
  digitalWrite(L2_in2, LOW);
  digitalWrite(R2_in1, LOW);
  digitalWrite(R2_in2, LOW);
  digitalWrite(convey_in1, LOW);
  digitalWrite(convey_in2, LOW);
}

//functions for wheels control
//if is max/min speed, return 1;
int isMaxSpeed(int speed)
{
  if (speed >= MAX_SPEED)
  {
    Serial.print("Is Max Speed!");
    return 1;
  }
  else
  {
    return 0;
  }
}

int isMinSpeed(int speed)
{
  if (speed <= 0)
  {
    Serial.print("Is Min Speed!");
    return 1;
  }
  else
  {
    return 0;
  }
}

int getsp(int x, int y)
{
  double xdistance = (double)abs(x-128);
  double ydistance = (double)abs(y-128);
  if ( (sqrt(xdistance*xdistance + ydistance*ydistance)) <= (128.0 - (double)HOLD))
  {
    sp = 0;
  }
  else
  {
    sp = (int)( MAX_SPEED*( sqrt(xdistance*xdistance + ydistance*ydistance) - 128.0 + (double)HOLD) / (double)HOLD );
  }

  return sp;
}

//functions for movements
void allstop()
{
  Serial.println("Car stop.");
  digitalWrite(L1_in1,LOW);
  digitalWrite(L1_in2,LOW);  
  digitalWrite(R1_in1,LOW);
  digitalWrite(R1_in2,LOW);
  digitalWrite(L2_in1,LOW);
  digitalWrite(L2_in2,LOW);
  digitalWrite(R2_in1,LOW);
  digitalWrite(R2_in2,LOW);
  //sp = 0;
}

void L1_forward(int sp)//左前轮前进
{
  analogWrite(L1_en,sp);
  digitalWrite(L1_in1,LOW);
  digitalWrite(L1_in2,HIGH);
}
void R1_forward(int sp)//右前轮前进
{
  analogWrite(R1_en,sp);
  digitalWrite(R1_in1,HIGH);
  digitalWrite(R1_in2,LOW);
}
void L2_forward(int sp)//左后轮前进
{
  analogWrite(L2_en,sp);
  digitalWrite(L2_in1,LOW);
  digitalWrite(L2_in2,HIGH);
}
void R2_forward(int sp)//右后轮前进
{
  analogWrite(R2_en,sp);
  digitalWrite(R2_in1,LOW);
  digitalWrite(R2_in2,HIGH);
}

void L1_backward(int sp)//左前轮后退
{
  analogWrite(L1_en,sp);
  digitalWrite(L1_in1,HIGH);
  digitalWrite(L1_in2,LOW);
}
void R1_backward(int sp)//右前轮后退
{
  analogWrite(R1_en,sp);
  digitalWrite(R1_in1,LOW);
  digitalWrite(R1_in2,HIGH);
}
void L2_backward(int sp)//左后轮后退
{
  analogWrite(L2_en,sp);
  digitalWrite(L2_in1,HIGH);
  digitalWrite(L2_in2,LOW);
}
void R2_backward(int sp)//右后轮后退
{
  analogWrite(R2_en,sp);
  digitalWrite(R2_in1,HIGH);
  digitalWrite(R2_in2,LOW);
}

void run(int sp)
{
  Serial.println("Car run.");
  L1_forward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_forward(sp);
  delay(DELAY);
}

void back(int sp)
{
  Serial.println("Car back.");
  L1_backward(sp);
  R1_backward(sp);
  L2_backward(sp);
  R2_backward(sp);
  delay(DELAY);
} 

void left(int sp)
{
  Serial.println("Car left.");
  L1_backward(sp);
  R1_forward(sp);
  L2_forward(sp);
  R2_backward(sp);
  delay(DELAY);
}

void right(int sp)
{
  Serial.println("Car right.");
  R1_backward(sp);
  L1_forward(sp);
  R2_forward(sp);
  L2_backward(sp);
  delay(DELAY);
}

void upleft(int sp)
{
  Serial.println("Car upleft.");
  R1_forward(sp);
  L2_forward(sp);
  delay(DELAY);
}

void upright(int sp)
{
  Serial.println("Car upright.");
  L1_forward(sp);
  R2_forward(sp);
  delay(DELAY);
}

void downleft(int sp)
{
  Serial.println("Car downleft.");
  L1_backward(sp);
  R2_backward(sp);
  delay(DELAY);
}

void downright(int sp)
{
  Serial.println("Car downright.");
  R1_backward(sp);
  L2_backward(sp);
  delay(DELAY);
}

void clock(int sp)
{
  Serial.println("Car clock.");
  L1_forward(sp);
  R1_backward(sp);
  L2_forward(sp);
  R2_backward(sp);
  delay(10);
}

void anticlock(int sp)
{
  Serial.println("Car anticlock.");
  L1_backward(sp);
  R1_forward(sp);
  L2_backward(sp);
  R2_forward(sp);
  delay(10);
}


//functions for ps2 control
void PS2_control(void)
{
  //摇杆方向值变量的定义
  int X1, Y1, X2, Y2;

  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  vibrate = ps2x.Analog(PSAB_CROSS);

  //DualShock Controller
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

//auto mode
  if(ps2x.ButtonPressed(PSB_PINK))
  {
      Serial.println("steer left."); 
      steerState = 1;
      stepperState = 1;
      gripState = 1;
      direction = 1;
      previousTime = millis();
  }
  else if(ps2x.ButtonPressed(PSB_RED))
  {    
      Serial.println("steer right.");
      steerState = 2;
      stepperState = 1;
      gripState = 1;
      direction = 2;
      previousTime = millis();
  }
  else{}

//stepper
  if (stepperState == 1 && gripState == 1 && (steerState != 0))
  {
    if (isDropHeight(step_count) == 0)
    {
        if (step_count < DROP_REV * split *steps_per_rev)
        {
            stepper_down(step_count);
        }
        else if (step_count > DROP_REV * split *steps_per_rev)
        {
            stepper_up(step_count);
        }
    }
    else
    {
      stepperState = 0;
    }
  }
  else{}

//rotate
  if (steerState == 1 && stepperState == 0 && gripState == 1)
  {
    if ((millis() - previousTime) < steerDelay)
    {
      steer.write(90 + steerSp);
    }
    else
    {
      steer.write(90);
      previousTime = millis();
      steerState = 0;
    }
  }
  else if(steerState == 2 && stepperState == 0 && gripState == 1)
  {
    if ((millis() - previousTime) < steerDelay)
    {
      steer.write(90 - steerSp);
    }
    else
    {
      steer.write(90);
      previousTime = millis();
      steerState = 0;
    }
  }
  else
  {
    steer.write(90);
  }

//grip
  if (steerState == 0 && stepperState == 0 && gripState == 1)
  {
    if ((millis() - previousTime) < gripDelay)
    {
      steer.write(90 + gripSp);
    }
    else
    {
      steer.write(90);
      delay(100);
      previousTime = millis();
      gripState = 0;
      (direction == 1) ? steerState = 1 : steerState =2;
    }
  }

//rotate
  if (steerState == 1 && gripState == 0)
  {
    if ((millis() - previousTime) < steerDelay)
    {
      steer.write(90 - steerSp);
    }
    else
    {
      steer.write(90);
      previousTime = millis();
      steerState = 0;
    }
  }
  else if(steerState == 2 && gripState == 0)
  {
    if ((millis() - previousTime) < steerDelay)
    {
      steer.write(90 + steerSp);
    }
    else
    {
      steer.write(90);
      previousTime = millis();
      steerState = 0;
    }
  }
  else
  {
    steer.write(90);
  }

//grip maunal
  if(ps2x.ButtonPressed(PSB_GREEN))
  {
    Serial.println("Grip closing.");
    gripper.write(90 + gripSp);
  }
  else if (ps2x.ButtonPressed(PSB_BLUE))
  {
    Serial.println("Grip opening.");
    gripper.write(90 - gripSp);
  }
  else if (grip)
  {
    gripper.write(90);
  }

//convey manual
  if(ps2x.Button(PSB_PAD_UP))
  {
    Serial.println("convey in work.");
    analogWrite(convey_en,CONVEY_SPEED);
    digitalWrite(convey_in1,HIGH);
    digitalWrite(convey_in2,LOW);
    delay(DELAY);
  }
<
  else if(ps2x.ButtonPressed(PSB_PAD_DOWN))
  {    
    Serial.println("convey in work.");
    analogWrite(convey_en,CONVEY_SPEED);
    digitalWrite(convey_in1,LOW);
    digitalWrite(convey_in2,HIGH);
    delay(DELAY);
  }

//steer manual
  if(ps2x.Button(PSB_PAD_LEFT))
  {
    Serial.println("steer left."); 
    steer.write(90 - steerSp);
    delay(steerDelay);
    steer.write(90);
  }
  else if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
  {    
    Serial.println("steer right.");
    delay(steerDelay);
    steer.write(90);
  }

  Serial.print("Stick R Values:");
  Serial.print(ps2x.Analog(PSS_RY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RX), DEC);
  Serial.print(" ");

  //读取摇杆的数据
  Y2 = ps2x.Analog(PSS_RY);
  X2 = ps2x.Analog(PSS_RX);

  if (Y2 < 128 - HOLD)         //上
  {
    if (isMaxHeight(step_count) == 0)
    {
      stepper_up(step_count);
    }
    delay(DELAY);
  }
  else if (Y2 > 128 + HOLD)
  {
    if (isMinHeight(step_count) == 0)
    {
      stepper_down(step_count);
    }
    delay(DELAY);
  }
  else{}

  //L1即手柄左侧前方下面的按键按下即小车减速
  if (ps2x.Button(PSB_L1))
  {
    Serial.println("L1 pressed");
    sp -= MINUSACC;
  }

  //R1即手柄右侧前方下面的按键按下即小车加速
  if (ps2x.Button(PSB_R1))
  {
    Serial.println("R1 pressed");
    sp += PLUSACC;
  }

  //When triggered
  //print stick values if either is TRUE
  Serial.println("Stick L Values:");
  Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC);
  Serial.print(" ");

  //读取摇杆的数据
  Y1 = ps2x.Analog(PSS_LY);
  X1 = ps2x.Analog(PSS_LX);

  /*左摇杆控制小车运动状态程序*/
  if (Y1 < 125 && X1 > 80 && X1 < 180)         //上
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enRUN;
  }
  else if (Y1 > 131 && X1 > 80 && X1 < 180) //下
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enBACK;
  }
  else if (X1 < 125 && Y1 > 80 && Y1 < 180)   //左
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enLEFT;
  }
  else if (Y1 > 80 && Y1 < 180 && X1 > 131) //右
  {
    //sp = getsp(X1, Y1);
    g_CarState = enRIGHT;
  }
  else if (Y1 <= 80 && X1 <= 80)           //左上
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enUPLEFT;
  }
  else if (Y1 <= 80 && X1 >= 180)          //右上
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enUPRIGHT;
  }
  else if (X1 <= 80 && Y1 >= 180)          //左下
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enDOWNLEFT;
  }
  else if (Y1 >= 180 && X1 >= 180)        //右下
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enDOWNRIGHT;
  }
  else                                  //停
  {
    if (ps2x.Button(PSB_R2))
    {
      Serial.println("L2 pressed");
      g_CarState = enRotateClock;
    }
    else if (ps2x.Button(PSB_L2))
    {
      Serial.println("r2 pressed");
      g_CarState = enRotateAntiClock;
    }
    else
    {
      g_CarState = enSTOP;
    }
  }
}

void loop()
{

  PS2_control();

  if (isMaxSpeed(sp) == 1)
  {
    sp = MAX_SPEED;
  }
  else if (isMinSpeed(sp) == 1)
  {
    sp = 0;
  }

  switch (g_CarState)
  {
    case enSTOP: allstop(); break;
    case enRUN: run(sp); break; 
    case enLEFT: left(sp); break;
    case enRIGHT: right(sp); break;
    case enBACK: back(sp); break;
    case enUPLEFT: upleft(sp); break;
    case enUPRIGHT: upright(sp); break;
    case enDOWNLEFT: downleft(sp); break;
    case enDOWNRIGHT: downright(sp); break;
    case enRotateClock: clock(RSPEED); break;
    case enRotateAntiClock: anticlock(RSPEED); break;
    default: break;
  }


  Serial.print("Speed:");
  Serial.println(sp);

 //下面的延时是必须要的,主要是为了避免过于频繁的发送手柄指令造成的不断重启
  delay(50);
}
