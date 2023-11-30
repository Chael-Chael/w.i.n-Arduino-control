#include <PS2X_lib.h> 
#include <math.h>
#include <Servo.h>
#include <AccelStepper.h>
#include "config.h"

//每次初始化请将夹爪升到最高点！！！
//steer
Servo steer;
//gripper
Servo gripper;
//stepper
AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// PS2X ps2x;     
// int error = 0;
// byte type = 0;
// byte vibrate = 0;

unsigned long previousTime;
//Wheels
int sp = 0;

int steerState = 0;
int stepperState = 0;
int gripState = 0;
int direction = 0;
int isAuto = 0;

// Motor L1 connections
const int L1_en = 9;
const int L1_in1 = 28;
const int L1_in2 = 26;
// Motor R1 connections
const int R1_en = 8;
const int R1_in1 = 24;
const int R1_in2 = 22;
//Motor L2 connections
const int L2_en = 6;
const int L2_in1 = 34;
const int L2_in2 = 36;
//Motor R2 connections
const int R2_en = 7;
const int R2_in1 = 30;
const int R2_in2 = 32;

//steerSp, must be no more than 90
int steerSp=10;
unsigned int steerDelay=250;
int gripSp=10;
unsigned int gripDelay=250;

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
  //unsigned long step_count = 100;//MAX_REV * split * steps_per_rev;
  //unsigned long previousTime = 0;

  pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  stepper.setMaxSpeed(MAX_STEER_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);

  //init split
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

  Serial.print("Stick L Values:");
  Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC);
  Serial.print(" ");


  //读取摇杆的数据
  Y1 = ps2x.Analog(PSS_LY);
  X1 = ps2x.Analog(PSS_LX);

  Serial.print("Stick R Values:");
  Serial.print(ps2x.Analog(PSS_RY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_RX), DEC);
  Serial.print(" ");

  //读取摇杆的数据
  Y2 = ps2x.Analog(PSS_RY);
  X2 = ps2x.Analog(PSS_RX);

//auto mode
  if(ps2x.ButtonPressed(PSB_PINK))
  {
      Serial.println("steer left."); 
      isAuto = 0;
      stepperState = 1;
      steerState = 0;
      gripState = 0;
      direction = 1;
      previousTime = millis();
  }
  else if(ps2x.ButtonPressed(PSB_RED))
  {    
      Serial.println("steer right.");
      isAuto = 1;
      stepperState = 1;
      steerState = 0;
      gripState = 0;
      direction = 2;
      previousTime = millis();
  }
  else{}

//auto loop
  if (isAuto == 1)
  {
    //stepper_up
    if (stepperState == 1)
    {
      stepper.setSpeed(SPEED);
      stepper.moveTo(0);
      stepper.runSpeed();

      steerState = 1;
      stepperState = 0;

      previousTime = millis();
    }

    if (stepperState == 0)
    {
      stepper.stop();
    }

    //rotate
    if (steerState == 1)
    {
      if (((millis() - previousTime) < steerDelay) && direction == 1)
      {
        steer.write(90 + steerSp);
      }
      else if (((millis() - previousTime) < steerDelay) && direction == 2)
      {
        steer.write(90 - steerSp);
      }
      else
      {
        steer.write(90);
        steerState = 2;
        gripState = 1;
        previousTime = millis(); 
      }
    }

    //grip
    if (gripState == 1)
    {
      if ((millis() - previousTime) < gripDelay)
      {
        steer.write(90 + gripSp);
      }
      else
      {
        steer.write(90);
        gripState = 0;
        delay(100);
        previousTime = millis();
      }
    }

    //rotate
    if (steerState == 2)
    {
      if (((millis() - previousTime) < steerDelay) && direction == 1)
      {
        steer.write(90 - steerSp);
      }
      else if (((millis() - previousTime) < steerDelay) && direction == 2)
      {
        steer.write(90 + steerSp);
      }
      else
      {
        steer.write(90);
        steerState = 0;
        previousTime = millis();
        stepperState = 2;
      }
    }

    if (stepperState == 2)
    {
      stepper.setSpeed(-SPEED);
      stepper.moveTo(MIN_POSITION);
      stepper.runSpeed();

      stepperState = 0;
      isAuto = 0;
      stepper.stop();
    }
  }


  if (isAuto == 0)
  {
    //grip maunal
    if(ps2x.ButtonPressed(PSB_GREEN))
    {
      Serial.println("Grip closing.");
      gripper.write(90 + gripSp);
      delay(gripDelay);
    }
    else if (ps2x.ButtonPressed(PSB_BLUE))
    {
      Serial.println("Grip opening.");
      gripper.write(90 - gripSp);
      delay(gripDelay);
    }
    else{} 

  //steer manual
    if(ps2x.Button(PSB_PAD_LEFT))
    {
      Serial.println("steer left."); 
      steer.write(90 + steerSp);
      delay(steerDelay);
      steer.write(90);
    }
    else if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
    {    
      Serial.println("steer right.");
      steer.write(90 - steerSp);
      delay(steerDelay);
      steer.write(90);
    }

    if (Y2 < 128 - HOLD)         //上
    {
      if (stepper.currentPosition() != 0)
      {
        Serial.println("stepper_up");
        stepper.setSpeed(SPEED);
        stepper.runSpeed();
      }
    }
    else if (Y2 > 128 + HOLD)
    {
      if (stepper.currentPosition() != MIN_POSITION)
      {
        Serial.println("stepper_down");
        stepper.setSpeed(-SPEED);
        stepper.runSpeed();
      }
    }
    else{
      stepper.stop();
    }
  }

  //convey manual
  if(ps2x.Button(PSB_PAD_UP))
  {
    Serial.println("convey in work.");
    analogWrite(convey_en,CONVEY_SPEED);
    digitalWrite(convey_in1,LOW);
    digitalWrite(convey_in2,HIGH);
    delay(DELAY);
  }
  else if(ps2x.Button(PSB_PAD_DOWN))
  {    
    Serial.println("convey in work.");
    analogWrite(convey_en,CONVEY_SPEED);
    digitalWrite(convey_in1,LOW);
    digitalWrite(convey_in2,HIGH);
    delay(DELAY);
  }

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
