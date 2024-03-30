#include <PS2X_lib.h>  //PS2的库文件，需先添加到Arduino IDE的库文件
#include <math.h>

//问题，关于sp
/*PS2引脚定义*/
#define PS2_DAT_PIN  A8
#define PS2_CMD_PIN  A9
#define PS2_SEL_PIN  A10
#define PS2_CLK_PIN  A11  

//car rotate & lifting speed(0 - 255)
#define RSPEED 180
#define LFSPEED 255

//brake acceleration
#define RANGE 10

//tweak delay between movements for better control
#define DELAY 0

//acceleration
#define PLUSACC 25
#define MINUSACC 25

//speed
#define MAX_SPEED 240

//axis threshold for accelerating, must be no more than 125
#define HOLD 100
#define pressures   false  //按键模式
#define rumble      false  //振动模式

//PS2X
PS2X ps2x;                //定义ps2x为PS2X类变量

int error = 0;
byte type = 0;
byte vibrate = 0;

//Wheels
//int ACC = 0;
int sp = 0;//speed initialization
// Motor L1 connections
int L1_en = 7;
int L1_in1 = 46;
int L1_in2 = 48;
// Motor R1 connections
int R1_en = 6; 
int R1_in1 = 50;
int R1_in2 = 52;
//Motor L2 connections
int L2_en =8 ;
int L2_in1 = 24;
int L2_in2 = 22;
//Motor R2 connections
int R2_en = 9;
int R2_in1 = 28;
int R2_in2 = 26;

//Board Lift
//Motor connection
int lift_motor_enA = 2;sss
int lift_motor_enB = 3;
int lift_motor_in1 = 31;
int lift_motor_in2 = 33;
int lift_motor_in3 = 35;
int lift_motor_in4 = 37;

//Lifting speed
int liftSpeed = 0;

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

void setup() {
  //PS2X Setup

  //串口波特率设置
  Serial.begin(9600);

  //在配置无线PS2模块之前，延时300ms以便于配置生效
  delay(300);

  /*PS2初始化*/
  //报错查询
  error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_SEL_PIN, PS2_DAT_PIN, pressures, rumble);
  if (error == 0)
  {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
    {
      Serial.println("true ");
    }
    else
    {
      Serial.println("false");
    }
    Serial.print("rumble = ");
    if (rumble)
    {
      Serial.println("true)");
    }
    else
    {
      Serial.println("false");
    }
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  //获取手柄类型
  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }

  //Motors setup
  //初始化电机驱动IO为输出方式
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

	// Turn off motors - Initial state
	digitalWrite(L1_in1, LOW);
	digitalWrite(L1_in2, LOW);
	digitalWrite(R1_in1, LOW);
	digitalWrite(R2_in2, LOW);
  digitalWrite(L2_in1, LOW);
  digitalWrite(L2_in2, LOW);
  digitalWrite(R2_in1, LOW);
  digitalWrite(R2_in2, LOW);

  //lift motor initialization
  pinMode(lift_motor_enA, OUTPUT);
  pinMode(lift_motor_enB, OUTPUT);
  pinMode(lift_motor_in1, OUTPUT);
  pinMode(lift_motor_in2, OUTPUT);
  pinMode(lift_motor_in3, OUTPUT);
  pinMode(lift_motor_in4, OUTPUT);

  digitalWrite(lift_motor_in1, LOW);
  digitalWrite(lift_motor_in2, LOW);
  digitalWrite(lift_motor_in3, LOW);
  digitalWrite(lift_motor_in4, LOW);
}

//functions for lift control
void lift(int sp)
{
  Serial.println("Lifting.");
  analogWrite(lift_motor_enA,sp);
  analogWrite(lift_motor_enB,sp);
  digitalWrite(lift_motor_in1,LOW);
  digitalWrite(lift_motor_in2,HIGH);
  digitalWrite(lift_motor_in3,LOW);
  digitalWrite(lift_motor_in4,HIGH);
  delay(DELAY);
}

void lower(int sp)
{
  analogWrite(lift_motor_enA,sp);
  analogWrite(lift_motor_enB,sp);
  Serial.println("Lowering.");
  digitalWrite(lift_motor_in1,HIGH);
  digitalWrite(lift_motor_in2,LOW);
  digitalWrite(lift_motor_in3,HIGH);
  digitalWrite(lift_motor_in4,LOW);
  delay(DELAY);
}

void liftstop()
{
  Serial.println("lift stop.");
  digitalWrite(lift_motor_in1,LOW);
  digitalWrite(lift_motor_in2,LOW);
  digitalWrite(lift_motor_in3,LOW);
  digitalWrite(lift_motor_in4,LOW);
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

//functions for speed
/*int speedplus(int ACC, int speed)
{
  if (isMaxSpeed(speed + ACC) == 0)
  {
    speed += ACC;
  }
  else
  {
    speed = MAX_SPEED;
  }
  return speed;
}*/

/*int getACC(int x, int y)
{
  double xdistance = (double)abs(x-128);
  double ydistance = (double)abs(y-128);
  if ( (sqrt(xdistance*xdistance + ydistance*ydistance)) <= (128.0 - (double)HOLD))
  {
    ACC = 0;
  }
  else
  {
    ACC = (int)( MAX_ACC*( sqrt(xdistance*xdistance + ydistance*ydistance) - 128.0 + (double)HOLD) / (double)HOLD );
  }
  return ACC;
}*/

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
  analogWrite(L1_en,0);
  analogWrite(L2_en,0);
  analogWrite(R1_en,0);
  analogWrite(R2_en,0);
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
  Serial.print("L1_f ");
  analogWrite(L1_en,sp);
  digitalWrite(L1_in1,LOW);
  digitalWrite(L1_in2,HIGH);
}
void R1_forward(int sp)//右前轮前进
{
  Serial.print("R1_f ");
  analogWrite(R1_en,sp);
  digitalWrite(R1_in1,LOW);
  digitalWrite(R1_in2,HIGH);
}
void L2_forward(int sp)//左后轮前进
{
  Serial.print("L2_f ");
  analogWrite(L2_en,sp);
  digitalWrite(L2_in1,LOW);
  digitalWrite(L2_in2,HIGH);
}
void R2_forward(int sp)//右后轮前进
{
  Serial.print("R2_f ");
  analogWrite(R2_en,sp);
  digitalWrite(R2_in1,HIGH);
  digitalWrite(R2_in2,LOW);
}

void L1_backward(int sp)//左前轮后退
{
  Serial.print("L1_b ");
  analogWrite(L1_en,sp);
  digitalWrite(L1_in1,HIGH);
  digitalWrite(L1_in2,LOW);
}
void R1_backward(int sp)//右前轮后退
{
  Serial.print("R1_b ");
  analogWrite(R1_en,sp);
  digitalWrite(R1_in1,HIGH);
  digitalWrite(R1_in2,LOW);
}
void L2_backward(int sp)//左后轮后退
{
  Serial.print("L2_b ");
  analogWrite(L2_en,sp);
  digitalWrite(L2_in1,HIGH);
  digitalWrite(L2_in2,LOW);
}
void R2_backward(int sp)//右后轮后退
{
  Serial.print("R2_b ");
  analogWrite(R2_en,sp);
  digitalWrite(R2_in1,LOW);
  digitalWrite(R2_in2,HIGH);
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

void right(int sp)
{
  Serial.println("Car left.");
  L2_backward(sp);
  R1_backward(sp);
  L1_forward(sp);
  R2_forward(sp);
  delay(DELAY);
}

void left(int sp)
{
  Serial.println("Car right.");
  R1_forward(sp);
  L2_forward(sp);
  R2_backward(sp);
  L1_backward(sp);
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
  L2_forward(sp);
  R1_backward(sp);
  L1_forward(sp);
  R2_backward(sp);
  delay(10);
}

void anticlock(int sp)
{
  Serial.println("Car anticlock.");
  L2_backward(sp);
  R1_forward(sp);
  L1_backward(sp);
  R2_forward(sp);
  delay(10);
}


//functions for ps2 control
void PS2_control(void)
{
  //摇杆方向值变量的定义
  int X1, Y1;
  //如果手柄初始化失败或者手柄类型没找到,则返回
  /*if (error == 1)           //skip loop if no controller found
    return;
  if (type != 1)            //skip loop if no controller found
    return;
  */

  //DualShock Controller
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  //手柄上的START键按下
  if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
    Serial.println("Start is being held");

  //手柄上的SELECT键按下
  if (ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held");

  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  vibrate = ps2x.Analog(PSAB_CROSS);

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
  Serial.print("Stick Values:");
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
    g_CarState = enRUN;
  }
  else if (Y1 <= 80 && X1 >= 180)          //右上
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enRUN;
  }
  else if (X1 <= 80 && Y1 >= 180)          //左下
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enBACK;
  }
  else if (Y1 >= 180 && X1 >= 180)        //右下
  {
    //sp = speedplus(getACC(X1, Y1), sp);
    //sp = getsp(X1, Y1);
    g_CarState = enBACK;
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
    case enUPLEFT: left(sp); break;
    case enUPRIGHT: right(sp); break;
    case enDOWNLEFT: left(sp); break;
    case enDOWNRIGHT: right(sp); break;
    case enRotateClock: clock(RSPEED); break;
    case enRotateAntiClock: anticlock(RSPEED); break;
    default: break;
  }

  if (ps2x.Button(PSB_PINK))
  {
    lift(LFSPEED);
  }
  else if (ps2x.Button(PSB_RED))
  {
    lower(LFSPEED);
  }
  else
  {
    liftstop();
  }

  Serial.print("Speed:");
  Serial.println(sp);

 //下面的延时是必须要的,主要是为了避免过于频繁的发送手柄指令造成的不断重启
  delay(50);
}
