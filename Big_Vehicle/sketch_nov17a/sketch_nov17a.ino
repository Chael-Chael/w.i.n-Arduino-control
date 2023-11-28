#include <PS2X_lib.h>

#include <PS2X_lib.h>

#include <PS2X_lib.h>

#include <PS2X_lib.h>  //PS2的库文件，需先添加到Arduino IDE的库文件
#include <math.h>

//WITHOUT SPEED CONTROL

/*PS2引脚定义*/
#define PS2_DAT_PIN  A3
#define PS2_CMD_PIN  A2
#define PS2_SEL_PIN  A1
#define PS2_CLK_PIN  A0   

//car rotate speed(0 - 255)
#define RSPEED 200

//tweak delay between movements for better control
#define DELAY 50
#define MAX_SPEED 100

//axis threshold for accelerating, must be no more than 125
#define HOLD 100
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   false  //按键模式
#define rumble      false  //振动模式

//PS2X
PS2X ps2x;                //定义ps2x为PS2X类变量

int error = 0;
byte type = 0;
byte vibrate = 0;

//Wheels
// Motor L1 connections
int L1_en = 6;
int L1_in1 = 46;
int L1_in2 = 48;
// Motor R1 connections
int R1_en = 7;
int R1_in1 = 50;
int R1_in2 = 52;
//Motor L2 connections
int L2_en = 4;
int L2_in1 = 26;
int L2_in2 = 28;
//Motor R2 connections
int R2_en = 5;
int R2_in1 = 22;
int R2_in2 = 24;
//Motor speed count
/*int L1Counter=0, R1Counter=0;
int L2Counter=0, R2Counter=0;
unsigned long time = 0, old_time = 0; // 时间标记
unsigned long time1 = 0; // 时间标记
int L1_Speed, R1_Speed, L2_Speed, R2_Speed;//左、右轮速度*/

//Board Lift
//Motor connection
int lift_motor_enA = 5;
int lift_motor_enB = 4;
int lift_motor_in1 = 30;
int lift_motor_in2 = 32;
int lift_motor_in3 = 34;
int lift_motor_in4 = 36;

//Lifting speed
int liftSpeed = 0;

/*小车运行状态枚举*/
enum CARSTATE{
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
  enRotateAntiClock
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

//speed detect
/*bool SpeedDetection()
{
  time = millis();//以毫秒为单位，计算当前时间 
  if(abs(time - old_time) >= 1000) // 如果计时时间已达1秒
  {  
    detachInterrupt(0); // 关闭外部中断0
    detachInterrupt(1); // 关闭外部中断1
    //把每一秒钟编码器码盘计得的脉冲数，换算为当前转速值
    //转速单位是每分钟多少转，即r/min。这个编码器码盘为20个空洞。
    Serial.print("left:");
    lv =(float)leftCounter*60/20;//小车车轮电机转速
    rv =(float)rightCounter*60/20;//小车车轮电机转速
    Serial.print("left:");
    Serial.print(lv);//向上位计算机上传左车轮电机当前转速的高、低字节
    Serial.print("     right:");
    Serial.println(rv);//向上位计算机上传左车轮电机当前转速的高、低字节
    //恢复到编码器测速的初始状态
    leftCounter = 0;   //把脉冲计数值清零，以便计算下一秒的脉冲计数
    rightCounter = 0;
    old_time=  millis();     // 记录每秒测速时的时间节点   
    attachInterrupt(0, RightCount_CallBack,FALLING); // 重新开放外部中断0
    attachInterrupt(1, LeftCount_CallBack,FALLING); // 重新开放外部中断0
    return 1;
  }
  else
    return 0;
}
*/

//functions for lift control
void lift(int sp)
{
  Serial.println("Lifting.");
  digitalWrite(lift_motor_in1,LOW);
  digitalWrite(lift_motor_in2,HIGH);
  digitalWrite(lift_motor_in3,LOW);
  digitalWrite(lift_motor_in4,HIGH);
  analogWrite(lift_motor_enA,sp);
  analogWrite(lift_motor_enB,sp);
  delay(DELAY);
}

void lower(int sp)
{
  Serial.println("Lowering.");
  digitalWrite(lift_motor_in1,HIGH);
  digitalWrite(lift_motor_in2,LOW);
  digitalWrite(lift_motor_in3,HIGH);
  digitalWrite(lift_motor_in4,LOW);
  analogWrite(lift_motor_enA,sp);
  analogWrite(lift_motor_enB,sp);
  delay(DELAY);
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

//functions for movements
void L1_forward()//左前轮前进
{
  digitalWrite(L1_in1,LOW);
  digitalWrite(L1_in2,HIGH);
}
void R1_forward()//右前轮前进
{
  digitalWrite(R1_in1,HIGH);
  digitalWrite(R1_in2,LOW);
}
void L2_forward()//左后轮前进
{
  digitalWrite(L2_in1,HIGH);
  digitalWrite(L2_in2,LOW);
}
void R2_forward()//右后轮前进
{
  digitalWrite(R2_in1,HIGH);
  digitalWrite(R2_in2,LOW);
}

void L1_backward()//左前轮后退
{
  digitalWrite(L1_in1,HIGH);
  digitalWrite(L1_in2,LOW);
}
void R1_backward()//右前轮后退
{
  digitalWrite(R1_in1,LOW);
  digitalWrite(R1_in2,HIGH);
}
void L2_backward()//左后轮后退
{
  digitalWrite(L2_in1,LOW);
  digitalWrite(L2_in2,HIGH);
}
void R2_backward()//右后轮后退
{
  digitalWrite(R2_in1,LOW);
  digitalWrite(R2_in2,HIGH);
}

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
}

void run()
{
  Serial.println("Car run.");
  L1_forward();
  R1_forward();
  L2_forward();
  R2_forward();
  delay(DELAY);
}

void back()
{
  Serial.println("Car back.");
  L1_backward();
  R1_backward();
  L2_backward();
  R2_backward();
  delay(DELAY);
} 

void left()
{
  Serial.println("Car left.");
  L1_backward();
  R1_forward();
  L2_forward();
  R2_backward();
  delay(DELAY);
}

void right()
{
  Serial.println("Car right.");
  L1_forward();
  L2_backward();
  R1_backward();
  R2_forward();
  delay(DELAY);
}

void upleft()
{
  Serial.println("Car upleft.");
  R1_forward();
  L2_forward();
  delay(DELAY);
}

void upright()
{
  Serial.println("Car upright.");
  L1_forward();
  R2_forward();
  delay(DELAY);
}

void downleft()
{
  Serial.println("Car downleft.");
  L1_backward();
  R2_backward();
  delay(DELAY);
}

void downright()
{
  Serial.println("Car downright.");
  R1_backward();
  L2_backward();
  delay(DELAY);
}

void clock()
{
  Serial.println("Car clock.");
  L1_forward();
  R1_backward();
  L2_forward();
  R2_backward();
  delay(DELAY);
}

void antiClock()
{
  Serial.println("Car antiClock.");
  L1_backward();
  R1_forward();
  L2_backward();
  R2_forward();
  delay(DELAY);
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

  //When triggered
  //print stick values if either is TRUE
  Serial.println("Stick Values:");
  Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC);
  Serial.print(" ");

  //读取摇杆的数据
  Y1 = ps2x.Analog(PSS_LY);
  X1 = ps2x.Analog(PSS_LX);

  //DualShock Controller
  //ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

  //手柄上的START键按下
  /*if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
    Serial.println("Start is being held");

  //手柄上的SELECT键按下
  if (ps2x.Button(PSB_SELECT))
    Serial.println("Select is being held")*/

  //手柄上的方向键上按下
  /*if (ps2x.Button(PSB_PAD_UP))      //will be TRUE as long as button is pressed
  {
    Serial.print("Up held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    g_CarState = enRUN;
  }
  //手柄上的方向键右按下
  else if (ps2x.Button(PSB_PAD_RIGHT))
  {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    g_CarState = enRIGHT;
  }
  //手柄上的方向键左按下
  else if (ps2x.Button(PSB_PAD_LEFT))
  {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    g_CarState = enLEFT;
  }
  //手柄上的方向键下按下
  else if (ps2x.Button(PSB_PAD_DOWN))
  {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    g_CarState = enBACK;
  }
  else
  {
    g_CarState = enSTOP;
  }*/

  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
  //vibrate = ps2x.Analog(PSAB_CROSS);

 
  //L2即手柄左侧前方下面的按键按下即小车顺时针转动
  if (ps2x.Button(PSB_L2))
  {
    Serial.println("L2 pressed");
    g_CarState = enRotateClock;
  }

  //R2即手柄右侧前方下面的按键按下即小车逆时针转动
  if (ps2x.Button(PSB_R2))
  {
    Serial.println("R2 pressed");
    g_CarState = enRotateAntiClock;
  }
  
  /*左摇杆控制小车运动状态程序*/
  if (Y1 < 125 && X1 > 80 && X1 < 180)         //上
  {
    g_CarState = enRUN;
  }
  else if (Y1 > 131 && X1 > 80 && X1 < 180) //下
  {
    g_CarState = enBACK;
  }
  else if (X1 < 125 && Y1 > 80 && Y1 < 180)   //左
  {
    g_CarState = enLEFT;
  }
  else if (Y1 > 80 && Y1 < 180 && X1 > 131) //右
  {
    g_CarState = enRIGHT;
  }
  else if (Y1 <= 80 && X1 <= 80)           //左上
  {
    g_CarState = enUPLEFT;
  }
  else if (Y1 <= 80 && X1 >= 180)          //右上
  {
    g_CarState = enUPRIGHT;
  }
  else if (X1 <= 80 && Y1 >= 180)          //左下
  {
    g_CarState = enDOWNLEFT;
  }
  else if (Y1 >= 180 && X1 >= 180)        //右下
  {
    g_CarState = enDOWNRIGHT;
  }
  else                                    //停
  {
    g_CarState = enSTOP;
  }
}


void loop()
{

  PS2_control();
  switch (g_CarState)
  {
    case enSTOP: allstop(); break;
    case enRUN: run(); break; 
    case enLEFT: left(); break;
    case enRIGHT: right(); break;
    case enBACK: back(); break;
    case enUPLEFT: upleft(); break;
    case enUPRIGHT: upright(); break;
    case enDOWNLEFT: downleft(); break;
    case enDOWNRIGHT: downright(); break;
    case enRotateClock: clock(); break;
    case enRotateAntiClock: antiClock(); break;
    default: break;
  }

  //lift control
  /*if (ps2x.Button(PSB_PINK) == 1)
  {
    Serial.println("X is being held");
    lift(liftSpeed);
  } //will be TRUE as long as button is pressed

  if (ps2x.Button(PSB_RED) == 1)
  {
    Serial.println("B is being held.");
    lower(liftSpeed);
  }*/

 //下面的延时是必须要的,主要是为了避免过于频繁的发送手柄指令造成的不断重启
  delay(50);
}

