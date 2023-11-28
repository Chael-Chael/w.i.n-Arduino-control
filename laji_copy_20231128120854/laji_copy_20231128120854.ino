#include <Servo.h>
Servo myservo1;
Servo myservo2;
#include <PS2X_lib.h>  //for v1.6
/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        A8  //14    
#define PS2_CMD        A9  //15
#define PS2_SEL        A10  //16
#define PS2_CLK        A11  //17
#define xinhao1 1
#define xinhao2 2
#define dirpin 4
#define steppin 5
#define enable 6
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false


PS2X ps2x; // create PS2 Controller Class
int error = 0;
byte type = 0;
byte vibrate = 0;


// Reset func 


void zheng()
{
  digitalWrite(dirpin,HIGH);
  for(int i=0;i<step;i++){
    digitalWrite(steppin,HIGH);
    delayMicroseconds(800);
    digitalWrite(steppin,LOW);
    delayMicroseconds(800);

  }
}
void fan()
{
  digitalWrite(dirpin,LOW);
  for(int i=0;i<step;i++){
    digitalWrite(steppin,HIGH);
    delayMicroseconds(800);
    digitalWrite(steppin,LOW);
    delayMicroseconds(800);

  }
}
void setup(){
  Serial.begin(9600);
  myservo1.attach(xinhao1);
  myservo2.attach(xinhao2);
  pinMode(enable,OUTPUT);
  pinMode(dirpin,OUTPUT);
  pinMode(steppin,OUTPUT);
  digitalWrite(enable,LOW);
 
}
void loop(){  
  ps2x.read_gamepad(false, vibrate); 
    if(ps2x.ButtonPressed(PSB_TRIANGLE)){
        Serial.println("Y"); 
          myservo1.write(myservo1speed2);
    delay(myservo1times2);
    myservo1.write(90);
    }

   else if(ps2x.ButtonPressed(PSB_CIRCLE))    {         //will be TRUE if button was JUST pressed
      Serial.print("B");
      
      zheng(); 
     }
   else if(ps2x.ButtonPressed(PSB_CROSS))   {          //will be TRUE if button was JUST pressed OR released
      Serial.print("A");
    
        fan(); 
     }

   else if(ps2x.ButtonPressed(PSB_SQUARE)){          //will be TRUE if button was JUST released
      Serial.println("X");
     myservo1.write(myservo1speed1);
delay(myservo1times1);
myservo1.write(90);
}
else if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      myservo2.write(0);
 delay(250);
 myservo2.write(90);
    }
 else if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      myservo2.write(180);
 delay(250); 
 myservo2.write(90);
    }
      delay(100);

}
