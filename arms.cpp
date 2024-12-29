#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Ultrasonic.h>

void LIFT(int x);
int angletopulse(int ang);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 
#define SERVOMAX  600 
#define USMIN  600 
#define USMAX  2400
#define SERVO_FREQ 50 
#define TRIG_PIN 12
#define ECHO_PIN 11

const int objectDistance = 8;
uint8_t servonum = 0;

int RU=1;
int RD=2;
int LU=3;
int LD=4;
int RLU=0;
int RLD=10;

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN); // Initialize ultrasonic sensor

void setup() {
  for (int y=0;y<8 ; y++)
  {pwm.setPWM(y, 0, angletopulse(90));
  delay(1000);}
  
   Serial.begin(9600);
  pwm.begin();
pwm.setPWMFreq(SERVO_FREQ); 

}

void loop() {
  

  int obstacleDistance = ultrasonic.read();
  Serial.println(obstacleDistance);
  if (obstacleDistance == objectDistance) {
    delay(1000);
    LIFT(RLU);
    delay(5000);
    LIFT(RLD);  
    }
  /*while(1)
  {}*/

  /*delay(2000);
  

delay(5000);
   LIFT(RLU);
    delay(5000);
  LIFT(RLD);*/
  
  
}


void LIFT(int x)
{ while(x==RU) 
  {
for (int ang =90;ang <= 175; ang++)//wrist
  {pwm.setPWM(5, 0, angletopulse(ang));
 delay(50);}
delay(100);

    
    for (int ang =90;ang >= 0; ang--)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
 delay(50);}
  for (int ang =0;ang <= 25; ang++)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
 delay(50);}
delay(100);

for (int ang =90;ang <=150 ; ang++)//shoulder
{pwm.setPWM(7, 0, angletopulse(ang));
  delay(50);  
    }
   x=50; }
////////////////////////////////////////
    while(x==RD) 
  {
    for (int ang =150;ang >=90 ; ang--)//shoulder
{pwm.setPWM(7, 0, angletopulse(ang));
  delay(50);  
    }
for (int ang =25;ang >= 0; ang--)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
 delay(50);}

 for (int ang =0;ang <=90; ang++)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
 delay(50);}

delay(100);
for (int ang =175;ang >= 90; ang--)//wrist
  {pwm.setPWM(5, 0, angletopulse(ang));
 delay(50);}
delay(100);

   x=50; }

///////////////////////////////////////////////////////
    while(x==LU) 
  {for (int ang =90;ang >=0 ; ang--)//wrist
  {pwm.setPWM(1, 0, angletopulse(ang));
 delay(100);}
delay(500);
    
    
    for (int ang =90;ang >= 0; ang--)//hand
  {pwm.setPWM(2, 0, angletopulse(ang));
 delay(100);}
  for (int ang =0;ang <= 35; ang++)
  {pwm.setPWM(2, 0, angletopulse(ang));
 delay(100);}
delay(500);

for (int ang =90;ang >=0 ; ang--)//shoulder
{pwm.setPWM(3, 0, angletopulse(ang));
  delay(20);  
    }
    x=50;}
    //////////////////////////////////////////////////////////
    while(x==LD) 
  {for (int ang =0;ang <=90 ; ang++)//shoulder
{pwm.setPWM(3, 0, angletopulse(ang));
  delay(20);  
    } 
    
     for (int ang =35;ang >= 0; ang--)//hand
  {pwm.setPWM(2, 0, angletopulse(ang));
 delay(100);}
delay(500);
for (int ang =0;ang <= 90; ang++)//hand
  {pwm.setPWM(2, 0, angletopulse(ang));
 delay(100);}
   
    for (int ang =0;ang <=90 ; ang++)//wrist
  {pwm.setPWM(1, 0, angletopulse(ang));
 delay(100);}
delay(500);


    x=50;}

//////////////////////////////////////////////////////////////////
    while(x==RLU) 
  {
    for (int ang =90;ang >= 30; ang--)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
   pwm.setPWM(2, 0, angletopulse(ang));
 delay(50);}
delay(500);


int ang1=90;
for (int ang =90;ang >= 55; ang--)//elbow
{pwm.setPWM(4, 0, angletopulse(ang));
if(ang1<=120)
{pwm.setPWM(0, 0, angletopulse(ang1));
ang1++;}
  delay(50);  
    }
    delay(500);
ang1=85;
for (int ang =90;ang <=150 ; ang++)//shoulder
{pwm.setPWM(7, 0, angletopulse(ang));
if(ang1>=30)
{pwm.setPWM(3, 0, angletopulse(ang1));
ang1--;}
  delay(50);  
    }x=50;}

////////////////////////////////////////////////
  while(x==RLD) 
  {int ang1=30;
    for (int ang =150;ang >=90 ; ang--)//shoulder
{pwm.setPWM(7, 0, angletopulse(ang));
if(ang1<=85)
{pwm.setPWM(3, 0, angletopulse(ang1));
ang1++;}
  delay(50);} 
  
  ang1=120;
   for (int ang =55;ang <= 90; ang++)//elbow
{pwm.setPWM(4, 0, angletopulse(ang));
if(ang1>=90)
{pwm.setPWM(0, 0, angletopulse(ang1));
ang1--;}

  delay(20);  
    }
    delay(500);

    for (int ang =30;ang <= 90; ang++)//hand
  {pwm.setPWM(6, 0, angletopulse(ang));
  pwm.setPWM(2, 0, angletopulse(ang));
 delay(20);}
delay(500);
 x=50;   
}
}


int angletopulse(int ang)
{
int pulse = map(ang,0,180,SERVOMIN,SERVOMAX);
return pulse;
}


