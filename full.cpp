#include <Wire.h>
#include <MPU6050.h>
#include <Ultrasonic.h>
#include <HardwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

// Motor control pins
#define Delay_Right 1800
#define Delay_Left 2100
#define ENA 5
#define IN1 7
#define IN2 6
#define ENB 10
#define IN3 9
#define IN4 8
// defines all used Ultrasonic
#define TRIG 12
#define ECHO 11
// #define TRIG_PIN_Obstecale 12
// #define ECHO_PIN_Obstecale 13

// defintion of the arms code
#define SERVOMIN 150
#define SERVOMAX 600
#define USMIN 600
#define USMAX 2400
#define SERVO_FREQ 50

// these defines is using to return to the starting point
#define FORWARD 1
#define BACKWARD -1
#define LEFT 2
#define RIGHT -2
#define TIME_BYPASS_OBSTACLE 500   //1 cm per 100 ms

// Movement tracking constants
#define MAX_MOVES 300

using namespace std;
// Arms
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int objectDistance = 8;
uint8_t servonum = 0;
int RU = 1;
int RD = 2;
int LU = 3;
int LD = 4;
int RLU = 0;
int RLD = 10;

// we have two ultraSonic one for obstecals and another to hold the desired object
Ultrasonic ultrasonic(TRIG, ECHO); // this responsible of run the arm code and and catch the desired object
// Ultrasonic ultrasonic_Obstecale(TRIG_PIN_Obstecale, ECHO_PIN_Obstecale); // to avoid Obstecale

// declaration of functions
void LIFT(int x);
int angletopulse(int ang);
bool moveDistance(int distance);
bool checkCamera();
bool avoidObstacle();
void stopMotors();
int lookRight();
int lookLeft();
void turnRightMPU();
void turnLeftMPU();
float getYaw();
void storeTraking(int direction, int distance = 0);
void moveForward();
bool check_Sensor();
void Hold_Obj();
void Leave_Obj();
void returnToStart();


// constant used in the code
const int obstacleThreshold = 40; // Obstacle detection threshold in cm
bool process_Done = false;
bool swap_left_right = true;
bool isInScope;

// Movement tracking arrays
int movementDistances[MAX_MOVES];  // 0 distance for turn left or right
int movementDirections[MAX_MOVES]; // 1 for forward, -1 for backward ,2 for left turn, -2 for right turn
int moveCount = 0;

// HardwareSerial Serial1(1); // Use UART1    on the esp we must comment that  'The Serial1 object is already provided by the ESP32 core library'

void setup()
{
  Serial.begin(115200);
  pinMode(2, INPUT);
  digitalWrite(2, LOW);
  Serial.println("ESP32-CAM Peripheral Initialized");
  // Set motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // // setup of the arms
  // for (int y = 0; y < 8; y++)
  // {
  //     pwm.setPWM(y, 0, angletopulse(90));
  //     delay(1000);
  // }
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Initialize MPU6050
  Wire.begin();
  // Serial.println("MPU6050 initialized successfully!");
}

void loop()
{

  bool Object_Detected1 = moveDistance(30); // the nubmer in cm
  Serial.print("Object_Detectedd loop 122:  ");
  Serial.println(Object_Detected1);

  if (Object_Detected1)
  {

    // in the following while loop , the robot will close to the desired object until the ultrasonic_Arm detect the object to run the code of arm and hold the object
    while (true)
    {
      // check the sensor that is responsible of catch the object
      isInScope = check_Sensor();
      if (isInScope)
      {
        // the code of arm ...
        Hold_Obj();
        process_Done = true;
        break;
      }
      else
      {
        moveDistanceToHoldObjOrReturn(2); // 2 cm
        storeTraking(FORWARD, 2);
      }
    }
  }
  if (process_Done)
  {
    // execute the functions that is responsible of returning to the sarting site
    // first rotate 180 degree
    turnLeftMPU();
    turnLeftMPU();
    returnToStart();
    Leave_Obj();
    // the end
    while (1)
      ;
  }

  // once turn left and once turn right
  if (swap_left_right)
  {
    turnLeftMPU();
    storeTraking(LEFT);
    swap_left_right = !swap_left_right;
  }
  else
  {
    turnRightMPU();
    storeTraking(RIGHT);
    swap_left_right = !swap_left_right;
  }
}

// Function to move a specific distance in cm
bool moveDistance(int distance)
{

  Serial.print("moveDistance:  ");
    int traveled = 0;
    while (traveled < distance)
    {
      int obstacleDistance = ultrasonic.read();
      Serial.println(obstacleDistance);

      if (obstacleDistance > 0 && obstacleDistance < obstacleThreshold)
      {
        stopMotors();
        // Obstacle detected
        Serial.println("Obstacle detected! . Avoiding or catch it ... 149");
        // check and read from the camera

        bool Object_Detected = checkCamera();
        if (!Object_Detected)
        {
          for (int i = 0; i < 6000; i++)
          {
            Object_Detected = checkCamera();
            if (Object_Detected)
            {
              Serial.print("return 203:  ");
              return 1;
            }
          }
        }
        if (Object_Detected)
        {
          return 1;
        }
        else
        {
          bool found_Object_During_Return = avoidObstacle();
          Serial.print("found_Object_During_Return: 223 ");

          if (found_Object_During_Return)
          {
             return 1;
            Serial.print("found_Object_During_Return 228:  ");
          }
        }
      }
      else
      {
        Serial.println("else : moveForward();");
        // No obstacle, keep moving forward
        moveForward();
        // Store the movement
        storeTraking(FORWARD, 10); // 10 cm
        delay(1000);               // Approximation: 1 cm per 100 ms (adjust as needed)
        traveled += 10;
      }
    }
  
  stopMotors();
  return 0;
}

bool moveDistanceToHoldObjOrReturn(int distance){
  int traveled = 0;

    while (traveled < distance)
    {
            // No obstacle, keep moving forward
            moveForward();
            // Store the movement
            storeTraking(FORWARD, 1); // 1 cm
            delay(100);               // Approximation: 1 cm per 100 ms (adjust as needed)
            traveled += 1;
  
    }
    stopMotors();
}
// Function to avoid an obstacle
bool avoidObstacle()
{
    stopMotors();
    delay(1000);

    int rightDistance = lookRight();
    if (rightDistance == -1) // -1 means the robot find the desired object while he turn right
    {
        storeTraking(RIGHT);
        return true;
    }
  int leftDistance = lookLeft();
  Serial.println("lookLeft");
  if (leftDistance == -1) // -1 means the robot find the desired object while he turn left
  {
    storeTraking(LEFT);
    return true;
  }

  if (leftDistance > rightDistance)
  {
    storeTraking(LEFT);
  }
  else
  {
    turnRightMPU();
    turnRightMPU();
    storeTraking(RIGHT);
  }
  // Bypass obstacle
  moveForward();
  delay(TIME_BYPASS_OBSTACLE);
  storeTraking(FORWARD, TIME_BYPASS_OBSTACLE);

  // Return to original orientation
  if (leftDistance > rightDistance)
  {    
    turnRightMPU();
    storeTraking(RIGHT);
  }
  else
  {
    turnLeftMPU();
    storeTraking(LEFT);
  }
  return false;
}

// Turn left 90 degrees using MPU6050
void turnLeftMPU()
{

  analogWrite(ENA, 150);
  analogWrite(ENB, 140);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  delay(Delay_Left); // Adjust for 90-degree turn
  stopMotors();
}



// Turn right 90 degrees using MPU6050
void turnRightMPU()
{

  analogWrite(ENA, 150);
  analogWrite(ENB, 140);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);

  delay(Delay_Right); // Adjust for 90-degree turn
  stopMotors();
}

// Look right
int lookRight()
{
  turnRightMPU();
  bool Object_Detected = checkCamera();
  for (int i = 0; i < 6000; i++)
  {
    Object_Detected = checkCamera();
    if (Object_Detected)
    {
      Serial.print("return 203:  ");
      return -1;
    }
  }

  int distance = ultrasonic.read();
  turnLeftMPU();
  return distance;
}

// Look left
int lookLeft()
{
  turnLeftMPU();
  bool Object_Detected = checkCamera();
  for (int i = 0; i < 6000; i++)
  {
    Object_Detected = checkCamera();
    if (Object_Detected)
    {
      Serial.print("return 203:  ");
      return -1;
    }
  }

  int distance = ultrasonic.read();

  return distance;
}

// Motor control functions
void moveForward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 84); // Speed control
  analogWrite(ENB, 90);
}

void stopMotors()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


void storeTraking(int direction, int distance = 0)
{
  if (process_Done)
  {
    return;
  }

  if (moveCount < MAX_MOVES)
  {

    movementDistances[moveCount] = distance;
    movementDirections[moveCount] = direction;
    moveCount++;
  }else {
    stopMotors();
  }
}

void returnToStart()
{
  Serial.println("returnToStart...");
  int i;
  for (i = moveCount; i >= 0; i--)
  {

    switch (movementDirections[i])
    {
    case FORWARD:
      moveDistanceToHoldObjOrReturn(movementDistances[i]);
      break;
    case BACKWARD:
      break;
    case LEFT:
      turnLeftMPU();
      break;
    case RIGHT:
      turnRightMPU();
      break;
    default:
      break;
    }
  }
}

bool checkCamera()
{
  // Check for incoming data from ESP32-CAM
  int Q = digitalRead(2);
  Serial.print("Q:  ");
  Serial.println(Q);
  Serial.println("checkCamera");

  // Serial.println(x);
  if (Q)
  {

    Serial.println("object detected #####################");
    stopMotors();
    return true;
  }
  return false;
}

bool check_Sensor()
{

  int obstacleDistance = ultrasonic.read();
  Serial.println(obstacleDistance);
  if (obstacleDistance == objectDistance || obstacleDistance < objectDistance)
  {
    return true;
  }
  return false;
}

void Hold_Obj()
{
  delay(1000);
  LIFT(RLU);
}
void Leave_Obj()
{
    LIFT(RLD);
  delay(1000);

}

// the function of the Arms
void LIFT(int x)
{
  while (x == RU)
  {
    for (int ang = 90; ang <= 175; ang++) // wrist
    {
      pwm.setPWM(5, 0, angletopulse(ang));
      delay(50);
    }
    delay(100);

    for (int ang = 90; ang >= 0; ang--) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      delay(50);
    }
    for (int ang = 0; ang <= 25; ang++) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      delay(50);
    }
    delay(100);

    for (int ang = 90; ang <= 150; ang++) // shoulder
    {
      pwm.setPWM(7, 0, angletopulse(ang));
      delay(50);
    }
    x = 50;
  }
  ////////////////////////////////////////
  while (x == RD)
  {
    for (int ang = 150; ang >= 90; ang--) // shoulder
    {
      pwm.setPWM(7, 0, angletopulse(ang));
      delay(50);
    }
    for (int ang = 25; ang >= 0; ang--) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      delay(50);
    }

    for (int ang = 0; ang <= 90; ang++) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      delay(50);
    }

    delay(100);
    for (int ang = 175; ang >= 90; ang--) // wrist
    {
      pwm.setPWM(5, 0, angletopulse(ang));
      delay(50);
    }
    delay(100);

    x = 50;
  }

  ///////////////////////////////////////////////////////
  while (x == LU)
  {
    for (int ang = 90; ang >= 0; ang--) // wrist
    {
      pwm.setPWM(1, 0, angletopulse(ang));
      delay(100);
    }
    delay(500);

    for (int ang = 90; ang >= 0; ang--) // hand
    {
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(100);
    }
    for (int ang = 0; ang <= 35; ang++)
    {
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(100);
    }
    delay(500);

    for (int ang = 90; ang >= 0; ang--) // shoulder
    {
      pwm.setPWM(3, 0, angletopulse(ang));
      delay(20);
    }
    x = 50;
  }
  //////////////////////////////////////////////////////////
  while (x == LD)
  {
    for (int ang = 0; ang <= 90; ang++) // shoulder
    {
      pwm.setPWM(3, 0, angletopulse(ang));
      delay(20);
    }

    for (int ang = 35; ang >= 0; ang--) // hand
    {
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(100);
    }
    delay(500);
    for (int ang = 0; ang <= 90; ang++) // hand
    {
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(100);
    }

    for (int ang = 0; ang <= 90; ang++) // wrist
    {
      pwm.setPWM(1, 0, angletopulse(ang));
      delay(100);
    }
    delay(500);

    x = 50;
  }

  //////////////////////////////////////////////////////////////////
  while (x == RLU)
  {
    for (int ang = 90; ang >= 30; ang--) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(50);
    }
    delay(500);

    int ang1 = 90;
    for (int ang = 90; ang >= 55; ang--) // elbow
    {
      pwm.setPWM(4, 0, angletopulse(ang));
      if (ang1 <= 120)
      {
        pwm.setPWM(0, 0, angletopulse(ang1));
        ang1++;
      }
      delay(50);
    }
    delay(500);
    ang1 = 85;
    for (int ang = 90; ang <= 150; ang++) // shoulder
    {
      pwm.setPWM(7, 0, angletopulse(ang));
      if (ang1 >= 30)
      {
        pwm.setPWM(3, 0, angletopulse(ang1));
        ang1--;
      }
      delay(50);
    }
    x = 50;
  }

  ////////////////////////////////////////////////
  while (x == RLD)
  {
    int ang1 = 30;
    for (int ang = 150; ang >= 90; ang--) // shoulder
    {
      pwm.setPWM(7, 0, angletopulse(ang));
      if (ang1 <= 85)
      {
        pwm.setPWM(3, 0, angletopulse(ang1));
        ang1++;
      }
      delay(50);
    }

    ang1 = 120;
    for (int ang = 55; ang <= 90; ang++) // elbow
    {
      pwm.setPWM(4, 0, angletopulse(ang));
      if (ang1 >= 90)
      {
        pwm.setPWM(0, 0, angletopulse(ang1));
        ang1--;
      }

      delay(20);
    }
    delay(500);

    for (int ang = 30; ang <= 90; ang++) // hand
    {
      pwm.setPWM(11, 0, angletopulse(ang));
      pwm.setPWM(2, 0, angletopulse(ang));
      delay(20);
    }
    delay(500);
    x = 50;
  }
}

int angletopulse(int ang)
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}