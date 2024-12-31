#include <Wire.h>
#include <MPU6050.h>
#include <Ultrasonic.h>
#include <HardwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

// Motor control pins
#define X 150
#define Y 150
#define ENA 3
#define IN1 6
#define IN2 5
#define ENB 9
#define IN3 7
#define IN4 8

// defines all used Ultrasonic
#define TRIG_PIN_Arm 11
#define ECHO_PIN_Arm 10
#define TRIG_PIN_Obstecale 12
#define ECHO_PIN_Obstecale 13

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
#define TIME_BYPASS_OBSTACLE 100

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
Ultrasonic ultrasonic_Arm(TRIG_PIN_Arm, ECHO_PIN_Arm);                   // this responsible of run the arm code and and catch the desired object
Ultrasonic ultrasonic_Obstecale(TRIG_PIN_Obstecale, ECHO_PIN_Obstecale); // to avoid Obstecale

// mpu
MPU6050 mpu;

// declaration of functions
void LIFT(int x);
int angletopulse(int ang);
void moveDistance(int distance);
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
void returnToStart();

// constant used in the code
const int obstacleThreshold = 20; // Obstacle detection threshold in cm
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
    // setup the communication with camera
    // Serial1.begin(9600, SERIAL_8N1, 16, 17); // UART1 on GPIO16 (RX) and GPIO17 (TX)  delete this on arduino
    Serial.println("ESP32-CAM Peripheral Initialized");
    // Set motor pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // setup of the arms
    for (int y = 0; y < 8; y++)
    {
        pwm.setPWM(y, 0, angletopulse(90));
        delay(1000);
    }
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (true)
            ; // Stop execution
    }
    Serial.println("MPU6050 initialized successfully!");
}

void loop()
{
    // Move forward 0.5 meters
    moveDistance(50); // the nubmer in cm

    steady();

    // check and read from the camera
    bool Object_Detected = checkCamera();

    if (Object_Detected)
    {
        stopMotors();
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
                moveDistance(20); // 20 cm
                storeTraking(FORWARD, 20);
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
void moveDistance(int distance)
{
    int traveled = 0;

    while (traveled < distance)
    {
        int obstacleDistance = ultrasonic_Obstecale.read();
        Serial.println(obstacleDistance);

        if (obstacleDistance > 0 && obstacleDistance < obstacleThreshold)
        {
            // Obstacle detected
            Serial.println("Obstacle detected! . Avoiding or catch it ... ");
            // check and read from the camera
            bool Object_Detected = checkCamera();
            if (Object_Detected)
            {
                break;
            }
            else
            {
                bool found_Object_During_Return = avoidObstacle();
                if (found_Object_During_Return)
                {
                    break;
                }
            }
        }
        else
        {
            // No obstacle, keep moving forward
            moveForward();
            // Store the movement
            storeTraking(FORWARD, 1); // 1 cm
            delay(100);               // Approximation: 1 cm per 100 ms (adjust as needed)
            traveled += 1;
        }
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
    if (leftDistance == -1) // -1 means the robot find the desired object while he turn left
    {
        storeTraking(LEFT);
        return true;
    }

    if (rightDistance > leftDistance)
    {
        turnRightMPU();
        storeTraking(RIGHT);
    }
    else
    {
        turnLeftMPU();
        storeTraking(LEFT);
    }
    // Bypass obstacle
    moveForward();
    delay(TIME_BYPASS_OBSTACLE);
    storeTraking(FORWARD, TIME_BYPASS_OBSTACLE);

    // Return to original orientation
    if (rightDistance > leftDistance)
    {
        turnLeftMPU();
        storeTraking(LEFT);
    }
    else
    {
        turnRightMPU();
        storeTraking(RIGHT);
    }
    return false;
}

// Function to calculate yaw using MPU6050
float getYaw()
{
    static unsigned long lastTime = 0;
    static float yaw = 0;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
    lastTime = currentTime;

    int16_t gyroZ = mpu.getRotationZ();
    yaw += (gyroZ / 131.0) * deltaTime; // Integrate gyro Z-axis
    yaw = fmod(yaw + 360, 360);         // Keep yaw in range [0, 360)
    return yaw;
}

// Turn left 90 degrees using MPU6050
void turnLeftMPU()
{
    float initialYaw = getYaw();
    float targetYaw = fmod(initialYaw + 90, 360);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, X);
    analogWrite(ENB, Y);

    while (true)
    {
        float currentYaw = getYaw();
        if (fabs(currentYaw - targetYaw) < 2.0)
        { // Allowable error margin
            break;
        }
        delay(10); // Allow time for MPU6050 readings
    }
    stopMotors();
}

// Turn right 90 degrees using MPU6050
void turnRightMPU()
{
    float initialYaw = getYaw();
    float targetYaw = fmod(initialYaw - 90 + 360, 360);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, X);
    analogWrite(ENB, Y);

    while (true)
    {
        float currentYaw = getYaw();
        if (fabs(currentYaw - targetYaw) < 2.0)
        { // Allowable error margin
            break;
        }
        delay(10); // Allow time for MPU6050 readings
    }
    stopMotors();
}

// Look right
 int lookRight()
{
    turnRightMPU();
    bool Object_Detected = checkCamera();
    if (Object_Detected)
    {
        return -1; // -1 means the object is detected
    }
    int distance = ultrasonic_Obstecale.read();
    turnLeftMPU();
    return distance;
}

// Look left
 int lookLeft()
{
    turnLeftMPU();
    bool Object_Detected = checkCamera();
    if (Object_Detected)
    {
        return -1;
    }
    int distance = ultrasonic_Obstecale.read();
    turnRightMPU();
    return distance;
}

// Motor control functions
void moveForward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, X);
    analogWrite(ENB, Y);
}

void stopMotors()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void steady()
{
    stopMotors();
    delay(2000);
}

void storeTraking(int direction, int distance = 0)
{
    if (!process_Done)
    {
        return;
    }

    if (moveCount < MAX_MOVES)
    {

        movementDistances[moveCount] = distance;
        movementDirections[moveCount] = direction;
        moveCount++;
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
            moveDistance(movementDistances[i]);
            break;
        case BACKWARD:
            break;
        case LEFT:
             turnRightMPU();
            break;
        case RIGHT:
           turnLeftMPU();
            break;
        default:
            break;
        }
    }
}

bool checkCamera()
{
    // // Check for incoming data from ESP32-CAM
    // if (Serial1.available())
    // {
    //     String receivedData = Serial1.readStringUntil('\n');
    //     Serial.print("Received from ESP32-CAM: ");
    //     Serial.println(receivedData);
    //     delay(1000); // Delay for demonstration
    //                  // the camera will send "OK" when detect the desired object...
    //     if (receivedData == "OK")
    //     {
    //         return true;
    //     }
    //     return false;
    // }
    // return false;
}

bool check_Sensor()
{

    int obstacleDistance = ultrasonic_Arm.read();
    Serial.println(obstacleDistance);
    if (obstacleDistance == objectDistance)
    {
        return true;
    }
    return false;
}

void Hold_Obj()
{
    delay(1000);
    LIFT(RLU);
    delay(5000);
    LIFT(RLD);
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
            pwm.setPWM(6, 0, angletopulse(ang));
            delay(50);
        }
        for (int ang = 0; ang <= 25; ang++) // hand
        {
            pwm.setPWM(6, 0, angletopulse(ang));
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
            pwm.setPWM(6, 0, angletopulse(ang));
            delay(50);
        }

        for (int ang = 0; ang <= 90; ang++) // hand
        {
            pwm.setPWM(6, 0, angletopulse(ang));
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
            pwm.setPWM(6, 0, angletopulse(ang));
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
            pwm.setPWM(6, 0, angletopulse(ang));
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