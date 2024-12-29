#include <Wire.h>
#include <MPU6050.h>
#include <Ultrasonic.h>
#include <HardwareSerial.h>
#include <stack>
#include <string>
// Motor control pins
#define x 150
#define y 150
#define TRIG_PIN 12
#define ECHO_PIN 11
#define ENA 5
#define IN1 7
#define IN2 6
#define ENB 10
#define IN3 8
#define IN4 9
// new
#define FORWARD 1
#define BACKWARD -1
#define LEFT 2
#define RIGHT -2
#define TIME_BYPASS_OBSTACLE 100

using namespace std;
// Obstacle detection threshold in cm
const int obstacleThreshold = 20;
bool process_Done = false;
bool swap_left_right = true;

// Movement tracking constants
#define MAX_MOVES 100

// Movement tracking arrays
stack<int> movementDistances;  // 0 distance for turn left or right
stack<int> movementDirections; // 1 for forward, -1 for backward ,2 for left turn, -2 for right turn

int moveCount = 0;

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN); // Initialize ultrasonic sensor
MPU6050 mpu;
HardwareSerial Serial1(1); // Use UART1

void setup()
{
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, 16, 17); // UART1 on GPIO16 (RX) and GPIO17 (TX)
    Serial.println("ESP32-CAM Peripheral Initialized");
    // Set motor pins as outputs
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (1)
            ; // Stop execution
    }
    Serial.println("MPU6050 initialized successfully!");
}

void loop()
{
    // Move forward 2 meters
    moveDistance(200);

    steady();

    // check and read from the camera
    bool Object_Detected = checkCamera();

    if (Object_Detected)
    {
        stopMotors();
        while (true)
        {
            // check the sensor
            bool isInScope = checkSensor()
            if (IN_Scope)
            {
                // the code of arm ...
                process_Done = true;
                break;
            }
            else
            {
                moveDistance(20); // 20 cm
            }
        }
    }
    if (process_Done)
    {
        // execute the functions that is responsible of returning to the sarting site
        turnLeftMPU();
        turnLeftMPU();
        returnToStart();
        while ()
            ; // the end
    }

    // once turn left and once turn right
    if (swap_left_right)
    {
        turnLeftMPU();
        storeTraking(LEFT, 0) !swap_left_right;
    }
    else
    {
        turnRightMPU();
        storeTraking(RIGHT, 0) !swap_left_right;
    }
}

// Function to move a specific distance in cm
void moveDistance(int distance)
{
    int traveled = 0;

    while (traveled < distance)
    {
        int obstacleDistance = ultrasonic.read();
        Serial.println(obstacleDistance);

        if (obstacleDistance > 0 && obstacleDistance < obstacleThreshold)
        {
            // Obstacle detected
            Serial.println("Obstacle detected! Avoiding...");
            //...
            avoidObstacle();
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
void avoidObstacle()
{
    stopMotors();
    delay(1000);

    int rightDistance = lookRight();
    int leftDistance = lookLeft();

    if (rightDistance > leftDistance)
    {
        turnRightMPU();
        storeTraking(RIGHT, TIME_BYPASS_OBSTACLE);
    }
    else
    {
        turnLeftMPU();
        storeTraking(LEFT, TIME_BYPASS_OBSTACLE);
    }

    moveForward();
    delay(TIME_BYPASS_OBSTACLE); // Bypass obstacle
    storeTraking(FORWARD, TIME_BYPASS_OBSTACLE);

    // Return to original orientation
    if (rightDistance > leftDistance)
    {
        turnLeftMPU();
        storeTraking(LEFT, 0);
    }
    else
    {
        turnRightMPU();
        storeTraking(RIGHT, 0);
    }
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
    analogWrite(ENA, x);
    analogWrite(ENB, y);

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
    analogWrite(ENA, x);
    analogWrite(ENB, y);

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
    int distance = ultrasonic.read();
    turnLeftMPU();
    return distance;
}

// Look left
int lookLeft()
{
    turnLeftMPU();
    int distance = ultrasonic.read();
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
    analogWrite(ENA, x);
    analogWrite(ENB, y);
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

void storeTraking(int direction, int distance)
{
    if (!process_Done)
    {
        return;
    }

    if (moveCount < MAX_MOVES)
    {

        movementDistances.push(distance);
        movementDirections.push(direction);
        moveCount++;
    }
}

void returnToStart()
{
    Serial.println("returnToStart...");

    while (!movementDirections.empty())
    {
        switch (movementDirections.top())
        {
        case FORWARD:
            moveDistance(movementDistances.top());
            break;
        case BACKWARD:
            break;
        case LEFT:
            turnLeftMPU();
            break;
        case RIGHT:
            turnRightMPU();
            break;
        }
    }
}

bool checkCamera()
{
    // Check for incoming data from ESP32-CAM
    if (Serial1.available())
    {
        string receivedData = Serial1.readStringUntil('\n');
        Serial.print("Received from ESP32-CAM: ");
        Serial.println(receivedData);
        delay(1000); // Delay for demonstration

        if (receivedData == "OK")
        {
            return true;
        }
        return false;
    }
    delay(1000); // Delay for demonstration
    return false;
}