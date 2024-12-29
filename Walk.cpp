#include <Wire.h>
#include <MPU6050.h>
#include <Ultrasonic.h>

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

// Obstacle detection threshold in cm
const int obstacleThreshold = 20;

// Movement tracking constants
#define MAX_MOVES 20

// Movement tracking arrays
int movementDistances[MAX_MOVES];
int movementDirections[MAX_MOVES]; // 1 for forward, 2 for left turn, 3 for right turn
int moveCount = 0;

Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN); // Initialize ultrasonic sensor
MPU6050 mpu;

void setup() {
  Serial.begin(9600);

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
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1); // Stop execution
  }
  Serial.println("MPU6050 initialized successfully!");
}

void loop() {
  // Step 1: Move forward 2 meters
  moveDistance(200);

  steady();

  // Step 2: Turn left 90 degrees using MPU6050
  turnLeftMPU();

  // Step 3: Move 3 meters to the left
  moveDistance(300);

  // Stop the robot after completing the task
  stopMotors();
  while (1); // Stop execution
}

// Function to move a specific distance in cm
void moveDistance(int distance) {
  int traveled = 0;

  // Store the movement
  if (moveCount < MAX_MOVES) {
    movementDistances[moveCount] = distance;
    movementDirections[moveCount] = 1; // Forward movement
    moveCount++;
  }

  while (traveled < distance) {
    int obstacleDistance = ultrasonic.read();
    Serial.println(obstacleDistance);

    if (obstacleDistance > 0 && obstacleDistance < obstacleThreshold) {
      // Obstacle detected
      Serial.println("Obstacle detected! Avoiding...");
      avoidObstacle();
    } else {
      // No obstacle, keep moving forward
      moveForward();
      delay(100); // Approximation: 1 cm per 100 ms (adjust as needed)
      traveled += 1;
    }
  }
  stopMotors();
}

// Function to avoid an obstacle
void avoidObstacle() {
  stopMotors();
  delay(1000);

  int rightDistance = lookRight();
  int leftDistance = lookLeft();

  if (rightDistance > leftDistance) {
    turnRightMPU();
  } else {
    turnLeftMPU();
  }

  moveForward();
  delay(1000); // Bypass obstacle

  // Return to original orientation
  if (rightDistance > leftDistance) {
    turnLeftMPU();
  } else {
    turnRightMPU();
  }
}

// Function to calculate yaw using MPU6050
float getYaw() {
  static unsigned long lastTime = 0;
  static float yaw = 0;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;

  int16_t gyroZ = mpu.getRotationZ();
  yaw += (gyroZ / 131.0) * deltaTime; // Integrate gyro Z-axis
  yaw = fmod(yaw + 360, 360); // Keep yaw in range [0, 360)
  return yaw;
}

// Turn left 90 degrees using MPU6050
void turnLeftMPU() {
  float initialYaw = getYaw();
  float targetYaw = fmod(initialYaw + 90, 360);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, x);
  analogWrite(ENB, y);

  while (true) {
    float currentYaw = getYaw();
    if (fabs(currentYaw - targetYaw) < 2.0) { // Allowable error margin
      break;
    }
    delay(10); // Allow time for MPU6050 readings
  }
  stopMotors();
}

// Turn right 90 degrees using MPU6050
void turnRightMPU() {
  float initialYaw = getYaw();
  float targetYaw = fmod(initialYaw - 90 + 360, 360);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, x);
  analogWrite(ENB, y);

  while (true) {
    float currentYaw = getYaw();
    if (fabs(currentYaw - targetYaw) < 2.0) { // Allowable error margin
      break;
    }
    delay(10); // Allow time for MPU6050 readings
  }
  stopMotors();
}

// Look right
int lookRight() {
  turnRightMPU();
  int distance = ultrasonic.read();
  turnLeftMPU();
  return distance;
}

// Look left
int lookLeft() {
  turnLeftMPU();
  int distance = ultrasonic.read();
  turnRightMPU();
  return distance;
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, x);
  analogWrite(ENB, y);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void steady() {
  stopMotors();
  delay(2000);
}
