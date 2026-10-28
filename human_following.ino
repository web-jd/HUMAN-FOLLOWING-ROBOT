
#include <NewPing.h>
#include <Servo.h>

// ================ SENSOR PINS ================
#define TRIG_PIN 11
#define ECHO_PIN 12
#define IR_RIGHT 2
#define IR_LEFT  3

// ================ SERVO ================
#define SERVO_PIN 4
#define SERVO_CENTER 90
#define SERVO_LEFT 45
#define SERVO_RIGHT 135

Servo scanServo;

// ================ L298N MOTOR PINS ================
// Right motor: OUT1 and OUT2
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;

// Left motor: OUT3 and OUT4
const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

// ================ SETTINGS ================
const int MAX_DISTANCE = 200;

const int MIN_FOLLOW_DISTANCE = 12;
const int MAX_FOLLOW_DISTANCE = 35;
const int OBSTACLE_DISTANCE = 15;

const int MIN_SPEED = 110;
const int MAX_SPEED = 190;
const int TURN_SPEED = 145;
const int REVERSE_SPEED = 125;
const int SEARCH_SPEED = 115;

const unsigned long SENSOR_INTERVAL = 60;
const unsigned long LOST_DELAY = 700;
const unsigned long SEARCH_TIMEOUT = 5000;
const unsigned long REVERSE_TIME = 300;
const unsigned long TURN_TIME = 450;

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// ================ ROBOT STATES ================
enum RobotState {
  FOLLOW,
  AVOID_REVERSE,
  AVOID_TURN,
  SEARCH,
  STOPPED
};

RobotState state = FOLLOW;

unsigned long lastSensorTime = 0;
unsigned long stateStartTime = 0;
unsigned long lostStartTime = 0;

int distanceCm = 0;
int leftDistance = 0;
int rightDistance = 0;

// Direction convention:
// +1 = pivot left
// -1 = pivot right
int lastDirection = 1;
int avoidDirection = 1;

bool rightDetected = false;
bool leftDetected = false;

// ================ SETUP ================
void setup() {
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);
  delay(500);

  stopMotors();

  Serial.println("Human Following Robot Ready");
  Serial.println("Servo centred at 90 degrees");
}

// ================ MAIN LOOP ================
void loop() {
  unsigned long now = millis();

  // Finish an active reverse maneuver.
  if (state == AVOID_REVERSE) {
    if (now - stateStartTime < REVERSE_TIME) {
      moveMotors(-REVERSE_SPEED, -REVERSE_SPEED);
      return;
    }

    stopMotors();

    // Scan the surroundings only when avoiding an obstacle.
    avoidDirection = scanAndChooseDirection();

    state = AVOID_TURN;
    stateStartTime = millis();
    return;
  }

  // Turn toward the side with more measured clearance.
  if (state == AVOID_TURN) {
    if (millis() - stateStartTime < TURN_TIME) {
      moveMotors(
        avoidDirection * TURN_SPEED,
        -avoidDirection * TURN_SPEED
      );
      return;
    }

    stopMotors();
    scanServo.write(SERVO_CENTER);

    state = FOLLOW;
    lostStartTime = 0;
    lastSensorTime = 0;
    return;
  }

  // Read sensors at a controlled interval.
  if (now - lastSensorTime < SENSOR_INTERVAL) {
    return;
  }

  lastSensorTime = now;

  readSensors();
  printStatus();

  // Obstacle avoidance:
  // Only trigger when the target is not detected by either IR sensor.
  if (distanceCm > 0 &&
      distanceCm <= OBSTACLE_DISTANCE &&
      !rightDetected &&
      !leftDetected) {

    stopMotors();

    state = AVOID_REVERSE;
    stateStartTime = millis();

    // Return to forward position before the maneuver.
    scanServo.write(SERVO_CENTER);

    lostStartTime = 0;

    Serial.println("Obstacle detected: reversing");
    return;
  }

  // Remember the last detected side.
  if (rightDetected && !leftDetected) {
    lastDirection = -1;  // Search/turn right
  } else if (leftDetected && !rightDetected) {
    lastDirection = 1;   // Search/turn left
  }

  // Target is visible through the IR sensors or is
  // at a valid distance in front of the robot.
  bool targetVisible =
      rightDetected ||
      leftDetected ||
      (distanceCm >= MIN_FOLLOW_DISTANCE &&
       distanceCm <= MAX_FOLLOW_DISTANCE);

  if (targetVisible) {
    lostStartTime = 0;
    state = FOLLOW;

    // Keep the sensor pointing forward while following.
    scanServo.write(SERVO_CENTER);

    // Turn toward the detected hand.
    if (rightDetected && !leftDetected) {
      moveMotors(-TURN_SPEED, TURN_SPEED);
      return;
    }

    if (leftDetected && !rightDetected) {
      moveMotors(TURN_SPEED, -TURN_SPEED);
      return;
    }

    // Both sensors detect the hand, or distance alone
    // indicates the target is in front.
    if (distanceCm >= MIN_FOLLOW_DISTANCE &&
        distanceCm <= MAX_FOLLOW_DISTANCE) {

      int speed = map(
        distanceCm,
        MIN_FOLLOW_DISTANCE,
        MAX_FOLLOW_DISTANCE,
        MIN_SPEED,
        MAX_SPEED
      );

      speed = constrain(speed, MIN_SPEED, MAX_SPEED);

      moveMotors(speed, speed);
    } else {
      stopMotors();
    }

    return;
  }

  // Target has been lost.
  if (lostStartTime == 0) {
    lostStartTime = millis();
    stopMotors();
    state = SEARCH;
  }

  unsigned long lostDuration = millis() - lostStartTime;

  // Pause before searching.
  if (lostDuration < LOST_DELAY) {
    stopMotors();
    return;
  }

  // Stop searching after the timeout.
  if (lostDuration >= LOST_DELAY + SEARCH_TIMEOUT) {
    stopMotors();
    state = STOPPED;
    Serial.println("Target lost: stopped");
    return;
  }

  // Search in the last known direction.
  state = SEARCH;

  scanServo.write(SERVO_CENTER);

  moveMotors(
    lastDirection * SEARCH_SPEED,
    -lastDirection * SEARCH_SPEED
  );
}

// ================ SENSOR FILTERING ================
void readSensors() {
  // Majority vote for active-low IR sensors.
  int rightLowCount = 0;
  int leftLowCount = 0;

  for (int i = 0; i < 3; i++) {
    if (digitalRead(IR_RIGHT) == LOW) {
      rightLowCount++;
    }

    if (digitalRead(IR_LEFT) == LOW) {
      leftLowCount++;
    }

    delay(2);
  }

  rightDetected = (rightLowCount >= 2);
  leftDetected = (leftLowCount >= 2);

  // Median of three valid ultrasonic readings.
  int readings[3];
  int count = 0;

  for (int i = 0; i < 3; i++) {
    unsigned int d = sonar.ping_cm();

    if (d > 0 && d <= MAX_DISTANCE) {
      readings[count++] = (int)d;
    }

    delay(25);
  }

  if (count == 0) {
    distanceCm = 0;
    return;
  }

  // Sort readings from smallest to largest.
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (readings[j] < readings[i]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }

  distanceCm = readings[count / 2];
}

// ================ SERVO OBSTACLE SCAN ================
int scanAndChooseDirection() {
  int leftReading = 0;
  int rightReading = 0;

  stopMotors();

  // 1. Scan left.
  scanServo.write(SERVO_LEFT);
  delay(400);
  leftReading = readSingleDistance();

  Serial.print("Left clearance: ");
  Serial.println(leftReading);

  // 2. Scan right.
  scanServo.write(SERVO_RIGHT);
  delay(500);
  rightReading = readSingleDistance();

  Serial.print("Right clearance: ");
  Serial.println(rightReading);

  // 3. Always return to the forward-facing position.
  scanServo.write(SERVO_CENTER);
  delay(350);

  // A zero reading means no echo was received.
  // Treat it as unknown, not guaranteed clear space.
  if (leftReading == 0 && rightReading == 0) {
    Serial.println("Both sides unknown: using last direction");
    return lastDirection;
  }

  if (leftReading == 0) {
    return -1;  // Right side has a valid reading
  }

  if (rightReading == 0) {
    return 1;   // Left side has a valid reading
  }

  // Choose the side with greater measured distance.
  if (leftReading > rightReading) {
    return 1;   // Pivot left
  }

  if (rightReading > leftReading) {
    return -1;  // Pivot right
  }

  return lastDirection;
}

// One distance measurement at the current servo angle.
int readSingleDistance() {
  unsigned int d = sonar.ping_cm();

  if (d == 0 || d > MAX_DISTANCE) {
    return 0;
  }

  return (int)d;
}

// ================ MOTOR CONTROL ================
// First value = right motor; second value = left motor.
// Positive = forward; negative = reverse.
void moveMotors(int rightSpeed, int leftSpeed) {
  rightSpeed = constrain(rightSpeed, -255, 255);
  leftSpeed = constrain(leftSpeed, -255, 255);

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(rightSpeed));
  analogWrite(ENB, abs(leftSpeed));
}

// ================ STOP MOTORS ================
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ================ SERIAL DIAGNOSTICS ================
void printStatus() {
  Serial.print("Right IR: ");
  Serial.print(rightDetected ? "DETECTED" : "CLEAR");

  Serial.print(" | Left IR: ");
  Serial.print(leftDetected ? "DETECTED" : "CLEAR");

  Serial.print(" | Front distance: ");

  if (distanceCm == 0) {
    Serial.print("NO ECHO");
  } else {
    Serial.print(distanceCm);
    Serial.print(" cm");
  }

  Serial.print(" | Mode: ");

  switch (state) {
    case FOLLOW:
      Serial.println("FOLLOW");
      break;

    case AVOID_REVERSE:
      Serial.println("AVOID REVERSE");
      break;

    case AVOID_TURN:
      Serial.println("AVOID TURN");
      break;

    case SEARCH:
      Serial.println("SEARCH");
      break;

    case STOPPED:
      Serial.println("STOPPED");
      break;
  }
}
