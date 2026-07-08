/*
  4WD Obstacle-Avoidance Robot with 4-DOF Arm
  Arduino Mega 2560
  PID-only obstacle avoidance steering
  -------------------------------------------
  - 4x DC motors via 2x L298N
  - 4x HC-SR04 ultrasonic sensors (FL, FR, R, L)
  - PCA9685 (I2C 0x40) driving 4 arm servos (Base, Shoulder, Elbow, Wrist)
  - PID steering based on side distance error
  - Non-blocking control loop
  - Extensive Serial debugging
  - Serial command interface for manual testing

  Libraries required: Wire.h, Adafruit_PWMServoDriver.h
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TRIG_FL 22
#define ECHO_FL 23
#define TRIG_FR 24
#define ECHO_FR 25
#define TRIG_R  26
#define ECHO_R  27
#define TRIG_L  28
#define ECHO_L  29

#define L_FRONT_ENA 5
#define L_FRONT_IN1 30
#define L_FRONT_IN2 31
#define L_REAR_ENA  6
#define L_REAR_IN1  32
#define L_REAR_IN2  33
#define R_FRONT_ENA 7
#define R_FRONT_IN1 34
#define R_FRONT_IN2 35
#define R_REAR_ENA  8
#define R_REAR_IN1  36
#define R_REAR_IN2  37

#define PCA9685_ADDR 0x40
#define SERVO_FREQ 50
#define CH_BASE 0
#define CH_SHOULDER 1
#define CH_ELBOW 2
#define CH_WRIST 3

const uint8_t SPEED_BASE   = 90;
const uint8_t SPEED_MAX    = 150;
const uint8_t SPEED_MIN    = 0;
const uint8_t SPEED_TURN_LIMIT = 70;

const float DIST_TARGET_CM   = 25.0;
const float DIST_CAUTION_CM  = 40.0;
const float DIST_STOP_CM     = 18.0;
const float SIDE_MIN_VALID   = 2.0;
const float SIDE_MAX_VALID   = 400.0;

const unsigned long SENSOR_INTERVAL_MS   = 100;
const unsigned long DEBUG_PRINT_INTERVAL = 500;
const unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;

float Kp = 3.5;
float Ki = 0.0;
float Kd = 1.2;

const float INTEGRAL_LIMIT = 50.0;

struct ServoLimits { int minAngle; int maxAngle; int homeAngle; };
const ServoLimits BASE_LIMITS     = {0, 180, 90};
const ServoLimits SHOULDER_LIMITS = {20, 160, 90};
const ServoLimits ELBOW_LIMITS    = {0, 160, 90};
const ServoLimits WRIST_LIMITS    = {0, 180, 90};
const int SERVO_PULSE_MIN = 150;
const int SERVO_PULSE_MAX = 600;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

bool autoMode = true;
unsigned long lastSensorRead = 0;
unsigned long lastDebugPrint = 0;
unsigned long lastPIDTime = 0;
unsigned long lastErrorTime = 0;

struct SensorData {
  float distFL;
  float distFR;
  float distR;
  float distL;
  bool validFL;
  bool validFR;
  bool validR;
  bool validL;
};

SensorData sensors;

float pidIntegral = 0.0;
float lastError = 0.0;

void motorsInit();
void setMotor(int enaPin, int in1Pin, int in2Pin, int speed, bool forward);
void driveForward(uint8_t leftSpeed, uint8_t rightSpeed);
void driveBackward(uint8_t speed);
void pivotLeft(uint8_t speed);
void pivotRight(uint8_t speed);
void stopMotors();
void printMotorCommand(const char* label, int speed, const char* dir);

void sensorsInit();
float readUltrasonicCM(int trigPin, int echoPin, bool &validOut);
void updateAllSensors();
void printDistances();

void armInit();
void setServoAngle(uint8_t channel, int angle, const ServoLimits &limits, const char* name);
void armHome();
void armTest();

void handleSerialCommands();
void printStatus();
void resetPID(const char* reason);
float computePID(float error, float dt);
void runPIDObstacleAvoidance();

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("================================================"));
  Serial.println(F(" 4WD Obstacle-Avoidance Robot - PID Steering Boot"));
  Serial.println(F("================================================"));

  motorsInit();
  sensorsInit();
  armInit();

  stopMotors();
  lastSensorRead = millis();
  lastPIDTime = millis();

  Serial.println(F("[SETUP] Motors initialized."));
  Serial.println(F("[SETUP] Sensors initialized."));
  Serial.println(F("[SETUP] Arm/PCA9685 initialized."));
  Serial.println(F("[SETUP] AUTO mode enabled. Send 'M' for manual stop."));
  Serial.println(F("[SETUP] Commands: A,M,F,B,L,R,X,D,S,H,T,P"));
}

void loop() {
  handleSerialCommands();

  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    updateAllSensors();
  }

  if (now - lastDebugPrint >= DEBUG_PRINT_INTERVAL) {
    lastDebugPrint = now;
    printDistances();
    printStatus();
  }

  if (autoMode) runPIDObstacleAvoidance();
}

void motorsInit() {
  pinMode(L_FRONT_ENA, OUTPUT); pinMode(L_FRONT_IN1, OUTPUT); pinMode(L_FRONT_IN2, OUTPUT);
  pinMode(L_REAR_ENA, OUTPUT);  pinMode(L_REAR_IN1, OUTPUT);  pinMode(L_REAR_IN2, OUTPUT);
  pinMode(R_FRONT_ENA, OUTPUT); pinMode(R_FRONT_IN1, OUTPUT); pinMode(R_FRONT_IN2, OUTPUT);
  pinMode(R_REAR_ENA, OUTPUT);  pinMode(R_REAR_IN1, OUTPUT);  pinMode(R_REAR_IN2, OUTPUT);
  stopMotors();
}

void setMotor(int enaPin, int in1Pin, int in2Pin, int speed, bool forward) {
  speed = constrain(speed, 0, 255);
  if (forward) { digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); }
  else { digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); }
  analogWrite(enaPin, speed);
}

void stopAllMotorPins() {
  digitalWrite(L_FRONT_IN1, LOW); digitalWrite(L_FRONT_IN2, LOW);
  digitalWrite(L_REAR_IN1, LOW);  digitalWrite(L_REAR_IN2, LOW);
  digitalWrite(R_FRONT_IN1, LOW); digitalWrite(R_FRONT_IN2, LOW);
  digitalWrite(R_REAR_IN1, LOW);  digitalWrite(R_REAR_IN2, LOW);
  analogWrite(L_FRONT_ENA, 0); analogWrite(L_REAR_ENA, 0);
  analogWrite(R_FRONT_ENA, 0); analogWrite(R_REAR_ENA, 0);
}

void driveForward(uint8_t leftSpeed, uint8_t rightSpeed) {
  setMotor(L_FRONT_ENA, L_FRONT_IN1, L_FRONT_IN2, leftSpeed, true);
  setMotor(L_REAR_ENA,  L_REAR_IN1,  L_REAR_IN2,  leftSpeed, true);
  setMotor(R_FRONT_ENA, R_FRONT_IN1, R_FRONT_IN2, rightSpeed, true);
  setMotor(R_REAR_ENA,  R_REAR_IN1,  R_REAR_IN2,  rightSpeed, true);
  Serial.print(F("[MOTOR] FORWARD L=")); Serial.print(leftSpeed); Serial.print(F(" R=")); Serial.println(rightSpeed);
}

void driveBackward(uint8_t speed) {
  setMotor(L_FRONT_ENA, L_FRONT_IN1, L_FRONT_IN2, speed, false);
  setMotor(L_REAR_ENA,  L_REAR_IN1,  L_REAR_IN2,  speed, false);
  setMotor(R_FRONT_ENA, R_FRONT_IN1, R_FRONT_IN2, speed, false);
  setMotor(R_REAR_ENA,  R_REAR_IN1,  R_REAR_IN2,  speed, false);
  printMotorCommand("ALL", speed, "BACKWARD");
}

void pivotLeft(uint8_t speed) {
  setMotor(L_FRONT_ENA, L_FRONT_IN1, L_FRONT_IN2, speed, false);
  setMotor(L_REAR_ENA,  L_REAR_IN1,  L_REAR_IN2,  speed, false);
  setMotor(R_FRONT_ENA, R_FRONT_IN1, R_FRONT_IN2, speed, true);
  setMotor(R_REAR_ENA,  R_REAR_IN1,  R_REAR_IN2,  speed, true);
  printMotorCommand("TURN", speed, "LEFT");
}

void pivotRight(uint8_t speed) {
  setMotor(L_FRONT_ENA, L_FRONT_IN1, L_FRONT_IN2, speed, true);
  setMotor(L_REAR_ENA,  L_REAR_IN1,  L_REAR_IN2,  speed, true);
  setMotor(R_FRONT_ENA, R_FRONT_IN1, R_FRONT_IN2, speed, false);
  setMotor(R_REAR_ENA,  R_REAR_IN1,  R_REAR_IN2,  speed, false);
  printMotorCommand("TURN", speed, "RIGHT");
}

void stopMotors() {
  stopAllMotorPins();
  printMotorCommand("ALL", 0, "STOP");
}

void printMotorCommand(const char* label, int speed, const char* dir) {
  Serial.print(F("[MOTOR] ")); Serial.print(label); Serial.print(F(" -> dir="));
  Serial.print(dir); Serial.print(F(" speed=")); Serial.println(speed);
}

void sensorsInit() {
  pinMode(TRIG_FL, OUTPUT); pinMode(ECHO_FL, INPUT);
  pinMode(TRIG_FR, OUTPUT); pinMode(ECHO_FR, INPUT);
  pinMode(TRIG_R, OUTPUT);  pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT);  pinMode(ECHO_L, INPUT);
  digitalWrite(TRIG_FL, LOW); digitalWrite(TRIG_FR, LOW);
  digitalWrite(TRIG_R, LOW);  digitalWrite(TRIG_L, LOW);
  sensors.distFL = sensors.distFR = sensors.distR = sensors.distL = -1;
  sensors.validFL = sensors.validFR = sensors.validR = sensors.validL = false;
}

float readUltrasonicCM(int trigPin, int echoPin, bool &validOut) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) { validOut = false; return -1.0; }
  float distanceCM = (duration * 0.0343) / 2.0;
  if (distanceCM < SIDE_MIN_VALID || distanceCM > SIDE_MAX_VALID) { validOut = false; return distanceCM; }
  validOut = true; return distanceCM;
}

void updateAllSensors() {
  sensors.distFL = readUltrasonicCM(TRIG_FL, ECHO_FL, sensors.validFL); delayMicroseconds(500);
  sensors.distFR = readUltrasonicCM(TRIG_FR, ECHO_FR, sensors.validFR); delayMicroseconds(500);
  sensors.distR  = readUltrasonicCM(TRIG_R, ECHO_R, sensors.validR);   delayMicroseconds(500);
  sensors.distL  = readUltrasonicCM(TRIG_L, ECHO_L, sensors.validL);
}

void printDistances() {
  Serial.println(F("---- Distances (cm) ----"));
  Serial.print(F("FL: ")); if (sensors.validFL) Serial.print(sensors.distFL); else Serial.print(F("INVALID"));
  Serial.print(F("  FR: ")); if (sensors.validFR) Serial.print(sensors.distFR); else Serial.print(F("INVALID"));
  Serial.print(F("  R: ")); if (sensors.validR) Serial.print(sensors.distR); else Serial.print(F("INVALID"));
  Serial.print(F("  L: ")); if (sensors.validL) Serial.println(sensors.distL); else Serial.println(F("INVALID"));
}

float computePID(float error, float dt) {
  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = (dt > 0.0) ? ((error - lastError) / dt) : 0.0;
  lastError = error;
  return (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
}

void resetPID(const char* reason) {
  pidIntegral = 0.0;
  lastError = 0.0;
  Serial.print(F("[PID] Reset: ")); Serial.println(reason);
}

void runPIDObstacleAvoidance() {
  float leftDist = sensors.validL ? sensors.distL : (sensors.validFL ? sensors.distFL : DIST_TARGET_CM);
  float rightDist = sensors.validR ? sensors.distR : (sensors.validFR ? sensors.distFR : DIST_TARGET_CM);

  bool frontTooClose = false;
  if (sensors.validFL && sensors.distFL <= DIST_STOP_CM) frontTooClose = true;
  if (sensors.validFR && sensors.distFR <= DIST_STOP_CM) frontTooClose = true;
  if (!sensors.validFL && !sensors.validFR) frontTooClose = true;

  if (frontTooClose) {
    resetPID("front blocked");
    driveBackward(90);
    delay(250);
    stopMotors();
    if (leftDist > rightDist) pivotLeft(85); else pivotRight(85);
    delay(350);
    stopMotors();
    return;
  }

  float error = rightDist - leftDist;
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt <= 0.0) dt = 0.1;
  lastPIDTime = now;

  float pidOut = computePID(error, dt);
  int turn = (int)constrain(pidOut, -SPEED_TURN_LIMIT, SPEED_TURN_LIMIT);

  int leftSpeed = SPEED_BASE - turn;
  int rightSpeed = SPEED_BASE + turn;
  leftSpeed = constrain(leftSpeed, SPEED_MIN, SPEED_MAX);
  rightSpeed = constrain(rightSpeed, SPEED_MIN, SPEED_MAX);

  if (leftDist < DIST_CAUTION_CM || rightDist < DIST_CAUTION_CM) {
    leftSpeed = min(leftSpeed, (int)SPEED_BASE);
    rightSpeed = min(rightSpeed, (int)SPEED_BASE);
  }

  driveForward((uint8_t)leftSpeed, (uint8_t)rightSpeed);

  Serial.print(F("[PID] L=")); Serial.print(leftDist);
  Serial.print(F(" R=")); Serial.print(rightDist);
  Serial.print(F(" err=")); Serial.print(error);
  Serial.print(F(" out=")); Serial.print(pidOut);
  Serial.print(F(" turn=")); Serial.print(turn);
  Serial.print(F(" LS=")); Serial.print(leftSpeed);
  Serial.print(F(" RS=")); Serial.println(rightSpeed);
}

void armInit() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  Serial.println(F("[ARM] PCA9685 initialized at 50Hz."));
  armHome();
}

void setServoAngle(uint8_t channel, int angle, const ServoLimits &limits, const char* name) {
  int clamped = constrain(angle, limits.minAngle, limits.maxAngle);
  int pulse = map(clamped, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  pwm.setPWM(channel, 0, pulse);
  Serial.print(F("[SERVO] ")); Serial.print(name); Serial.print(F(" -> angle="));
  Serial.print(clamped); Serial.print(F(" pulse=")); Serial.println(pulse);
}

void armHome() {
  setServoAngle(CH_BASE, BASE_LIMITS.homeAngle, BASE_LIMITS, "Base");
  delay(150);
  setServoAngle(CH_SHOULDER, SHOULDER_LIMITS.homeAngle, SHOULDER_LIMITS, "Shoulder");
  delay(150);
  setServoAngle(CH_ELBOW, ELBOW_LIMITS.homeAngle, ELBOW_LIMITS, "Elbow");
  delay(150);
  setServoAngle(CH_WRIST, WRIST_LIMITS.homeAngle, WRIST_LIMITS, "Wrist");
  delay(150);
}

void armTest() {
  setServoAngle(CH_BASE, BASE_LIMITS.homeAngle + 20, BASE_LIMITS, "Base"); delay(300);
  setServoAngle(CH_BASE, BASE_LIMITS.homeAngle, BASE_LIMITS, "Base"); delay(300);
  setServoAngle(CH_SHOULDER, SHOULDER_LIMITS.homeAngle + 15, SHOULDER_LIMITS, "Shoulder"); delay(300);
  setServoAngle(CH_SHOULDER, SHOULDER_LIMITS.homeAngle, SHOULDER_LIMITS, "Shoulder"); delay(300);
  setServoAngle(CH_ELBOW, ELBOW_LIMITS.homeAngle + 15, ELBOW_LIMITS, "Elbow"); delay(300);
  setServoAngle(CH_ELBOW, ELBOW_LIMITS.homeAngle, ELBOW_LIMITS, "Elbow"); delay(300);
  setServoAngle(CH_WRIST, WRIST_LIMITS.homeAngle + 15, WRIST_LIMITS, "Wrist"); delay(300);
  setServoAngle(CH_WRIST, WRIST_LIMITS.homeAngle, WRIST_LIMITS, "Wrist"); delay(300);
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  if (cmd == '\n' || cmd == '\r') return;

  switch (cmd) {
    case 'A': case 'a':
      autoMode = true;
      resetPID("AUTO enabled");
      Serial.println(F("[CMD] AUTO mode ENABLED."));
      break;
    case 'M': case 'm':
      autoMode = false;
      stopMotors();
      Serial.println(F("[CMD] MANUAL mode."));
      break;
    case 'F': case 'f':
      autoMode = false; driveForward(90, 90); delay(3000); stopMotors(); break;
    case 'B': case 'b':
      autoMode = false; driveBackward(90); delay(3000); stopMotors(); break;
    case 'L': case 'l':
      autoMode = false; pivotLeft(85); delay(1000); stopMotors(); break;
    case 'R': case 'r':
      autoMode = false; pivotRight(85); delay(1000); stopMotors(); break;
    case 'X': case 'x':
      autoMode = false; stopMotors(); break;
    case 'D': case 'd':
      updateAllSensors(); printDistances(); break;
    case 'S': case 's':
      printStatus(); break;
    case 'H': case 'h':
      armHome(); break;
    case 'T': case 't':
      armTest(); break;
    case 'P': case 'p':
      Kp += 0.5; Serial.print(F("[PID] Kp=")); Serial.println(Kp); break;
    default:
      Serial.println(F("[CMD] Valid commands: A,M,F,B,L,R,X,D,S,H,T,P"));
      break;
  }
}

void printStatus() {
  Serial.println(F("================ STATUS ================"));
  Serial.print(F("Auto mode: ")); Serial.println(autoMode ? "ENABLED" : "DISABLED");
  Serial.print(F("Kp: ")); Serial.println(Kp);
  Serial.print(F("Ki: ")); Serial.println(Ki);
  Serial.print(F("Kd: ")); Serial.println(Kd);
  Serial.print(F("Target distance: ")); Serial.println(DIST_TARGET_CM);
  Serial.print(F("Current left dist: ")); Serial.println(sensors.validL ? sensors.distL : -1);
  Serial.print(F("Current right dist: ")); Serial.println(sensors.validR ? sensors.distR : -1);
  Serial.println(F("========================================"));
}
