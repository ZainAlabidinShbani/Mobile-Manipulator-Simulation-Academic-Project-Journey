#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== Hardware map =====================
const uint8_t PIN_I2C_SDA = 20;
const uint8_t PIN_I2C_SCL = 21;

// Ultrasonic sensors
const uint8_t TRIG_FRONT_LEFT   = 22;
const uint8_t ECHO_FRONT_LEFT   = 23;

const uint8_t TRIG_FRONT_RIGHT  = 24;
const uint8_t ECHO_FRONT_RIGHT  = 25;

const uint8_t TRIG_RIGHT        = 26;
const uint8_t ECHO_RIGHT        = 27;

const uint8_t TRIG_LEFT         = 28;
const uint8_t ECHO_LEFT         = 29;

// ===================== L298N - LEFT SIDE =====================
// Left motor 1
const uint8_t LEFT_FRONT_ENA    = 5;
const uint8_t LEFT_FRONT_IN1    = 30;
const uint8_t LEFT_FRONT_IN2    = 31;

// Left motor 2
const uint8_t LEFT_REAR_ENA     = 6;
const uint8_t LEFT_REAR_IN1     = 32;
const uint8_t LEFT_REAR_IN2     = 33;

// ===================== L298N - RIGHT SIDE =====================
const uint8_t RIGHT_FRONT_ENA   = 7;
const uint8_t RIGHT_FRONT_IN1   = 34;
const uint8_t RIGHT_FRONT_IN2   = 35;

const uint8_t RIGHT_REAR_ENA    = 8;
const uint8_t RIGHT_REAR_IN1    = 36;
const uint8_t RIGHT_REAR_IN2    = 37;

// ===================== Servo channels (kept for future) =====================
const uint8_t SERVO_BASE        = 0;
const uint8_t SERVO_SHOULDER    = 1;
const uint8_t SERVO_ELBOW       = 2;
const uint8_t SERVO_WRIST       = 3;

const uint8_t SERVO_COUNT = 4;
const uint8_t servoChannel[SERVO_COUNT] = {
  SERVO_BASE, SERVO_SHOULDER, SERVO_ELBOW, SERVO_WRIST
};

const int servoHome[SERVO_COUNT]     = {90, 90, 90, 90};
int       servoCurrent[SERVO_COUNT]  = {90, 90, 90, 90};

const int servoMinAngle[SERVO_COUNT] = {0, 15, 15, 0};
const int servoMaxAngle[SERVO_COUNT] = {180, 165, 165, 180};

const uint16_t SERVO_FREQ      = 50;
const uint16_t SERVO_MIN_PULSE = 102;
const uint16_t SERVO_MAX_PULSE = 512;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ===================== Ultrasonic settings =====================
const unsigned long ECHO_TIMEOUT_US   = 30000UL;
const unsigned long SENSOR_GAP_MS     = 18;
const long          DIST_INVALID      = -1;
const int           DIST_MAX_VALID_CM = 250;

// ===================== Motion settings =====================
const int BASE_SPEED       = 100;
const int CRUISE_SPEED     = 112;
const int CAUTION_SPEED    = 92;
const int REVERSE_SPEED    = 92;
const int PIVOT_SPEED      = 105;
const int MAX_CMD_SPEED    = 140;
const int MIN_CMD_SPEED    = 75;

// ===================== Avoidance thresholds =====================
const int FRONT_BLOCK_CM   = 26;
const int FRONT_CAUTION_CM = 45;
const int FRONT_CLEAR_CM   = 60;
const int SIDE_SAFE_CM     = 25;
const int SIDE_OPEN_CM     = 40;
const int VERY_CLOSE_CM    = 12;

// ===================== Timing =====================
const unsigned long CONTROL_DT_MS      = 70;
const unsigned long STATUS_PRINT_MS    = 450;
const unsigned long STOP_BEFORE_BACK_MS = 120;
const unsigned long BACKUP_TIME_MS     = 320;
const unsigned long PIVOT_TIME_MS      = 360;
const unsigned long RECOVERY_WAIT_MS   = 90;

// ===================== State machine =====================
enum RobotState {
  STATE_STARTUP,
  STATE_CRUISE,
  STATE_CAUTION,
  STATE_AVOID_BACKUP,
  STATE_AVOID_TURN
};

enum TurnDirection {
  TURN_LEFT,
  TURN_RIGHT
};

// ===================== Motor structures =====================
struct MotorPins {
  uint8_t ena;
  uint8_t in1;
  uint8_t in2;
};

enum MotorIndex {
  MOTOR_LEFT_FRONT = 0,
  MOTOR_RIGHT_FRONT,
  MOTOR_LEFT_REAR,
  MOTOR_RIGHT_REAR,
  MOTOR_COUNT
};

const MotorPins motorPins[MOTOR_COUNT] = {
  {LEFT_FRONT_ENA,  LEFT_FRONT_IN1,  LEFT_FRONT_IN2},
  {RIGHT_FRONT_ENA, RIGHT_FRONT_IN1, RIGHT_FRONT_IN2},
  {LEFT_REAR_ENA,   LEFT_REAR_IN1,   LEFT_REAR_IN2},
  {RIGHT_REAR_ENA,  RIGHT_REAR_IN1,  RIGHT_REAR_IN2}
};

// ===================== Sensor data =====================
struct SensorReadings {
  long frontLeft;
  long frontRight;
  long right;
  long left;
  long frontAvg;
  long leftScore;
  long rightScore;
};

SensorReadings sense;

// ===================== Runtime =====================
RobotState currentState = STATE_STARTUP;
TurnDirection lastTurn = TURN_RIGHT;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;
unsigned long stateStartTime = 0;

bool autoMode = true;
bool armLocked = true;

int backDir = 1;

// ===================== Utility =====================
long clampLong(long x, long lo, long hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

long sanitizeDistance(long d) {
  if (d <= 0 || d > DIST_MAX_VALID_CM) return DIST_INVALID;
  return d;
}

long median3(long a, long b, long c) {
  if (a > b) { long t = a; a = b; b = t; }
  if (b > c) { long t = b; b = c; c = t; }
  if (a > b) { long t = a; a = b; b = t; }
  return b;
}

// ===================== Servo functions (kept for future) =====================
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void writeServo(uint8_t index, int angle) {
  if (index >= SERVO_COUNT) return;
  angle = constrain(angle, servoMinAngle[index], servoMaxAngle[index]);
  servoCurrent[index] = angle;
  pca.setPWM(servoChannel[index], 0, angleToPulse(angle));
}

void lockArmHome() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (servoCurrent[i] != servoHome[i]) {
      writeServo(i, servoHome[i]);
      delay(60);
    }
  }
}

void moveAllHome() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    writeServo(i, servoHome[i]);
    delay(180);
  }
}

void startupServoTest() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    int testAngle = servoHome[i] + 15;
    if (testAngle > servoMaxAngle[i]) testAngle = servoHome[i] - 15;
    testAngle = constrain(testAngle, servoMinAngle[i], servoMaxAngle[i]);
    writeServo(i, testAngle);
    delay(500);
    writeServo(i, servoHome[i]);
    delay(250);
  }
}

// ===================== Ultrasonic functions =====================
void initUltrasonicPins() {
  pinMode(TRIG_FRONT_LEFT, OUTPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);

  pinMode(TRIG_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  digitalWrite(TRIG_FRONT_LEFT, LOW);
  digitalWrite(TRIG_FRONT_RIGHT, LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  digitalWrite(TRIG_LEFT, LOW);
}

long readDistanceRawCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return DIST_INVALID;

  long distance = duration / 58;
  if (distance < 2 || distance > 400) return DIST_INVALID;
  return distance;
}

long readDistanceFilteredCM(uint8_t trigPin, uint8_t echoPin) {
  long a = readDistanceRawCM(trigPin, echoPin);
  delay(SENSOR_GAP_MS);
  long b = readDistanceRawCM(trigPin, echoPin);
  delay(SENSOR_GAP_MS);
  long c = readDistanceRawCM(trigPin, echoPin);

  if (a == DIST_INVALID && b == DIST_INVALID && c == DIST_INVALID) return DIST_INVALID;
  if (a == DIST_INVALID) a = DIST_MAX_VALID_CM;
  if (b == DIST_INVALID) b = DIST_MAX_VALID_CM;
  if (c == DIST_INVALID) c = DIST_MAX_VALID_CM;

  return median3(a, b, c);
}

void updateSensors() {
  sense.frontLeft  = readDistanceFilteredCM(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  delay(SENSOR_GAP_MS);
  sense.frontRight = readDistanceFilteredCM(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  delay(SENSOR_GAP_MS);
  sense.right      = readDistanceFilteredCM(TRIG_RIGHT, ECHO_RIGHT);
  delay(SENSOR_GAP_MS);
  sense.left       = readDistanceFilteredCM(TRIG_LEFT, ECHO_LEFT);

  if (sense.frontLeft == DIST_INVALID && sense.frontRight == DIST_INVALID) {
    sense.frontAvg = DIST_INVALID;
  } else {
    long fl = (sense.frontLeft == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontLeft;
    long fr = (sense.frontRight == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontRight;
    sense.frontAvg = (fl + fr) / 2;
  }

  long ls = (sense.left == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.left;
  long rs = (sense.right == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.right;
  long fl = (sense.frontLeft == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontLeft;
  long fr = (sense.frontRight == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontRight;

  sense.leftScore = ls + (fl / 2);
  sense.rightScore = rs + (fr / 2);
}

bool sensorMissingOrBad() {
  return (sense.frontLeft == DIST_INVALID || sense.frontRight == DIST_INVALID ||
          sense.left == DIST_INVALID || sense.right == DIST_INVALID);
}

void printDistances() {
  Serial.print("FL:");
  Serial.print(sense.frontLeft);
  Serial.print(" FR:");
  Serial.print(sense.frontRight);
  Serial.print(" L:");
  Serial.print(sense.left);
  Serial.print(" R:");
  Serial.print(sense.right);
  Serial.print(" FAVG:");
  Serial.print(sense.frontAvg);
  Serial.print(" LS:");
  Serial.print(sense.leftScore);
  Serial.print(" RS:");
  Serial.println(sense.rightScore);
}

// ===================== Motor functions =====================
void initMotorPins() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motorPins[i].ena, OUTPUT);
    pinMode(motorPins[i].in1, OUTPUT);
    pinMode(motorPins[i].in2, OUTPUT);
    analogWrite(motorPins[i].ena, 0);
    digitalWrite(motorPins[i].in1, LOW);
    digitalWrite(motorPins[i].in2, LOW);
  }
}

void setMotor(MotorIndex motor, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(motorPins[motor].in1, HIGH);
    digitalWrite(motorPins[motor].in2, LOW);
    analogWrite(motorPins[motor].ena, pwm);
  } else if (pwm < 0) {
    digitalWrite(motorPins[motor].in1, LOW);
    digitalWrite(motorPins[motor].in2, HIGH);
    analogWrite(motorPins[motor].ena, -pwm);
  } else {
    digitalWrite(motorPins[motor].in1, LOW);
    digitalWrite(motorPins[motor].in2, LOW);
    analogWrite(motorPins[motor].ena, 0);
  }
}

void setDrive4(int leftFront, int rightFront, int leftRear, int rightRear) {
  setMotor(MOTOR_LEFT_FRONT, leftFront);
  setMotor(MOTOR_RIGHT_FRONT, rightFront);
  setMotor(MOTOR_LEFT_REAR, leftRear);
  setMotor(MOTOR_RIGHT_REAR, rightRear);
}

void stopMotors() {
  setDrive4(0, 0, 0, 0);
}

void driveForward(int l, int r) {
  setDrive4(abs(l), abs(r), abs(l), abs(r));
}

void driveBackward(int l, int r) {
  setDrive4(-abs(l), -abs(r), -abs(l), -abs(r));
}

void pivotLeft(int pwm) {
  setDrive4(-abs(pwm), abs(pwm), -abs(pwm), abs(pwm));
}

void pivotRight(int pwm) {
  setDrive4(abs(pwm), -abs(pwm), abs(pwm), -abs(pwm));
}

// ===================== Decision logic =====================
bool frontBlocked() {
  long fl = (sense.frontLeft == DIST_INVALID) ? 0 : sense.frontLeft;
  long fr = (sense.frontRight == DIST_INVALID) ? 0 : sense.frontRight;
  return (fl <= FRONT_BLOCK_CM || fr <= FRONT_BLOCK_CM);
}

bool dangerClose() {
  long fl = (sense.frontLeft == DIST_INVALID) ? 0 : sense.frontLeft;
  long fr = (sense.frontRight == DIST_INVALID) ? 0 : sense.frontRight;
  return (fl <= VERY_CLOSE_CM || fr <= VERY_CLOSE_CM);
}

bool sideTight() {
  long l = (sense.left == DIST_INVALID) ? 0 : sense.left;
  long r = (sense.right == DIST_INVALID) ? 0 : sense.right;
  return (l < SIDE_SAFE_CM || r < SIDE_SAFE_CM);
}

TurnDirection chooseBestTurn() {
  long l = (sense.left == DIST_INVALID) ? 0 : sense.left;
  long r = (sense.right == DIST_INVALID) ? 0 : sense.right;
  long leftMargin = (sense.leftScore == DIST_INVALID ? 0 : sense.leftScore) - (sense.rightScore == DIST_INVALID ? 0 : sense.rightScore);

  if (l > SIDE_OPEN_CM && r <= SIDE_SAFE_CM) return TURN_LEFT;
  if (r > SIDE_OPEN_CM && l <= SIDE_SAFE_CM) return TURN_RIGHT;
  if (leftMargin > 10) return TURN_LEFT;
  if (leftMargin < -10) return TURN_RIGHT;
  if (l > r) return TURN_LEFT;
  if (r > l) return TURN_RIGHT;
  return lastTurn == TURN_LEFT ? TURN_RIGHT : TURN_LEFT;
}

void setState(RobotState newState) {
  currentState = newState;
  stateStartTime = millis();
}

void runCruiseControl() {
  long fl = (sense.frontLeft == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontLeft;
  long fr = (sense.frontRight == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.frontRight;
  long l  = (sense.left == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.left;
  long r  = (sense.right == DIST_INVALID) ? DIST_MAX_VALID_CM : sense.right;

  int frontBalance = (int)(fl - fr);
  int sideBalance  = (int)(l - r);
  int steering = (frontBalance * 2 + sideBalance) / 3;
  steering = constrain(steering, -35, 35);

  int targetSpeed = CRUISE_SPEED;
  if (sense.frontAvg != DIST_INVALID && sense.frontAvg < FRONT_CAUTION_CM) targetSpeed = CAUTION_SPEED;

  int leftCmd  = constrain(targetSpeed + steering, MIN_CMD_SPEED, MAX_CMD_SPEED);
  int rightCmd = constrain(targetSpeed - steering, MIN_CMD_SPEED, MAX_CMD_SPEED);

  driveForward(leftCmd, rightCmd);
}

void runCautionControl() {
  TurnDirection preferred = chooseBestTurn();
  int bias = 18;

  if (preferred == TURN_LEFT) driveForward(CAUTION_SPEED - bias, CAUTION_SPEED + bias);
  else driveForward(CAUTION_SPEED + bias, CAUTION_SPEED - bias);
}

void runAvoidance() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed < STOP_BEFORE_BACK_MS) {
    stopMotors();
    return;
  }

  if (elapsed < STOP_BEFORE_BACK_MS + BACKUP_TIME_MS) {
    long l = (sense.left == DIST_INVALID) ? 0 : sense.left;
    long r = (sense.right == DIST_INVALID) ? 0 : sense.right;
    backDir = (l > r) ? 1 : -1;
    driveBackward(REVERSE_SPEED + 8, REVERSE_SPEED + 8);
    return;
  }

  if (elapsed < STOP_BEFORE_BACK_MS + BACKUP_TIME_MS + PIVOT_TIME_MS) {
    TurnDirection turn = chooseBestTurn();
    lastTurn = turn;
    if (turn == TURN_LEFT) pivotLeft(PIVOT_SPEED);
    else pivotRight(PIVOT_SPEED);
    return;
  }

  setState(STATE_CRUISE);
}

void autonomousStep() {
  if (armLocked) {
    // kept for future use only
  }

  updateSensors();

  if (sensorMissingOrBad() && sense.frontLeft == DIST_INVALID && sense.frontRight == DIST_INVALID) {
    stopMotors();
    delay(80);
    driveBackward(REVERSE_SPEED, REVERSE_SPEED);
    delay(240);
    TurnDirection t = chooseBestTurn();
    if (t == TURN_LEFT) pivotLeft(PIVOT_SPEED);
    else pivotRight(PIVOT_SPEED);
    delay(320);
    stopMotors();
    return;
  }

  if (dangerClose()) {
    if (currentState != STATE_AVOID_BACKUP && currentState != STATE_AVOID_TURN) {
      setState(STATE_AVOID_BACKUP);
    }
  } else {
    switch (currentState) {
      case STATE_STARTUP:
        setState(STATE_CRUISE);
        break;
      case STATE_CRUISE:
        if (frontBlocked()) setState(STATE_AVOID_BACKUP);
        else if (sense.frontAvg != DIST_INVALID && (sense.frontAvg < FRONT_CAUTION_CM || sideTight()))
          setState(STATE_CAUTION);
        break;
      case STATE_CAUTION:
        if (frontBlocked()) setState(STATE_AVOID_BACKUP);
        else if (sense.frontAvg != DIST_INVALID && sense.frontAvg >= FRONT_CLEAR_CM && !sideTight())
          setState(STATE_CRUISE);
        break;
      case STATE_AVOID_BACKUP:
      case STATE_AVOID_TURN:
        break;
    }
  }

  switch (currentState) {
    case STATE_CRUISE:       runCruiseControl(); break;
    case STATE_CAUTION:      runCautionControl(); break;
    case STATE_AVOID_BACKUP: runAvoidance(); break;
    case STATE_AVOID_TURN:   runAvoidance(); break;
    default:                 stopMotors(); break;
  }
}

// ===================== Serial commands =====================
void printHelp() {
  Serial.println("A -> auto mode");
  Serial.println("M -> manual stop");
  Serial.println("H -> arm home lock");
  Serial.println("T -> servo startup test");
  Serial.println("D -> distances");
  Serial.println("S -> status");
  Serial.println("F -> forward");
  Serial.println("B -> backward");
  Serial.println("L -> pivot left");
  Serial.println("R -> pivot right");
  Serial.println("X -> stop");
}

void printStatus() {
  Serial.print("STATE:");
  switch (currentState) {
    case STATE_STARTUP:       Serial.print("STARTUP"); break;
    case STATE_CRUISE:        Serial.print("CRUISE"); break;
    case STATE_CAUTION:       Serial.print("CAUTION"); break;
    case STATE_AVOID_BACKUP:  Serial.print("AVOID_BACKUP"); break;
    case STATE_AVOID_TURN:    Serial.print("AVOID_TURN"); break;
  }
  Serial.print(" AUTO:");
  Serial.print(autoMode ? "1" : "0");
  Serial.print(" ARM_LOCK:");
  Serial.println(armLocked ? "1" : "0");
}

void parseCommand(String cmd) {
  cmd.trim();

  if (cmd == "A") { autoMode = true; setState(STATE_CRUISE); return; }
  if (cmd == "M") { autoMode = false; stopMotors(); return; }
  if (cmd == "H") { armLocked = true; lockArmHome(); return; }
  if (cmd == "T") { autoMode = false; stopMotors(); startupServoTest(); return; }
  if (cmd == "D") { updateSensors(); printDistances(); return; }
  if (cmd == "S") { printStatus(); return; }
  if (cmd == "F") { autoMode = false; driveForward(95, 95); return; }
  if (cmd == "B") { autoMode = false; driveBackward(85, 85); return; }
  if (cmd == "L") { autoMode = false; pivotLeft(95); return; }
  if (cmd == "R") { autoMode = false; pivotRight(95); return; }
  if (cmd == "X") { autoMode = false; stopMotors(); return; }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);

  initUltrasonicPins();
  initMotorPins();

  moveAllHome();
  delay(700);
  lockArmHome();

  updateSensors();
  setState(STATE_CRUISE);
  printHelp();
}

// ===================== Loop =====================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }

  if (autoMode && millis() - lastControlTime >= CONTROL_DT_MS) {
    lastControlTime = millis();
    autonomousStep();
  }

  if (millis() - lastPrintTime >= STATUS_PRINT_MS) {
    lastPrintTime = millis();
    updateSensors();
    printDistances();
  }
}
