#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===================== I2C / PCA9685 =====================
const uint8_t PIN_I2C_SDA = 20;
const uint8_t PIN_I2C_SCL = 21;

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ===================== Servo config =====================
const uint8_t SERVO_BASE     = 0;
const uint8_t SERVO_SHOULDER = 1;
const uint8_t SERVO_ELBOW    = 2;
const uint8_t SERVO_WRIST    = 3;

const uint8_t SERVO_COUNT = 4;
const uint8_t servoChannel[SERVO_COUNT] = {
  SERVO_BASE, SERVO_SHOULDER, SERVO_ELBOW, SERVO_WRIST
};

const int servoHome[SERVO_COUNT]    = {90, 90, 90, 90};
int servoCurrent[SERVO_COUNT]       = {90, 90, 90, 90};

const int servoMinAngle[SERVO_COUNT] = {0, 15, 15, 0};
const int servoMaxAngle[SERVO_COUNT] = {180, 165, 165, 180};

const uint16_t SERVO_FREQ      = 50;
const uint16_t SERVO_MIN_PULSE = 102;
const uint16_t SERVO_MAX_PULSE = 512;

// ===================== Helpers =====================
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

void moveAllHome() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    writeServo(i, servoHome[i]);
    delay(200);
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

void printStatus() {
  Serial.print("S1:"); Serial.print(servoCurrent[0]);
  Serial.print(" S2:"); Serial.print(servoCurrent[1]);
  Serial.print(" S3:"); Serial.print(servoCurrent[2]);
  Serial.print(" S4:"); Serial.println(servoCurrent[3]);
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("H        -> all home");
  Serial.println("T        -> startup test");
  Serial.println("S        -> status");
  Serial.println("1 0..180 -> servo 1 angle");
  Serial.println("2 0..180 -> servo 2 angle");
  Serial.println("3 0..180 -> servo 3 angle");
  Serial.println("4 0..180 -> servo 4 angle");
}

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pca.begin();
  pca.setOscillatorFrequency(27000000);
  pca.setPWMFreq(SERVO_FREQ);

  delay(10);
  moveAllHome();
  printHelp();
}

// ===================== Loop =====================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "H") {
      moveAllHome();
      Serial.println("HOME");
      return;
    }

    if (cmd == "T") {
      startupServoTest();
      return;
    }

    if (cmd == "S") {
      printStatus();
      return;
    }

    int sp = cmd.indexOf(' ');
    if (sp > 0) {
      int idx = cmd.substring(0, sp).toInt();
      int ang = cmd.substring(sp + 1).toInt();

      if (idx >= 1 && idx <= 4) {
        writeServo(idx - 1, ang);
        Serial.print("SERVO ");
        Serial.print(idx);
        Serial.print(" -> ");
        Serial.println(ang);
      }
    }
  }
}
