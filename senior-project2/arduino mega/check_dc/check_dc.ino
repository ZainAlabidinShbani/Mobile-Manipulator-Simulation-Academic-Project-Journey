// ===================== DC Motor Serial Test =====================
// Commands:
// 1F 1S 1B -> Left Front
// 2F 2S 2B -> Right Front
// 3F 3S 3B -> Left Rear
// 4F 4S 4B -> Right Rear
// X -> stop all

const uint8_t LEFT_FRONT_ENA    = 5;
const uint8_t LEFT_FRONT_IN1    = 30;
const uint8_t LEFT_FRONT_IN2    = 31;

const uint8_t LEFT_REAR_ENA     = 6;
const uint8_t LEFT_REAR_IN1     = 32;
const uint8_t LEFT_REAR_IN2     = 33;

const uint8_t RIGHT_FRONT_ENA   = 7;
const uint8_t RIGHT_FRONT_IN1   = 34;
const uint8_t RIGHT_FRONT_IN2   = 35;

const uint8_t RIGHT_REAR_ENA    = 8;
const uint8_t RIGHT_REAR_IN1    = 36;
const uint8_t RIGHT_REAR_IN2    = 37;

const int TEST_SPEED = 120;

struct MotorPins {
  uint8_t ena;
  uint8_t in1;
  uint8_t in2;
};

MotorPins motors[4] = {
  {LEFT_FRONT_ENA,  LEFT_FRONT_IN1,  LEFT_FRONT_IN2},
  {RIGHT_FRONT_ENA, RIGHT_FRONT_IN1, RIGHT_FRONT_IN2},
  {LEFT_REAR_ENA,   LEFT_REAR_IN1,   LEFT_REAR_IN2},
  {RIGHT_REAR_ENA,  RIGHT_REAR_IN1,  RIGHT_REAR_IN2}
};

void initMotorPins() {
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].ena, OUTPUT);
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    analogWrite(motors[i].ena, 0);
    digitalWrite(motors[i].in1, LOW);
    digitalWrite(motors[i].in2, LOW);
  }
}

void setMotor(int m, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(motors[m].in1, HIGH);
    digitalWrite(motors[m].in2, LOW);
    analogWrite(motors[m].ena, pwm);
  } else if (pwm < 0) {
    digitalWrite(motors[m].in1, LOW);
    digitalWrite(motors[m].in2, HIGH);
    analogWrite(motors[m].ena, -pwm);
  } else {
    digitalWrite(motors[m].in1, LOW);
    digitalWrite(motors[m].in2, LOW);
    analogWrite(motors[m].ena, 0);
  }
}

void stopAll() {
  for (int i = 0; i < 4; i++) setMotor(i, 0);
}

void setup() {
  Serial.begin(9600);
  initMotorPins();
  stopAll();

  Serial.println("Motor test ready");
  Serial.println("Use: 1F 1S 1B | 2F 2S 2B | 3F 3S 3B | 4F 4S 4B | X");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "X") {
      stopAll();
      Serial.println("ALL STOP");
      return;
    }

    if (cmd.length() < 2) return;

    char motorId = cmd.charAt(0);
    char action  = cmd.charAt(1);

    int m = -1;
    if (motorId == '1') m = 0;
    if (motorId == '2') m = 1;
    if (motorId == '3') m = 2;
    if (motorId == '4') m = 3;

    if (m < 0) return;

    if (action == 'F') {
      setMotor(m, TEST_SPEED);
      Serial.print("Motor ");
      Serial.print(motorId);
      Serial.println(" FORWARD");
    } else if (action == 'B') {
      setMotor(m, -TEST_SPEED);
      Serial.print("Motor ");
      Serial.print(motorId);
      Serial.println(" BACKWARD");
    } else if (action == 'S') {
      setMotor(m, 0);
      Serial.print("Motor ");
      Serial.print(motorId);
      Serial.println(" STOP");
    }
  }
}
