#include <Wire.h>

// ---------------------------------------------------------------------------
// 1. HARDWARE CONSTANTS & PCA9685 CHANNELS
// ---------------------------------------------------------------------------
#define PCA9685_ADDRESS 0x40

#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0_ON_L 0x06

#define PULSE_MIN 130 // 0 Grad
#define PULSE_MAX 490 // 180 Grad

// Servo PCA9685 Channels
#define PIN_BASE       6  
#define PIN_FINGER     1  
#define PIN_WRIST      2  
#define PIN_ELBOW      3  
#define PIN_ARM        4  
#define PIN_DUAL_LEFT  10
#define PIN_DUAL_RIGHT 11

// Funduino Shield Pins
#define JOY_X   A0
#define JOY_Y   A1
#define BTN_A   2
#define BTN_B   3
#define BTN_C   4
#define BTN_D   5

// ---------------------------------------------------------------------------
// 2. STATE VARIABLES
// ---------------------------------------------------------------------------
int ANGLE_STEP = 3; 

int angleBase   = 90;
int angleFinger = 90;
int angleWrist  = 90;
int angleArm    = 90;
int angleElbow  = 90;
int angleDual   = 90;

int joyXCenter = 512;
int joyYCenter = 512;
const int DEADZONE = 45;

// ---------------------------------------------------------------------------
// 3. LOW-LEVEL PCA9685 DRIVER
// ---------------------------------------------------------------------------
void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setServo(byte channel, int pulse) {
  byte reg = LED0_ON_L + 4 * channel;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(0);
  Wire.write(0);
  Wire.write(pulse & 0xFF);
  Wire.write(pulse >> 8);
  Wire.endTransmission();
}

void setServoAngle(byte channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, PULSE_MIN, PULSE_MAX);
  setServo(channel, pulse);
}

void setDualOpposingServos(int leftAngle) {
  leftAngle = constrain(leftAngle, 0, 180);
  int rightAngle = 180 - leftAngle;
  setServoAngle(PIN_DUAL_LEFT, leftAngle);
  setServoAngle(PIN_DUAL_RIGHT, rightAngle);
}

void updateServos() {
  setServoAngle(PIN_BASE, angleBase);
  setServoAngle(PIN_FINGER, angleFinger);
  setServoAngle(PIN_WRIST, angleWrist);
  setServoAngle(PIN_ARM, angleArm);
  setServoAngle(PIN_ELBOW, angleElbow);
  setDualOpposingServos(angleDual);
}

void printStatus() {
  Serial.print("POS:");
  Serial.print(angleBase); Serial.print(",");
  Serial.print(angleFinger); Serial.print(",");
  Serial.print(angleWrist); Serial.print(",");
  Serial.print(angleArm); Serial.print(",");
  Serial.print(angleElbow); Serial.print(",");
  Serial.println(angleDual);
}

// ---------------------------------------------------------------------------
// 4. SETUP & MAIN LOOP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_D, INPUT_PULLUP);

  writeRegister(MODE1, 0x00);
  delay(10);
  writeRegister(MODE1, 0x10);
  writeRegister(PRESCALE, 121);
  writeRegister(MODE1, 0x00);
  delay(10);
  writeRegister(MODE1, 0xA1);

  // Calibration Joystick Center
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 20; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(10);
  }
  joyXCenter = sumX / 20;
  joyYCenter = sumY / 20;

  updateServos();
  Serial.println("ARDUINO_BEREIT");
}

void processSerialKey(char key) {
  switch (key) {
    case 'a': case 'A': angleBase = constrain(angleBase - ANGLE_STEP, 0, 180); break;
    case 'd': case 'D': angleBase = constrain(angleBase + ANGLE_STEP, 0, 180); break;
    case 'q': case 'Q': angleFinger = constrain(angleFinger - ANGLE_STEP, 0, 180); break;
    case 'e': case 'E': angleFinger = constrain(angleFinger + ANGLE_STEP, 0, 180); break;
    case 'j': case 'J': angleWrist = constrain(angleWrist + ANGLE_STEP, 0, 180); break;
    case 'k': case 'K': angleWrist = constrain(angleWrist - ANGLE_STEP, 0, 180); break;
    case 'i': case 'I': angleArm = constrain(angleArm + ANGLE_STEP, 0, 180); break;
    case 'o': case 'O': angleArm = constrain(angleArm - ANGLE_STEP, 0, 180); break;
    case 'm': case 'M': angleElbow = constrain(angleElbow + ANGLE_STEP, 0, 180); break;
    case 'n': case 'N': angleElbow = constrain(angleElbow - ANGLE_STEP, 0, 180); break;
    case 'w': case 'W': angleDual = constrain(angleDual + ANGLE_STEP, 0, 180); break;
    case 's': case 'S': angleDual = constrain(angleDual - ANGLE_STEP, 0, 180); break;
    case 'r': case 'R': resetServos(); break;
  }
}

void resetServos() {
  angleBase   = 90;
  angleFinger = 90;
  angleWrist  = 90;
  angleArm    = 90;
  angleElbow  = 90;
  angleDual   = 90;
}

void parseDirectPosition(String data) {
  // Format: "P:base,finger,wrist,arm,elbow,dual"
  int first = data.indexOf(':');
  if (first == -1) return;
  
  String coords = data.substring(first + 1);
  int vals[6];
  int idx = 0;
  
  while (coords.length() > 0 && idx < 6) {
    int comma = coords.indexOf(',');
    if (comma == -1) {
      vals[idx++] = coords.toInt();
      break;
    } else {
      vals[idx++] = coords.substring(0, comma).toInt();
      coords = coords.substring(comma + 1);
    }
  }

  if (idx == 6) {
    angleBase   = constrain(vals[0], 0, 180);
    angleFinger = constrain(vals[1], 0, 180);
    angleWrist  = constrain(vals[2], 0, 180);
    angleArm    = constrain(vals[3], 0, 180);
    angleElbow  = constrain(vals[4], 0, 180);
    angleDual   = constrain(vals[5], 0, 180);
  }
}

void loop() {
  bool moved = false;

  // 1. Check Serial Commands (PC / Python GUI)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("P:")) {
      parseDirectPosition(input);
      moved = true;
    } else if (input.length() == 1) {
      processSerialKey(input.charAt(0));
      moved = true;
    }
  }

  // 2. Check Joystick Controls
  int xVal = analogRead(JOY_X);
  
  int yVal = analogRead(JOY_Y);

  // Report raw joystick values to the GUI (live indicator)
  Serial.print("JS:");
  Serial.print(xVal); Serial.print(",");
  Serial.println(yVal);

  bool btnA = (digitalRead(BTN_A) == LOW);
  bool btnB = (digitalRead(BTN_B) == LOW);
  bool btnC = (digitalRead(BTN_C) == LOW);
  bool btnD = (digitalRead(BTN_D) == LOW);

  if (btnA) {
    if (xVal < (joyXCenter - DEADZONE)) { angleWrist = constrain(angleWrist - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleWrist = constrain(angleWrist + ANGLE_STEP, 0, 180); moved = true; }
  } else if (btnB) {
    if (yVal < (joyYCenter - DEADZONE)) { angleElbow = constrain(angleElbow - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleElbow = constrain(angleElbow + ANGLE_STEP, 0, 180); moved = true; }
  } else if (btnC) {
    if (xVal < (joyXCenter - DEADZONE)) { angleFinger = constrain(angleFinger - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleFinger = constrain(angleFinger + ANGLE_STEP, 0, 180); moved = true; }
  } else if (btnD) {
    if (yVal < (joyYCenter - DEADZONE)) { angleArm = constrain(angleArm - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleArm = constrain(angleArm + ANGLE_STEP, 0, 180); moved = true; }
  } else {
    if (xVal < (joyXCenter - DEADZONE)) { angleBase = constrain(angleBase - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleBase = constrain(angleBase + ANGLE_STEP, 0, 180); moved = true; }
    if (yVal < (joyYCenter - DEADZONE)) { angleDual = constrain(angleDual - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleDual = constrain(angleDual + ANGLE_STEP, 0, 180); moved = true; }
  }

  if (moved) {
    updateServos();
    printStatus();
  }

  delay(20);
}