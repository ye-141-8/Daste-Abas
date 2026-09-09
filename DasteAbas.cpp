#include <Wire.h>

#define PCA9685_ADDRESS 0x40

#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0_ON_L 0x06

#define PULSE_MIN 205 // ~0 degrees
#define PULSE_MAX 410 // ~180 degrees

// ---------------------------------------------------------------------------
// 1. PIN DEFINITIONS (PCA9685 Board Channels)
// ---------------------------------------------------------------------------
#define PIN_BASE       6   // Fixed: Match code pin to Channel 6
#define PIN_FINGER     1   // Motor 2 (Finger - Channel 1)
#define PIN_WRIST      2   // Motor 3 (Wrist - Channel 2)
#define PIN_ELBOW      3   // Motor 5 (Elbow - Channel 3)
#define PIN_ARM        4   // Motor 4 (Arm - Channel 4)
#define PIN_DUAL_LEFT  10  // Motor 6 (Left opposing - Channel 10)
#define PIN_DUAL_RIGHT 11  // Motor 7 (Right opposing - Channel 11)

// ---------------------------------------------------------------------------
// 2. CONFIGURATION & STATE
// ---------------------------------------------------------------------------
int ANGLE_STEP = 5; // Step adjustment per keypress in degrees

// Initial joint angles — Set to 90 degrees so the arm stands straight up on boot
int angleBase   = 90;
int angleFinger = 90;
int angleWrist  = 90;
int angleArm    = 90;
int angleElbow  = 90;
int angleDual   = 90; // Left motor at 90; Right automatically mirrors to (180 - 90) = 90

// ---------------------------------------------------------------------------
// HARDWARE DRIVER FUNCTIONS
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
  int rightAngle = 180 - leftAngle; // Inverse axis so dual motors mirror each other
  
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
  Serial.print("M1 Base:"); Serial.print(angleBase);
  Serial.print(" | M2 Finger:"); Serial.print(angleFinger);
  Serial.print(" | M3 Wrist:"); Serial.print(angleWrist);
  Serial.print(" | M4 Arm:"); Serial.print(angleArm);
  Serial.print(" | M5 Elbow:"); Serial.print(angleElbow);
  Serial.print(" | M6/7 Dual:"); Serial.print(angleDual);
  Serial.print("/"); Serial.println(180 - angleDual);
}

// ---------------------------------------------------------------------------
// SETUP & LOOP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize PCA9685 @ 50 Hz PWM rate
  writeRegister(MODE1, 0x10);
  writeRegister(PRESCALE, 121);
  writeRegister(MODE1, 0x00);
  delay(10);
  writeRegister(MODE1, 0xA1);

  // Trigger position setup immediately to force 90-degree upright stance
  updateServos();
  Serial.println("ARDUINO_READY");
}

void loop() {
  if (Serial.available() > 0) {
    char key = Serial.read();

    switch (key) {
      // Motor 1: Base (A / D)
      case 'a': case 'A': angleBase = constrain(angleBase - ANGLE_STEP, 0, 180); break;
      case 'd': case 'D': angleBase = constrain(angleBase + ANGLE_STEP, 0, 180); break;

      // Motor 2: Finger (Q / E)
      case 'q': case 'Q': angleFinger = constrain(angleFinger - ANGLE_STEP, 0, 180); break;
      case 'e': case 'E': angleFinger = constrain(angleFinger + ANGLE_STEP, 0, 180); break;

      // Motor 3: Wrist (J / K)
      case 'j': case 'J': angleWrist = constrain(angleWrist + ANGLE_STEP, 0, 180); break;
      case 'k': case 'K': angleWrist = constrain(angleWrist - ANGLE_STEP, 0, 180); break;

      // Motor 4: Arm (I / O)
      case 'i': case 'I': angleArm = constrain(angleArm + ANGLE_STEP, 0, 180); break;
      case 'o': case 'O': angleArm = constrain(angleArm - ANGLE_STEP, 0, 180); break;

      // Motor 5: Elbow (M / N)
      case 'm': case 'M': angleElbow = constrain(angleElbow + ANGLE_STEP, 0, 180); break;
      case 'n': case 'N': angleElbow = constrain(angleElbow - ANGLE_STEP, 0, 180); break;

      // Motor 6 & 7: Dual Opposing (W / S)
      case 'w': case 'W': angleDual = constrain(angleDual + ANGLE_STEP, 0, 180); break;
      case 's': case 'S': angleDual = constrain(angleDual - ANGLE_STEP, 0, 180); break;

      default: return;
    }

    updateServos();
    printStatus();
  }
}