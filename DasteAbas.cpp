#include <Wire.h>

#define PCA9685_ADDRESS 0x40

#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0_ON_L 0x06

#define PULSE_MIN 205 // ~0 degrees
#define PULSE_MAX 410 // ~180 degrees

// ---------------------------------------------------------------------------
// 1. PIN DEFINITIONS
// ---------------------------------------------------------------------------
#define PIN_BASE       1   // Motor 1 (Ground anchor base / Spaceship plate)
#define PIN_FINGER     2   // Motor 2 (Gripper finger)
#define PIN_WRIST      4   // Motor 3 (Wrist rotation)
#define PIN_ARM        5   // Motor 4 (Main arm/shoulder joint)
#define PIN_ELBOW      7   // Motor 5 (Elbow joint)
#define PIN_DUAL_LEFT  10  // Left opposing motor
#define PIN_DUAL_RIGHT 11  // Right opposing motor (Mirrors Pin 10)

// ---------------------------------------------------------------------------
// 2. ANGLE CONFIGURATION
// ---------------------------------------------------------------------------
const int BASE_START   = 0;    const int BASE_TARGET   = 180;  
const int FINGER_START = 30;   const int FINGER_TARGET = 180; 
const int WRIST_START  = 30;   const int WRIST_TARGET  = 180;  
const int ARM_START    = 30;   const int ARM_TARGET    = 90;   
const int ELBOW_START  = 30;   const int ELBOW_TARGET  = 90;   
const int DUAL_START   = 30;   const int DUAL_TARGET   = 90;   

const int SLOW_HOME_SPEED = 30; // 30ms * 100 steps = 3000ms (3 seconds total homing time)

// Global state tracking to perform gentle motion from unknown boot state
int currentBase   = 90;
int currentFinger = 90;
int currentWrist  = 90;
int currentArm    = 90;
int currentElbow  = 90;
int currentDual   = 90;

// ---------------------------------------------------------------------------
// HARDWARE HELPER FUNCTIONS
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

// MIRRORED DUAL MOTOR FUNCTION (Pins 10 & 11)
void setDualOpposingServos(int leftAngle) {
  leftAngle = constrain(leftAngle, 0, 180);
  int rightAngle = 180 - leftAngle; // Inverse mapping for opposing motor
  
  setServoAngle(PIN_DUAL_LEFT, leftAngle);
  setServoAngle(PIN_DUAL_RIGHT, rightAngle);
}

// ---------------------------------------------------------------------------
// SLOW HOME POSITIONING (SAFE POWER-ON RESTART - 3 SECONDS)
// ---------------------------------------------------------------------------
void slowHomePosition() {
  Serial.println("Moving all motors slowly to home positions over 3 seconds...");
  
  for (int step = 0; step <= 100; step++) {
    float progress = step / 100.0;

    int baseAngle   = currentBase   + (progress * (BASE_START   - currentBase));
    int fingerAngle = currentFinger + (progress * (FINGER_START - currentFinger));
    int wristAngle  = currentWrist  + (progress * (WRIST_START  - currentWrist));
    int armAngle    = currentArm    + (progress * (ARM_START    - currentArm));
    int elbowAngle  = currentElbow  + (progress * (ELBOW_START  - currentElbow));
    int dualAngle   = currentDual   + (progress * (DUAL_START   - currentDual));

    setServoAngle(PIN_BASE, baseAngle);
    setServoAngle(PIN_FINGER, fingerAngle);
    setServoAngle(PIN_WRIST, wristAngle);
    setServoAngle(PIN_ARM, armAngle);
    setServoAngle(PIN_ELBOW, elbowAngle);
    setDualOpposingServos(dualAngle);

    delay(SLOW_HOME_SPEED);
  }

  // Update tracked positions to match completed start state
  currentBase   = BASE_START;
  currentFinger = FINGER_START;
  currentWrist  = WRIST_START;
  currentArm    = ARM_START;
  currentElbow  = ELBOW_START;
  currentDual   = DUAL_START;
}

// ---------------------------------------------------------------------------
// DIAGNOSTIC TEST FUNCTIONS
// ---------------------------------------------------------------------------

// FIXED BASE MOTOR TEST (Rotates out to angle and returns back, 2 TIMES)
void testBaseMotorTwice(byte pin, int startAngle, int targetAngle) {
  Serial.print("Testing Base Plate Motor (Pin "); Serial.print(pin); Serial.println(")...");

  for (int pass = 1; pass <= 2; pass++) {
    for (int a = startAngle; a <= targetAngle; a++) { 
      setServoAngle(pin, a); 
      delay(15); 
    }
    delay(200);
    for (int a = targetAngle; a >= startAngle; a--) { 
      setServoAngle(pin, a); 
      delay(15); 
    }
    delay(300);
  }
}

// Single motor test (sweeps out and back TWICE)
void testMotorGentle(byte pin, const char* name, int startAngle) {
  Serial.print("Testing "); Serial.print(name); Serial.print(" on Pin "); Serial.println(pin);

  for (int pass = 1; pass <= 2; pass++) {
    for (int a = startAngle; a <= startAngle + 30; a++) { setServoAngle(pin, a); delay(15); }
    for (int a = startAngle + 30; a >= startAngle; a--) { setServoAngle(pin, a); delay(15); }
    delay(150);
  }
  delay(200);
}

// Dual opposing motors test (Pins 10 & 11 move together in opposite directions TWICE)
void testDualMotorsGentle(int startLeftAngle) {
  Serial.println("Testing Dual Opposing Motors (Pins 10 & 11)...");

  for (int pass = 1; pass <= 2; pass++) {
    for (int a = startLeftAngle; a <= startLeftAngle + 30; a++) { setDualOpposingServos(a); delay(15); }
    for (int a = startLeftAngle + 30; a >= startLeftAngle; a--) { setDualOpposingServos(a); delay(15); }
    delay(150);
  }
  delay(200);
}

// ALL MOTORS TOGETHER TEST
void testAllMotorsTogether() {
  Serial.println("Testing ALL MOTORS TOGETHER...");
  
  // Sweep out
  for (int step = 0; step <= 50; step++) {
    float p = step / 50.0;
    setServoAngle(PIN_BASE,   BASE_START   + (p * 45));
    setServoAngle(PIN_FINGER, FINGER_START + (p * 30));
    setServoAngle(PIN_WRIST,  WRIST_START  + (p * 30));
    setServoAngle(PIN_ARM,    ARM_START    + (p * 30));
    setServoAngle(PIN_ELBOW,  ELBOW_START  + (p * 30));
    setDualOpposingServos(DUAL_START + (p * 30));
    delay(20);
  }
  // Sweep back
  for (int step = 50; step >= 0; step--) {
    float p = step / 50.0;
    setServoAngle(PIN_BASE,   BASE_START   + (p * 45));
    setServoAngle(PIN_FINGER, FINGER_START + (p * 30));
    setServoAngle(PIN_WRIST,  WRIST_START  + (p * 30));
    setServoAngle(PIN_ARM,    ARM_START    + (p * 30));
    setServoAngle(PIN_ELBOW,  ELBOW_START  + (p * 30));
    setDualOpposingServos(DUAL_START + (p * 30));
    delay(20);
  }
  delay(500);
}

// ---------------------------------------------------------------------------
// ARDUINO SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize PCA9685 clock for 50 Hz PWM
  writeRegister(MODE1, 0x10);
  writeRegister(PRESCALE, 121);
  writeRegister(MODE1, 0x00);
  delay(10);
  writeRegister(MODE1, 0xA1);

  // Slow-start homing sequence (3 seconds) to prevent gear damage on startup
  slowHomePosition();
  delay(1000);

  Serial.println("\n--- STEP 1: INDIVIDUAL MOTOR DIAGNOSTICS (2 PASSES EACH) ---");
  testBaseMotorTwice(PIN_BASE, BASE_START, 90);                    // Motor 1 (Pin 1) - Rotates 0 -> 90 -> 0 TWICE
  testMotorGentle(PIN_FINGER, "Motor 2 - Finger", FINGER_START);    // Motor 2 (Pin 2)
  testMotorGentle(PIN_WRIST,  "Motor 3 - Wrist",  WRIST_START);     // Motor 3 (Pin 4)
  testMotorGentle(PIN_ARM,    "Motor 4 - Arm",    ARM_START);       // Motor 4 (Pin 5)
  testMotorGentle(PIN_ELBOW,  "Motor 5 - Elbow",  ELBOW_START);     // Motor 5 (Pin 7)
  testDualMotorsGentle(DUAL_START);                                 // Dual Motors (Pins 10 & 11)

  Serial.println("\n--- STEP 2: ALL MOTORS TOGETHER DIAGNOSTIC ---");
  testAllMotorsTogether();

  Serial.println("--- DIAGNOSTICS COMPLETE. MOTORS HOLDING POSITION. ---");
}

// ---------------------------------------------------------------------------
// MAIN MOTION LOOP (Empty to prevent continuous motion)
// ---------------------------------------------------------------------------
void loop() {
  // Intentionally empty: Motors will stay still at their starting positions.
}
