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

// Smooth-motion easing: each loop() the current angle moves a fraction
// of the way towards its target. Set SMOOTH_EASE to 1.0 for instant motion.
float SMOOTH_EASE = 0.25; // 0.0..1.0, higher = faster / snappier
float EASE_SNAP  = 1.0;   // degrees within which we snap to the target

// Current angles (what the servos are actually set to / what POS: reports)
int angleBase   = 90;
int angleFinger = 90;
int angleWrist  = 90;
int angleArm    = 90;
int angleElbow  = 90;
int angleDual   = 90;

// Target angles (goals we slowly move towards when easing is active)
int targetBase   = 90;
int targetFinger = 90;
int targetWrist  = 90;
int targetArm    = 90;
int targetElbow  = 90;
int targetDual   = 90;

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

// Steps a target angle (clamped to 0..180).
void stepTarget(int *target, int delta) {
  *target = constrain(*target + delta, 0, 180);
}

void processSerialKey(char key) {
  switch (key) {
    case 'a': case 'A': stepTarget(&targetBase,   -ANGLE_STEP); break;
    case 'd': case 'D': stepTarget(&targetBase,   +ANGLE_STEP); break;
    case 'q': case 'Q': stepTarget(&targetFinger, -ANGLE_STEP); break;
    case 'e': case 'E': stepTarget(&targetFinger, +ANGLE_STEP); break;
    case 'j': case 'J': stepTarget(&targetWrist,  +ANGLE_STEP); break;
    case 'k': case 'K': stepTarget(&targetWrist,  -ANGLE_STEP); break;
    case 'i': case 'I': stepTarget(&targetArm,    +ANGLE_STEP); break;
    case 'o': case 'O': stepTarget(&targetArm,    -ANGLE_STEP); break;
    case 'm': case 'M': stepTarget(&targetElbow,  +ANGLE_STEP); break;
    case 'n': case 'N': stepTarget(&targetElbow,  -ANGLE_STEP); break;
    case 'w': case 'W': stepTarget(&targetDual,   +ANGLE_STEP); break;
    case 's': case 'S': stepTarget(&targetDual,   -ANGLE_STEP); break;
  }
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
    targetBase   = constrain(vals[0], 0, 180);
    targetFinger = constrain(vals[1], 0, 180);
    targetWrist  = constrain(vals[2], 0, 180);
    targetArm    = constrain(vals[3], 0, 180);
    targetElbow  = constrain(vals[4], 0, 180);
    targetDual   = constrain(vals[5], 0, 180);
  }
}

// Moves each current angle towards its target by the easing factor.
// Returns true if any angle changed (used to trigger a servo update + report).
bool applyEasing() {
  bool changed = false;
  int *angles[6]  = { &angleBase,   &angleFinger, &angleWrist,
                      &angleArm,    &angleElbow,  &angleDual };
  int *targets[6] = { &targetBase,  &targetFinger, &targetWrist,
                      &targetArm,   &targetElbow,  &targetDual };
  for (int i = 0; i < 6; i++) {
    float diff = *targets[i] - *angles[i];
    if (diff == 0) continue;
    if (fabs(diff) <= EASE_SNAP) {
      *angles[i] = *targets[i];           // snap
    } else {
      *angles[i] += (int)(diff * SMOOTH_EASE);
    }
    changed = true;
  }
  return changed;
}

void loop() {
  // 1. Check Serial Commands (PC / Python GUI)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("P:")) {
      parseDirectPosition(input);
    } else if (input == "?") {
      printStatus();                      // on-demand position report
    } else if (input.length() == 1) {
      processSerialKey(input.charAt(0));
    }
  }

  // 2. Check Joystick Controls (steps the targets)
  int xVal = analogRead(JOY_X);
  int yVal = analogRead(JOY_Y);

  bool btnA = (digitalRead(BTN_A) == LOW);
  bool btnB = (digitalRead(BTN_B) == LOW);
  bool btnC = (digitalRead(BTN_C) == LOW);
  bool btnD = (digitalRead(BTN_D) == LOW);

  if (btnA) {
    if (xVal < (joyXCenter - DEADZONE)) stepTarget(&targetWrist, -ANGLE_STEP);
    if (xVal > (joyXCenter + DEADZONE)) stepTarget(&targetWrist, +ANGLE_STEP);
  } else if (btnB) {
    if (yVal < (joyYCenter - DEADZONE)) stepTarget(&targetElbow, -ANGLE_STEP);
    if (yVal > (joyYCenter + DEADZONE)) stepTarget(&targetElbow, +ANGLE_STEP);
  } else if (btnC) {
    if (xVal < (joyXCenter - DEADZONE)) stepTarget(&targetFinger, -ANGLE_STEP);
    if (xVal > (joyXCenter + DEADZONE)) stepTarget(&targetFinger, +ANGLE_STEP);
  } else if (btnD) {
    if (yVal < (joyYCenter - DEADZONE)) stepTarget(&targetArm, -ANGLE_STEP);
    if (yVal > (joyYCenter + DEADZONE)) stepTarget(&targetArm, +ANGLE_STEP);
  } else {
    if (xVal < (joyXCenter - DEADZONE)) stepTarget(&targetBase, -ANGLE_STEP);
    if (xVal > (joyXCenter + DEADZONE)) stepTarget(&targetBase, +ANGLE_STEP);
    if (yVal < (joyYCenter - DEADZONE)) stepTarget(&targetDual, -ANGLE_STEP);
    if (yVal > (joyYCenter + DEADZONE)) stepTarget(&targetDual, +ANGLE_STEP);
  }

  // 3. Smooth motion + servo update
  if (applyEasing()) {
    updateServos();
    printStatus();
  }

  delay(20);
}