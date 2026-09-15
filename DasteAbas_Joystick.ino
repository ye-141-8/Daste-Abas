#include <Wire.h>


// ---------------------------------------------------------------------------
// 1. HARDWARE CONSTANTS & PCA9685 CHANNELS
// ---------------------------------------------------------------------------
#define PCA9685_ADDRESS 0x40


#define MODE1     0x00
#define PRESCALE  0xFE
#define LED0_ON_L 0x06


// Standard PCA9685 PWM Impulse für 50Hz (ca. 0.5ms bis 2.5ms)
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
// 2. STATE VARIABLES & DEADBAND CONFIGURATION
// ---------------------------------------------------------------------------
int ANGLE_STEP = 2; // Grad pro Update


// Startwinkel
int angleBase   = 90;
int angleFinger = 90;
int angleWrist  = 90;
int angleArm    = 90;
int angleElbow  = 90;
int angleDual   = 90;


// Variablen für automatische Kalibrierung der Joystick-Mitte
int joyXCenter = 512;
int joyYCenter = 512;
const int DEADZONE = 40; // Empfindlicher Bereich um die Mitte


// ---------------------------------------------------------------------------
// 3. LOW-LEVEL PCA9685 DRIVER FUNCTIONS
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
  int rightAngle = 180 - leftAngle; // Spiegelung für Schulter
 
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


  // PCA9685 Reset & Setup für 50Hz Servos
  writeRegister(MODE1, 0x00);
  delay(10);
  writeRegister(MODE1, 0x10); // Sleep Modus aktivieren
  writeRegister(PRESCALE, 121); // 50 Hz Frequenz
  writeRegister(MODE1, 0x00); // Sleep Modus beenden
  delay(10);
  writeRegister(MODE1, 0xA1); // Auto-Increment aktivieren


  // Joystick-Mitte beim Einschalten messen
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 20; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(10);
  }
  joyXCenter = sumX / 20;
  joyYCenter = sumY / 20;


  updateServos();
  Serial.println("SYSTEM_BEREIT");
}


void loop() {
  int xVal = analogRead(JOY_X);
  int yVal = analogRead(JOY_Y);


  bool btnA = (digitalRead(BTN_A) == LOW);
  bool btnB = (digitalRead(BTN_B) == LOW);
  bool btnC = (digitalRead(BTN_C) == LOW);
  bool btnD = (digitalRead(BTN_D) == LOW);


  bool moved = false;


  // -------------------------------------------------------------------------
  // BUTTON A: Wrist / Handgelenk (Joystick X)
  // -------------------------------------------------------------------------
  if (btnA) {
    if (xVal < (joyXCenter - DEADZONE)) { angleWrist = constrain(angleWrist - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleWrist = constrain(angleWrist + ANGLE_STEP, 0, 180); moved = true; }
  }
  // -------------------------------------------------------------------------
  // BUTTON B: Elbow / Ellbogen (Joystick Y)
  // -------------------------------------------------------------------------
  else if (btnB) {
    if (yVal < (joyYCenter - DEADZONE)) { angleElbow = constrain(angleElbow - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleElbow = constrain(angleElbow + ANGLE_STEP, 0, 180); moved = true; }
  }
  // -------------------------------------------------------------------------
  // BUTTON C: Finger / Greifer (Joystick X)
  // -------------------------------------------------------------------------
  else if (btnC) {
    if (xVal < (joyXCenter - DEADZONE)) { angleFinger = constrain(angleFinger - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleFinger = constrain(angleFinger + ANGLE_STEP, 0, 180); moved = true; }
  }
  // -------------------------------------------------------------------------
  // BUTTON D: Arm / Oberarm (Joystick Y)
  // -------------------------------------------------------------------------
  else if (btnD) {
    if (yVal < (joyYCenter - DEADZONE)) { angleArm = constrain(angleArm - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleArm = constrain(angleArm + ANGLE_STEP, 0, 180); moved = true; }
  }
  // -------------------------------------------------------------------------
  // KEIN KNOPF: Base (Joystick X) & Schulter (Joystick Y)
  // -------------------------------------------------------------------------
  else {
    // Base Drehung
    if (xVal < (joyXCenter - DEADZONE)) { angleBase = constrain(angleBase - ANGLE_STEP, 0, 180); moved = true; }
    if (xVal > (joyXCenter + DEADZONE)) { angleBase = constrain(angleBase + ANGLE_STEP, 0, 180); moved = true; }


    // Schulter Heben / Senken
    if (yVal < (joyYCenter - DEADZONE)) { angleDual = constrain(angleDual - ANGLE_STEP, 0, 180); moved = true; }
    if (yVal > (joyYCenter + DEADZONE)) { angleDual = constrain(angleDual + ANGLE_STEP, 0, 180); moved = true; }
  }


  // Bei Bewegung ausführen und Daten senden
  if (moved) {
    updateServos();


    Serial.print("Base:"); Serial.print(angleBase);
    Serial.print(" | Schulter:"); Serial.print(angleDual);
    Serial.print(" | Finger:"); Serial.print(angleFinger);
    Serial.print(" | Wrist:"); Serial.print(angleWrist);
    Serial.print(" | Elbow:"); Serial.print(angleElbow);
    Serial.print(" | Arm:"); Serial.println(angleArm);
  }


  delay(30);
}

