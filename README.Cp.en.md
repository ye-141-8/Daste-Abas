# 🤖 Universal 6-DoF Robot Arm Control System

A fully integrated control system for a 6-DoF (Degree of Freedom) robot arm. The project combines an **Arduino C++ firmware** with hybrid input handling (Funduino Joystick Shield + serial keyboard commands) and a **Python desktop app** with an English Tkinter GUI, forward-kinematics 2D simulation, and a teach-in sequence player.

---

## 📸 Features

- **Hybrid Control:** Parallel input via physical joystick (Funduino Shield), PC keyboard hotkeys, or GUI sliders.
- **Real-Time Kinematics Simulation:** Graphical 2D preview of the arm positions in the Python application.
- **Teach-In & Sequence Player:** Save any motion routine as steps and have the robot arm play them back automatically.
- **Import/Export:** Save and load motion sequences in `JSON` format.
- **PWM PCA9685 Driver:** Precise servo control over I2C at 50 Hz.

---

## 🛠️ Hardware Components

- **Microcontroller:** Arduino Uno (or Mega)
- **Servo Driver:** PCA9685 16-channel 12-bit PWM Servo Driver Board (I2C addr. `0x40`)
- **Input:** Funduino / Joypad Shield (Joystick on A0/A1, buttons on pins 2, 3, 4, 5)
- **Actuators:** 7x servomotors (shoulder is driven synchronously by 2 counter-rotating servos)

### 🔌 Pin Assignment (PCA9685 Servo Shield)

| Servo / Joint | PCA9685 Channel |
| :--- | :--- |
| **Finger / Gripper** | Channel 1 |
| **Wrist** | Channel 2 |
| **Elbow** | Channel 3 |
| **Upper Arm** | Channel 4 |
| **Base (Rotation)** | Channel 6 |
| **Shoulder Left** | Channel 10 |
| **Shoulder Right (Mirrored)** | Channel 11 |

---

## 💻 Software Requirements & Installation

### 1. Arduino Setup
1. Open the `Arduino IDE`.
2. Make sure the standard `<Wire.h>` library is available.
3. Upload the Arduino C++ code to your Arduino.
4. Connect the **Funduino Shield** and the **PCA9685** (SDA -> A4, SCL -> A5) to the Arduino.

### 2. Setting up the Python environment
Make sure Python (3.8 or newer) is installed.

Install the required Python packages via your terminal / command prompt:

```bash
pip install pyserial
```