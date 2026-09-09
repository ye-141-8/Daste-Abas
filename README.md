# Daste Abas — Industrial Robot Arm (Prototype)

> A compact prototype of an **industrial robot arm** that can **move**, **rotate**, and **pick up / release**
> objects. The hand (gripper) is controlled directly from the **keyboard**.

This repository is a small proof-of-concept robotics project. It consists of an **Arduino firmware**
in C++ and a lightweight **Python desktop GUI** that lets you drive the arm live from your keyboard.

---

## Overview

The arm is built from servomotors driven by a **PCA9685 16-channel PWM/servo driver** over I2C.
Although a full industrial arm normally uses many joints and precise motion planning, this prototype
captures the core idea: a multi-joint arm with a gripper that the operator steers in real time.

The project is split into two parts:

| File | Language | Purpose |
| --- | --- | --- |
| `DasteAbas.cpp` | C++ (Arduino) | Firmware: reads serial keypresses and sets each servo angle |
| `arm_gui.py` | Python | Desktop GUI: forwards keyboard input to the Arduino over serial |

---

## Features

- **6 motorized axes** on 7 PWM channels
  - **Motor 1 – Base:** rotating ground / base plate
  - **Motor 2 – Finger (gripper):** the "hand" — opens and closes to **pick up / release** objects
  - **Motor 3 – Wrist:** rotates the hand
  - **Motor 4 – Arm:** the main lifting segment
  - **Motor 5 – Elbow:** bends the arm
  - **Motors 6 & 7 – Dual opposing:** a mirrored pair that works together
- **Keyboard control:** every joint responds to a dedicated key, 5° per press
- **Upright boot position:** all joints start at 90° so the arm stands straight up
- **Serial status reporting:** live angle feedback printed for every servo
- **Real-time GUI:** `arm_gui.py` renders the controls and streams keys to the Arduino

---

## Project Structure

```
daste_abas/
├── DasteAbas.cpp   # Arduino firmware (C++)
├── arm_gui.py      # Python keyboard-control GUI
├── README.md       # This file (English)
└── README.de.md    # German documentation
```

---

## Hardware Requirements

- **Arduino board** with I2C / `Wire` support
- **PCA9685** 16-channel servo driver
- **7-Bus servo motors** (or a robot arm kit with these joints)
- A suitable **power supply** for the servos
- A computer with **Python 3** and the `pyserial` package installed

---

## Wiring

| PCA9685 channel | Firmware pin | Motor |
| --- | --- | --- |
| 6  | `PIN_BASE`       | Motor 1 – Base |
| 1  | `PIN_FINGER`     | Motor 2 – Gripper finger |
| 2  | `PIN_WRIST`      | Motor 3 – Wrist |
| 4  | `PIN_ARM`        | Motor 4 – Main arm |
| 3  | `PIN_ELBOW`      | Motor 5 – Elbow |
| 10 | `PIN_DUAL_LEFT`  | Motor 6 – Dual (left) |
| 11 | `PIN_DUAL_RIGHT` | Motor 7 – Dual (right, mirrored) |

Connect the PCA9685 `SDA`, `SCL`, `VCC`, and `GND` lines to the Arduino and power the
servos from a separate supply.

---

## Getting Started

### 1. Upload the firmware

Open `DasteAbas.cpp` in the Arduino IDE, select your board, and upload it.

### 2. Install the Python dependencies

```bash
pip install pyserial
```

### 3. Set the serial port

Edit `ARDUINO_PORT` in `arm_gui.py` to match your system (e.g. `COM6` on Windows,
`/dev/ttyUSB0` on Linux). **Make sure the Arduino IDE Serial Monitor is closed.**

### 4. Run the GUI

```bash
python arm_gui.py
```

Connect your physical arm, then **control it with the keys** below.

---

## Keyboard Controls

The step size (default **5°** per press, `ANGLE_STEP` / `STEP_ANGLE`) applies to all joints.

| Motor | Axis | Decrease | Increase |
| --- | --- | --- | --- |
| 1 | Base | `A` | `D` |
| 2 | Gripper finger | `Q` | `E` |
| 3 | Wrist | `K` | `J` |
| 4 | Main arm | `O` | `I` |
| 5 | Elbow | `N` | `M` |
| 6 & 7 | Dual opposing | `S` | `W` |

> **To pick something up:** lower the arm, close the finger (`Q`) to grip the object,
> then lift the arm (`I`) and release it by opening the finger (`E`).

---

## Configuration

Firmware settings live at the top of `DasteAbas.cpp`:

- `ANGLE_STEP` — degrees per keypress (default `5`)
- `PULSE_MIN` / `PULSE_MAX` — map 0°–180° to servo pulse widths
- `PRESCALE` — PCA9685 clock for 50 Hz PWM
- Initial angles — all joints start at 90° for an upright stance

---

## How It Works

1. `setup()` initializes the PCA9685 for 50 Hz PWM and applies the 90° upright stance.
2. The firmware listens on the serial port for keypress characters.
3. Each key updates the corresponding joint angle, clamped to 0–180°.
4. `updateServos()` writes the new angles to every servo.
5. `printStatus()` prints the current angles on the serial line for the GUI to display.

On the desktop side, `arm_gui.py` opens the same serial port, listens for keyboard input,
and forwards each pressed key to the Arduino.

---

## License

Provided as-is for educational and hobby use.