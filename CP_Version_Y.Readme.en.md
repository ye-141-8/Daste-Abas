# Daste Abas — Industrial Robot Arm (Prototype)

> A compact prototype of an **industrial robot arm** that can **move**, **rotate**, and **pick up / release**
> objects. The gripper "hand" opens and closes. Everything is driven live from a **Python desktop GUI**
> or directly from the **keyboard**.

This repository is a small proof-of-concept robotics project. It consists of an **Arduino firmware**
in C++ and a feature-rich **Python desktop GUI** that lets you drive the arm live, record positions,
and play them back with 2D/3D simulations.

---

## Overview

The arm is built from servomotors driven by a **PCA9685 16-channel PWM/servo driver** over I2C.
Although a full industrial arm normally uses many joints and precise motion planning, this prototype
captures the core idea: a multi-joint arm with a gripper that the operator steers in real time.

The project is split into two parts:

| File | Language | Purpose |
| --- | --- | --- |
| `CP_Version.cpp` | C++ (Arduino) | Firmware: reads serial commands/keys and sets each servo angle |
| `CP_Version.py` | Python | Desktop GUI: sliders, teach-in player, 2D + 3D simulator |

> Files `DasteAbas.cpp` and `arm_gui.py` are earlier/legacy versions. `CP_Version.c*` is the
> current, full-featured build.

---

## Features

- **6 motorized axes** on 7 PWM channels
  - **Base:** rotating ground / base plate
  - **Finger (gripper):** the "hand" — opens and closes to **pick up / release** objects
  - **Wrist:** rotates the hand
  - **Arm:** the main lifting segment
  - **Elbow:** bends the arm
  - **Dual opposing (2 motors):** a mirrored pair that works together
- **Software reset:** bring the whole arm back to 90° (straight up) with one button or the `R` key
- **Teach-in sequence player:** save positions, then
  - play the **entire sequence** (with optional endless loop), or
  - play **a single selected position** once
- **Real-time recording (REC):** no need to add positions one by one — press `● REC`, move the arm
  (slider, keyboard or joystick), then press `■ Stop Aufn.` and the whole time-based motion is captured
  and played back faithfully with `► Aufn. Abspielen`
- **Time display:** the duration of every recording and playback is shown live (`MM:SS`), including the
  classic teach-in playback
- **Live joystick indicator:** a crosshair widget shows the current X/Y joystick position in real time
- **Motor diagnosis (test):** `⚡ Teste Motoren` opens a sci-fi diagnosis window that sweeps **each motor
  individually** from fully closed (20°) to fully open (160°) and asks for visual confirmation. The
  **Dual opposing motors are tested as a single entry** — only one value is driven, while left/right run
  **mirrored against each other** automatically.
- **Smooth motion:** positions are approached in small interpolated steps instead of jumping
- **Adjustable speed:** playback speed slider
- **2D kinematics simulator** rendered in real time
- **3D simulator** (matplotlib) showing the base rotating around the Z axis
- **Gripper indicator:** visual open/closed state + angle readout
- **Live angle readout** for all joints
- **Grouped toolbar** (Edit left / Playback + Record stacked in the middle / File right) and consistent button styling
- **Configurable serial port** persisted to `robot_config.json`
- **Status bar** with connection state
- **Keyboard control** for every joint, 5° per press
- **Serial status reporting** — the GUI syncs back the angle state from the Arduino

---

## Project Structure

```
daste_abas/
├── CP_Version.cpp   # Arduino firmware (C++)  <- current
├── CP_Version.py    # Python control GUI      <- current
├── DasteAbas.cpp       # Legacy firmware
├── arm_gui.py          # Legacy keyboard GUI
├── CP_Version.md       # English documentation
└── CP_Version.de.md    # German documentation
```

---

## Hardware Requirements

- **Arduino board** with I2C / `Wire` support
- **PCA9685** 16-channel servo driver
- **Servomotors** (7 units, or a robot arm kit with these joints)
- A suitable **power supply** for the servos
- A computer with **Python 3** and `pyserial` + `matplotlib` installed

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

Open `CP_Version.cpp` in the Arduino IDE, select your board, and upload it.

### 2. Install the Python dependencies

```bash
pip install pyserial matplotlib
```

### 3. Set the serial port

Enter the port in the GUI (bottom-left, e.g. `COM6` on Windows, `/dev/ttyUSB0` on Linux) and
press **Verbinden**. The port is saved to `robot_config.json`. Alternatively edit it directly:

```json
{ "port": "COM6" }
```

**Make sure the Arduino IDE Serial Monitor is closed** so the port is free.

### 4. Run the GUI

```bash
python CP_Version.py
```

Connect your physical arm, then move the sliders or use the keys below.

---

## GUI Overview

| Area | What it does |
| --- | --- |
| Sliders (left) | Mechanically control each joint |
| Geschwindigkeit (left) | Playback speed |
| 2D / 3D sim | Real-time kinematics preview |
| Live readout | Current angle of every joint |
| Position Speichern / Löschen | Add / remove teach-in steps |
| Abspielen (Alle / Einzel) | Play the full sequence or one step |
| ■ Stop / Leertaste | Halt playback |
| Endlos wiederholen | Loop the sequence |
| ● REC / ■ Stop Aufn. / ► Aufn. Abspielen | Record real-time motion, stop & play it back |
| Time display | Duration of recording & playback (`MM:SS`) |
| Joystick (left) | Live X/Y joystick position |
| ⚡ Teste Motoren | Sci-fi motor diagnosis: sweep motors + confirm (Dual mirrored) |
| Datei Laden / Speichern | Save / load sequences (JSON) |
| ⊕ Reset (90°) / `R` | Return the arm to the upright stance |

---

## Keyboard Controls

The step size (default **5°** per press) applies to all joints.

| Motor | Axis | Decrease | Increase |
| --- | --- | --- | --- |
| 1 | Base | `A` | `D` |
| 2 | Gripper finger | `Q` | `E` |
| 3 | Wrist | `K` | `J` |
| 4 | Main arm | `O` | `I` |
| 5 | Elbow | `N` | `M` |
| 6 & 7 | Dual opposing | `S` | `W` |
| – | Reset (90°) | `R` | – |
| – | Stop playback | `Space` | – |

> **To pick something up:** lower the arm, close the finger (`Q`) to grip the object,
> then lift the arm (`I`) and release it by opening the finger (`E`).

---

## Serial Protocol

The GUI and firmware talk over serial with simple text commands:

- `P:base,finger,wrist,arm,elbow,dual` — set all angles directly (e.g. `P:90,90,90,90,90,90`)
- Single character keys (`a`, `d`, `q`, …) — nudge a single joint
- `R` — reset all angles to 90°
- Firmware reports each state as `POS:base,finger,wrist,arm,elbow,dual`
- Firmware reports the raw joystick values as `JS:x,y` for the live GUI indicator

---

## Configuration

Firmware settings live at the top of `CP_Version.cpp`:

- `ANGLE_STEP` — degrees per keypress (default `3`)
- `PULSE_MIN` / `PULSE_MAX` — map 0°–180° to servo pulse widths
- `PRESCALE` — PCA9685 clock for 50 Hz PWM
- Initial angles — all joints start at 90° for an upright stance

---

## How It Works

1. `setup()` initializes the PCA9685 for 50 Hz PWM and applies the 90° upright stance.
2. `loop()` reads serial commands/keys and camera polling from the shield.
3. `parseDirectPosition()` sets all angles from a `P:` command.
4. `updateServos()` writes the angles to every servo.
5. `printStatus()` prints the current angles over serial so the GUI can mirror them.

On the desktop side, `CP_Version.py` sends `P:` commands for slider/keyboard/reset/playback,
interpolates smooth motion, and renders the live 2D/3D simulation.

---

## Troubleshooting

- **Servo doesn't move** — first confirm the code path: that joint's slider sends a `P:` value and
  the firmware writes that channel. If the logic is correct, test the hardware: swap the motor to a
  known-good channel (e.g. set `PIN_FINGER` to the wrist channel temporarily). If it moves there,
  the original channel/pin/cable is at fault.
- **No connection** — close the Arduino Serial Monitor and check the COM port.

---

## License

Provided as-is for educational and hobby use.