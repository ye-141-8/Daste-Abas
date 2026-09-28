# Daste Abas — 6-DoF Robot Arm Control System

A complete control system for a **6-DoF (Degree of Freedom) robot arm** (`Daste Abas`). The project
combines **Arduino firmware** (C++) with **Python desktop GUIs**. The arm is driven by servomotors
via a **PCA9685 16-channel PWM/servo driver** over I2C and can be steered live from a physical
joystick, keyboard hotkeys, or the GUI.

This document is a **top-level guide**: it explains what the whole repository contains, what each file
does, and which files are connected to each other.

> 💬 The repository has been developed together by two contributors:
> **EM** = [`EFN360`](https://github.com/EFN360) and **Y** = [`ye-141-8`](https://github.com/ye-141-8).

---

## System Overview

The hardware arm is controlled by an Arduino. Two separate, parallel "versions" of the firmware +
GUI exist (one per contributor), plus some earlier `legacy` files. The two versions **do not talk to
each other** — pick **one** firmware and the matching GUI when you set up your arm.

```
                       ┌────────────────────────────┐
   Physical joystick ─▶│         Arduino           │──▶ PCA9685 ─▶ Servos
   (Funduino shield)   │  (firmware .cpp / .ino)   │
                       └────────────┬───────────────┘
                                    │  Serial (USB, 9600 baud)
                                    ▼
                       ┌────────────────────────────┐
                       │      Python desktop GUI    │
                       └────────────────────────────┘
```

**Two active variants:**

| Variant | Firmware | GUI | Config files | Author |
| :--- | :--- | :--- | :--- | :--- |
| **Version Y** | `CP_Version_Y.cpp` | `CP_Version_Y.py` | `robot_config.json` | Y (`ye-141-8`) |
| **Version EM** | `Control_Panel_EM.cpp` | `Control_Panel_EM.py` | `config.ini`, `presets.json` | EM (`EFN360`) |

**Legacy files** (older, replaced by the versions above): `DasteAbas.cpp`, `arm_gui.py`,
`DasteAbas_Joystick.ino`.

---

## File-by-File Guide

### Arduino Firmware (C++)

Upload only the firmware that matches the GUI you want to run.

| File | What it does | Pairs with |
| :--- | :--- | :--- |
| `CP_Version_Y.cpp` | Firmware for **Version Y**. Reads serial commands (`P:` positions, single keys) **and** the physical joystick. Reports `POS:...` (angles) and `JS:x,y` (raw joystick) over serial. | `CP_Version_Y.py` |
| `Control_Panel_EM.cpp` | Firmware for **Version EM**. Equivalent to Version Y but with **smooth motion / easing** (`SMOOTH_EASE`, `EASE_SNAP`), a `?` position-request command, and current+target angle states. Reports `POS:...`. | `Control_Panel_EM.py` |
| `DasteAbas.cpp` | **Legacy** firmware. Simplest version: reads single keyboard characters, drives each servo, reports `M1 Base:... \| M2 Finger:...`. No `P:` protocol. | `arm_gui.py` |
| `DasteAbas_Joystick.ino` | **Legacy / standalone** firmware. Joystick-only control (no GUI). Buttons select which joint the joystick steers; prints `Base:90 \| Schulter:90 ...`. German comments. | none (self-contained) |

### Python Desktop GUIs

| File | What it does | Connects to |
| :--- | :--- | :--- |
| `CP_Version_Y.py` | Full-featured GUI for **Version Y** (German UI). Sliders, teach-in sequence player, real-time **REC** recording with playback, live joystick indicator, motor-diagnosis window, **2D + 3D** (matplotlib) kinematics simulator. Sends `P:...` / keys, reads `POS:` + `JS:`. | `CP_Version_Y.cpp` + `robot_config.json` |
| `Control_Panel_EM.py` | Modern **CustomTkinter** dark-theme GUI for **Version EM**. Joint sliders, mouse-driven virtual joystick, **2D side-view kinematics** with target/actual ghost, motor test sweep, motion recorder, teach-in sequencer, preset poses, console panel, E-Stop (`ESC`). Sends `P:...` + `?`, reads `POS:`. | `Control_Panel_EM.cpp` + `config.ini` + `presets.json` |
| `arm_gui.py` | **Legacy** simple Tkinter GUI. Forwards single keyboard characters to the old firmware. | `DasteAbas.cpp` |

### Configuration Files

| File | Used by | Contents |
| :--- | :--- | :--- |
| `robot_config.json` | `CP_Version_Y.py` | Stores the serial COM port (e.g. `{ "port": "COM6" }`). |
| `config.ini` | `Control_Panel_EM.py` | Serial port/baud, safe-home pose (safety), recorder speed/loop, window geometry. |
| `presets.json` | `Control_Panel_EM.py` | Saved named poses (presets) for the EM GUI. |

### Documentation (READMEs)

Each version/firmware already has its own README. Use the one that matches your setup.

| File | Covers |
| :--- | :--- |
| `CP_Version_Y.Readme.en.md` / `CP_Version_Y.Readme.de.md` | Full docs for **Version Y** (`CP_Version_Y.c*`). |
| `README.Cp_EM.md` / `README_EM.md` | Full docs for **Version EM** (`Control_Panel_EM.c*`). |
| `README.Cp.md` / `README.Cp.en.md` | Docs for the general/legacy **keyboard control** firmware + `arm_gui.py`. |
| `Joystick_Readme.md` | Docs for the standalone `DasteAbas_Joystick.ino` firmware. |
| `README.md` | **This file** — top-level overview of the whole repository. |

---

## How the Files Are Connected

```
┌── VERSION Y (author Y / ye-141-8) ──────────────────────────────┐
│                                                                │
│   CP_Version_Y.py ──P:/keys, POS:/JS:──▶ CP_Version_Y.cpp      │
│        │  ◀──reads──                                  │          │
│        └──────── robot_config.json                     │          │
│                                                   PCA9685 ─▶ servos
│                                                                │
├── VERSION EM (author EM / EFN360) ─────────────────────────────┤
│                                                                │
│   Control_Panel_EM.py ──P:/? , POS:──▶ Control_Panel_EM.cpp    │
│        │  ◀──reads──                    (smooth easing) │       │
│        ├──────── config.ini                        PCA9685 ─▶ servos
│        └──────── presets.json
│
└── LEGACY ───────────────────────────────────────────────────────┘
   arm_gui.py ──single keys──▶ DasteAbas.cpp ─▶ PCA9685 ─▶ servos
   DasteAbas_Joystick.ino ─▶ (standalone, joystick only)
```

### Serial Protocol

All active GUIs talk to their firmware over a serial connection at `9600 baud`:

- `P:base,finger,wrist,arm,elbow,dual` — set all joint angles directly (e.g. `P:90,90,90,90,90,90`)
- Single-character keys (`a`, `d`, `q`, …) — nudge a single joint by the step angle
- `?` (EM only) — ask the firmware to report its current position
- Firmware **echoes** its state as `POS:base,finger,wrist,arm,elbow,dual`
- Version Y also reports raw joystick values as `JS:x,y` for the GUI indicator

**PCA9685 channel mapping** (identical on both versions):

| Joint | Channel |
| :--- | :--- |
| Finger / Gripper | 1 |
| Wrist | 2 |
| Elbow | 3 |
| Upper Arm | 4 |
| Base (rotation) | 6 |
| Shoulder Left | 10 |
| Shoulder Right (mirrored) | 11 |

The shoulder is dual: two counter-rotating servos (`10` + `11`) form one shoulder axis.

---

## Quick Start

```bash
# 1) Arduino: open your chosen firmware in the Arduino IDE, upload it.
#    (CP_Version_Y.cpp  or  Control_Panel_EM.cpp)

# 2) Install the Python dependency for your GUI:
pip install pyserial            # Version Y
pip install pyserial customtkinter matplotlib   # Version EM

# 3) Set the serial port (in the GUI or in the config file),
#    close the Arduino Serial Monitor, then run:
python CP_Version_Y.py         # Version Y
#  or
python Control_Panel_EM.py     # Version EM
```

> Note: `DasteAbas.cpp` and `arm_gui.py` are legacy and are kept only for reference/history.
> Use the matching current pair (`_Y` or `_EM`) for a new setup.

---

## License

Provided as-is for educational and hobby use.
