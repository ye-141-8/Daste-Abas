# Daste Abas — Cyber-Arm IK v0.4

> The **Inverse-Kinematics** version of the Daste Abas robot-arm control system, built on **PySide6 + Matplotlib**.
> Drives the arm live, simulates it in 3D, and lets you build **Pick & Place / Automation** sequences.

Two files belong together — the Python GUI and the matching Arduino firmware — and use the serial `P:`/`?`/`G:` protocol over 9600 baud:

| File | Language | Purpose |
| --- | --- | --- |
| `CP_Version_IK.py` | Python | Desktop studio: sliders, joystick, 3D simulator, IK target control, shape objects, Automation Studio |
| `CP_Version_IK.cpp` | C++ (Arduino) | Firmware: reads serial commands and drives the PCA9685 servos (+ physical joystick) |

---

## Setup

```bash
pip install pyserial PySide6 matplotlib numpy
# upload CP_Version_IK.cpp to the Arduino, close the Serial Monitor, then:
python CP_Version_IK.py
```

If the port isn't found (or **Simulation** is checked) the app runs fully without hardware so you can plan and test everything first.

## Joint model (decoupled)

- **Base** — spins the whole arm horizontally (yaw)
- **Dual (Shoulder)** — lifts the whole arm up/down on top of the base
- **Arm** — independent mid joint (separate from the shoulder)
- **Elbow** — bends the forearm up toward the wrist
- **Wrist** — left/right (roll)
- **Finger** — opens/closes the gripper

PCA9685 channel map: Finger `1`, Wrist `2`, Elbow `3`, Arm `4`, Base `6`, Shoulder Left/Right `10/11`.

## Main features

- **Manual control** — sliders with motor color-coded labels, keyboard shortcuts, virtual joystick, gripper open/close.
- **3D simulator** — neon sci-fi render of the color-coded arm + gripper jaws, draggable/free camera with presets (Top/Side/Front), and a **click-to-set target** on a chosen height plane.
- **Coordinate / IK control** — type or click a point (X,Y,Z in cm) and the arm moves there via IK. Base sits ~10 cm above the floor.
- **Shape objects** — add spheres / cubes / boxes / pyramids / cylinders with auto-colors and short names (`o1`, `o2`, …). Move/edit/delete them on the 3D view or in the studio.
- **Automation Studio** — build an ordered sequence of actions (MOVE / GRAB / PLACE / open-finger / close-finger / WAIT), reorder them, loop with repeat count, and run it. Object positions update as they are picked up and dropped.
- **Live movement log & system console** — every movement (joint, direction, from→to degrees, speed), plus a full system console.
- **Recorder / Sequencer / Presets / Motor-test sweep / Reach-R-Z** — all carried over from earlier versions.

## Persistence

- Scenes + sequences auto-save to `automation/scene.json`.
- Presets / config / saved sequences are kept in `presets.json`, `config.ini`, and `sequences/`.