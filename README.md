# Daste Abas - 6-Axis Robotic Arm Controller

Arduino sketch for controlling a **6-axis robotic arm** using a **PCA9685 16-channel PWM/servo driver** over I2C. The program performs a gentle homing sequence on power-up, runs individual motor diagnostics, exercises all motors together, and then continuously sweeps the arm between the start and target positions.

## Features

- **PCA9685 servo driver** via I2C (`0x40`)
- **6 motors / axes** driven by 7 PWM channels:
  - Motor 1 - Base plate / ground anchor (Pin 1)
  - Motor 2 - Gripper finger (Pin 2)
  - Motor 3 - Wrist rotation (Pin 4)
  - Motor 4 - Main arm / shoulder (Pin 5)
  - Motor 5 - Elbow (Pin 7)
  - Dual opposing motors (Pins 10 & 11) that mirror each other
- **Slow 3-second homing** on startup to avoid gear damage
- **Individual motor diagnostics** - every axis sweeps out and back 2 times
- **All-motors-together test** for synchronization check
- **Continuous main loop** - arm moves from start to target angles and back

## Hardware Requirements

- Arduino board (with I2C / `Wire` support)
- PCA9685 servo driver board
- 6 servo motors
- Power supply suitable for your servos

## Wiring

| PCA9685 channel | Arduino pin | Motor                                                        |
| --------------- | ----------- | ------------------------------------------------------------ |
| 1               | `PIN_BASE`  | Motor 1 - Base plate / ground anchor                         |
| 2               | `PIN_FINGER`| Motor 2 - Gripper finger                                     |
| 4               | `PIN_WRIST` | Motor 3 - Wrist rotation                                     |
| 5               | `PIN_ARM`   | Motor 4 - Main arm / shoulder                                |
| 7               | `PIN_ELBOW` | Motor 5 - Elbow                                              |
| 10, 11          | `PIN_DUAL_LEFT` / `PIN_DUAL_RIGHT` | Dual opposing motors (mirrored) |

Connect the PCA9685 `SDA`, `SCL`, `VCC`, and `GND` lines to the Arduino and power the servos separately.

## Configuration

Tune the motion in `DasteAbas.cpp`:

- **Angle range** - `BASE_START` / `BASE_TARGET` and the other `*_START` / `*_TARGET` constants define each axis home and target angle.
- **Speed** - `STEP_DELAY` (ms per smoothing step) and `SLOW_HOME_SPEED` (homing step delay). Lower values = faster motion.
- **Pulse limits** - `PULSE_MIN` / `PULSE_MAX` map 0° to 180° to servo pulse widths.
- **PCA9685 clock** - `PRESCALE` value sets the 50 Hz PWM frequency.

## How It Works

1. `setup()` initializes the PCA9685 for 50 Hz PWM.
2. `slowHomePosition()` gently moves every motor to the start angle over ~3 seconds.
3. Step 1 of diagnostics tests each motor individually (2 passes).
4. Step 2 moves all motors together.
5. `loop()` then repeatedly sweeps the arm forward (start -> target) and back (target -> start).

## License

This project is provided as-is for educational and hobby use.