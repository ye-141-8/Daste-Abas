# Daste Abas - Steuerung für einen 6-Achsen-Roboterarm

Arduino-Sketch zur Steuerung eines **6-Achsen-Roboterarms** mit einem **PCA9685-16-Kanal-PWM/Servo-Treiber** über I2C. Das Programm führt beim Einschalten eine sanfte Homing-Sequenz aus, testet die einzelnen Motoren, bewegt alle Motoren gemeinsam und schwenkt den Arm anschließend kontinuierlich zwischen Start- und Zielpositionen hin und her.

## Funktionen

- **PCA9685-Servo-Treiber** über I2C (`0x40`)
- **6 Motoren / Achsen**, angesteuert über 7 PWM-Kanäle:
  - Motor 1 - Basisplatte / Bodenanker (Pin 1)
  - Motor 2 - Greiferfinger (Pin 2)
  - Motor 3 - Handgelenks-Drehung (Pin 4)
  - Motor 4 - Hauptarm / Schulter (Pin 5)
  - Motor 5 - Ellenbogen (Pin 7)
  - Duo-Gegenmotoren (Pins 10 & 11), die sich spiegelbildlich bewegen
- **Langsames Homing über 3 Sekunden** beim Start zum Schutz der Zahnräder
- **Einzelmotor-Diagnose** - jede Achse schwenkt 2-mal aus und zurück
- **Gemeinsamer Motortest** zur Überprüfung der Synchronisation
- **Endlosschleife** - der Arm bewegt sich von den Start- zu den Zielwinkeln und zurück

## Benötigte Hardware

- Arduino-Board (mit I2C / `Wire`-Unterstützung)
- PCA9685-Servo-Treiber
- 6 Servomotoren
- Passendes Netzteil für die Servos

## Verkabelung

| PCA9685-Kanal | Arduino-Pin | Motor                                                    |
| --------------- | ----------- | -------------------------------------------------------- |
| 1               | `PIN_BASE`  | Motor 1 - Basisplatte / Bodenanker                       |
| 2               | `PIN_FINGER`| Motor 2 - Greiferfinger                                  |
| 4               | `PIN_WRIST` | Motor 3 - Handgelenks-Drehung                            |
| 5               | `PIN_ARM`   | Motor 4 - Hauptarm / Schulter                            |
| 7               | `PIN_ELBOW` | Motor 5 - Ellenbogen                                     |
| 10, 11          | `PIN_DUAL_LEFT` / `PIN_DUAL_RIGHT` | Duo-Gegenmotoren (gespiegelt)          |

Verbinden Sie die `SDA`-, `SCL`-, `VCC`- und `GND`-Leitungen des PCA9685 mit dem Arduino und versorgen Sie die Servos separat mit Strom.

## Konfiguration

Die Bewegung wird in `DasteAbas.cpp` eingestellt:

- **Winkelbereich** - `BASE_START` / `BASE_TARGET` sowie die anderen `*_START`- / `*_TARGET`-Konstanten legen den Home- und Zielwinkel jeder Achse fest.
- **Geschwindigkeit** - `STEP_DELAY` (ms pro Glättungsschritt) und `SLOW_HOME_SPEED` (Verzögerung beim Homing). Niedrigere Werte = schnellere Bewegung.
- **Pulsgrenzen** - `PULSE_MIN` / `PULSE_MAX` bilden 0° bis 180° auf Servo-Pulsbreiten ab.
- **PCA9685-Takt** - Der `PRESCALE`-Wert stellt die 50-Hz-PWM-Frequenz ein.

## Funktionsweise

1. `setup()` initialisiert den PCA9685 für 50-Hz-PWM.
2. `slowHomePosition()` bewegt jeden Motor über ca. 3 Sekunden sanft in den Startwinkel.
3. Schritt 1 der Diagnose testet jeden Motor einzeln (2 Durchläufe).
4. Schritt 2 bewegt alle Motoren gemeinsam.
5. `loop()` schwenkt den Arm anschließend wiederholt vor (Start -> Ziel) und zurück (Ziel -> Start).

## Lizenz

Dieses Projekt wird wie besehen für Ausbildungs- und Hobbyzwecke bereitgestellt.