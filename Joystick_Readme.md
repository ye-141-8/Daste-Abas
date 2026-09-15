# DasteAbas Joystick Firmware

Firmware für einen **6-Achsen-Roboterarm (Daste Abas)**, gesteuert über einen **analogen Joystick** und **4 Taster** auf dem Arduino/Funduino-Schild. Die Servos werden über einen **PCA9685**-Servocontroller angesteuert.

---

## Inhalt

- [Übersicht](#übersicht)
- [Hardware-Anforderungen](#hardware-anforderungen)
- [Verkabelung / Pins](#verkabelung--pins)
- [Funktionsweise](#funktionsweise)
- [Installation](#installation)
- [Kalibrierung](#kalibrierung)
- [Serielle Ausgabe](#serielle-ausgabe)
- [Konfiguration](#konfiguration)
- [Lizenz](#lizenz)

---

## Übersicht

Dieses Projekt steuert einen Roboterarm mit bis zu **6 Servos**. Ein zweiachsiger
Joystick wird gemeinsam mit vier Tastern verwendet, um **verschiedene Gelenke**
unabhängig voneinander zu bewegen. Dadurch lässt sich der Arm präzise und ohne
zusätzliche Fernbedienung bedienen.

**Achsentypen:**

| Achse          | Beschreibung                  |
| -------------- | ----------------------------- |
| `Base`         | Rotation des Arms             |
| `Schulter`     | Heben/Senken (2 gekoppelte Servos, gegenläufig) |
| `Finger`       | Öffnen/Schließen des Greifers |
| `Wrist`        | Handgelenk                    |
| `Elbow`        | Ellbogen                      |
| `Arm`          | Oberarm                       |

---

## Hardware-Anforderungen

| Komponente                | Menge |
| ------------------------- | ----- |
| Arduino / Funduino Board  | 1  |
| PCA9685 Servocontroller   | 1  |
| Analoger Joystick (2 Achsen) | 1 |
| Taster / Schalter         | 4  |
| Servos (50 Hz)            | bis zu 6 |

---

## Verkabelung / Pins

### PCA9685 Servo-Channels

| Kanal | Servo        |
| ----- | ------------ |
| 1     | Finger       |
| 2     | Wrist        |
| 3     | Elbow        |
| 4     | Arm          |
| 6     | Base         |
| 10    | Schulter links |
| 11    | Schulter rechts |

> **Hinweis:** Die beiden Schulter-Servos (Kanal 10 & 11) werden **spiegelbildlich**
> angesteuert (gegenläufig), sodass sie sich als eine gemeinsame Schulterachse verhalten.

### Arduino-Pins

| Pin | Funktion       |
| --- | -------------- |
| `A0` | Joystick X   |
| `A1` | Joystick Y   |
| `2`  | Taster A      |
| `3`  | Taster B      |
| `4`  | Taster C      |
| `5`  | Taster D      |

---

## Funktionsweise

Der Joystick liefert analoge Werte (0–1023). Bei eingeschaltetem Gerät wird
automatisch die **Joystick-Mitte** kalibriert. Ein **Deadzone**-Bereich um die
Mitte verhindert ungewollte Bewegungen.

Je nach gehaltener **Taster-Kombination** steuert der Joystick unterschiedliche
Achsen:

| Taster | Joystick X        | Joystick Y   |
| ------ | ----------------- | ------------ |
| –      | Base (Rotation)   | Schulter     |
| A      | Wrist             | –            |
| B      | –                 | Elbow        |
| C      | Finger (Greifer) | –            |
| D      | –                 | Arm          |

Der Roboterarm bewegt sich dabei in festen **Gradschritten** (standardmäßig 2°).
Bei jeder Bewegung wird ein Statusstring über die serielle Schnittstelle ausgegeben.

---

## Installation

1. Öffne die Datei `DasteAbas_Joystick.ino` in der **Arduino IDE**.
2. Installiere bei Bedarf die Bibliothek **`Wire`** (Teil der Standard-IDE).
3. Wähle das passende **Board** und den **Port** aus.
4. Lade den Sketch auf dein Board:

   ```bash
   # Alternativ über die Arduino-CLI
   arduino-cli compile --fqbn <BOARD> DasteAbas_Joystick.ino
   arduino-cli upload <BOARD> DasteAbas_Joystick.ino
   ```

---

## Kalibrierung

Die Joystick-Mitte wird beim **Start automatisch** aus 20 Messwerten gemittelt.
Für eine korrekte Kalibrierung muss der Joystick beim Einschalten **in der
Mittelposition** und **ohne Berührung** gehalten werden.

> Die automatische Mitte kann bei Bedarf über die Variablen
> `joyXCenter` und `joyYCenter` überschrieben werden.

---

## Serielle Ausgabe

Auf der seriellen Schnittstelle (**9600 Baud**) wird bei jeder Bewegung der
aktuelle Zustand aller Achsen ausgegeben:

```
Base:90 | Schulter:90 | Finger:90 | Wrist:88 | Elbow:90 | Arm:90
```

Beim Start erscheint `SYSTEM_BEREIT`, sobald das System einsatzbereit ist.

---

## Konfiguration

Die wichtigsten Einstellungen befinden sich am Anfang der Datei und lassen sich
einfach anpassen:

| Parameter           | Default | Beschreibung                              |
| ------------------- | ------- | ----------------------------------------- |
| `ANGLE_STEP`        | `2`     | Grad pro Update                           |
| `DEADZONE`          | `40`    | Empfindlichkeit um die Joystick-Mitte     |
| `PULSE_MIN`         | `130`   | PWM-Impuls für 0°                         |
| `PULSE_MAX`         | `490`   | PWM-Impuls für 180°                       |
| `PCA9685_ADDRESS`   | `0x40`  | I²C-Adresse des PCA9685                   |

---

## Lizenz

Dieses Projekt wird unter der **MIT-Lizenz** veröffentlicht. Siehe `LICENSE` für
weitere Informationen.