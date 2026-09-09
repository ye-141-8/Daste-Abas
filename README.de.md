# Daste Abas — Industrie-Roboterarm (Prototyp)

> Ein **kompakter Prototyp eines Industrie-Roboterarms**, der sich **bewegen**, **drehen** und Objekte
> **aufnehmen oder loslassen** kann. Die **Hand (Greifer) wird direkt über die Tastatur gesteuert**.

Dieses Repository ist ein kleines Proof-of-Concept-Projekt aus der Robotik. Es besteht aus einer
**Arduino-Firmware** in C++ und einer **schlanken Python-Desktop-GUI**, mit der Sie den Arm live über
Ihre Tastatur steuern können.

---

## Überblick

Der Arm wird durch Servomotoren angetrieben, die über einen **PCA9685-16-Kanal-PWM/Servo-Treiber**
per I2C angesteuert werden. Ein vollständiger Industriearm nutzt normalerweise viele Gelenke und
präzise Bewegungsplanung — dieser Prototyp bildet aber den Kern ab: einen mehrachsigen Arm mit
Greifer, den der Bediener in Echtzeit steuert.

Das Projekt besteht aus zwei Teilen:

| Datei | Sprache | Zweck |
| --- | --- | --- |
| `DasteAbas.cpp` | C++ (Arduino) | Firmware: liest Tasteneingaben über seriell und setzt jeden Servowinkel |
| `arm_gui.py` | Python | Desktop-GUI: leitet Tastatureingaben über seriell an den Arduino weiter |

---

## Funktionen

- **6 motorisierte Achsen** auf 7 PWM-Kanälen
  - **Motor 1 – Basis:** drehbarer Boden / Grundplatte
  - **Motor 2 – Finger (Greifer):** die „Hand" — öffnet und schließt, um Objekte **aufzunehmen / loszulassen**
  - **Motor 3 – Handgelenk:** dreht die Hand
  - **Motor 4 – Arm:** das hauptsächliche Hubsegment
  - **Motor 5 – Ellenbogen:** beugt den Arm
  - **Motoren 6 & 7 – Duo-Gegenmotoren:** ein gespiegeltes Paar, das gemeinsam arbeitet
- **Tastatursteuerung:** jedes Gelenk reagiert auf eine eigene Taste, 5° pro Tastendruck
- **Aufrechte Startposition:** alle Gelenke starten bei 90°, sodass der Arm gerade steht
- **Serielle Statusausgabe:** Live-Winkelrückmeldung für jeden Servo
- **Echtzeit-GUI:** `arm_gui.py` rendert die Steuerung und leitet Tasten an den Arduino weiter

---

## Projektstruktur

```
daste_abas/
├── DasteAbas.cpp   # Arduino-Firmware (C++)
├── arm_gui.py      # Python-Tastatursteuerungs-GUI
├── README.md       # Englische Doku
└── README.de.md    # Deutsche Doku
```

---

## Benötigte Hardware

- **Arduino-Board** mit I2C / `Wire`-Unterstützung
- **PCA9685**-16-Kanal-Servo-Treiber
- **Servomotoren** (7 Stück bzw. ein Roboterarm-Bausatz mit diesen Gelenken)
- Ein geeignetes **Netzteil** für die Servos
- Ein Computer mit **Python 3** und installiertem `pyserial`-Paket

---

## Verkabelung

| PCA9685-Kanal | Firmware-Pin | Motor |
| --- | --- | --- |
| 6  | `PIN_BASE`       | Motor 1 – Basis |
| 1  | `PIN_FINGER`     | Motor 2 – Greiferfinger |
| 2  | `PIN_WRIST`      | Motor 3 – Handgelenk |
| 4  | `PIN_ARM`        | Motor 4 – Hauptarm |
| 3  | `PIN_ELBOW`      | Motor 5 – Ellenbogen |
| 10 | `PIN_DUAL_LEFT`  | Motor 6 – Duo (links) |
| 11 | `PIN_DUAL_RIGHT` | Motor 7 – Duo (rechts, gespiegelt) |

Verbinden Sie die `SDA`-, `SCL`-, `VCC`- und `GND`-Leitungen des PCA9685 mit dem Arduino und
versorgen Sie die Servos über ein separates Netzteil.

---

## Erste Schritte

### 1. Firmware hochladen

Öffnen Sie `DasteAbas.cpp` in der Arduino-IDE, wählen Sie Ihr Board aus und laden Sie den Sketch hoch.

### 2. Python-Abhängigkeiten installieren

```bash
pip install pyserial
```

### 3. Seriellen Port festlegen

Bearbeiten Sie `ARDUINO_PORT` in `arm_gui.py`, damit er zu Ihrem System passt (z. B. `COM6` unter
Windows, `/dev/ttyUSB0` unter Linux). **Achten Sie darauf, dass der serielle Monitor der Arduino-IDE
geschlossen ist.**

### 4. GUI starten

```bash
python arm_gui.py
```

Verbinden Sie Ihren echten Arm und **steuern Sie ihn mithilfe der Tasten** unten.

---

## Tastatursteuerung

Die Schrittweite (Standard **5°** pro Tastendruck, `ANGLE_STEP` / `STEP_ANGLE`) gilt für alle Gelenke.

| Motor | Achse | Verringern | Erhöhen |
| --- | --- | --- | --- |
| 1 | Basis | `A` | `D` |
| 2 | Greiferfinger | `Q` | `E` |
| 3 | Handgelenk | `K` | `J` |
| 4 | Hauptarm | `O` | `I` |
| 5 | Ellenbogen | `N` | `M` |
| 6 & 7 | Duo-Gegenmotoren | `S` | `W` |

> **So nehmen Sie etwas auf:** Arm senken, Finger schließen (`Q`), um das Objekt zu greifen,
> dann den Arm heben (`I`) und das Objekt durch Öffnen des Fingers (`E`) wieder loslassen.

---

## Konfiguration

Die Firmware-Einstellungen stehen oben in `DasteAbas.cpp`:

- `ANGLE_STEP` — Grad pro Tastendruck (Standard `5`)
- `PULSE_MIN` / `PULSE_MAX` — bilden 0°–180° auf Servo-Pulsbreiten ab
- `PRESCALE` — PCA9685-Takt für 50-Hz-PWM
- Anfangswinkel — alle Gelenke starten bei 90° für eine aufrechte Haltung

---

## Funktionsweise

1. `setup()` initialisiert den PCA9685 für 50-Hz-PWM und setzt die aufrechte 90°-Haltung.
2. Die Firmware lauscht auf dem seriellen Port auf Tastendrücke.
3. Jede Taste aktualisiert den passenden Gelenkwinkel, begrenzt auf 0–180°.
4. `updateServos()` schreibt die neuen Winkel auf jeden Servo.
5. `printStatus()` gibt die aktuellen Winkel über seriell aus, damit die GUI sie anzeigen kann.

Auf der Desktop-Seite öffnet `arm_gui.py` denselben seriellen Port, lauscht auf Tastatureingaben
und sendet jede gedrückte Taste an den Arduino.

---

## Lizenz

Bereitgestellt wie besehen für Ausbildungs- und Hobbyzwecke.