# Daste Abas — Industrie-Roboterarm (Prototyp)

> Ein **kompakter Prototyp eines Industrie-Roboterarms**, der sich **bewegen**, **drehen** und Objekte
> **aufnehmen oder loslassen** kann. Die **Greifer-Hand öffnet und schließt**. Alles wird live über eine
> **Python-Desktop-GUI** oder direkt über die **Tastatur** gesteuert.

Dieses Repository ist ein kleines Proof-of-Concept-Projekt aus der Robotik. Es besteht aus einer
**Arduino-Firmware** in C++ und einer **funktionsreichen Python-Desktop-GUI**, mit der Sie den Arm
steuern, Positionen aufnehmen und mit 2D-/3D-Simulation abspielen können.

---

## Überblick

Der Arm wird durch Servomotoren angetrieben, die über einen **PCA9685-16-Kanal-PWM/Servo-Treiber**
per I2C angesteuert werden. Ein vollständiger Industriearm nutzt normalerweise viele Gelenke und
präzise Bewegungsplanung — dieser Prototyp bildet aber den Kern ab: einen mehrachsigen Arm mit
Greifer, den der Bediener in Echtzeit steuert.

Das Projekt besteht aus zwei Teilen:

| Datei | Sprache | Zweck |
| --- | --- | --- |
| `CP_Version.cpp` | C++ (Arduino) | Firmware: liest serielle Befehle/Tasten und setzt jeden Servowinkel |
| `CP_Version.py` | Python | Desktop-GUI: Schieberegler, Teach-In-Player, 2D- + 3D-Simulator |

> `DasteAbas.cpp` und `arm_gui.py` sind frühere/Legacy-Versionen. `CP_Version.*` ist die
> aktuelle, vollwertige Version.

---

## Funktionen

- **6 motorisierte Achsen** auf 7 PWM-Kanälen
  - **Basis:** drehbarer Boden / Grundplatte
  - **Finger (Greifer):** die „Hand" — öffnet und schließt, um Objekte **aufzunehmen / loszulassen**
  - **Handgelenk:** dreht die Hand
  - **Arm:** das hauptsächliche Hubsegment
  - **Ellenbogen:** beugt den Arm
  - **Duo-Gegenmotoren (2):** ein gespiegeltes Paar, das gemeinsam arbeitet
- **Software-Reset:** ganzen Arm mit einem Klick oder der Taste `R` auf 90° (gerade nach oben) zurücksetzen
- **Teach-In-Player:** Positionen speichern, dann
  - **gesamte Sequenz** abspielen (optional Endlos-Schleife), oder
  - **eine einzelne ausgewählte Position** einmal abspielen
- **Echtzeit-Aufnahme (REC):** Aufnahme ohne einzelnes Hinzufügen — `● REC` drücken, den Arm bewegen
  (Slider, Tastatur oder Joystick), `■ Stop Aufn.` drücken → die gesamte zeitliche Bewegung wird
  aufgezeichnet und mit `► Aufn. Abspielen` originalgetreu wiedergegeben
- **Zeitanzeige:** Dauer jeder Aufnahme und Wiedergabe wird live angezeigt (`MM:SS`), auch bei der
  klassischen Teach-In-Wiedergabe
- **Joystick-Live-Anzeige:** ein Crosshair-Widget zeigt die aktuelle X/Y-Joystick-Position in Echtzeit
- **Motor-Diagnose (Test):** `⚡ Teste Motoren` öffnet ein SciFi-Diagnose-Fenster, das **jeden Motor einzeln**
  von ganz geschlossen (20°) zu ganz offen (160°) durchfährt und zur visuellen Bestätigung auffordert.
  Die **Duo-Gegenmotoren werden als ein Eintrag gemeinsam getestet** — gedreht wird nur ein Wert, woraufhin
  links/rechts automatisch **gespiegelt gegeneinander** laufen.
- **Sanfte Bewegung:** Positionen werden in kleinen Interpolationsschritten statt springend angefahren
- **Einstellbare Geschwindigkeit:** Geschwindigkeitsregler
- **2D-Kinematik-Simulator** in Echtzeit
- **3D-Simulator** (matplotlib), zeigt die Rotation der Basis um die Z-Achse
- **Greifer-Anzeige:** visueller offen/geschlossen-Zustand + Winkelwert
- **Live-Winkelanzeige** für alle Gelenke
- **Gruppierte Toolbar** (Bearbeiten links / Wiedergabe + Aufnahme mittig übereinander / Datei rechts) mit einheitlichem Button-Design
- **Konfigurierbarer serieller Port**, gespeichert in `robot_config.json`
- **Statusleiste** mit Verbindungszustand
- **Tastatursteuerung** für jedes Gelenk, 5° pro Tastendruck
- **Serielle Statusausgabe** — die GUI übernimmt den Winkelzustand vom Arduino

---

## Projektstruktur

```
daste_abas/
├── CP_Version.cpp   # Arduino-Firmware (C++)  <- aktuell
├── CP_Version.py    # Python-Steuerungs-GUI    <- aktuell
├── DasteAbas.cpp       # Legacy-Firmware
├── arm_gui.py          # Legacy-Tastatur-GUI
├── CP_Version.md       # Englische Dokumentation
└── CP_Version.de.md    # Deutsche Dokumentation
```

---

## Benötigte Hardware

- **Arduino-Board** mit I2C / `Wire`-Unterstützung
- **PCA9685**-16-Kanal-Servo-Treiber
- **Servomotoren** (7 Stück bzw. ein Roboterarm-Bausatz mit diesen Gelenken)
- Ein geeignetes **Netzteil** für die Servos
- Ein Computer mit **Python 3** und installiertem `pyserial`- + `matplotlib`-Paket

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

Öffnen Sie `CP_Version.cpp` in der Arduino-IDE, wählen Sie Ihr Board aus und laden Sie den Sketch hoch.

### 2. Python-Abhängigkeiten installieren

```bash
pip install pyserial matplotlib
```

### 3. Seriellen Port festlegen

Geben Sie den Port unten links in der GUI ein (z. B. `COM6` unter Windows, `/dev/ttyUSB0` unter Linux)
und drücken Sie **Verbinden**. Der Port wird in `robot_config.json` gespeichert. Alternativ direkt editieren:

```json
{ "port": "COM6" }
```

**Achten Sie darauf, dass der serielle Monitor der Arduino-IDE geschlossen ist.**

### 4. GUI starten

```bash
python CP_Version.py
```

Verbinden Sie Ihren echten Arm und bewegen Sie die Schieberegler oder nutzen Sie die Tasten unten.

---

## GUI-Übersicht

| Bereich | Funktion |
| --- | --- |
| Schieberegler (links) | Jedes Gelenk mechanisch steuern |
| Geschwindigkeit (links) | Abspielgeschwindigkeit |
| 2D-/3D-Sim | Echtzeit-Kinematik-Vorschau |
| Live-Anzeige | Aktueller Winkel jedes Gelenks |
| Position Speichern / Löschen | Teach-In-Schritte hinzufügen / entfernen |
| Abspielen (Alle / Einzel) | Komplette Sequenz oder einen Schritt abspielen |
| ■ Stop / Leertaste | Wiedergabe anhalten |
| Endlos wiederholen | Sequenz in Schleife abspielen |
| ● REC / ■ Stop Aufn. / ► Aufn. Abspielen | Echtzeit-Aufnahme aufzeichnen, beenden & wiedergeben |
| Zeit-Anzeige | Dauer von Aufnahme & Wiedergabe (`MM:SS`) |
| Joystick (links) | Live X/Y-Stellung des Joysticks |
| ⚡ Teste Motoren | SciFi-Motor-Diagnose: Motoren durchfahren + bestätigen (Duo gespiegelt) |
| Datei Laden / Speichern | Sequenzen speichern / laden (JSON) |
| ⊕ Reset (90°) / `R` | Arm in die aufrechte Position zurückführen |

---

## Tastatursteuerung

Die Schrittweite (Standard **5°** pro Tastendruck) gilt für alle Gelenke.

| Motor | Achse | Verringern | Erhöhen |
| --- | --- | --- | --- |
| 1 | Basis | `A` | `D` |
| 2 | Greiferfinger | `Q` | `E` |
| 3 | Handgelenk | `K` | `J` |
| 4 | Hauptarm | `O` | `I` |
| 5 | Ellenbogen | `N` | `M` |
| 6 & 7 | Duo-Gegenmotoren | `S` | `W` |
| – | Reset (90°) | `R` | – |
| – | Wiedergabe stoppen | `Leertaste` | – |

> **So nehmen Sie etwas auf:** Arm senken, Finger schließen (`Q`), um das Objekt zu greifen,
> dann den Arm heben (`I`) und das Objekt durch Öffnen des Fingers (`E`) wieder loslassen.

---

## Serielles Protokoll

GUI und Firmware kommunizieren über seriell mit einfachen Textbefehlen:

- `P:base,finger,wrist,arm,elbow,dual` — alle Winkel direkt setzen (z. B. `P:90,90,90,90,90,90`)
- Einzelne Zeichen (`a`, `d`, `q`, …) — ein Gelenk schrittweise verstellen
- `R` — alle Winkel auf 90° zurücksetzen
- Die Firmware meldet jeden Zustand als `POS:base,finger,wrist,arm,elbow,dual`
- Die Firmware meldet die Roh-Joystick-Werte als `JS:x,y` für die Live-Anzeige in der GUI

---

## Konfiguration

Die Firmware-Einstellungen stehen oben in `CP_Version.cpp`:

- `ANGLE_STEP` — Grad pro Tastendruck (Standard `3`)
- `PULSE_MIN` / `PULSE_MAX` — bilden 0°–180° auf Servo-Pulsbreiten ab
- `PRESCALE` — PCA9685-Takt für 50-Hz-PWM
- Anfangswinkel — alle Gelenke starten bei 90° für eine aufrechte Haltung

---

## Funktionsweise

1. `setup()` initialisiert den PCA9685 für 50-Hz-PWM und setzt die aufrechte 90°-Haltung.
2. `loop()` liest serielle Befehle/Tasten und die Joystick-Werte des Shields.
3. `parseDirectPosition()` setzt alle Winkel aus einem `P:`-Befehl.
4. `updateServos()` schreibt die Winkel auf jeden Servo.
5. `printStatus()` gibt die aktuellen Winkel über seriell aus, damit die GUI sie spiegeln kann.

Auf der Desktop-Seite sendet `CP_Version.py` `P:`-Befehle für Schieberegler/Tastatur/Reset/Wiedergabe,
interpoliert sanfte Bewegungen und rendert die Live-2D-/3D-Simulation.

---

## Fehlerbehebung

- **Servo bewegt sich nicht** — prüfen Sie zuerst den Codepfad: Der Schieberegler des Gelenks sendet
  einen `P:`-Wert und die Firmware schreibt diesen Kanal. Ist die Logik korrekt, testen Sie die Hardware:
  Stecken Sie den Motor provisorisch an einen bekannten Kanal (z. B. `PIN_FINGER` auf den Handgelenk-Kanal).
  Bewegt er sich dort, liegt es am Originalkanal/Pin/Kabel.
- **Keine Verbindung** — schließen Sie den seriellen Monitor der Arduino-IDE und prüfen Sie den COM-Port.

---

## Lizenz

Bereitgestellt wie besehen für Ausbildungs- und Hobbyzwecke.