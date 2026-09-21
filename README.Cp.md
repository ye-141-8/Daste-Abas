# 🤖 Universal 6-DoF Roboterarm Steuerung System

Ein voll integriertes Steuerungssystem für einen 6-DoF (Degree of Freedom) Roboterarm. Das Projekt kombiniert eine **Arduino C++ Firmware** mit hybrider Eingabeverarbeitung (Funduino Joystick Shield + Serielle Tastaturbefehle) und eine **Python Desktop App** mit deutscher Tkinter-GUI, Vorwärtskinematik-2D-Simulation sowie einem Teach-In Sequenz-Player.

---

## 📸 Features

- **Hybrid-Steuerung:** Parallele Eingabe über physischen Joystick (Funduino Shield), PC-Tastatur-Hotkeys oder GUI-Schieberegler.
- **Echtzeit-Kinematik-Simulation:** Grafische 2D-Vorschau der Armpositionen in der Python-Anwendung.
- **Teach-In & Sequenz-Player:** Speichern von beliebigen Bewegungsabläufen als Schritte und automatisches Abspielen durch den Roboterarm.
- **Import/Export:** Speichern und Laden von Bewegungssequenzen im `JSON`-Format.
- **PWM PCA9685 Treiber:** Präzise Servoansteuerung über I2C bei 50 Hz.

---

## 🛠️ Hardware-Komponenten

- **Mikrocontroller:** Arduino Uno (oder Mega)
- **Servo-Treiber:** PCA9685 16-Kanal 12-Bit PWM Servo Driver Board (I2C Adr. `0x40`)
- **Eingabe:** Funduino / Joypad Shield (Joystick an A0/A1, Buttons an Pin 2, 3, 4, 5)
- **Aktoren:** 7x Servomotoren (Schulter wird synchron über 2 gegeneinander laufende Servos gesteuert)

### 🔌 Pin-Belegung (PCA9685 Servo Shield)

| Servo / Gelenk | PCA9685 Kanal |
| :--- | :--- |
| **Finger / Greifer** | Kanal 1 |
| **Handgelenk** | Kanal 2 |
| **Ellbogen** | Kanal 3 |
| **Oberarm** | Kanal 4 |
| **Basis (Drehung)** | Kanal 6 |
| **Schulter Links** | Kanal 10 |
| **Schulter Rechts (Spiegelung)** | Kanal 11 |

---

## 💻 Software-Voraussetzungen & Installation

### 1. Arduino Setup
1. Öffne die `Arduino IDE`.
2. Stelle sicher, dass die Standard-Bibliothek `<Wire.h>` vorhanden ist.
3. Lade den Arduino C++ Code auf deinen Arduino hoch.
4. Schließe das **Funduino Shield** und den **PCA9685** (SDA -> A4, SCL -> A5) an den Arduino an.

### 2. Python Umgebung einrichten
Stelle sicher, dass Python (3.8 oder neuer) installiert ist.

Installiere die benötigten Python-Pakete über dein Terminal/Eingabeaufforderung:

```bash
pip install pyserial