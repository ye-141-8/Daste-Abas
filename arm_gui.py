import tkinter as tk
import serial
import time

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
ARDUINO_PORT = 'COM6'  # Update port to match your system
BAUD_RATE = 9600
STEP_ANGLE = 5

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)  # Wait for Arduino reset so it executes the 90-degree stance setup
    print(f"Connected to Arduino on {ARDUINO_PORT}")
except Exception as e:
    print(f"Connection failed on {ARDUINO_PORT}: {e}")
    print("Ensure the Arduino IDE Serial Monitor is CLOSED.")
    exit()

def send_command(char):
    arduino.write(char.encode('utf-8'))
    time.sleep(0.01)
    
    if arduino.in_waiting > 0:
        try:
            response = arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore').strip()
            if response:
                print(f"[Arduino]: {response}")
        except Exception:
            pass

# ---------------------------------------------------------------------------
# CONTROL WINDOW
# ---------------------------------------------------------------------------
class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Keybind Controller")
        self.root.geometry("480x380")
        self.root.configure(bg="#222222")

        info_text = (
            "ROBOT ARM CONTROLLER ACTIVE\n\n"
            f"Step Angle: {STEP_ANGLE}° per press\n\n"
            "• Motor 1 (Base - Ch 6):     [A] / [D]\n"
            "• Motor 2 (Finger - Ch 1):   [Q] / [E]\n"
            "• Motor 3 (Wrist - Ch 2):    [J] / [K]\n"
            "• Motor 4 (Arm - Ch 4):      [I] / [O]\n"
            "• Motor 5 (Elbow - Ch 3):    [M] / [N]\n"
            "• Motor 6 & 7 (Dual Ch 10/11): [W] / [S]\n"
        )
        
        self.label = tk.Label(
            root, text=info_text, fg="#00FF66", bg="#222222",
            font=("Consolas", 10), justify="left", padx=20, pady=20
        )
        self.label.pack(fill="both", expand=True)

        self.root.bind("<Key>", self.on_key_press)

    def on_key_press(self, event):
        key = (event.char if event.char else event.keysym).lower()

        key_mappings = {
            'a': 'a', 'd': 'd',  # Base
            'q': 'q', 'e': 'e',  # Finger
            'j': 'j', 'k': 'k',  # Wrist
            'i': 'i', 'o': 'o',  # Arm
            'm': 'm', 'n': 'n',  # Elbow
            'w': 'w', 's': 's'   # Dual
        }

        if key in key_mappings:
            send_command(key_mappings[key])

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()
    if 'arduino' in globals() and arduino.is_open:
        arduino.close()