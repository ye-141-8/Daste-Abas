import math
import json
import time
import threading
import tkinter as tk
from tkinter import messagebox, filedialog
import serial

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
ARDUINO_PORT = 'COM6'  # Port anpassen
BAUD_RATE = 9600

class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Roboterarm Steuerung & Kinematik Simulator")
        self.root.geometry("1000x680")
        self.root.configure(bg="#1E1E1E")

        # System State
        self.angles = {
            "Base": 90,
            "Finger": 90,
            "Wrist": 90,
            "Arm": 90,
            "Elbow": 90,
            "Dual": 90
        }
        self.saved_sequence = []
        self.is_playing = False
        self.serial_conn = None

        self.connect_arduino()
        self.build_ui()
        
        # Keyboard Event Bindings
        self.root.bind("<Key>", self.on_key_press)

        # Serial Polling Thread
        self.stop_thread = False
        self.read_thread = threading.Thread(target=self.poll_serial, daemon=True)
        self.read_thread.start()

    def connect_arduino(self):
        try:
            self.serial_conn = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            print(f"Verbunden mit Arduino auf {ARDUINO_PORT}")
        except Exception as e:
            print(f"Verbindung zu {ARDUINO_PORT} fehlgeschlagen: {e}")

    def build_ui(self):
        # Header Panel
        header = tk.Frame(self.root, bg="#2D2D2D", height=40)
        header.pack(fill="x", side="top")
        title = tk.Label(header, text="ROBOTERARM STEUERUNGSSYSTEM", fg="#00FF66", bg="#2D2D2D", font=("Consolas", 14, "bold"))
        title.pack(pady=5)

        # Main Layout
        left_frame = tk.Frame(self.root, bg="#1E1E1E", width=420)
        left_frame.pack(side="left", fill="y", padx=10, pady=10)

        right_frame = tk.Frame(self.root, bg="#1E1E1E")
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        # 1. SIMULATION CANVAS
        sim_label = tk.Label(right_frame, text="Echtzeit-Kinematik Simulation (2D)", fg="#FFFFFF", bg="#1E1E1E", font=("Arial", 11, "bold"))
        sim_label.pack(anchor="w")
        
        self.canvas = tk.Canvas(right_frame, bg="#000000", highlightbackground="#333333", height=380)
        self.canvas.pack(fill="x", pady=5)

        # 2. SEQUENCE EDITOR
        seq_label = tk.Label(right_frame, text="Bewegungssequenz / Teach-In Player", fg="#FFFFFF", bg="#1E1E1E", font=("Arial", 11, "bold"))
        seq_label.pack(anchor="w", pady=(10, 0))

        self.listbox = tk.Listbox(right_frame, bg="#252526", fg="#00FF66", selectbackground="#007ACC", font=("Consolas", 10), height=8)
        self.listbox.pack(fill="both", expand=True, pady=5)

        btn_row = tk.Frame(right_frame, bg="#1E1E1E")
        btn_row.pack(fill="x")

        tk.Button(btn_row, text="Position Speichern", command=self.save_current_pos, bg="#0e639c", fg="white").pack(side="left", padx=2)
        tk.Button(btn_row, text="Ausgewählte Löschen", command=self.delete_pos, bg="#a80000", fg="white").pack(side="left", padx=2)
        tk.Button(btn_row, text="Sequenz Abspielen", command=self.start_sequence_thread, bg="#16825d", fg="white").pack(side="left", padx=2)
        tk.Button(btn_row, text="Datei Speichern", command=self.export_file, bg="#333333", fg="white").pack(side="right", padx=2)
        tk.Button(btn_row, text="Datei Laden", command=self.import_file, bg="#333333", fg="white").pack(side="right", padx=2)

        # 3. MANUAL SLIDERS & CONTROLS
        tk.Label(left_frame, text="Gelenke Manuell Steuern", fg="#FFFFFF", bg="#1E1E1E", font=("Arial", 11, "bold")).pack(anchor="w")

        self.sliders = {}
        controls = [
            ("Base (Basis) [A/D]", "Base"),
            ("Schulter (Dual) [W/S]", "Dual"),
            ("Oberarm [I/O]", "Arm"),
            ("Ellbogen [M/N]", "Elbow"),
            ("Handgelenk [J/K]", "Wrist"),
            ("Greifer/Finger [Q/E]", "Finger"),
        ]

        for label_text, key in controls:
            frame = tk.Frame(left_frame, bg="#1E1E1E")
            frame.pack(fill="x", pady=2)
            lbl = tk.Label(frame, text=label_text, fg="#CCCCCC", bg="#1E1E1E", width=22, anchor="w")
            lbl.pack(side="left")
            slider = tk.Scale(frame, from_=0, to=180, orient="horizontal", bg="#252526", fg="#FFFFFF",
                              highlightthickness=0, command=lambda v, k=key: self.on_slider_move(k, v))
            slider.set(90)
            slider.pack(side="right", fill="x", expand=True)
            self.sliders[key] = slider

        self.draw_simulation()

    # -----------------------------------------------------------------------
    # SIMULATION DRAWING (Forward Kinematics Projection)
    # -----------------------------------------------------------------------
    def draw_simulation(self):
        self.canvas.delete("all")
        cx, cy = 250, 320  # Base point on canvas

        # Segment lengths
        l1, l2, l3, l4 = 60, 80, 70, 40

        # Calculate angles relative to origin (converting servo angles to radians)
        a_dual = math.radians(180 - self.angles["Dual"])
        a_arm = a_dual + math.radians(self.angles["Arm"] - 90)
        a_elbow = a_arm + math.radians(self.angles["Elbow"] - 90)
        a_wrist = a_elbow + math.radians(self.angles["Wrist"] - 90)

        # Kinematic points
        x1, y1 = cx, cy - l1
        x2 = x1 + l2 * math.cos(a_arm)
        y2 = y1 - l2 * math.sin(a_arm)

        x3 = x2 + l3 * math.cos(a_elbow)
        y3 = y2 - l3 * math.sin(a_elbow)

        x4 = x3 + l4 * math.cos(a_wrist)
        y4 = y3 - l4 * math.sin(a_wrist)

        # Draw Base Platform
        self.canvas.create_rectangle(cx - 50, cy, cx + 50, cy + 20, fill="#444444", outline="#666666")
        self.canvas.create_line(cx, cy, x1, y1, fill="#00FF66", width=8)  # Base pillar

        # Links
        self.canvas.create_line(x1, y1, x2, y2, fill="#00E5FF", width=6)  # Shoulder-to-Arm
        self.canvas.create_line(x2, y2, x3, y3, fill="#FFD700", width=5)  # Arm-to-Elbow
        self.canvas.create_line(x3, y3, x4, y4, fill="#FF007F", width=4)  # Wrist-to-Hand

        # Joint points
        for px, py in [(cx, cy), (x1, y1), (x2, y2), (x3, y3), (x4, y4)]:
            self.canvas.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#FFFFFF")

        # Base Angle Indicator (Top View Arc)
        base_rad = math.radians(self.angles["Base"])
        bx = 420 + 30 * math.cos(base_rad)
        by = 60 - 30 * math.sin(base_rad)
        self.canvas.create_text(420, 20, text=f"Basis: {self.angles['Base']}°", fill="#FFFFFF")
        self.canvas.create_oval(390, 30, 450, 90, outline="#555555")
        self.canvas.create_line(420, 60, bx, by, fill="#00FF66", width=3)

    # -----------------------------------------------------------------------
    # CONTROL LOGIC
    # -----------------------------------------------------------------------
    def on_slider_move(self, key, value):
        val = int(value)
        if self.angles[key] != val:
            self.angles[key] = val
            self.send_position_to_arduino()
            self.draw_simulation()

    def on_key_press(self, event):
        key = (event.char if event.char else event.keysym).lower()
        mapping = {
            'a': ('Base', -5), 'd': ('Base', 5),
            'q': ('Finger', -5), 'e': ('Finger', 5),
            'j': ('Wrist', 5), 'k': ('Wrist', -5),
            'i': ('Arm', 5), 'o': ('Arm', -5),
            'm': ('Elbow', 5), 'n': ('Elbow', -5),
            'w': ('Dual', 5), 's': ('Dual', -5)
        }
        if key in mapping:
            joint, delta = mapping[key]
            new_val = max(0, min(180, self.angles[joint] + delta))
            self.angles[joint] = new_val
            self.sliders[joint].set(new_val)
            self.send_position_to_arduino()
            self.draw_simulation()

    def send_position_to_arduino(self):
        if self.serial_conn and self.serial_conn.is_open:
            cmd = f"P:{self.angles['Base']},{self.angles['Finger']},{self.angles['Wrist']},{self.angles['Arm']},{self.angles['Elbow']},{self.angles['Dual']}\n"
            try:
                self.serial_conn.write(cmd.encode('utf-8'))
            except Exception as e:
                print(f"Übertragungsfehler: {e}")

    def poll_serial(self):
        while not self.stop_thread:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("POS:"):
                            vals = list(map(int, line.replace("POS:", "").split(",")))
                            if len(vals) == 6:
                                self.angles["Base"] = vals[0]
                                self.angles["Finger"] = vals[1]
                                self.angles["Wrist"] = vals[2]
                                self.angles["Arm"] = vals[3]
                                self.angles["Elbow"] = vals[4]
                                self.angles["Dual"] = vals[5]
                                # Update UI from main thread safely
                                self.root.after(0, self.sync_ui_state)
                except Exception:
                    pass
            time.sleep(0.05)

    def sync_ui_state(self):
        for key, val in self.angles.items():
            self.sliders[key].set(val)
        self.draw_simulation()

    # -----------------------------------------------------------------------
    # TEACH-IN SEQUENCE MANAGEMENT
    # -----------------------------------------------------------------------
    def save_current_pos(self):
        pos = dict(self.angles)
        self.saved_sequence.append(pos)
        idx = len(self.saved_sequence)
        text_entry = f"Schritt {idx:02d} -> Basis:{pos['Base']}° | Schulter:{pos['Dual']}° | Arm:{pos['Arm']}° | Ellbogen:{pos['Elbow']}° | Hand:{pos['Wrist']}° | Greifer:{pos['Finger']}°"
        self.listbox.insert(tk.END, text_entry)

    def delete_pos(self):
        sel = self.listbox.curselection()
        if sel:
            idx = sel[0]
            self.listbox.delete(idx)
            del self.saved_sequence[idx]

    def start_sequence_thread(self):
        if not self.saved_sequence:
            messagebox.showwarning("Warnung", "Keine Positionen in der Sequenz gespeichert!")
            return
        if not self.is_playing:
            threading.Thread(target=self.play_sequence, daemon=True).start()

    def play_sequence(self):
        self.is_playing = True
        for idx, pos in enumerate(self.saved_sequence):
            if not self.is_playing:
                break
            self.angles = dict(pos)
            self.root.after(0, self.sync_ui_state)
            self.send_position_to_arduino()
            time.sleep(1.2)  # Zeit für die Servobewegung
        self.is_playing = False

    def export_file(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])
        if file_path:
            with open(file_path, 'w') as f:
                json.dump(self.saved_sequence, f, indent=4)
            messagebox.showinfo("Erfolg", "Sequenz erfolgreich gespeichert!")

    def import_file(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if file_path:
            with open(file_path, 'r') as f:
                self.saved_sequence = json.load(f)
            self.listbox.delete(0, tk.END)
            for idx, pos in enumerate(self.saved_sequence):
                text_entry = f"Schritt {idx+1:02d} -> Basis:{pos['Base']}° | Schulter:{pos['Dual']}° | Arm:{pos['Arm']}° | Ellbogen:{pos['Elbow']}° | Hand:{pos['Wrist']}° | Greifer:{pos['Finger']}°"
                self.listbox.insert(tk.END, text_entry)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()