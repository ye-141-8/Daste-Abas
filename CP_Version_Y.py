import math
import json
import time
import os
import threading
import tkinter as tk
from tkinter import messagebox, filedialog
import serial

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
DEFAULT_BAUD = 9600
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
STEP_STEP = 3  # Grad pro Interpolations-Schritt bei sanfter Bewegung


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except Exception:
            pass
    return {"port": "COM6"}


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=4)
    except Exception:
        pass


class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Roboterarm Steuerung & Kinematik Simulator")
        self.root.geometry("1220x840")
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
        self.motor_meta = {
            "Base":   {"channel": 6,  "dual": False},
            "Finger": {"channel": 1,  "dual": False},
            "Wrist":  {"channel": 2,  "dual": False},
            "Arm":    {"channel": 4,  "dual": False},
            "Elbow":  {"channel": 3,  "dual": False},
            "Dual":   {"channel": "10/11", "dual": True},
        }
        self.saved_sequence = []          # teach-in: list of dict positions
        self.recorded_sequence = []       # time-based rec: list of (time_sec, pos)
        self.is_playing = False
        self.is_recording = False
        self.rec_start_time = 0.0
        self.stop_event = threading.Event()
        self.serial_conn = None
        self.joy_x, self.joy_y = 512, 512

        self.connect_arduino()

        # Buttons (fetched later for enable/disable during playback)
        self.btn_play_all = None
        self.btn_play_one = None
        self.btn_stop = None
        self.btn_rec = None
        self.btn_rec_stop = None
        self.btn_rec_play = None
        self.rec_time_label = None
        self.play_time_label = None

        self.build_ui()

        # Keyboard Event Bindings
        self.root.bind("<Key>", self.on_key_press)

        # Serial Polling Thread
        self.stop_thread = False
        self.read_thread = threading.Thread(target=self.poll_serial, daemon=True)
        self.read_thread.start()

    def connect_arduino(self):
        cfg = load_config()
        port = cfg.get("port", "COM6")
        try:
            self.serial_conn = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)
            time.sleep(2)
            print(f"Verbunden mit Arduino auf {port}")
        except Exception as e:
            print(f"Verbindung zu {port} fehlgeschlagen: {e}")
        if hasattr(self, "status_label"):
            self._update_status()

    def _styled_button(self, parent, text, command, bg, fg="#FFFFFF", state="normal", accent=None):
        return tk.Button(
            parent, text=text, command=command, bg=bg, fg=fg,
            activebackground=accent or bg, activeforeground=fg,
            relief="flat", bd=0, cursor="hand2", state=state,
            padx=12, pady=3, font=("Segoe UI", 9, "bold"),
        )

    def build_ui(self):
        # Header Panel
        header = tk.Frame(self.root, bg="#2D2D2D", height=40)
        header.pack(fill="x", side="top")
        tk.Label(header, text="ROBOTERARM STEUERUNGSSYSTEM", fg="#00FF66",
                 bg="#2D2D2D", font=("Consolas", 14, "bold")).pack(pady=5)

        # Main Layout
        left_frame = tk.Frame(self.root, bg="#1E1E1E", width=420)
        left_frame.pack(side="left", fill="y", padx=10, pady=10)

        right_frame = tk.Frame(self.root, bg="#1E1E1E")
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        # --- Live angle readout ---
        self.live_label = tk.Label(right_frame, text="", fg="#00FF66", bg="#1E1E1E",
                                   font=("Consolas", 10, "bold"), anchor="w")
        self.live_label.pack(anchor="w")

        # --- Simulation canvases (2D + 3D) ---
        sim_row = tk.Frame(right_frame, bg="#1E1E1E")
        sim_row.pack(fill="x", pady=5)

        col2d = tk.Frame(sim_row, bg="#1E1E1E")
        col2d.pack(side="left", fill="both", expand=True, padx=2)
        tk.Label(col2d, text="Kinematik (2D)", fg="#FFFFFF", bg="#1E1E1E",
                 font=("Arial", 10, "bold")).pack(anchor="w")
        self.canvas = tk.Canvas(col2d, bg="#000000", highlightbackground="#333333", width=560, height=360)
        self.canvas.pack(fill="x", pady=2)

        if MATPLOTLIB_AVAILABLE:
            col3d = tk.Frame(sim_row, bg="#1E1E1E")
            col3d.pack(side="right", fill="both", expand=True, padx=2)
            tk.Label(col3d, text="3D Simulation (Basis rotiert)", fg="#FFFFFF", bg="#1E1E1E",
                     font=("Arial", 10, "bold")).pack(anchor="w")
            self.fig = Figure(figsize=(5, 3.6), dpi=96, facecolor="#111111")
            self.fig_axes = self.fig.add_subplot(111, projection="3d")
            self.fig_axes.set_facecolor("#111111")
            self.fig_axes.grid(False)
            self.mpl_canvas = FigureCanvasTkAgg(self.fig, master=col3d)
            self.mpl_canvas.get_tk_widget().pack(fill="both", expand=True)
        else:
            self.fig = None
            tk.Label(col2d, text="3D nicht verfügbar (matplotlib nicht installiert)",
                     fg="#888888", bg="#1E1E1E").pack(fill="x")

        # --- Sequence editor ---
        tk.Label(right_frame, text="Bewegungssequenz / Teach-In Player", fg="#FFFFFF",
                 bg="#1E1E1E", font=("Arial", 11, "bold")).pack(anchor="w", pady=(8, 0))

        self.listbox = tk.Listbox(right_frame, bg="#252526", fg="#00FF66",
                                  selectbackground="#007ACC", font=("Consolas", 10), height=7)
        self.listbox.pack(fill="both", expand=True, pady=4)

        # --- Grouped toolbar: Edit | Play+Record (middle) | File ---
        toolbar = tk.Frame(right_frame, bg="#1E1E1E")
        toolbar.pack(fill="x", pady=(6, 2))

        edit_box = tk.Frame(toolbar, bg="#282828")
        edit_box.pack(side="left", padx=4, ipadx=4, ipady=2)
        tk.Label(edit_box, text="BEARBEITEN", fg="#9FA3A8", bg="#282828", font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=2)
        edit_row = tk.Frame(edit_box, bg="#282828")
        edit_row.pack(padx=2, pady=2)
        self._styled_button(edit_row, "＋ Position Speichern", self.save_current_pos, "#0e639c").pack(side="left", padx=2)
        self._styled_button(edit_row, "✖ Ausgewählte Löschen", self.delete_pos, "#a80000").pack(side="left", padx=2)

        # Middle column: Wiedergabe on top, Aufnahme below it
        mid_box = tk.Frame(toolbar, bg="#1E1E1E")
        mid_box.pack(side="left", expand=True)

        play_box = tk.Frame(mid_box, bg="#282828")
        play_box.pack(side="top", pady=2)
        tk.Label(play_box, text="WIEDERGABE", fg="#9FA3A8", bg="#282828", font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=2)
        play_row = tk.Frame(play_box, bg="#282828")
        play_row.pack(padx=2, pady=2)
        self.btn_play_all = self._styled_button(play_row, "► Alle Abspielen", self.start_sequence_thread, "#16825d")
        self.btn_play_all.pack(side="left", padx=2)
        self.btn_play_one = self._styled_button(play_row, "► Einzelposition", self.start_single_thread, "#16825d")
        self.btn_play_one.pack(side="left", padx=2)
        self.btn_stop = self._styled_button(play_row, "■ Stop", self.stop_playback, "#e53935", state="disabled")
        self.btn_stop.pack(side="left", padx=2)

        rec_box = tk.Frame(mid_box, bg="#282828")
        rec_box.pack(side="top", pady=2)
        tk.Label(rec_box, text="AUFNAHME (ZEIT)", fg="#9FA3A8", bg="#282828", font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=2)
        rec_row = tk.Frame(rec_box, bg="#282828")
        rec_row.pack(padx=2, pady=2)
        self.btn_rec = self._styled_button(rec_row, "● REC", self.start_recording, "#c62828")
        self.btn_rec.pack(side="left", padx=2)
        self.btn_rec_stop = self._styled_button(rec_row, "■ Stop Aufn.", self.stop_recording, "#e53935", state="disabled")
        self.btn_rec_stop.pack(side="left", padx=2)
        self.btn_rec_play = self._styled_button(rec_row, "► Aufn. Abspielen", self.start_recorded_thread, "#16825d")
        self.btn_rec_play.pack(side="left", padx=2)

        self.rec_time_label = tk.Label(right_frame, text="  ● Aufnahme: 00:00",
                                       fg="#FF7043", bg="#1E1E1E", font=("Consolas", 10, "bold"), anchor="w")
        self.play_time_label = tk.Label(right_frame, text="  ► Wiedergabe: 00:00 / 00:00",
                                        fg="#29B6F6", bg="#1E1E1E", font=("Consolas", 10, "bold"), anchor="w")
        self.rec_time_label.pack(anchor="w")
        self.play_time_label.pack(anchor="w")

        file_box = tk.Frame(toolbar, bg="#282828")
        file_box.pack(side="right", padx=4, ipadx=4, ipady=2)
        tk.Label(file_box, text="DATEI", fg="#9FA3A8", bg="#282828", font=("Segoe UI", 8, "bold")).pack(anchor="w", padx=2)
        file_row = tk.Frame(file_box, bg="#282828")
        file_row.pack(padx=2, pady=2)
        self._styled_button(file_row, "Datei Laden", self.import_file, "#333333").pack(side="left", padx=2)
        self._styled_button(file_row, "Datei Speichern", self.export_file, "#333333").pack(side="left", padx=2)

        # --- Loop toggle + status line ---
        loop_row = tk.Frame(right_frame, bg="#1E1E1E")
        loop_row.pack(fill="x", pady=(2, 0), padx=4)
        self.loop_enabled = tk.BooleanVar(value=False)
        tk.Checkbutton(loop_row, text="Endlos wiederholen  |  Leertaste = Stop",
                       variable=self.loop_enabled, bg="#1E1E1E", fg="#9FA3A8",
                       activebackground="#1E1E1E", activeforeground="#00FF66",
                       selectcolor="#1E1E1E", cursor="hand2").pack(side="left")

        # --- Manual sliders ---
        tk.Label(left_frame, text="Gelenke Manuell Steuern", fg="#FFFFFF", bg="#1E1E1E",
                 font=("Arial", 11, "bold")).pack(anchor="w")

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
            tk.Label(frame, text=label_text, fg="#CCCCCC", bg="#1E1E1E", width=22, anchor="w").pack(side="left")
            slider = tk.Scale(frame, from_=0, to=180, orient="horizontal", bg="#252526",
                              fg="#FFFFFF", highlightthickness=0,
                              command=lambda v, k=key: self.on_slider_move(k, v))
            slider.set(90)
            slider.pack(side="right", fill="x", expand=True)
            self.sliders[key] = slider

        # --- Speed control (under the joints) ---
        speed_frame = tk.Frame(left_frame, bg="#1E1E1E")
        speed_frame.pack(fill="x", pady=8)
        tk.Label(speed_frame, text="Geschwindigkeit", fg="#CCCCCC", bg="#1E1E1E",
                 width=15, anchor="w").pack(side="left")
        self.speed_slider = tk.Scale(speed_frame, from_=10, to=500, orient="horizontal",
                                     bg="#252526", fg="#FFFFFF", highlightthickness=0)
        self.speed_slider.set(150)
        self.speed_slider.pack(side="right", fill="x", expand=True)

        # --- Joystick live indicator ---
        joy_frame = tk.Frame(left_frame, bg="#1E1E1E")
        joy_frame.pack(fill="x", pady=8)
        tk.Label(joy_frame, text="Joystick (X/Y)", fg="#FFFFFF", bg="#1E1E1E",
                 font=("Arial", 10, "bold")).pack(anchor="w")
        self.joy_canvas = tk.Canvas(joy_frame, bg="#101010", highlightbackground="#333333",
                                    width=180, height=140)
        self.joy_canvas.pack(pady=2)
        self.joy_canvas.create_oval(20, 20, 160, 120, outline="#2E7D32", width=2)
        self.joy_canvas.create_line(90, 20, 90, 120, fill="#444444", dash=(2, 2))
        self.joy_canvas.create_line(20, 70, 160, 70, fill="#444444", dash=(2, 2))
        self.joy_dot = self.joy_canvas.create_oval(85, 65, 95, 75, fill="#00E5FF", outline="#00E5FF")
        self.joy_label = tk.Label(joy_frame, text="X: 512  Y: 512", fg="#00FF66",
                                  bg="#1E1E1E", font=("Consolas", 9, "bold"))
        self.joy_label.pack()

        # --- Connection & reset controls ---
        conn_frame = tk.Frame(left_frame, bg="#1E1E1E")
        conn_frame.pack(fill="x", pady=8)
        tk.Label(conn_frame, text="COM-Port:", fg="#CCCCCC", bg="#1E1E1E").pack(side="left")
        self.port_var = tk.StringVar(value=load_config().get("port", "COM6"))
        self.port_entry = tk.Entry(conn_frame, textvariable=self.port_var, width=8, bg="#252526")
        self.port_entry.pack(side="left", padx=2)
        self._styled_button(conn_frame, "Verbinden", self.reconnect, "#333333").pack(side="left", padx=2)
        self._styled_button(conn_frame, "⊕ Reset (90°)", self.reset_angles, "#b58900").pack(side="left", padx=4)
        self._styled_button(conn_frame, "⚡ Teste Motoren", self.open_motor_test, "#6a1b9a", fg="#E1B8FF").pack(side="left", padx=4)

        self.draw_simulation()
        if self.fig is not None:
            self.draw_3d()

        # --- Status bar ---
        footer = tk.Frame(self.root, bg="#2D2D2D", height=26)
        footer.pack(fill="x", side="bottom")
        self.status_label = tk.Label(footer, text="", bg="#2D2D2D", fg="#9FA3A8",
                                     anchor="w", font=("Segoe UI", 9))
        self.status_label.pack(fill="x", side="left")
        self._update_status()

    def _update_status(self):
        if self.serial_conn and self.serial_conn.is_open:
            text = f"Arduino verbunden ({self.port_var.get()})"
        else:
            text = "Keine Verbindung zum Arduino"
        self.status_label.config(text="  " + text)

    # -----------------------------------------------------------------------
    # CONNECTION
    # -----------------------------------------------------------------------
    def reconnect(self):
        port = self.port_var.get().strip()
        cfg = load_config()
        cfg["port"] = port
        save_config(cfg)
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        try:
            self.serial_conn = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)
            time.sleep(2)
            print(f"Verbunden mit Arduino auf {port}")
        except Exception as e:
            print(f"Verbindung zu {port} fehlgeschlagen: {e}")

    def reset_angles(self):
        for key in self.angles:
            self.angles[key] = 90
        for key, slider in self.sliders.items():
            slider.set(90)
        self.send_position_to_arduino()
        self.draw_simulation()
        if self.fig is not None:
            self.draw_3d()

    # -----------------------------------------------------------------------
    # SIMULATION DRAWING (Forward Kinematics Projection)
    # -----------------------------------------------------------------------
    def compute_points(self):
        # Returns 2D arm points in the (x, y) plane
        cx, cy = 0, 320  # local coords; caller offsets
        l1, l2, l3, l4 = 60, 80, 70, 40

        a_dual = math.radians(180 - self.angles["Dual"])
        a_arm = a_dual + math.radians(self.angles["Arm"] - 90)
        a_elbow = a_arm + math.radians(self.angles["Elbow"] - 90)
        a_wrist = a_elbow + math.radians(self.angles["Wrist"] - 90)

        x1, y1 = cx, cy - l1
        x2 = x1 + l2 * math.cos(a_arm)
        y2 = y1 - l2 * math.sin(a_arm)
        x3 = x2 + l3 * math.cos(a_elbow)
        y3 = y2 - l3 * math.sin(a_elbow)
        x4 = x3 + l4 * math.cos(a_wrist)
        y4 = y3 - l4 * math.sin(a_wrist)
        return [(cx, cy), (x1, y1), (x2, y2), (x3, y3), (x4, y4)]

    def draw_simulation(self):
        self.canvas.delete("all")
        cx, cy = 240, 300
        # translate kinematics origin (base at (0,320)) to canvas center (cx, cy)
        base_y = self.compute_points()[0][1]
        abs_pts = [(px + cx, py + (cy - base_y)) for px, py in self.compute_points()]

        # Draw Base Platform (no green pillar line)
        self.canvas.create_rectangle(cx - 50, cy, cx + 50, cy + 20, fill="#444444", outline="#666666")

        # Links
        self.canvas.create_line(abs_pts[0][0], abs_pts[0][1], abs_pts[1][0], abs_pts[1][1], fill="#4A4A4A", width=8)
        self.canvas.create_line(abs_pts[1][0], abs_pts[1][1], abs_pts[2][0], abs_pts[2][1], fill="#00E5FF", width=6)
        self.canvas.create_line(abs_pts[2][0], abs_pts[2][1], abs_pts[3][0], abs_pts[3][1], fill="#FFD700", width=5)
        self.canvas.create_line(abs_pts[3][0], abs_pts[3][1], abs_pts[4][0], abs_pts[4][1], fill="#FF007F", width=4)

        # Joint points
        for px, py in abs_pts:
            self.canvas.create_oval(px - 5, py - 5, px + 5, py + 5, fill="#FFFFFF")

        # Base angle text (instead of green top-view arc)
        self.canvas.create_text(20, 20, anchor="nw",
                                text=f"Basis: {self.angles['Base']}°",
                                fill="#FFFFFF", font=("Consolas", 10, "bold"))

        # Gripper indicator at wrist tip
        self.draw_gripper(abs_pts[4], xoff=cx, yoff=cy)

    def update_live(self):
        parts = [f"{k}:{v}°" for k, v in self.angles.items()]
        self.live_label.config(text="  " + "  |  ".join(parts))

    # -----------------------------------------------------------------------
    # CONTROL LOGIC
    # -----------------------------------------------------------------------
    def on_slider_move(self, key, value):
        val = int(value)
        if self.angles[key] != val:
            self.angles[key] = val
            self.send_position_to_arduino()
            self.draw_simulation()
            if self.fig is not None:
                self.draw_3d()
            self.update_live()

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
        if key == 'r':
            self.reset_angles()
            self.update_live()
            return
        if key == ' ' or key == 'space':
            self.stop_playback()
            return
        if key in mapping:
            joint, delta = mapping[key]
            new_val = max(0, min(180, self.angles[joint] + delta))
            self.angles[joint] = new_val
            self.sliders[joint].set(new_val)
            self.send_position_to_arduino()
            self.draw_simulation()
            if self.fig is not None:
                self.draw_3d()
            self.update_live()

    def send_position_to_arduino(self):
        if self.serial_conn and self.serial_conn.is_open:
            cmd = (f"P:{self.angles['Base']},{self.angles['Finger']},{self.angles['Wrist']},"
                   f"{self.angles['Arm']},{self.angles['Elbow']},{self.angles['Dual']}\n")
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
                                self.root.after(0, self.sync_ui_state)
                        elif line.startswith("JS:"):
                            parts = line.replace("JS:", "").split(",")
                            if len(parts) == 2:
                                try:
                                    self.joy_x = int(parts[0])
                                    self.joy_y = int(parts[1])
                                    self.root.after(0, self.update_joystick_display)
                                except ValueError:
                                    pass
                except Exception:
                    pass
            time.sleep(0.05)

    def sync_ui_state(self):
        for key, val in self.angles.items():
            self.sliders[key].set(val)
        self.draw_simulation()
        if self.fig is not None:
            self.draw_3d()
        self.update_live()

    def update_joystick_display(self):
        x = self.joy_x
        y = self.joy_y
        # map 0..1023 to canvas center region (90,70)
        cx = 90 + int((x - 512) / 1024 * 60)
        cy = 70 + int((y - 512) / 1024 * 45)
        cx = max(25, min(155, cx))
        cy = max(25, min(115, cy))
        self.joy_canvas.coords(self.joy_dot, cx - 5, cy - 5, cx + 5, cy + 5)
        self.joy_label.config(text=f"X: {x}  Y: {y}")

    # -----------------------------------------------------------------------
    # 3D DRAWING
    # -----------------------------------------------------------------------
    def draw_3d(self):
        if self.fig is None:
            return
        ax = self.fig_axes
        ax.clear()
        pts = self.compute_points()
        base_y = pts[0][1]  # base height in kinematics space
        base_rad = math.radians(self.angles["Base"])
        # Arm plane points -> 3D (x/y = reach rotated by base, z = height up, corrected orientation)
        pts3 = []
        for (px, py) in pts:
            x = px * math.cos(base_rad)
            y = px * math.sin(base_rad)
            z = base_y - py  # invert so the arm points up (+z)
            pts3.append((x, y, z))
        xs = [p[0] for p in pts3]
        ys = [p[1] for p in pts3]
        zs = [p[2] for p in pts3]
        ax.plot(xs, ys, zs, color="#00E5FF", linewidth=4)
        ax.scatter(xs, ys, zs, color="white", s=30)
        # Base pillar (sits on the ground at z = 0)
        ax.plot([0, 0], [0, 0], [0, 40], color="#555555", linewidth=6)
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        ax.set_zlim(0, 260)
        ax.set_facecolor("#111111")
        ax.view_init(elev=25, azim=-45)
        self.mpl_canvas.draw_idle()

    # -----------------------------------------------------------------------
    # GRIPPER INDICATOR
    # -----------------------------------------------------------------------
    def draw_gripper(self, tip, xoff, yoff):
        px, py = tip
        opening = abs(self.angles["Finger"] - 90) * 0.5  # pixels
        gap = max(2, min(40, opening))
        # two "fingers" diverging from wrist tip
        self.canvas.create_line(px, py, px - gap, py + 12, fill="#FF007F", width=3)
        self.canvas.create_line(px, py, px + gap, py + 12, fill="#FF007F", width=3)
        if self.angles["Finger"] > 105:
            status = "OFFEN"
            color = "#00FF66"
        elif self.angles["Finger"] < 75:
            status = "GESCHLOSSEN"
            color = "#FF4D4D"
        else:
            status = "NEUTRAL"
            color = "#FFD700"
        self.canvas.create_text(px, py - 16, text=f"Greifer: {self.angles['Finger']}° ({status})",
                                fill=color, font=("Consolas", 9, "bold"))

    # -----------------------------------------------------------------------
    # TEACH-IN SEQUENCE MANAGEMENT
    # -----------------------------------------------------------------------
    def save_current_pos(self):
        pos = dict(self.angles)
        self.saved_sequence.append(pos)
        idx = len(self.saved_sequence)
        text_entry = (f"Schritt {idx:02d} -> Basis:{pos['Base']}° | Schulter:{pos['Dual']}° | "
                      f"Arm:{pos['Arm']}° | Ellbogen:{pos['Elbow']}° | Hand:{pos['Wrist']}° | "
                      f"Greifer:{pos['Finger']}°")
        self.listbox.insert(tk.END, text_entry)

    def delete_pos(self):
        sel = self.listbox.curselection()
        if sel:
            idx = sel[0]
            self.listbox.delete(idx)
            del self.saved_sequence[idx]

    # -----------------------------------------------------------------------
    # REAL-TIME RECORDING (ZEIT-BASIERT)
    # -----------------------------------------------------------------------
    def start_recording(self):
        if self.is_playing:
            messagebox.showwarning("Warnung", "Bitte Wiedergabe zuerst stoppen.")
            return
        self.recorded_sequence = []
        self.is_recording = True
        self.rec_start_time = time.time()
        self.btn_rec.config(state="disabled")
        self.btn_rec_stop.config(state="normal")
        threading.Thread(target=self._record_loop, daemon=True).start()
        self.root.after(0, self._rec_timer_tick)

    def _record_loop(self):
        while self.is_recording:
            elapsed = time.time() - self.rec_start_time
            self.recorded_sequence.append((elapsed, dict(self.angles)))
            time.sleep(0.05)

    def _rec_timer_tick(self):
        if not self.is_recording:
            self.root.after(0, lambda: self.rec_time_label.config(
                text=f"  ● Aufnahme: {self._fmt_time(time.time() - self.rec_start_time)}"))
            return
        self.rec_time_label.config(text=f"  ● Aufnahme: {self._fmt_time(time.time() - self.rec_start_time)}")
        self.root.after(100, self._rec_timer_tick)

    def stop_recording(self):
        self.is_recording = False
        self.btn_rec.config(state="normal")
        self.btn_rec_stop.config(state="disabled")
        if self.recorded_sequence:
            total = self.recorded_sequence[-1][0]
            self.rec_time_label.config(text=f"  ● Aufnahme fertig: {self._fmt_time(total)} ({len(self.recorded_sequence)} Punkte)")
            messagebox.showinfo("Aufnahme", f"Aufnahme gespeichert: {self._fmt_time(total)} ({len(self.recorded_sequence)} Punkte)")
        else:
            self.rec_time_label.config(text="  ● Aufnahme leer")

    @staticmethod
    def _fmt_time(sec):
        m = int(sec) // 60
        s = int(sec) % 60
        return f"{m:02d}:{s:02d}"

    def start_recorded_thread(self):
        if not self.recorded_sequence:
            messagebox.showwarning("Warnung", "Keine Aufnahme vorhanden! Erst mit ● REC aufzeichnen.")
            return
        if self.is_playing or self.is_recording:
            return
        self.is_playing = True
        self.stop_event.clear()
        self._set_play_buttons(playing=True)
        threading.Thread(target=self.play_recorded, daemon=True).start()

    def play_recorded(self):
        try:
            seq = list(self.recorded_sequence)
            if not seq:
                return
            start_ts = seq[0][0]
            total = seq[-1][0] - start_ts
            base = dict(self.angles)
            t0 = time.time()
            last = base
            # Interpolate between consecutive recorded samples for smooth motion
            for i in range(len(seq) - 1):
                if self.stop_event.is_set():
                    return
                t_a, p_a = seq[i]
                t_b, p_b = seq[i + 1]
                dur = max(0.0, (t_b - start_ts) - (t_a - start_ts))
                if dur <= 0:
                    last = p_b
                    self._apply_position(p_b)
                    self.root.after(0, lambda d=t_b - start_ts: self.play_time_label.config(
                        text=f"  ► Wiedergabe: {self._fmt_time(d)} / {self._fmt_time(total)}"))
                    continue
                steps = max(1, int(dur / 0.02))
                for s in range(1, steps + 1):
                    if self.stop_event.is_set():
                        return
                    t = s / steps
                    p = {k: round(p_a[k] + (p_b[k] - p_a[k]) * t) for k in p_a}
                    self._apply_position(p)
                    elapsed = time.time() - t0
                    # show time synchronized to recording timeline
                    self.root.after(0, lambda d=t_a-start_ts: self.play_time_label.config(
                        text=f"  ► Wiedergabe: {self._fmt_time(d)} / {self._fmt_time(total)}"))
                    time.sleep(dur / steps)
                last = p_b
            # final
            self.root.after(0, lambda: self.play_time_label.config(
                text=f"  ► Wiedergabe: {self._fmt_time(total)} / {self._fmt_time(total)} — fertig"))
        finally:
            self.root.after(0, self._finish_playback)

    def start_sequence_thread(self):
        if not self.saved_sequence:
            messagebox.showwarning("Warnung", "Keine Positionen in der Sequenz gespeichert!")
            return
        if not self.is_playing:
            self.is_playing = True
            self.stop_event.clear()
            self._set_play_buttons(playing=True)
            threading.Thread(target=self.play_sequence, daemon=True).start()

    def start_single_thread(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning("Warnung", "Bitte zuerst eine Position in der Liste auswählen!")
            return
        if not self.is_playing:
            self.is_playing = True
            self.stop_event.clear()
            self._set_play_buttons(playing=True)
            pos = dict(self.saved_sequence[sel[0]])
            threading.Thread(target=self.play_single, args=(pos,), daemon=True).start()

    def stop_playback(self):
        self.stop_event.set()

    def _set_play_buttons(self, playing):
        state_play = "disabled" if playing else "normal"
        state_stop = "normal" if playing else "disabled"
        self.btn_play_all.config(state=state_play)
        self.btn_play_one.config(state=state_play)
        self.btn_stop.config(state=state_stop)
        if self.btn_rec_play is not None:
            self.btn_rec_play.config(state=state_play)
        # REC must stay available when not playing (unless actively recording)
        rec_state = "disabled" if (playing or self.is_recording) else "normal"
        if self.btn_rec is not None:
            self.btn_rec.config(state=rec_state)

    def _interpolate(self, start, end):
        steps = max(1, max(abs(start[k] - end[k]) for k in start) // STEP_STEP)
        for i in range(1, steps + 1):
            t = i / steps
            p = {k: round(start[k] + (end[k] - start[k]) * t) for k in start}
            yield p

    def _step_delay(self):
        # speed slider: 10 (langsam) .. 500 (schnell) -> ms per step
        speed = self.speed_slider.get()
        return max(0.005, (510 - speed) / 1000.0)

    def _apply_position(self, pos):
        self.angles = dict(pos)
        self.root.after(0, self.sync_ui_state)
        self.send_position_to_arduino()

    def play_single(self, pos):
        try:
            start = dict(self.angles)
            steps_total = max(1, max(abs(start[k] - pos[k]) for k in start) // STEP_STEP)
            t_single = steps_total * self._step_delay()
            t0 = time.time()
            for p in self._interpolate(start, pos):
                if self.stop_event.is_set():
                    break
                self._apply_position(p)
                self.root.after(0, lambda d=t0: self.play_time_label.config(
                    text=f"  ► Wiedergabe: {self._fmt_time(time.time() - d)} / {self._fmt_time(t_single)}"))
                time.sleep(self._step_delay())
            self.root.after(0, lambda: self.play_time_label.config(
                text=f"  ► Wiedergabe: {self._fmt_time(t_single)} / {self._fmt_time(t_single)} — fertig"))
        finally:
            self.root.after(0, self._finish_playback)

    def play_sequence(self):
        try:
            seq = list(self.saved_sequence)
            # estimate total duration
            est = 0.0
            cur = dict(self.angles)
            for pos in seq:
                steps = max(1, max(abs(cur[k] - pos[k]) for k in cur) // STEP_STEP)
                est += steps * self._step_delay()
                cur = dict(pos)
            t0 = time.time()
            while True:
                current = dict(self.angles)
                for pos in seq:
                    if self.stop_event.is_set():
                        return
                    for p in self._interpolate(current, pos):
                        if self.stop_event.is_set():
                            return
                        self._apply_position(p)
                        self.root.after(0, lambda d=t0: self.play_time_label.config(
                            text=f"  ► Wiedergabe: {self._fmt_time(time.time() - d)} / {self._fmt_time(est)}"))
                        time.sleep(self._step_delay())
                    current = dict(pos)
                if not self.loop_enabled.get():
                    break
            self.root.after(0, lambda: self.play_time_label.config(
                text=f"  ► Wiedergabe: {self._fmt_time(est)} / {self._fmt_time(est)} — fertig"))
        finally:
            self.root.after(0, self._finish_playback)

    def _finish_playback(self):
        self.is_playing = False
        self.stop_event.clear()
        self._set_play_buttons(playing=False)

    # -----------------------------------------------------------------------
    # FILE I/O
    # -----------------------------------------------------------------------
    def export_file(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".json",
                                                 filetypes=[("JSON Files", "*.json")])
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
                text_entry = (f"Schritt {idx+1:02d} -> Basis:{pos['Base']}° | "
                              f"Schulter:{pos['Dual']}° | Arm:{pos['Arm']}° | "
                              f"Ellbogen:{pos['Elbow']}° | Hand:{pos['Wrist']}° | "
                              f"Greifer:{pos['Finger']}°")
                self.listbox.insert(tk.END, text_entry)

    # -----------------------------------------------------------------------
    # MOTOR TEST WINDOW (SCI-FI)
    # -----------------------------------------------------------------------
    def open_motor_test(self):
        if self.is_playing or self.is_recording:
            messagebox.showwarning("Warnung", "Bitte Wiedergabe/Aufnahme zuerst stoppen.")
            return
        if not (self.serial_conn and self.serial_conn.is_open):
            messagebox.showwarning("Keine Verbindung",
                                   "Kein Arduino verbunden. Der Test kann nur durchgeführt werden, "
                                   "wenn ein Gerät per COM verbunden ist.")
            return
        win = tk.Toplevel(self.root)
        win.title("⚡ MOTOR-DIAGNOSE")
        win.geometry("460x520")
        win.configure(bg="#0B0F14")
        try:
            win.attributes("-topmost", True)
        except Exception:
            pass
        self._test_win = win
        self._test_abort = False

        tk.Label(win, text="MOTOR-DIAGNOSE-SYSTEM", fg="#00E5FF", bg="#0B0F14",
                 font=("Consolas", 14, "bold")).pack(pady=6)
        tk.Label(win, text="Alle Motoren werden einzeln durchgefahren. Bestätige jeden visuell.",
                 fg="#9FA3A8", bg="#0B0F14", font=("Consolas", 9)).pack(pady=2)

        self._test_status = {}
        frame = tk.Frame(win, bg="#0B0F14")
        frame.pack(fill="both", expand=True, padx=10, pady=6)
        for name, meta in self.motor_meta.items():
            row = tk.Frame(frame, bg="#0B0F14")
            row.pack(fill="x", pady=3)
            label = (f"Dual L/R — Kanal 10/11" if meta["dual"] else f"{name} — Kanal {meta['channel']}")
            tk.Label(row, text=label, fg="#E1B8FF", bg="#0B0F14",
                     font=("Consolas", 10, "bold"), width=24, anchor="w").pack(side="left")
            status_label = tk.Label(row, text="○ OFF", fg="#666666", bg="#0B0F14",
                                    font=("Consolas", 9, "bold"), width=14)
            status_label.pack(side="left")
            btn_ok = tk.Button(row, text="✔ OK", command=lambda n=name, sl=status_label: self._confirm_motor(n, sl),
                               bg="#1B5E20", fg="#FFFFFF", relief="flat", bd=0, cursor="hand2",
                               padx=8, activebackground="#2E7D32", state="disabled")
            btn_ok.pack(side="left", padx=4)
            self._test_status[name] = {"label": status_label, "btn": btn_ok, "done": False}

        btn_row = tk.Frame(win, bg="#0B0F14")
        btn_row.pack(fill="x", pady=4)
        tk.Button(btn_row, text="▶ TEST STARTEN", command=self.run_motor_test,
                  bg="#6a1b9a", fg="#E1B8FF", relief="flat", bd=0, cursor="hand2",
                  padx=12, pady=4, activebackground="#8E24AA").pack(side="left", padx=4)
        tk.Button(btn_row, text="■ Abbrechen", command=lambda: setattr(self, "_test_abort", True),
                  bg="#a80000", fg="#FFFFFF", relief="flat", bd=0, cursor="hand2",
                  padx=12, pady=4, activebackground="#e53935").pack(side="left", padx=4)
        tk.Button(btn_row, text="✖ Schließen", command=win.destroy,
                  bg="#333333", fg="#FFFFFF", relief="flat", bd=0, cursor="hand2",
                  padx=12, pady=4).pack(side="left", padx=4)

        self._test_live = tk.Label(win, text="Bereit.", fg="#00FF66", bg="#0B0F14",
                                   font=("Consolas", 10, "bold"))
        self._test_live.pack(pady=4)

    def _confirm_motor(self, name, status_label):
        status_label.config(text="● OK", fg="#00FF66")
        self._test_status[name]["btn"].config(state="disabled")
        self._test_status[name]["done"] = True
        all_done = all(v["done"] for v in self._test_status.values())
        if all_done:
            self._test_live.config(text="Alle Motoren getestet. ✔", fg="#00FF66")

    def set_test_status(self, name, text, color):
        rec = self._test_status.get(name)
        if rec:
            rec["label"].config(text=text, fg=color)

    def run_motor_test(self):
        if self.is_playing or self.is_recording:
            return
        self._test_abort = False
        for rec in self._test_status.values():
            rec["label"].config(text="○ OFF", fg="#666666")
            rec["btn"].config(state="disabled")
            rec["done"] = False
        threading.Thread(target=self._motor_test_worker, daemon=True).start()

    def _motor_test_worker(self):
        try:
            # Reset all to 90 first
            self._test_live.config(text="Reset auf 90° ...", fg="#FFD700")
            self._send_test_pos(None, 90)
            time.sleep(1.0)

            # Order: single motors then dual last
            order = ["Base", "Finger", "Wrist", "Arm", "Elbow", "Dual"]
            for name in order:
                if self._test_abort:
                    return
                if name == "Dual":
                    # Dual = 2 mirrored motors; moving one value drives both opposite.
                    self.set_test_status(name, "● GEGENTEST", "#29B6F6")
                    self._test_live.config(text="Dual L/R: gegenläufig prüfen (links/rechts gegeneinander)", fg="#29B6F6")
                    self._send_test_pos(name, 30)   # links 30° / rechts 150°
                    time.sleep(1.2)
                    self._send_test_pos(name, 150)  # links 150° / rechts 30°
                    time.sleep(1.2)
                    self._send_test_pos(name, 90)
                else:
                    self.set_test_status(name, "● TESTET", "#FFD700")
                    self._test_live.config(
                        text=f"Teste {name} (Kanal {self.motor_meta[name]['channel']}): ganz auf / ganz zu",
                        fg="#FFD700")
                    self._send_test_pos(name, 20)   # ganz geschlossen / eine Endlage
                    time.sleep(1.2)
                    self._send_test_pos(name, 160)  # ganz offen / andere Endlage
                    time.sleep(1.2)
                    self._send_test_pos(name, 90)
                self.set_test_status(name, "○ BESTÄTIGEN?", "#9FA3A8")
                self._test_status[name]["btn"].config(state="normal")
                self._test_live.config(text=f"{name} durchgefahren. Prüfe visuell und bestätige mit ✔ OK.", fg="#00E5FF")
                # wait for confirmation or abort
                deadline = time.time() + 30
                while not self._test_status[name]["done"] and not self._test_abort:
                    if time.time() > deadline:
                        self.set_test_status(name, "○ NICHT BESTÄTIGT", "#FF5252")
                        break
                    time.sleep(0.1)
            if not self._test_abort:
                self._test_live.config(text="Motor-Diagnose abgeschlossen.", fg="#00FF66")
            else:
                self._test_live.config(text="Abbruch.", fg="#FF5252")
        finally:
            self._send_test_pos(None, 90)

    def _send_test_pos(self, joint, ang):
        if self.serial_conn and self.serial_conn.is_open:
            keep = {"Base": 90, "Finger": 90, "Wrist": 90,
                    "Arm": 90, "Elbow": 90, "Dual": 90}
            if joint is not None:
                keep[joint] = ang
            cmd = (f"P:{keep['Base']},{keep['Finger']},{keep['Wrist']},"
                   f"{keep['Arm']},{keep['Elbow']},{keep['Dual']}\n")
            try:
                self.serial_conn.write(cmd.encode('utf-8'))
            except Exception as e:
                print(f"Test-Übertragungsfehler: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()