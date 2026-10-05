import sys
import math
import time
import serial
import threading
import numpy as np

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSlider, QTextEdit, QDoubleSpinBox, QMessageBox, QComboBox
)

# Matplotlib für die 3D Sci-Fi Visualisierung
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ===========================================================================
# ⚙️ HIER BEARBEITEST DU DIE ABSTÄNDE UND MOTOR-LÄNGEN IM CODE (in cm)
# ===========================================================================
L1_CM = 6.00   # Abstand Motor 1 (Basis) bis Motor 2/6 (Schulter)
L2_CM = 12.00  # Oberarm: Abstand Schulter-Motor bis Ellbogen-Motor
L3_CM = 10.00  # Unterarm: Abstand Ellbogen-Motor bis Handgelenk-Motor
L4_CM = 6.00   # Greifer: Abstand Handgelenk-Motor bis Greiferspitze

GREIFER_OFFEN = 180
GREIFER_ZU = 30
DEFAULT_BAUD = 9600


class SerialWorker(QThread):
    pos_received = Signal(list)
    
    def __init__(self, port="COM6", baud=DEFAULT_BAUD):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.serial_conn = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
        except Exception:
            return

        while self.running:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("POS:"):
                            vals = list(map(int, line.replace("POS:", "").split(",")))
                            if len(vals) == 6:
                                self.pos_received.emit(vals)
                except Exception:
                    pass
            time.sleep(0.02)

    def send_cmd(self, cmd_str):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(cmd_str.encode('utf-8'))
            except Exception as e:
                print(f"Schreibfehler: {e}")

    def stop(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.wait()


class SciFiRobotGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚡ CYBER-ARM v3.0 // ADVANCED NEURAL INTERFACE")
        self.resize(1400, 900)

        self.angles = {
            "Base": 90, "Finger": GREIFER_OFFEN, "Wrist": 90,
            "Arm": 90, "Elbow": 90, "Dual": 90
        }

        self.serial_worker = None
        self.loop_active = False
        self.current_xyz = (0.0, 0.0, 0.0)

        self.init_ui()
        self.apply_scifi_stylesheet()
        self.recalculate_fk_and_update()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # -------------------------------------------------------------------
        # LINKE SEITE: STEUERUNG, KOORDINATEN & LOOP
        # -------------------------------------------------------------------
        left_layout = QVBoxLayout()

        # 1. Verbindung
        conn_box = QGroupBox("📡 SYSTEM-LINK (HARDWARE INTERFACE)")
        conn_layout = QHBoxLayout(conn_box)
        self.port_input = QLineEdit("COM6")
        self.btn_connect = QPushButton("⚡ VERBINDEN")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(QLabel("PORT:"))
        conn_layout.addWidget(self.port_input)
        conn_layout.addWidget(self.btn_connect)
        left_layout.addWidget(conn_box)

        # 2. Live Tracker (Vorwärtskinematik - FK)
        tracker_box = QGroupBox("📍 ECHTZEIT-POSITIONS-TRACKER (FK)")
        tracker_layout = QGridLayout(tracker_box)

        self.lbl_curr_pos = QLabel("X: 0.00 cm | Y: 0.00 cm | Z: 0.00 cm")
        self.lbl_curr_pos.setStyleSheet("color: #00ffcc; font-size: 13px; font-weight: bold;")
        tracker_layout.addWidget(self.lbl_curr_pos, 0, 0, 1, 2)

        self.btn_copy_pos = QPushButton("📋 POS IN PICK/DROP EINFÜGEN")
        self.btn_copy_pos.clicked.connect(self.copy_current_pos)
        tracker_layout.addWidget(self.btn_copy_pos, 1, 0, 1, 2)

        left_layout.addWidget(tracker_box)

        # 3. Geschwindigkeitsregler
        speed_box = QGroupBox("⚡ GESCHWINDIGKEIT / SPEED")
        speed_layout = QVBoxLayout(speed_box)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.lbl_speed_status = QLabel("Geschwindigkeit: Normal (50 ms/Schritt)")
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.lbl_speed_status)
        speed_layout.addWidget(self.speed_slider)
        left_layout.addWidget(speed_box)

        # 4. Greifer Direct Controls
        grip_box = QGroupBox("🦾 GREIFER STEUERUNG")
        grip_layout = QHBoxLayout(grip_box)
        self.btn_open = QPushButton("🔓 ÖFFNEN (180°)")
        self.btn_open.clicked.connect(self.open_gripper)
        self.btn_close = QPushButton("🔒 SCHLIESSEN (30°)")
        self.btn_close.clicked.connect(self.close_gripper)
        grip_layout.addWidget(self.btn_open)
        grip_layout.addWidget(self.btn_close)
        left_layout.addWidget(grip_box)

        # 5. Punkt Anfahren
        cmd_box = QGroupBox("🎯 KOORDINATEN ANFAHREN & HANDGELENK")
        cmd_layout = QGridLayout(cmd_box)

        cmd_layout.addWidget(QLabel("Punkt (X, Y, Z in cm):"), 0, 0)
        self.txt_point = QLineEdit("8, 15, 0")
        cmd_layout.addWidget(self.txt_point, 0, 1)

        cmd_layout.addWidget(QLabel("Wrist-Ausrichtung:"), 1, 0)
        self.combo_wrist = QComboBox()
        self.combo_wrist.addItems(["Horizontal (90°)", "Vertikal (0°)", "Diagonal Links (45°)", "Diagonal Rechts (135°)"])
        cmd_layout.addWidget(self.combo_wrist, 1, 1)

        self.btn_goto = QPushButton("▶ GEHE ZU KOORDINATEN")
        self.btn_goto.clicked.connect(self.on_goto_clicked)
        cmd_layout.addWidget(self.btn_goto, 2, 0, 1, 2)

        left_layout.addWidget(cmd_box)

        # 6. Endlos Pick-and-Place Loop
        pnp_box = QGroupBox("🔄 REPETITIVER PICK & DROP LOOP")
        pnp_layout = QGridLayout(pnp_box)

        pnp_layout.addWidget(QLabel("Pick-Punkt (X,Y,Z):"), 0, 0)
        self.txt_pick = QLineEdit("3, 2, 18")
        pnp_layout.addWidget(self.txt_pick, 0, 1)

        pnp_layout.addWidget(QLabel("Drop-Punkt (X,Y,Z):"), 1, 0)
        self.txt_drop = QLineEdit("-10, 12, 2")
        pnp_layout.addWidget(self.txt_drop, 1, 1)

        self.btn_start_loop = QPushButton("🔄 LOOP STARTEN")
        self.btn_start_loop.clicked.connect(self.start_loop)
        pnp_layout.addWidget(self.btn_start_loop, 2, 0)

        self.btn_stop_loop = QPushButton("⛔ LOOP STOPPEN")
        self.btn_stop_loop.clicked.connect(self.stop_loop)
        pnp_layout.addWidget(self.btn_stop_loop, 2, 1)

        left_layout.addWidget(pnp_box)

        # Konsolen Log
        log_box = QGroupBox("🖥️ SYSTEM LOG")
        log_layout = QVBoxLayout(log_box)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(log_box)

        main_layout.addLayout(left_layout, stretch=1)

        # -------------------------------------------------------------------
        # RECHTE SEITE: 3D INTERAKTIONS-CANVAS & SLIDER
        # -------------------------------------------------------------------
        right_layout = QVBoxLayout()

        # Matplotlib 3D Canvas
        vis_box = QGroupBox("🌐 3D NEURAL TRACKING & KAMERA STEUERUNG")
        vis_layout = QVBoxLayout(vis_box)
        
        self.fig = Figure(figsize=(5, 4), facecolor='#06060c')
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        vis_layout.addWidget(self.canvas)

        cam_btn_layout = QHBoxLayout()
        btn_cam_reset = QPushButton("📷 Ansicht Reset")
        btn_cam_reset.clicked.connect(self.reset_camera)
        btn_cam_top = QPushButton("📐 Draufsicht (Top)")
        btn_cam_top.clicked.connect(self.top_camera)
        cam_btn_layout.addWidget(btn_cam_reset)
        cam_btn_layout.addWidget(btn_cam_top)
        vis_layout.addLayout(cam_btn_layout)

        right_layout.addWidget(vis_box, stretch=2)

        # Servowinkel Sliders
        sliders_box = QGroupBox("📊 MANUELLE SERVO STEUERUNG")
        sliders_layout = QGridLayout(sliders_box)

        self.sliders = {}
        joints = ["Base", "Dual", "Arm", "Elbow", "Wrist", "Finger"]
        for idx, joint in enumerate(joints):
            lbl = QLabel(f"{joint}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(self.angles[joint])
            slider.valueChanged.connect(lambda val, j=joint: self.on_slider_moved(j, val))
            val_lbl = QLabel(f"{self.angles[joint]}°")
            
            sliders_layout.addWidget(lbl, idx, 0)
            sliders_layout.addWidget(slider, idx, 1)
            sliders_layout.addWidget(val_lbl, idx, 2)
            self.sliders[joint] = (slider, val_lbl)

        btn_reset_90 = QPushButton("↺ ALLE SERVOS AUF 90° RESETTEN")
        btn_reset_90.clicked.connect(self.reset_all_servos)
        sliders_layout.addWidget(btn_reset_90, len(joints), 0, 1, 3)

        right_layout.addWidget(sliders_box, stretch=1)

        main_layout.addLayout(right_layout, stretch=2)

    def log(self, text):
        self.log_text.append(f"> {text}")

    # -----------------------------------------------------------------------
    # VORWÄRTSKINEMATIK (FK) & REALTIME TRACKING
    # -----------------------------------------------------------------------
    def recalculate_fk_and_update(self):
        """Berechnet aus den aktuellen Servowinkeln die X, Y, Z Position."""
        b_rad = math.radians(self.angles["Base"] - 90.0)
        s_rad = math.radians(self.angles["Arm"])
        e_rad = math.radians(180.0 - self.angles["Elbow"])

        p0 = [0, 0, 0]
        p1 = [0, 0, L1_CM]

        r_arm = L2_CM * math.cos(s_rad) + L3_CM * math.cos(s_rad - e_rad) + L4_CM
        z_arm = L1_CM + L2_CM * math.sin(s_rad) + L3_CM * math.sin(s_rad - e_rad)

        x = r_arm * math.cos(b_rad)
        y = r_arm * math.sin(b_rad)
        z = max(0.0, z_arm)

        self.current_xyz = (round(x, 2), round(y, 2), round(z, 2))
        self.lbl_curr_pos.setText(f"X: {self.current_xyz[0]} cm | Y: {self.current_xyz[1]} cm | Z: {self.current_xyz[2]} cm")

        # Visualisierung
        p2 = [
            p1[0] + L2_CM * math.cos(s_rad) * math.cos(b_rad),
            p1[1] + L2_CM * math.cos(s_rad) * math.sin(b_rad),
            p1[2] + L2_CM * math.sin(s_rad)
        ]
        p3 = [x, y, z]
        self.update_3d_plot(p0, p1, p2, p3)

    def copy_current_pos(self):
        coord_str = f"{self.current_xyz[0]}, {self.current_xyz[1]}, {self.current_xyz[2]}"
        self.txt_pick.setText(coord_str)
        self.log(f"Position {coord_str} in Pick-Feld übernommen.")

    # -----------------------------------------------------------------------
    # KAMERA & GESCHWINDIGKEIT
    # -----------------------------------------------------------------------
    def reset_camera(self):
        self.ax.view_init(elev=25, azim=-60)
        self.canvas.draw()

    def top_camera(self):
        self.ax.view_init(elev=90, azim=-90)
        self.canvas.draw()

    def get_step_delay(self):
        val = self.speed_slider.value()
        return (11 - val) * 0.01

    def on_speed_changed(self, val):
        delay_ms = int(self.get_step_delay() * 1000)
        self.lbl_speed_status.setText(f"Geschwindigkeit: Level {val} ({delay_ms} ms/Schritt)")

    def get_wrist_angle(self):
        txt = self.combo_wrist.currentText()
        if "Vertikal" in txt: return 0
        elif "Diagonal Links" in txt: return 45
        elif "Diagonal Rechts" in txt: return 135
        return 90

    # -----------------------------------------------------------------------
    # INVERSE KINEMATIK (IK)
    # -----------------------------------------------------------------------
    def parse_xyz(self, text):
        try:
            parts = [float(p.strip()) for p in text.split(",")]
            if len(parts) == 3:
                return parts[0], parts[1], parts[2]
        except Exception:
            pass
        return None

    def calculate_ik(self, x_cm, y_cm, z_cm):
        l1, l2, l3, l4 = L1_CM, L2_CM, L3_CM, L4_CM

        base_angle = math.degrees(math.atan2(y_cm, x_cm)) + 90.0
        if not (0 <= base_angle <= 180):
            return False, "Punkt liegt außerhalb des 180° Basis-Drehbereichs!", None

        r = math.sqrt(x_cm**2 + y_cm**2)
        r_w = r
        z_w = z_cm + l4 - l1

        d = math.sqrt(r_w**2 + z_w**2)

        if d > (l2 + l3) or d < abs(l2 - l3):
            return False, f"Punkt ({x_cm},{y_cm},{z_cm}) außer Reichweite!", None

        cos_elbow = (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        angle_elbow = math.degrees(math.acos(cos_elbow))

        beta = math.atan2(z_w, r_w)
        cos_alpha = (l2**2 + d**2 - l3**2) / (2 * l2 * d)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)

        angle_shoulder = math.degrees(beta + alpha)

        dual_angle = 180.0 - angle_shoulder
        arm_angle = angle_shoulder
        elbow_angle = 180.0 - angle_elbow

        rad_b = math.radians(base_angle - 90.0)
        p0 = [0, 0, 0]
        p1 = [0, 0, l1]
        p2 = [
            p1[0] + l2 * math.cos(beta + alpha) * math.cos(rad_b),
            p1[1] + l2 * math.cos(beta + alpha) * math.sin(rad_b),
            p1[2] + l2 * math.sin(beta + alpha)
        ]
        p3 = [x_cm, y_cm, z_cm]

        target_angles = {
            "Base": int(round(base_angle)),
            "Dual": int(round(max(0, min(180, dual_angle)))),
            "Arm": int(round(max(0, min(180, arm_angle)))),
            "Elbow": int(round(max(0, min(180, elbow_angle)))),
            "Wrist": self.get_wrist_angle(),
            "Finger": self.angles["Finger"]
        }
        return True, target_angles, (p0, p1, p2, p3)

    # -----------------------------------------------------------------------
    # 3D RENDERER
    # -----------------------------------------------------------------------
    def update_3d_plot(self, p0, p1, p2, p3):
        self.ax.clear()
        self.ax.set_facecolor('#06060c')
        
        self.ax.xaxis.pane.set_edgecolor('#00ffcc')
        self.ax.yaxis.pane.set_edgecolor('#00ffcc')
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.tick_params(colors='#00ffcc', labelsize=8)

        xs = [p0[0], p1[0], p2[0], p3[0]]
        ys = [p0[1], p1[1], p2[1], p3[1]]
        zs = [p0[2], p1[2], p2[2], p3[2]]

        self.ax.plot(xs, ys, zs, '-o', color='#00ffcc', linewidth=4, markersize=8, markerfacecolor='#ff0055')
        self.ax.scatter([p3[0]], [p3[1]], [p3[2]], color='#ff0055', s=90, label='Greifer')

        self.ax.set_xlim([-25, 25])
        self.ax.set_ylim([-25, 25])
        self.ax.set_zlim([0, 30])
        self.ax.set_xlabel('X (cm)', color='#00ffcc')
        self.ax.set_ylabel('Y (cm)', color='#00ffcc')
        self.ax.set_zlabel('Z (cm)', color='#00ffcc')

        self.canvas.draw()

    # -----------------------------------------------------------------------
    # BEWEGUNGEN & LOOPS
    # -----------------------------------------------------------------------
    def move_smoothly(self, target_angles, pts_3d=None):
        steps = 25
        delay = self.get_step_delay()
        start_angles = dict(self.angles)

        for i in range(1, steps + 1):
            t = i / steps
            interp = {}
            for k in start_angles:
                interp[k] = int(round(start_angles[k] + (target_angles[k] - start_angles[k]) * t))
            
            self.send_angles(interp)
            time.sleep(delay)

        if pts_3d:
            self.update_3d_plot(*pts_3d)

    def send_angles(self, angles_dict):
        self.angles = dict(angles_dict)
        for k, v in self.angles.items():
            if k in self.sliders:
                self.sliders[k][0].blockSignals(True)
                self.sliders[k][0].setValue(v)
                self.sliders[k][1].setText(f"{v}°")
                self.sliders[k][0].blockSignals(False)

        cmd = f"P:{self.angles['Base']},{self.angles['Finger']},{self.angles['Wrist']},{self.angles['Arm']},{self.angles['Elbow']},{self.angles['Dual']}\n"
        if self.serial_worker:
            self.serial_worker.send_cmd(cmd)

    def on_goto_clicked(self):
        pt = self.parse_xyz(self.txt_point.text())
        if not pt:
            QMessageBox.warning(self, "Fehler", "Ungültiges Format! Bitte z.B. '8, 15, 0' nutzen.")
            return

        ok, target, pts = self.calculate_ik(*pt)
        if ok:
            self.log(f"Fahre zu Koordinaten {pt} ...")
            threading.Thread(target=self.move_smoothly, args=(target, pts), daemon=True).start()
        else:
            self.log(f"❌ {target}")
            QMessageBox.critical(self, "IK Fehler", target)

    def start_loop(self):
        if self.loop_active:
            return
        
        pick_pt = self.parse_xyz(self.txt_pick.text())
        drop_pt = self.parse_xyz(self.txt_drop.text())

        if not pick_pt or not drop_pt:
            QMessageBox.warning(self, "Fehler", "Pick- oder Drop-Format ungültig!")
            return

        ok1, ik_pick, pts_pick = self.calculate_ik(*pick_pt)
        ok2, ik_drop, pts_drop = self.calculate_ik(*drop_pt)

        if not ok1 or not ok2:
            QMessageBox.critical(self, "IK Fehler", "Eine der Positionen ist nicht erreichbar! Loop abgebrochen.")
            return

        def loop_thread():
            self.loop_active = True
            self.log("🔁 LOOPOPERATION GESTARTET...")
            cycle_count = 0

            while self.loop_active:
                cycle_count += 1
                self.log(f"--- DURCHLAUF #{cycle_count} ---")

                # 1. Greifer Öffnen
                pos = dict(self.angles)
                pos["Finger"] = GREIFER_OFFEN
                self.move_smoothly(pos)

                # 2. Pick Hover (+4 cm)
                ok, hover_pick, pts_h1 = self.calculate_ik(pick_pt[0], pick_pt[1], pick_pt[2] + 4.0)
                if not ok or not self.loop_active: break
                hover_pick["Finger"] = GREIFER_OFFEN
                self.move_smoothly(hover_pick, pts_h1)

                # 3. Absenken & Greifen
                ik_pick["Finger"] = GREIFER_OFFEN
                self.move_smoothly(ik_pick, pts_pick)
                time.sleep(0.3)
                ik_pick["Finger"] = GREIFER_ZU
                self.send_angles(ik_pick)
                time.sleep(0.5)

                # 4. Anheben & zu Drop Hover fahren
                self.move_smoothly(hover_pick, pts_h1)

                ok, hover_drop, pts_h2 = self.calculate_ik(drop_pt[0], drop_pt[1], drop_pt[2] + 4.0)
                if not ok or not self.loop_active: break
                hover_drop["Finger"] = GREIFER_ZU
                self.move_smoothly(hover_drop, pts_h2)

                # 5. Absenken & Loslassen
                ik_drop["Finger"] = GREIFER_ZU
                self.move_smoothly(ik_drop, pts_drop)
                time.sleep(0.3)

                ik_drop["Finger"] = GREIFER_OFFEN
                self.send_angles(ik_drop)
                time.sleep(0.5)

                # 6. Zurück zur Sicherheitshöhe
                self.move_smoothly(hover_drop, pts_h2)

            self.log("🛑 LOOP BEENDET OR MANUELL GESTOPPT.")
            self.loop_active = False

        threading.Thread(target=loop_thread, daemon=True).start()

    def stop_loop(self):
        self.loop_active = False
        self.log("Stopp-Signal gesendet...")

    def open_gripper(self):
        pos = dict(self.angles)
        pos["Finger"] = GREIFER_OFFEN
        self.send_angles(pos)
        self.log("Greifer Manuell Geöffnet.")

    def close_gripper(self):
        pos = dict(self.angles)
        pos["Finger"] = GREIFER_ZU
        self.send_angles(pos)
        self.log("Greifer Manuell Geschlossen.")

    def reset_all_servos(self):
        for k in self.angles:
            self.angles[k] = 90
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self.log("Alle Servomotoren auf 90° Reset gesetzt.")

    def on_slider_moved(self, joint, val):
        self.angles[joint] = val
        self.sliders[joint][1].setText(f"{val}°")
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()

    def toggle_connection(self):
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.stop()
            self.serial_worker = None
            self.btn_connect.setText("⚡ VERBINDEN")
            self.log("Verbindung getrennt.")
        else:
            port = self.port_input.text().strip()
            self.serial_worker = SerialWorker(port=port)
            self.serial_worker.start()
            self.btn_connect.setText("⛔ TRENNEN")
            self.log(f"Verbindung zu {port} hergestellt.")

    # -----------------------------------------------------------------------
    # CYBERPUNK SCI-FI STYLESHEET
    # -----------------------------------------------------------------------
    def apply_scifi_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #06060c; }
            QGroupBox { 
                color: #00ffcc; 
                font-size: 11px;
                font-weight: bold; 
                border: 1px solid #00ffcc; 
                border-radius: 6px;
                margin-top: 8px; 
                padding-top: 12px; 
                background-color: #0a0d18;
            }
            QLabel { color: #a0f0ff; font-weight: bold; font-family: Consolas, monospace; }
            QLineEdit, QComboBox, QTextEdit { 
                background-color: #03050a; 
                color: #00ffcc; 
                border: 1px solid #00a8ff; 
                border-radius: 4px;
                padding: 4px; 
                font-family: Consolas, monospace;
            }
            QPushButton { 
                background-color: #0f172a; 
                color: #00ffcc; 
                font-weight: bold; 
                border: 1px solid #00ffcc; 
                padding: 6px; 
                border-radius: 4px; 
                font-family: Consolas, monospace;
            }
            QPushButton:hover { 
                background-color: #00ffcc; 
                color: #06060c; 
                border: 1px solid #ffffff;
            }
            QSlider::groove:horizontal { height: 6px; background: #1a2238; border-radius: 3px; }
            QSlider::handle:horizontal { background: #ff0055; width: 16px; margin: -5px 0; border-radius: 8px; }
        """)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SciFiRobotGUI()
    window.show()
    sys.exit(app.exec())