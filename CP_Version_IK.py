import sys
import math
import time
import serial
import threading

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSlider, QTextEdit, QMessageBox
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# ===========================================================================
# ⚙️ ROBOT ARM GEOMETRY (in cm)
# ===========================================================================
L_BASE    = 0.00   # Basishöhe
L1_DUAL   = 22.00  # Oberarm (Dual bis Arm)
L2_ARM    = 10.00  # Unterarm (Arm bis Elbow)
L3_ELBOW  = 7.00   # Handgelenk-Segment (Elbow bis Wrist)
L4_FINGER = 15.00  # Greifer (Wrist bis Fingerspitze)

L_EFF = L2_ARM + L3_ELBOW + L4_FINGER  # 32.0 cm
R_MAX = L1_DUAL + L_EFF                 # 54.0 cm (Max Streckung)
R_MIN = abs(L1_DUAL - L_EFF)            # 10.0 cm (Min Nahgrenze)

MAX_GRIPPER_WIDTH_CM = 8.0  # Volle Öffnung bei 180°
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
        self.setWindowTitle("⚡ CYBER-ARM v3.5 // FULL CONTROL, TRACKER & QUICK PRESETS")
        self.resize(1550, 980)

        self.angles = {
            "Base": 90, "Finger": 180, "Wrist": 90,
            "Arm": 90, "Elbow": 90, "Dual": 90
        }

        self.serial_worker = None
        self.loop_active = False
        self.current_xyz = (0.0, 0.0, 0.0)
        self.current_r = 0.0

        self.init_ui()
        self.apply_scifi_stylesheet()
        self.recalculate_fk_and_update()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # -------------------------------------------------------------------
        # LINKE SEITE: SYSTEM, TRACKER, DECOUPLED CONTROL & THROW
        # -------------------------------------------------------------------
        left_layout = QVBoxLayout()

        # 1. HARDWARE VERBINDUNG
        conn_box = QGroupBox("📡 SYSTEM-LINK (HARDWARE INTERFACE)")
        conn_layout = QHBoxLayout(conn_box)
        self.port_input = QLineEdit("COM6")
        self.btn_connect = QPushButton("⚡ VERBINDEN")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(QLabel("PORT:"))
        conn_layout.addWidget(self.port_input)
        conn_layout.addWidget(self.btn_connect)
        left_layout.addWidget(conn_box)

        # 2. ECHTZEIT TRACKER (LIVE POS & WINKELEINGABE / KOPIEREN)
        tracker_box = QGroupBox("📍 LIVE TRACKER & KOORDINATEN MONITOING")
        tracker_layout = QGridLayout(tracker_box)

        self.lbl_curr_xyz = QLabel("X: 0.0 cm | Y: 0.0 cm | Z: 0.0 cm | R: 0.0 cm")
        self.lbl_curr_xyz.setStyleSheet("color: #00ffcc; font-size: 12px; font-weight: bold;")
        tracker_layout.addWidget(self.lbl_curr_xyz, 0, 0, 1, 2)

        self.lbl_curr_angles = QLabel("Winkel: Base: 90° | Dual: 90° | Arm: 90° | Elbow: 90° | Wrist: 90° | Finger: 180°")
        self.lbl_curr_angles.setStyleSheet("color: #00a8ff; font-size: 11px;")
        tracker_layout.addWidget(self.lbl_curr_angles, 1, 0, 1, 2)

        self.btn_copy_to_pick = QPushButton("📋 POS ALS PICK SPEICHERN")
        self.btn_copy_to_pick.clicked.connect(self.copy_to_pick)
        tracker_layout.addWidget(self.btn_copy_to_pick, 2, 0)

        self.btn_copy_to_drop = QPushButton("📋 POS ALS DROP SPEICHERN")
        self.btn_copy_to_drop.clicked.connect(self.copy_to_drop)
        tracker_layout.addWidget(self.btn_copy_to_drop, 2, 1)

        left_layout.addWidget(tracker_box)

        # 3. ENTKOPPELTE ARM- & ENDEFFECTOR-STEUERUNG
        ctrl_box = QGroupBox("🎯 STEUERUNG (BASIS, REICHWEITE, WRIST & FINGER)")
        ctrl_layout = QGridLayout(ctrl_box)

        ctrl_layout.addWidget(QLabel("Basis Grad (0-180°):"), 0, 0)
        self.txt_base_angle = QLineEdit("90")
        ctrl_layout.addWidget(self.txt_base_angle, 0, 1)

        ctrl_layout.addWidget(QLabel("Reichweite R (cm) & Höhe Z (cm):"), 1, 0)
        rz_layout = QHBoxLayout()
        self.txt_reach_r = QLineEdit("25.0")
        self.txt_height_z = QLineEdit("10.0")
        rz_layout.addWidget(self.txt_reach_r)
        rz_layout.addWidget(QLabel("Z:"))
        rz_layout.addWidget(self.txt_height_z)
        ctrl_layout.addLayout(rz_layout, 1, 1)

        # WRIST EINGABE & BUTTONS
        ctrl_layout.addWidget(QLabel("Wrist (Grad 0-180°):"), 2, 0)
        wrist_v_layout = QVBoxLayout()
        self.txt_wrist_deg = QLineEdit("90")
        wrist_v_layout.addWidget(self.txt_wrist_deg)
        
        wrist_btn_layout = QHBoxLayout()
        btn_w_0 = QPushButton("0° Vert")
        btn_w_0.clicked.connect(lambda: self.txt_wrist_deg.setText("0"))
        btn_w_45 = QPushButton("45° DiagL")
        btn_w_45.clicked.connect(lambda: self.txt_wrist_deg.setText("45"))
        btn_w_90 = QPushButton("90° Horiz")
        btn_w_90.clicked.connect(lambda: self.txt_wrist_deg.setText("90"))
        btn_w_135 = QPushButton("135° DiagR")
        btn_w_135.clicked.connect(lambda: self.txt_wrist_deg.setText("135"))
        
        wrist_btn_layout.addWidget(btn_w_0)
        wrist_btn_layout.addWidget(btn_w_45)
        wrist_btn_layout.addWidget(btn_w_90)
        wrist_btn_layout.addWidget(btn_w_135)
        wrist_v_layout.addLayout(wrist_btn_layout)
        ctrl_layout.addLayout(wrist_v_layout, 2, 1)

        # FINGER EINGABE & BUTTONS
        ctrl_layout.addWidget(QLabel("Finger / Greifer (Grad 0-180°):"), 3, 0)
        finger_v_layout = QVBoxLayout()
        self.txt_finger_deg = QLineEdit("180")
        finger_v_layout.addWidget(self.txt_finger_deg)

        finger_btn_layout = QHBoxLayout()
        btn_f_close = QPushButton("✊ Zu (0°)")
        btn_f_close.clicked.connect(lambda: self.txt_finger_deg.setText("0"))
        btn_f_half = QPushButton("🤏 Halb (90°)")
        btn_f_half.clicked.connect(lambda: self.txt_finger_deg.setText("90"))
        btn_f_open = QPushButton("🖐️ Offen (180°)")
        btn_f_open.clicked.connect(lambda: self.txt_finger_deg.setText("180"))
        
        finger_btn_layout.addWidget(btn_f_close)
        finger_btn_layout.addWidget(btn_f_half)
        finger_btn_layout.addWidget(btn_f_open)
        finger_v_layout.addLayout(finger_btn_layout)
        ctrl_layout.addLayout(finger_v_layout, 3, 1)

        self.btn_move_decoupled = QPushButton("▶ PARAMETER ANFAHREN")
        self.btn_move_decoupled.clicked.connect(self.on_move_decoupled_clicked)
        ctrl_layout.addWidget(self.btn_move_decoupled, 4, 0, 1, 2)

        left_layout.addWidget(ctrl_box)

        # 4. PICK, ROTATE & DROP / THROW SEQUENZ
        throw_box = QGroupBox("📦 AUTONOM PICK, ROTATE & ABWERFEN / DROP")
        throw_layout = QGridLayout(throw_box)

        throw_layout.addWidget(QLabel("Pick: Base°, R cm, Z cm, Wrist°:"), 0, 0)
        self.txt_pick_params = QLineEdit("45, 25.0, 5.0, 90")
        throw_layout.addWidget(self.txt_pick_params, 0, 1)

        throw_layout.addWidget(QLabel("Drop: Base°, R cm, Z cm, Wrist°:"), 1, 0)
        self.txt_drop_params = QLineEdit("135, 25.0, 20.0, 90")
        throw_layout.addWidget(self.txt_drop_params, 1, 1)

        throw_layout.addWidget(QLabel("Greif-Winkel (Finger Grad 0-180°):"), 2, 0)
        self.txt_throw_finger_deg = QLineEdit("60")
        throw_layout.addWidget(self.txt_throw_finger_deg, 2, 1)

        self.btn_exec_throw = QPushButton("💥 OBJEKT GREIFEN, DREHEN & ABWERFEN")
        self.btn_exec_throw.clicked.connect(self.execute_pick_and_throw)
        throw_layout.addWidget(self.btn_exec_throw, 3, 0, 1, 2)

        left_layout.addWidget(throw_box)

        # LOG KONSOLE
        log_box = QGroupBox("🖥️ SYSTEM LOG")
        log_layout = QVBoxLayout(log_box)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        left_layout.addWidget(log_box)

        main_layout.addLayout(left_layout, stretch=1)

        # -------------------------------------------------------------------
        # RECHTE SEITE: 3D MODEL & MANUELLE SERVO SLIDER
        # -------------------------------------------------------------------
        right_layout = QVBoxLayout()

        vis_box = QGroupBox("🌐 3D VISUALISIERUNG")
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

        # SPEED & SLIDERS
        speed_box = QGroupBox("⚡ GESCHWINDIGKEIT")
        speed_layout = QVBoxLayout(speed_box)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.lbl_speed_status = QLabel("Geschwindigkeit: Level 5")
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        speed_layout.addWidget(self.lbl_speed_status)
        speed_layout.addWidget(self.speed_slider)
        right_layout.addWidget(speed_box)

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

        btn_reset_90 = QPushButton("↺ ALLE SERVOS RESET (90°)")
        btn_reset_90.clicked.connect(self.reset_all_servos)
        sliders_layout.addWidget(btn_reset_90, len(joints), 0, 1, 3)

        right_layout.addWidget(sliders_box, stretch=1)
        main_layout.addLayout(right_layout, stretch=2)

    def log(self, text):
        self.log_text.append(f"> {text}")

    # -----------------------------------------------------------------------
    # TRACKER SPEICHER-FUNKTIONEN
    # -----------------------------------------------------------------------
    def copy_to_pick(self):
        b = self.angles["Base"]
        w = self.angles["Wrist"]
        str_val = f"{b}, {self.current_r:.1f}, {self.current_xyz[2]:.1f}, {w}"
        self.txt_pick_params.setText(str_val)
        self.log(f"Position in Pick-Feld übernommen: {str_val}")

    def copy_to_drop(self):
        b = self.angles["Base"]
        w = self.angles["Wrist"]
        str_val = f"{b}, {self.current_r:.1f}, {self.current_xyz[2]:.1f}, {w}"
        self.txt_drop_params.setText(str_val)
        self.log(f"Position in Drop-Feld übernommen: {str_val}")

    # -----------------------------------------------------------------------
    # 2D INVERSE KINEMATIK
    # -----------------------------------------------------------------------
    def calculate_2d_ik(self, base_deg, r_cm, z_cm, wrist_deg=90, finger_deg=180):
        z_w = z_cm - L_BASE
        d = math.sqrt(r_cm**2 + z_w**2)

        if d > R_MAX:
            return False, f"Reichweite zu groß ({round(d,1)} cm > {R_MAX} cm)!", None
        if d < R_MIN:
            return False, f"Zu nah an Basis ({round(d,1)} cm < {R_MIN} cm)!", None

        cos_elbow = (L1_DUAL**2 + L_EFF**2 - d**2) / (2 * L1_DUAL * L_EFF)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        angle_elbow = math.degrees(math.acos(cos_elbow))

        beta = math.atan2(z_w, r_cm)
        cos_alpha = (L1_DUAL**2 + d**2 - L_EFF**2) / (2 * L1_DUAL * d)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)

        angle_shoulder = math.degrees(beta + alpha)

        base_deg = max(0, min(180, int(round(base_deg))))
        wrist_deg = max(0, min(180, int(round(wrist_deg))))
        finger_deg = max(0, min(180, int(round(finger_deg))))

        target_angles = {
            "Base": base_deg,
            "Dual": int(round(max(0, min(180, 180.0 - angle_shoulder)))),
            "Arm": int(round(max(0, min(180, angle_shoulder)))),
            "Elbow": int(round(max(0, min(180, 180.0 - angle_elbow)))),
            "Wrist": wrist_deg,
            "Finger": finger_deg
        }

        # 3D Punkte für Visualisierung
        rad_b = math.radians(base_deg - 90.0)
        p0 = [0, 0, 0]
        p1 = [0, 0, L_BASE]
        p2 = [
            p1[0] + L1_DUAL * math.cos(beta + alpha) * math.cos(rad_b),
            p1[1] + L1_DUAL * math.cos(beta + alpha) * math.sin(rad_b),
            p1[2] + L1_DUAL * math.sin(beta + alpha)
        ]
        p3 = [
            p2[0] + (L2_ARM + L3_ELBOW) * math.cos(beta + alpha - math.radians(180.0 - angle_elbow)) * math.cos(rad_b),
            p2[1] + (L2_ARM + L3_ELBOW) * math.cos(beta + alpha - math.radians(180.0 - angle_elbow)) * math.sin(rad_b),
            p2[2] + (L2_ARM + L3_ELBOW) * math.sin(beta + alpha - math.radians(180.0 - angle_elbow))
        ]
        p4 = [r_cm * math.cos(rad_b), r_cm * math.sin(rad_b), z_cm]

        return True, target_angles, (p0, p1, p2, p3, p4)

    def on_move_decoupled_clicked(self):
        try:
            b_deg = float(self.txt_base_angle.text().strip())
            r_cm = float(self.txt_reach_r.text().strip())
            z_cm = float(self.txt_height_z.text().strip())
            w_deg = float(self.txt_wrist_deg.text().strip())
            f_deg = float(self.txt_finger_deg.text().strip())
        except ValueError:
            QMessageBox.warning(self, "Fehler", "Ungültige Zahlen im Eingabefeld!")
            return

        ok, target, pts = self.calculate_2d_ik(b_deg, r_cm, z_cm, w_deg, f_deg)
        if ok:
            self.log(f"Fahre zu: Base={b_deg}°, R={r_cm}cm, Z={z_cm}cm, Wrist={w_deg}°, Finger={f_deg}°")
            threading.Thread(target=self.move_smoothly, args=(target, pts), daemon=True).start()
        else:
            self.log(f"❌ {target}")
            QMessageBox.critical(self, "IK Fehler", target)

    # -----------------------------------------------------------------------
    # SEQUENZ: PICK, ROTATE & ABWERFEN
    # -----------------------------------------------------------------------
    def execute_pick_and_throw(self):
        try:
            p_parts = [float(i) for i in self.txt_pick_params.text().split(",")]
            d_parts = [float(i) for i in self.txt_drop_params.text().split(",")]
            f_grip_deg = float(self.txt_throw_finger_deg.text().strip())
        except Exception:
            QMessageBox.warning(self, "Fehler", "Format: 'Base, R, Z, Wrist' z.B.: '45, 25.0, 5.0, 90'")
            return

        p_base, p_r, p_z, p_w = p_parts
        d_base, d_r, d_z, d_w = d_parts

        ok1, ik_pick, pts_p = self.calculate_2d_ik(p_base, p_r, p_z, p_w, 180)
        ok2, ik_drop, pts_d = self.calculate_2d_ik(d_base, d_r, d_z, d_w, f_grip_deg)

        if not ok1 or not ok2:
            QMessageBox.critical(self, "IK Fehler", "Eine der Positionen liegt außerhalb der Reichweite!")
            return

        def sequence():
            self.log("🚀 STARTE PICK & THROW SEQUENZ...")

            # 1. Anfahren Pick (Offen)
            ik_pick["Finger"] = 180
            self.move_smoothly(ik_pick, pts_p)
            time.sleep(0.4)

            # 2. Greifen
            ik_pick["Finger"] = int(f_grip_deg)
            self.send_angles(ik_pick)
            self.log(f"Greife Objekt mit Finger={f_grip_deg}°...")
            time.sleep(0.5)

            # 3. Anheben (+10 cm)
            _, ik_lift, pts_l = self.calculate_2d_ik(p_base, p_r, p_z + 10.0, p_w, f_grip_deg)
            self.move_smoothly(ik_lift, pts_l)

            # 4. Drehen und Abwurf-Position anfahren
            ik_drop["Finger"] = int(f_grip_deg)
            self.move_smoothly(ik_drop, pts_d)
            self.log(f"Gedreht zu Base={d_base}° auf Abwurf-Höhe Z={d_z}cm.")
            time.sleep(0.4)

            # 5. ABWERFEN / DROP (Finger voll öffnen)
            ik_drop["Finger"] = 180
            self.send_angles(ik_drop)
            self.log("💥 OBJEKT ABGEWORFEN / FALLENGELASSEN!")
            time.sleep(0.5)

            # 6. Reset
            self.reset_all_servos()

        threading.Thread(target=sequence, daemon=True).start()

    # -----------------------------------------------------------------------
    # VORWÄRTSKINEMATIK & REALTIME TRACKING
    # -----------------------------------------------------------------------
    def recalculate_fk_and_update(self):
        b_rad = math.radians(self.angles["Base"] - 90.0)
        s_rad = math.radians(self.angles["Arm"])
        e_rad = math.radians(180.0 - self.angles["Elbow"])

        p0 = [0, 0, 0]
        p1 = [0, 0, L_BASE]

        r_arm = L1_DUAL * math.cos(s_rad) + L_EFF * math.cos(s_rad - e_rad)
        z_arm = L_BASE + L1_DUAL * math.sin(s_rad) + L_EFF * math.sin(s_rad - e_rad)

        x = round(r_arm * math.cos(b_rad), 2)
        y = round(r_arm * math.sin(b_rad), 2)
        z = round(max(0.0, z_arm), 2)

        self.current_xyz = (x, y, z)
        self.current_r = round(r_arm, 2)

        # Live Tracker Label Update
        self.lbl_curr_xyz.setText(f"X: {x} cm | Y: {y} cm | Z: {z} cm | Reichweite R: {self.current_r} cm")
        self.lbl_curr_angles.setText(
            f"Winkel: Base: {self.angles['Base']}° | Dual: {self.angles['Dual']}° | "
            f"Arm: {self.angles['Arm']}° | Elbow: {self.angles['Elbow']}° | "
            f"Wrist: {self.angles['Wrist']}° | Finger: {self.angles['Finger']}°"
        )

        p2 = [
            p1[0] + L1_DUAL * math.cos(s_rad) * math.cos(b_rad),
            p1[1] + L1_DUAL * math.cos(s_rad) * math.sin(b_rad),
            p1[2] + L1_DUAL * math.sin(s_rad)
        ]
        p3 = [
            p2[0] + (L2_ARM + L3_ELBOW) * math.cos(s_rad - e_rad) * math.cos(b_rad),
            p2[1] + (L2_ARM + L3_ELBOW) * math.cos(s_rad - e_rad) * math.sin(b_rad),
            p2[2] + (L2_ARM + L3_ELBOW) * math.sin(s_rad - e_rad)
        ]
        p4 = [x, y, z]

        self.update_3d_plot(p0, p1, p2, p3, p4)

    def update_3d_plot(self, p0, p1, p2, p3, p4):
        self.ax.clear()
        self.ax.set_facecolor('#06060c')
        self.ax.tick_params(colors='#00ffcc', labelsize=8)

        xs = [p0[0], p1[0], p2[0], p3[0], p4[0]]
        ys = [p0[1], p1[1], p2[1], p3[1], p4[1]]
        zs = [p0[2], p1[2], p2[2], p3[2], p4[2]]

        self.ax.plot(xs, ys, zs, '-o', color='#00ffcc', linewidth=4, markersize=7, markerfacecolor='#ff0055')
        self.ax.scatter([p4[0]], [p4[1]], [p4[2]], color='#ff0055', s=100)

        self.ax.set_xlim([-55, 55])
        self.ax.set_ylim([0, 55])
        self.ax.set_zlim([0, 55])

        self.canvas.draw()

    def reset_camera(self):
        self.ax.view_init(elev=25, azim=-60)
        self.canvas.draw()

    def top_camera(self):
        self.ax.view_init(elev=90, azim=-90)
        self.canvas.draw()

    def get_step_delay(self):
        return (11 - self.speed_slider.value()) * 0.01

    def on_speed_changed(self, val):
        self.lbl_speed_status.setText(f"Geschwindigkeit: Level {val}")

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

    def reset_all_servos(self):
        for k in self.angles:
            self.angles[k] = 90
        self.angles["Finger"] = 180
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()

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

    def apply_scifi_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #06060c; }
            QGroupBox { 
                color: #00ffcc; font-size: 11px; font-weight: bold; 
                border: 1px solid #00ffcc; border-radius: 6px;
                margin-top: 8px; padding-top: 12px; background-color: #0a0d18;
            }
            QLabel { color: #a0f0ff; font-weight: bold; font-family: Consolas, monospace; }
            QLineEdit, QComboBox, QTextEdit { 
                background-color: #03050a; color: #00ffcc; 
                border: 1px solid #00a8ff; border-radius: 4px;
                padding: 4px; font-family: Consolas, monospace;
            }
            QPushButton { 
                background-color: #0f172a; color: #00ffcc; font-weight: bold; 
                border: 1px solid #00ffcc; padding: 4px; border-radius: 4px; 
                font-family: Consolas, monospace; font-size: 10px;
            }
            QPushButton:hover { background-color: #00ffcc; color: #06060c; }
            QSlider::groove:horizontal { height: 6px; background: #1a2238; border-radius: 3px; }
            QSlider::handle:horizontal { background: #ff0055; width: 16px; margin: -5px 0; border-radius: 8px; }
        """)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SciFiRobotGUI()
    window.show()
    sys.exit(app.exec())