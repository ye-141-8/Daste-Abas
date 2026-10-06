import sys
import os
import json
import math
import time
import queue
import threading
import configparser

import serial
import numpy as np

from PySide6.QtCore import Qt, QObject, Signal, QTimer, QSize
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSlider, QTextEdit, QMessageBox, QComboBox, QStackedWidget,
    QListWidget, QListWidgetItem, QProgressBar, QCheckBox, QSpinBox,
    QFrame, QScrollArea, QSizePolicy
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

# ===========================================================================
# ⚙️ 2D REICHWEITE-MODELL (LENKEN ÜBER R & Z) — ye-141-8 / v3.5-Ebene
# ===========================================================================
L_BASE    = 0.00   # Basishöhe
L1_DUAL   = 22.00  # Oberarm (Dual bis Arm)
L2_ARM    = 10.00  # Unterarm (Arm bis Elbow)
L3_ELBOW  = 7.00   # Handgelenk-Segment (Elbow bis Wrist)
L4_FINGER = 15.00  # Greifer (Wrist bis Fingerspitze)

L_EFF = L2_ARM + L3_ELBOW + L4_FINGER  # 32.0 cm
R_MAX = L1_DUAL + L_EFF                 # 54.0 cm (Max Streckung)
R_MIN = abs(L1_DUAL - L_EFF)            # 10.0 cm (Min Nahgrenze)

# Gelenkreihenfolge im Protokoll (muss der Firmware "P:"-Format entsprechen)
JOINT_ORDER = ['Base', 'Finger', 'Wrist', 'Arm', 'Elbow', 'Dual']

# Tastaturlayout: Gelenk -> { 'decrease': Taste, 'increase': Taste }
KEYMAP = {
    'Base':   {'decrease': 'a', 'increase': 'd'},
    'Finger': {'decrease': 'q', 'increase': 'e'},
    'Wrist':  {'decrease': 'k', 'increase': 'j'},
    'Arm':    {'decrease': 'o', 'increase': 'i'},
    'Elbow':  {'decrease': 'n', 'increase': 'm'},
    'Dual':   {'decrease': 's', 'increase': 'w'},
}

STEP_ANGLE = 5

# Dateien für Einstellungen / Presets (neben diesem Skript)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, 'config.ini')
PRESET_FILE = os.path.join(BASE_DIR, 'presets.json')
SEQUENCE_DIR = os.path.join(BASE_DIR, 'sequences')


# ===========================================================================
# KONFIGURATION (config.ini)
# ===========================================================================
def load_config():
    cfg = configparser.ConfigParser()
    cfg['serial'] = {'port': 'COM6', 'baud': str(DEFAULT_BAUD),
                     'step_angle': str(STEP_ANGLE)}
    cfg['safety'] = {'safe_home': '90,90,90,90,90,90'}
    cfg['recorder'] = {'speed': '1.00', 'loop': 'false'}
    cfg['window'] = {'geometry': '1400x900'}
    try:
        if os.path.exists(CONFIG_FILE):
            cfg.read(CONFIG_FILE, encoding='utf-8')
    except Exception:
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            cfg.write(f)
    except Exception:
        pass


# ===========================================================================
# SERIELLE SCHICHT
# ===========================================================================
class SerialManager(QObject):
    """Besitzt den seriellen Port, liest bytesweise im Hintergrund-Thread
    und parst thread-sicher die 'POS:'-Zeilen. Sendet über ein Schreib-Lock."""
    pos_received = Signal(list)
    raw_line = Signal(str)
    status_loss = Signal()

    def __init__(self, port, baud, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._state = {j: 90 for j in JOINT_ORDER}
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self.connected = True
        self.last_pos_time = time.time()
        self._stop = threading.Event()
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(1)
                if not chunk:
                    continue
                byte = chunk[0]
            except Exception:
                break
            if byte in (0x0A, 0x0D):            # \n oder \r
                line = bytes(buf).decode('utf-8', 'ignore').strip()
                buf = bytearray()
                if line:
                    self._parse(line)
                    self.raw_line.emit(line)
            else:
                buf.append(byte)
        self.connected = False
        self.status_loss.emit()

    def _parse(self, line):
        if line.startswith('POS:'):
            self.last_pos_time = time.time()
            parts = line[4:].split(',')
            with self._state_lock:
                for i, j in enumerate(JOINT_ORDER):
                    if i < len(parts):
                        try:
                            self._state[j] = max(0, min(180, int(parts[i])))
                        except ValueError:
                            pass
            self.pos_received.emit(list(self._state.values()))

    def get_state(self):
        with self._state_lock:
            return dict(self._state), self.connected

    def send_abs(self, pos):
        cmd = 'P:' + ','.join(str(int(pos[j])) for j in JOINT_ORDER) + '\n'
        with self._write_lock:
            try:
                self.ser.write(cmd.encode('ascii'))
            except Exception:
                self.connected = False
                self.status_loss.emit()

    def request_position(self):
        with self._write_lock:
            try:
                self.ser.write(b'?\n')
            except Exception:
                self.connected = False
                self.status_loss.emit()

    def close(self):
        self._stop.set()
        try:
            with self._write_lock:
                self.ser.close()
        except Exception:
            pass


class SimulatedManager(QObject):
    """Simulierter Seriell-Backend, damit die Oberfläche ohne Hardware läuft.

    Spiegelt die SerialManager-API; Befehle aktualisieren nur eine
    In-Memory-Position, sodass sich der Arm am Bildschirm bewegt."""
    sim = True
    pos_received = Signal(list)
    raw_line = Signal(str)
    status_loss = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port = 'COM7 (Simulation)'
        self.connected = True
        self.last_pos_time = time.time()
        self._state = {j: 90 for j in JOINT_ORDER}

    def get_state(self):
        return dict(self._state), self.connected

    def send_abs(self, pos):
        self._state.update(pos)
        self.last_pos_time = time.time()
        self.pos_received.emit(list(self._state.values()))

    def request_position(self):
        self.last_pos_time = time.time()

    def close(self):
        pass


# ===========================================================================
# VIRTUELLER JOYSTICK (Zeichenfläche mit Maus)
# ===========================================================================
class JoystickWidget(QWidget):
    moved = Signal(float, float)     # (vx, vy)
    released = Signal()

    def __init__(self):
        super().__init__()
        self.setFixedSize(220, 165)
        self.setMinimumSize(220, 165)
        self._active = False
        self._cx, self._cy = 110, 82
        self._r = 44
        self._vx = 0.0
        self._vy = 0.0
        self._knob_x, self._knob_y = self._cx, self._cy

    def paintEvent(self, ev):
        from PySide6.QtGui import QPainter, QColor, QPen
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#0f1216'))
        painter.setPen(QPen(QColor('#3a4751'), 2))
        painter.drawEllipse(self._cx - self._r, self._cy - self._r,
                            self._r * 2, self._r * 2)
        painter.setBrush(QColor('#00ffcc'))
        painter.setPen(QColor('#00ffcc'))
        painter.drawEllipse(self._cx - 3, self._cy - 3, 6, 6)
        painter.drawEllipse(self._knob_x - 18, self._knob_y - 18, 36, 36)
        painter.end()

    def sizeHint(self):
        return QSize(220, 165)

    def _joy_move(self, x, y):
        dx, dy = x - self._cx, y - self._cy
        dist = math.hypot(dx, dy)
        if dist > self._r:
            dx = dx / dist * self._r
            dy = dy / dist * self._r
        self._vx = dx / self._r
        self._vy = dy / self._r
        self._knob_x = self._cx + dx
        self._knob_y = self._cy + dy
        self.update()
        self.moved.emit(self._vx, self._vy)

    def mousePressEvent(self, ev):
        self._active = True
        self._joy_move(ev.position().x(), ev.position().y())

    def mouseMoveEvent(self, ev):
        if self._active and (ev.buttons() & Qt.LeftButton):
            self._joy_move(ev.position().x(), ev.position().y())

    def mouseReleaseEvent(self, ev):
        self._active = False
        self._vx = self._vy = 0.0
        self._knob_x, self._knob_y = self._cx, self._cy
        self.update()
        self.released.emit()


# ===========================================================================
# HAUPTFENSTER
# ===========================================================================
class SciFiRobotGUI(QMainWindow):
    # Qt-Signale für thread-sichere GUI-Updates
    sig_pose = Signal(dict, object)               # (angles, pts_3d_oder_None)
    sig_log = Signal(str)
    sig_sweep_progress = Signal(int)
    sig_sweep_done = Signal()
    sig_seq_step = Signal(dict)
    sig_seq_done = Signal()
    sig_motion_step = Signal(int, float, dict, float)
    sig_motion_done = Signal()
    sig_loop_tick = Signal(int)
    sig_recorder_status = Signal()
    sig_conn_state = Signal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚡ Daste Abas — CYBER-ARM v4.0 // ADVANCED NEURAL INTERFACE")

        self.cfg = load_config()
        self.step_angle = int(self.cfg.get('serial', 'step_angle', fallback=str(STEP_ANGLE)))
        self.safe_home = {j: 90 for j in JOINT_ORDER}
        self._load_safe_home()
        self._load_presets()

        self.angles = {j: 90 for j in JOINT_ORDER}
        self.angles['Finger'] = GREIFER_OFFEN
        self.current_xyz = (0.0, 0.0, 0.0)

        self.manager = None
        self.port = self.cfg.get('serial', 'port', fallback='COM6')
        self.baud = int(self.cfg.get('serial', 'baud', fallback=str(DEFAULT_BAUD)))

        # Betriebszustände / Threads
        self.busy = 0
        self._abort = threading.Event()            # E-Stop
        self.sweep_cancel = threading.Event()
        self.seq_cancel = threading.Event()
        self.seq = []
        self.seq_playing = False
        self.loop_active = False
        self.cycle_count = 0

        self.motion_log = []
        self.recording = False
        self.rec_start = 0.0
        self.motion_playing = False
        self.motion_pause = threading.Event()
        self.motion_pause.set()
        self.motion_cancel = threading.Event()

        self.joystick = {'active': False, 'vx': 0.0, 'vy': 0.0}
        self.joy_mode = None
        self._syncing = False
        self._pending_pts = None
        self._plot_timer_active = False

        self.init_ui()
        self.apply_scifi_stylesheet()

        # Verbindungen der Signale
        self.sig_pose.connect(self._on_pose)
        self.sig_log.connect(self._on_log)
        self.sig_sweep_progress.connect(self._on_sweep_progress)
        self.sig_sweep_done.connect(self._on_sweep_done)
        self.sig_seq_step.connect(self._on_seq_step)
        self.sig_seq_done.connect(self._on_seq_done)
        self.sig_motion_step.connect(self._on_motion_step)
        self.sig_motion_done.connect(self._on_motion_done)
        self.sig_loop_tick.connect(self._on_loop_tick)
        self.sig_recorder_status.connect(self._render_recorder)
        self.sig_conn_state.connect(self._update_conn_label)

        # Status-Polling
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(40)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start()

        # Geometrie aus config.ini wiederherstellen
        g = self.cfg.get('window', 'geometry', fallback='1400x900')
        try:
            w, h = [int(x) for x in g.split('x')[:2]]
            self.resize(w, h)
        except Exception:
            self.resize(1400, 900)

        self._apply_recorder_config()
        self.recalculate_fk_and_update()

    # ================================================================== UI
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---------- Kopfbereich (dauerhaft) ----------
        root.addLayout(self._build_header())

        # ---------- Körper: Seitenleiste | Stapel-Ansichten | 3D ----------
        body = QHBoxLayout()

        # --- Seitenleiste ---
        self.sidebar = QListWidget()
        self.sidebar.setFixedWidth(170)
        for text in ["🕹  Manuell", "🎯  Koordinaten", "🔄  Pick & Drop",
                     "🔧  Motor-Test", "🎬  Recorder", "🗒  Sequencer",
                     "💾  Presets", "🎛  Reichweite-R/Z & THROW", "🖥  Konsole & Einstellungen"]:
            item = QListWidgetItem(text)
            self.sidebar.addItem(item)
        self.sidebar.currentRowChanged.connect(self._on_sidebar_changed)
        body.addWidget(self.sidebar, 0)

        # --- Stapel-Ansichten ---
        self.stack = QStackedWidget()
        self._page_manuell = self._build_manuell_page()
        self._page_koordinaten = self._build_koordinaten_page()
        self._page_pickdrop = self._build_pickdrop_page()
        self._page_test = self._build_test_page()
        self._page_recorder = self._build_recorder_page()
        self._page_sequencer = self._build_sequencer_page()
        self._page_presets = self._build_presets_page()
        self._page_rz = self._build_rz_page()
        self._page_settings = self._build_settings_page()
        for p in (self._page_manuell, self._page_koordinaten, self._page_pickdrop,
                  self._page_test, self._page_recorder, self._page_sequencer,
                  self._page_presets, self._page_rz, self._page_settings):
            self.stack.addWidget(p)
        self.sidebar.setCurrentRow(0)
        body.addWidget(self.stack, 3)

        # --- permanente 3D-Ansicht ---
        body.addLayout(self._build_3d_panel(), 2)

        root.addLayout(body)

    def _page_container(self, title, title_color='#00ffcc'):
        page = QWidget()
        outer = QVBoxLayout(page)
        bar = QLabel(f"  {title}  ")
        bar.setStyleSheet(
            f"background-color: {title_color}; color: #06060c; font-weight: bold;"
            " font-size: 12px; padding: 6px; border-radius: 4px;")
        outer.addWidget(bar)
        content = QVBoxLayout()
        outer.addLayout(content)
        outer.addStretch(0)
        return page, content

    # ----------------------------- Header -----------------------------
    def _build_header(self):
        box = QHBoxLayout()
        box.setSpacing(8)
        title = QLabel("⚡ DASTE ABAS // ARM STEUERUNG")
        title.setStyleSheet("color: #00ffcc; font-size: 18px; font-weight: bold;")
        box.addWidget(title)

        box.addStretch(1)

        box.addWidget(QLabel("PORT:"))
        self.port_input = QLineEdit(self.port)
        self.port_input.setFixedWidth(70)
        box.addWidget(self.port_input)

        self.btn_connect = QPushButton("⚡ VERBINDEN")
        self.btn_connect.clicked.connect(self.toggle_connection)
        box.addWidget(self.btn_connect)

        self.chk_sim = QCheckBox("Simulation")
        self.chk_sim.toggled.connect(self._on_sim_toggled)
        box.addWidget(self.chk_sim)

        self.lbl_status = QLabel("● GETRENNT")
        self.lbl_status.setStyleSheet("color: #e8b84b; font-weight: bold;")
        box.addWidget(self.lbl_status)

        self.btn_estop = QPushButton("⛔ E-STOP")
        self.btn_estop.setStyleSheet(
            "QPushButton { background-color: #c0392b; color: #ffffff; "
            "font-weight: bold; border: 1px solid #ff5555; padding: 8px; "
            "border-radius: 4px; font-size: 13px; }")
        self.btn_estop.clicked.connect(self.estop)
        box.addWidget(self.btn_estop)

        return box

    # ----------------------------- 3D-Panel -----------------------------
    def _build_3d_panel(self):
        layout = QVBoxLayout()
        vis_box = QGroupBox("🌐 3D NEURAL TRACKING")
        v = QVBoxLayout(vis_box)
        self.fig = Figure(figsize=(4.2, 4.2), facecolor='#06060c')
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        v.addWidget(self.canvas)

        self.lbl_curr_pos = QLabel("X: 0.00 cm | Y: 0.00 cm | Z: 0.00 cm")
        self.lbl_curr_pos.setStyleSheet("color: #00ffcc; font-size: 13px; font-weight: bold;")
        v.addWidget(self.lbl_curr_pos)

        cam = QHBoxLayout()
        b_reset = QPushButton("📷 Ansicht Reset")
        b_reset.clicked.connect(self.reset_camera)
        b_top = QPushButton("📐 Draufsicht")
        b_top.clicked.connect(self.top_camera)
        cam.addWidget(b_reset)
        cam.addWidget(b_top)
        v.addLayout(cam)
        layout.addWidget(vis_box)

        return layout

    # ----------------------------- Manuell -----------------------------
    def _build_manuell_page(self):
        page, c = self._page_container("MANUELL", '#00ffcc')

        # Servo-Slider mit Zahlenfeldern
        g = QGroupBox("📊 MANUELLE SERVO STEUERUNG")
        gl = QGridLayout(g)
        self.sliders = {}
        self.spins = {}
        for idx, joint in enumerate(JOINT_ORDER):
            gl.addWidget(QLabel(f"{joint}:"), idx, 0)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.setValue(self.angles[joint])
            slider.valueChanged.connect(lambda val, j=joint: self.on_slider_moved(j, val))
            gl.addWidget(slider, idx, 1)
            val_lbl = QLabel(f"{self.angles[joint]}°")
            gl.addWidget(val_lbl, idx, 2)
            spin = QSpinBox()
            spin.setRange(0, 180)
            spin.setValue(self.angles[joint])
            spin.valueChanged.connect(lambda val, j=joint: self.on_spin_changed(j, val))
            gl.addWidget(spin, idx, 3)
            self.sliders[joint] = (slider, val_lbl)
            self.spins[joint] = spin
        gl.addWidget(QLabel("Tipp: Wert + Enter übernimmt 0–180°."), len(JOINT_ORDER), 0, 1, 4)
        c.addWidget(g)

        btn_reset = QPushButton("↺ HOME / RESET 90°")
        btn_reset.clicked.connect(self.reset_all_servos)
        c.addWidget(btn_reset)

        # Greifer
        g2 = QGroupBox("🦾 GREIFER STEUERUNG")
        g2l = QHBoxLayout(g2)
        b_open = QPushButton("🔓 ÖFFNEN")
        b_open.clicked.connect(self.open_gripper)
        b_close = QPushButton("🔒 SCHLIESSEN")
        b_close.clicked.connect(self.close_gripper)
        g2l.addWidget(b_open)
        g2l.addWidget(b_close)
        c.addWidget(g2)

        # Geschwindigkeit
        g3 = QGroupBox("⚡ GESCHWINDIGKEIT / SPEED")
        g3l = QVBoxLayout(g3)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.lbl_speed_status = QLabel("Geschwindigkeit: Normal (50 ms/Schritt)")
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        g3l.addWidget(self.lbl_speed_status)
        g3l.addWidget(self.speed_slider)
        c.addWidget(g3)

        # Virtueller Joystick
        g4 = QGroupBox("🎮 VIRTUELLER JOYSTICK")
        g4l = QHBoxLayout(g4)
        self.joystick_widget = JoystickWidget()
        self._install_joystick(self.joystick_widget)
        g4l.addWidget(self.joystick_widget)
        mb = QVBoxLayout()
        lab = QLabel("MODUS")
        lab.setStyleSheet("color: #a0f0ff; font-size: 9px;")
        mb.addWidget(lab)
        self.joy_btns = {}
        for _id in ('A', 'B', 'C', 'D'):
            b = QPushButton(_id)
            b.setCheckable(True)
            b.clicked.connect(lambda _, k=_id: self._set_joy_mode(k))
            self.joy_btns[_id] = b
            mb.addWidget(b)
        self.lbl_joy_mode = QLabel("")
        self.lbl_joy_mode.setWordWrap(True)
        self.lbl_joy_mode.setStyleSheet("color: #00ffcc; font-size: 9px;")
        mb.addWidget(self.lbl_joy_mode)
        mb.addStretch(0)
        g4l.addLayout(mb)
        c.addWidget(g4)
        self._render_joy_mode()

        # Tastenkürzel-Hinweis
        hint = QLabel("Tasten:  A/D Base · Q/E Finger · K/J Wrist · O/I Arm · "
                      "N/M Elbow · S/W Shoulder   ·   ESC = E-Stop")
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #9aa4b2; font-size: 10px;")
        c.addWidget(hint)

        c.addStretch(0)
        return page

    def _install_joystick(self, widget):
        widget.moved.connect(self._on_joy_moved)
        widget.released.connect(self._on_joy_released)

    # ----------------------------- Koordinaten -----------------------------
    def _build_koordinaten_page(self):
        page, c = self._page_container("KOORDINATEN", '#00a8ff')
        g = QGroupBox("🎯 KOORDINATEN ANFAHREN")
        gl = QGridLayout(g)
        gl.addWidget(QLabel("Punkt (X, Y, Z in cm):"), 0, 0)
        self.txt_point = QLineEdit("8, 15, 0")
        gl.addWidget(self.txt_point, 0, 1)
        gl.addWidget(QLabel("Wrist-Ausrichtung:"), 1, 0)
        self.combo_wrist = QComboBox()
        self.combo_wrist.addItems(["Horizontal (90°)", "Vertikal (0°)",
                                   "Diagonal Links (45°)", "Diagonal Rechts (135°)"])
        gl.addWidget(self.combo_wrist, 1, 1)
        b_goto = QPushButton("▶ GEHE ZU KOORDINATEN")
        b_goto.clicked.connect(self.on_goto_clicked)
        gl.addWidget(b_goto, 2, 0, 1, 2)
        c.addWidget(g)

        b_copy = QPushButton("📋 POSITION IN PICK-FELD ÜBERNEHMEN")
        b_copy.clicked.connect(self.copy_current_pos)
        c.addWidget(b_copy)
        c.addStretch(0)
        return page

    # ----------------------------- Pick & Drop -----------------------------
    def _build_pickdrop_page(self):
        page, c = self._page_container("PICK & DROP", '#ff6a00')
        g = QGroupBox("🔄 REPETITIVER PICK & DROP LOOP")
        gl = QGridLayout(g)
        gl.addWidget(QLabel("Pick-Punkt (X,Y,Z):"), 0, 0)
        self.txt_pick = QLineEdit("3, 2, 18")
        gl.addWidget(self.txt_pick, 0, 1)
        gl.addWidget(QLabel("Drop-Punkt (X,Y,Z):"), 1, 0)
        self.txt_drop = QLineEdit("-10, 12, 2")
        gl.addWidget(self.txt_drop, 1, 1)
        b_start = QPushButton("🔄 LOOP STARTEN")
        b_start.clicked.connect(self.start_loop)
        gl.addWidget(b_start, 2, 0)
        b_stop = QPushButton("⛔ LOOP STOPPEN")
        b_stop.clicked.connect(self.stop_loop)
        gl.addWidget(b_stop, 2, 1)
        c.addWidget(g)

        self.lbl_cycles = QLabel("Durchläufe: 0")
        self.lbl_cycles.setStyleSheet("color: #00ffcc; font-weight: bold; font-size: 13px;")
        c.addWidget(self.lbl_cycles)
        c.addStretch(0)
        return page

    # ----------------------------- Motor-Test -----------------------------
    def _build_test_page(self):
        page, c = self._page_container("MOTOR-TEST", '#5b8cff')
        g = QGroupBox("🔧 SWEEP TEST (0→180→0)")
        gl = QGridLayout(g)
        gl.addWidget(QLabel("Gelenk:"), 0, 0)
        self.combo_sweep = QComboBox()
        self.combo_sweep.addItems(JOINT_ORDER)
        gl.addWidget(self.combo_sweep, 0, 1)
        gl.addWidget(QLabel("Verzögerung (ms):"), 1, 0)
        self.txt_sweep_delay = QLineEdit("15")
        gl.addWidget(self.txt_sweep_delay, 1, 1)
        b_run = QPushButton("▶ SWEEP AUSFÜHREN")
        b_run.clicked.connect(self.run_sweep)
        gl.addWidget(b_run, 2, 0)
        b_stop = QPushButton("⛔ STOP")
        b_stop.clicked.connect(self.sweep_cancel.set)
        gl.addWidget(b_stop, 2, 1)
        c.addWidget(g)

        self.sweep_progress = QProgressBar()
        self.sweep_progress.setRange(0, 180)
        self.sweep_progress.setValue(0)
        c.addWidget(self.sweep_progress)
        self.lbl_sweep_status = QLabel("Bereit.")
        c.addWidget(self.lbl_sweep_status)
        c.addStretch(0)
        return page

    # ----------------------------- Recorder -----------------------------
    def _build_recorder_page(self):
        page, c = self._page_container("MOTION RECORDER", '#e8b84b')
        self.rec_btn = QPushButton("● AUFNAHME")
        self.rec_btn.clicked.connect(self.toggle_record)
        c.addWidget(self.rec_btn)
        self.lbl_rec_status = QLabel("Bereit · 0 Punkte")
        c.addWidget(self.lbl_rec_status)

        b_play = QPushButton("▶ ABSPIELEN")
        b_play.clicked.connect(self.play_motion)
        b_pause = QPushButton("⏸  PAUSE")
        b_pause.clicked.connect(self.toggle_pause)
        b_stop = QPushButton("⛔ STOP")
        b_stop.clicked.connect(self.stop_motion)
        row = QHBoxLayout()
        row.addWidget(b_play)
        row.addWidget(b_pause)
        row.addWidget(b_stop)
        c.addLayout(row)

        opt = QHBoxLayout()
        self.chk_loop = QCheckBox("Loop")
        opt.addWidget(self.chk_loop)
        opt.addWidget(QLabel("Geschwindigkeit:"))
        self.rec_speed = QSlider(Qt.Horizontal)
        self.rec_speed.setRange(25, 400)     # 0.25x .. 4.0x
        self.rec_speed.setValue(100)
        self.lbl_rec_speed = QLabel("1.00×")
        self.rec_speed.valueChanged.connect(
            lambda v: self.lbl_rec_speed.setText(f"{v / 100.0:.2f}×"))
        opt.addWidget(self.rec_speed)
        opt.addWidget(self.lbl_rec_speed)
        c.addLayout(opt)

        self.rec_progress = QProgressBar()
        self.rec_progress.setRange(0, 1000)
        c.addWidget(self.rec_progress)

        b_save = QPushButton("💾 JSON SPEICHERN")
        b_save.clicked.connect(self.save_motion)
        b_load = QPushButton("📂 JSON LADEN")
        b_load.clicked.connect(self.load_motion)
        io = QHBoxLayout()
        io.addWidget(b_save)
        io.addWidget(b_load)
        c.addLayout(io)
        c.addStretch(0)
        return page

    # ----------------------------- Sequencer -----------------------------
    def _build_sequencer_page(self):
        page, c = self._page_container("SEQUENCER (TEACH-IN)", '#00ffcc')
        b_rec = QPushButton("● POSE AUFNEHMEN")
        b_rec.clicked.connect(self.record_pose)
        c.addWidget(b_rec)

        sp = QHBoxLayout()
        sp.addWidget(QLabel("Zeit pro Schritt (ms):"))
        self.txt_seq_speed = QLineEdit("200")
        sp.addWidget(self.txt_seq_speed)
        c.addLayout(sp)

        b_play = QPushButton("▶ ABSPIELEN")
        b_play.clicked.connect(self.play_seq)
        b_stop = QPushButton("⛔ STOP")
        b_stop.clicked.connect(self.seq_cancel.set)
        row = QHBoxLayout()
        row.addWidget(b_play)
        row.addWidget(b_stop)
        c.addLayout(row)

        b_save = QPushButton("💾 JSON SPEICHERN")
        b_save.clicked.connect(self.save_seq)
        b_load = QPushButton("📂 JSON LADEN")
        b_load.clicked.connect(self.load_seq)
        io = QHBoxLayout()
        io.addWidget(b_save)
        io.addWidget(b_load)
        c.addLayout(io)

        g = QGroupBox("🛑 WEGPUNKTE")
        gl = QVBoxLayout(g)
        self.seq_list = QListWidget()
        gl.addWidget(self.seq_list)
        b_del = QPushButton("🗑  AUSGEWÄHLTE POSE LÖSCHEN")
        b_del.clicked.connect(self.del_pose)
        gl.addWidget(b_del)
        self.lbl_seq_status = QLabel("0 Posen")
        gl.addWidget(self.lbl_seq_status)
        c.addWidget(g)
        c.addStretch(0)
        return page

    # ----------------------------- Presets -----------------------------
    def _build_presets_page(self):
        page, c = self._page_container("PRESETS", '#c678dd')
        g = QGroupBox("💾 POSE SPEICHERN")
        gl = QHBoxLayout(g)
        self.txt_preset_name = QLineEdit("Posename")
        gl.addWidget(self.txt_preset_name)
        b_save = QPushButton("SPEICHERN")
        b_save.clicked.connect(self.save_preset)
        gl.addWidget(b_save)
        c.addWidget(g)

        self.preset_list = QListWidget()
        c.addWidget(self.preset_list)

        b_apply = QPushButton("▶ ANWENDEN")
        b_apply.clicked.connect(self.apply_preset)
        b_del = QPushButton("🗑  LÖSCHEN")
        b_del.clicked.connect(self.delete_preset)
        row = QHBoxLayout()
        row.addWidget(b_apply)
        row.addWidget(b_del)
        c.addLayout(row)
        self._render_presets()
        c.addStretch(0)
        return page

    # ----------------------------- Konsole & Einstellungen -----------------------------
    def _build_settings_page(self):
        page, c = self._page_container("KONSOLE & EINSTELLUNGEN", '#00a8ff')
        g = QGroupBox("🖥 SYSTEM KONSOLE")
        gl = QVBoxLayout(g)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        gl.addWidget(self.log_text)
        c.addWidget(g)

        g2 = QGroupBox("⚙ EINSTELLUNGEN")
        g2l = QGridLayout(g2)
        g2l.addWidget(QLabel("Safe-Home (6 Werte, 0–180):"), 0, 0)
        sh = ','.join(str(self.safe_home[j]) for j in JOINT_ORDER)
        self.txt_safe_home = QLineEdit(sh)
        self.txt_safe_home.editingFinished.connect(self._save_safe_home)
        g2l.addWidget(self.txt_safe_home, 0, 1)
        g2l.addWidget(QLabel("Schrittwinkel (Tastatur):"), 1, 0)
        self.txt_step_angle = QSpinBox()
        self.txt_step_angle.setRange(1, 90)
        self.txt_step_angle.setValue(self.step_angle)
        self.txt_step_angle.valueChanged.connect(self._on_step_angle_changed)
        g2l.addWidget(self.txt_step_angle, 1, 1)
        g2l.addWidget(QLabel("Baudrate:"), 2, 0)
        self.txt_baud = QSpinBox()
        self.txt_baud.setRange(300, 115200)
        self.txt_baud.setValue(self.baud)
        self.txt_baud.valueChanged.connect(self._on_baud_changed)
        g2l.addWidget(self.txt_baud, 2, 1)
        c.addWidget(g2)
        c.addStretch(0)
        return page

    # ----------------------------- Reichweite-R/Z & THROW (ye-141-8) -----------------------------
    def _build_rz_page(self):
        page, c = self._page_container("REICHWEITE-R/Z & AUTONOM THROW", '#ff6a00')

        # 1. ECHTZEIT TRACKER (LIVE POS & KOPIEREN)
        tracker_box = QGroupBox("📍 LIVE TRACKER & KOORDINATEN MONITORING")
        tracker_layout = QGridLayout(tracker_box)
        self.lbl_curr_xyz = QLabel("X: 0.0 cm | Y: 0.0 cm | Z: 0.0 cm | R: 0.0 cm")
        self.lbl_curr_xyz.setStyleSheet("color: #00ffcc; font-size: 12px; font-weight: bold;")
        tracker_layout.addWidget(self.lbl_curr_xyz, 0, 0, 1, 2)
        self.lbl_curr_angles = QLabel("Winkel: Base: 90° | Dual: 90° | Arm: 90° | Elbow: 90° | Wrist: 90° | Finger: 180°")
        self.lbl_curr_angles.setStyleSheet("color: #00a8ff; font-size: 11px;")
        tracker_layout.addWidget(self.lbl_curr_angles, 1, 0, 1, 2)
        b_cp = QPushButton("📋 POS ALS PICK SPEICHERN")
        b_cp.clicked.connect(self.copy_to_pick)
        b_cd = QPushButton("📋 POS ALS DROP SPEICHERN")
        b_cd.clicked.connect(self.copy_to_drop)
        tracker_layout.addWidget(b_cp, 2, 0)
        tracker_layout.addWidget(b_cd, 2, 1)
        c.addWidget(tracker_box)

        # 2. ENTKOPPELTE ARM- & ENDEFFECTOR-STEUERUNG
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

        ctrl_layout.addWidget(QLabel("Wrist (Grad 0-180°):"), 2, 0)
        wrist_v_layout = QVBoxLayout()
        self.txt_wrist_deg = QLineEdit("90")
        wrist_v_layout.addWidget(self.txt_wrist_deg)
        wrist_btn_layout = QHBoxLayout()
        for lbl, val in [("0° Vert", "0"), ("45° DiagL", "45"), ("90° Horiz", "90"), ("135° DiagR", "135")]:
            b = QPushButton(lbl)
            b.clicked.connect(lambda _, v=val: self.txt_wrist_deg.setText(v))
            wrist_btn_layout.addWidget(b)
        wrist_v_layout.addLayout(wrist_btn_layout)
        ctrl_layout.addLayout(wrist_v_layout, 2, 1)

        ctrl_layout.addWidget(QLabel("Finger / Greifer (Grad 0-180°):"), 3, 0)
        finger_v_layout = QVBoxLayout()
        self.txt_finger_deg = QLineEdit("180")
        finger_v_layout.addWidget(self.txt_finger_deg)
        finger_btn_layout = QHBoxLayout()
        for lbl, val in [("✊ Zu (0°)", "0"), ("🤏 Halb (90°)", "90"), ("🖐️ Offen (180°)", "180")]:
            b = QPushButton(lbl)
            b.clicked.connect(lambda _, v=val: self.txt_finger_deg.setText(v))
            finger_btn_layout.addWidget(b)
        finger_v_layout.addLayout(finger_btn_layout)
        ctrl_layout.addLayout(finger_v_layout, 3, 1)

        b_move = QPushButton("▶ PARAMETER ANFAHREN")
        b_move.clicked.connect(self.on_move_decoupled_clicked)
        ctrl_layout.addWidget(b_move, 4, 0, 1, 2)
        c.addWidget(ctrl_box)

        # 3. PICK, ROTATE & DROP / THROW SEQUENZ
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
        b_exec = QPushButton("💥 OBJEKT GREIFEN, DREHEN & ABWERFEN")
        b_exec.clicked.connect(self.execute_pick_and_throw)
        throw_layout.addWidget(b_exec, 3, 0, 1, 2)
        c.addWidget(throw_box)

        c.addStretch(0)
        return page

    # ==================================================================
    # REICHWEITE-R/Z & THROW HANDLER (ye-141-8 / v3.5-Ebene)
    # ==================================================================
    def _refresh_rz_tracker(self):
        b_rad = math.radians(self.angles["Base"] - 90.0)
        s_rad = math.radians(self.angles["Arm"])
        e_rad = math.radians(180.0 - self.angles["Elbow"])
        r_arm = L1_DUAL * math.cos(s_rad) + L_EFF * math.cos(s_rad - e_rad)
        x = round(r_arm * math.cos(b_rad), 2)
        y = round(r_arm * math.sin(b_rad), 2)
        z = round(max(0.0, L_BASE + L1_DUAL * math.sin(s_rad) + L_EFF * math.sin(s_rad - e_rad)), 2)
        self.current_r = round(r_arm, 2)
        self.current_xyz = (x, y, z)
        if self.lbl_curr_xyz is not None:
            self.lbl_curr_xyz.setText(f"X: {x} cm | Y: {y} cm | Z: {z} cm | R: {self.current_r} cm")
            self.lbl_curr_angles.setText(
                f"Winkel: Base: {self.angles['Base']}° | Dual: {self.angles['Dual']}° | "
                f"Arm: {self.angles['Arm']}° | Elbow: {self.angles['Elbow']}° | "
                f"Wrist: {self.angles['Wrist']}° | Finger: {self.angles['Finger']}°")

    def copy_to_pick(self):
        b = self.angles["Base"]
        w = self.angles["Wrist"]
        self._refresh_rz_tracker()
        str_val = f"{b}, {self.current_r:.1f}, {self.current_xyz[2]:.1f}, {w}"
        self.txt_pick_params.setText(str_val)
        self.log(f"Position in Pick-Feld übernommen: {str_val}")

    def copy_to_drop(self):
        b = self.angles["Base"]
        w = self.angles["Wrist"]
        self._refresh_rz_tracker()
        str_val = f"{b}, {self.current_r:.1f}, {self.current_xyz[2]:.1f}, {w}"
        self.txt_drop_params.setText(str_val)
        self.log(f"Position in Drop-Feld übernommen: {str_val}")

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
            "Finger": finger_deg,
        }
        rad_b = math.radians(base_deg - 90.0)
        p0 = [0, 0, 0]
        p1 = [0, 0, L_BASE]
        p2 = [
            p1[0] + L1_DUAL * math.cos(beta + alpha) * math.cos(rad_b),
            p1[1] + L1_DUAL * math.cos(beta + alpha) * math.sin(rad_b),
            p1[2] + L1_DUAL * math.sin(beta + alpha),
        ]
        p4 = [r_cm * math.cos(rad_b), r_cm * math.sin(rad_b), z_cm]
        return True, target_angles, (p0, p1, p2, p4)

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
            ik_pick["Finger"] = 180
            self.move_smoothly(ik_pick, pts_p)
            time.sleep(0.4)
            ik_pick["Finger"] = int(f_grip_deg)
            self._send_angles(ik_pick)
            self.sig_pose.emit(dict(ik_pick), None)
            self.log(f"Greife Objekt mit Finger={f_grip_deg}°...")
            time.sleep(0.5)
            _, ik_lift, pts_l = self.calculate_2d_ik(p_base, p_r, p_z + 10.0, p_w, f_grip_deg)
            self.move_smoothly(ik_lift, pts_l)
            ik_drop["Finger"] = int(f_grip_deg)
            self.move_smoothly(ik_drop, pts_d)
            self.log(f"Gedreht zu Base={d_base}° auf Abwurf-Höhe Z={d_z}cm.")
            time.sleep(0.4)
            ik_drop["Finger"] = 180
            self._send_angles(ik_drop)
            self.sig_pose.emit(dict(ik_drop), None)
            self.log("💥 OBJEKT ABGEWORFEN / FALLENGELASSEN!")
            time.sleep(0.5)
            self.reset_all_servos()
            self._refresh_rz_tracker()

        threading.Thread(target=sequence, daemon=True).start()

    # ==================================================================
    # SEITENLEISTE & STACK
    # ==================================================================
    def _on_sidebar_changed(self, row):
        if 0 <= row < self.stack.count():
            self.stack.setCurrentIndex(row)

    # ==================================================================
    # LOG
    # ==================================================================
    def log(self, text):
        self.sig_log.emit(text)

    def _on_log(self, text):
        stamps = time.strftime('%H:%M:%S')
        if self.log_text is None:
            return
        self.log_text.append(f"[{stamps}] {text}")
        # auf 500 Zeilen begrenzen
        while self.log_text.document().blockCount() > 500:
            cur = self.log_text.textCursor()
            cur.movePosition(cur.Start)
            cur.movePosition(cur.EndOfLine, cur.KeepAnchor)
            cur.removeSelectedText()
            self.log_text.textCursor().deletePreviousChar()

    # ==================================================================
    # POLLING: Status, Auto-Resync, Joystick
    # ==================================================================
    def _poll(self):
        if self.manager is None:
            return
        # Verbindungsstatus
        if getattr(self.manager, 'sim', False):
            self.lbl_status.setText("● SIMULATION")
            self.lbl_status.setStyleSheet("color: #5b8cff; font-weight: bold;")
        else:
            _, connected = self.manager.get_state()
            live = (time.time() - self.manager.last_pos_time) < 1.5
            if not connected:
                self.lbl_status.setText("● LOST")
                self.lbl_status.setStyleSheet("color: #e0645a; font-weight: bold;")
            elif live:
                self.lbl_status.setText(f"{self.port} — ● CONNECTED")
                self.lbl_status.setStyleSheet("color: #34d399; font-weight: bold;")
            else:
                self.lbl_status.setText(f"{self.port} — ● NO DATA")
                self.lbl_status.setStyleSheet("color: #e8b84b; font-weight: bold;")
                if connected:
                    self.manager.request_position()   # Auto-Resync

        # Joystick saß im Polling (Manuell-Seite)
        if self.joystick['active'] and self.busy == 0 and self.manager is not None:
            self._joy_apply_vel()

        # Fortschrittsstatus des Recorders regelmäßig auffrischen
        if self.recording:
            self.sig_recorder_status.emit()

    # ==================================================================
    # VIDEOKAMERA & GESCHWINDIGKEIT
    # ==================================================================
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

    # ==================================================================
    # 3D RENDERER & VORWÄRTSKINEMATIK (FK)
    # ==================================================================
    def recalculate_fk_and_update(self):
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
        self.lbl_curr_pos.setText(
            f"X: {self.current_xyz[0]} cm | Y: {self.current_xyz[1]} cm | Z: {self.current_xyz[2]} cm")

        p2 = [
            p1[0] + L2_CM * math.cos(s_rad) * math.cos(b_rad),
            p1[1] + L2_CM * math.cos(s_rad) * math.sin(b_rad),
            p1[2] + L2_CM * math.sin(s_rad)
        ]
        p3 = [x, y, z]
        self.update_3d_plot(p0, p1, p2, p3)

    def update_3d_plot(self, p0, p1, p2, p3):
        self.ax.clear()
        self.ax.set_facecolor('#06060c')
        try:
            self.ax.xaxis.pane.set_edgecolor('#00ffcc')
            self.ax.yaxis.pane.set_edgecolor('#00ffcc')
        except Exception:
            pass
        self.ax.tick_params(colors='#00ffcc', labelsize=8)

        xs = [p0[0], p1[0], p2[0], p3[0]]
        ys = [p0[1], p1[1], p2[1], p3[1]]
        zs = [p0[2], p1[2], p2[2], p3[2]]

        self.ax.plot(xs, ys, zs, '-o', color='#00ffcc', linewidth=4,
                     markersize=8, markerfacecolor='#ff0055')
        self.ax.scatter([p3[0]], [p3[1]], [p3[2]], color='#ff0055', s=90, label='Greifer')

        self.ax.set_xlim([-25, 25])
        self.ax.set_ylim([-25, 25])
        self.ax.set_zlim([0, 30])
        self.ax.set_xlabel('X (cm)', color='#00ffcc')
        self.ax.set_ylabel('Y (cm)', color='#00ffcc')
        self.ax.set_zlabel('Z (cm)', color='#00ffcc')

        self.canvas.draw()

    def _update_tracker_from_coord(self, coord):
        self.current_xyz = (round(coord[0], 2), round(coord[1], 2), round(coord[2], 2))
        self.lbl_curr_pos.setText(
            f"X: {self.current_xyz[0]} cm | Y: {self.current_xyz[1]} cm | Z: {self.current_xyz[2]} cm")

    # ==================================================================
    # THREAD-SICHERER POSEN-UPDATE (Signal-Slot mit 3D-Throttling)
    # ==================================================================
    def _on_pose(self, angles_dict, pts_3d):
        self.angles = dict(angles_dict)
        self._syncing = True
        try:
            for k, v in angles_dict.items():
                if k in self.sliders:
                    sl, vl = self.sliders[k]
                    sl.setValue(v)
                    vl.setText(f"{v}°")
                if k in self.spins:
                    self.spins[k].setValue(v)
        finally:
            self._syncing = False
        self._pending_pts = pts_3d
        if not self._plot_timer_active:
            self._plot_timer_active = True
            QTimer.singleShot(50, self._flush_plot)     # max ~20-30 Hz

    def _flush_plot(self):
        self._plot_timer_active = False
        if self._pending_pts is not None:
            try:
                self.update_3d_plot(*self._pending_pts)
                self._update_tracker_from_coord(self._pending_pts[3])
            except Exception:
                pass
            self._pending_pts = None
        else:
            try:
                self.recalculate_fk_and_update()
            except Exception:
                pass

    # ==================================================================
    # INVERSE KINEMATIK (IK)
    # ==================================================================
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

    # ==================================================================
    # BEWEGUNGEN & SCHLEIFEN
    # ==================================================================
    def _abortable_sleep(self, seconds):
        seconds = max(0.002, seconds)
        end = time.time() + seconds
        while time.time() < end:
            if self._abort.is_set() or not self.loop_active:
                return False
            time.sleep(min(0.01, max(0.001, end - time.time())))
        return True

    def move_smoothly(self, target_angles, pts_3d=None):
        steps = 25
        delay = self.get_step_delay()
        start_angles = dict(self.angles)
        for i in range(1, steps + 1):
            if self._abort.is_set():
                return
            t = i / steps
            interp = {k: int(round(start_angles[k] + (target_angles[k] - start_angles[k]) * t))
                      for k in start_angles}
            self._send_angles(interp)
            self.sig_pose.emit(dict(interp), None)
            if not self._abortable_sleep(delay):
                return
        if pts_3d and not self._abort.is_set():
            self.sig_pose.emit(dict(self.angles), pts_3d)

    def _send_angles(self, angles_dict):
        """Sendet Winkel an den Manager (thread-sicher). Ändert die GUI nicht."""
        if self.manager is not None:
            self.manager.send_abs(angles_dict)

    def send_angles(self, angles_dict):
        """GUI-seitiger Sendepfad (Haupt-Thread): sendet und aktualisiert die Anzeige."""
        self.angles = dict(angles_dict)
        self._syncing = True
        try:
            for k, v in self.angles.items():
                if k in self.sliders:
                    self.sliders[k][0].setValue(v)
                    self.sliders[k][1].setText(f"{v}°")
                if k in self.spins:
                    self.spins[k].setValue(v)
        finally:
            self._syncing = False
        self._send_angles(self.angles)

    def on_goto_clicked(self):
        if self.busy > 0:
            return
        pt = self.parse_xyz(self.txt_point.text())
        if not pt:
            QMessageBox.warning(self, "Fehler", "Ungültiges Format! Bitte z.B. '8, 15, 0' nutzen.")
            return
        ok, target, pts = self.calculate_ik(*pt)
        if ok:
            self.log(f"Fahre zu Koordinaten {pt} ...")
            t = threading.Thread(target=self.move_smoothly, args=(target, pts), daemon=True)
            t.start()
        else:
            self.log(f"❌ {target}")
            QMessageBox.critical(self, "IK Fehler", target)

    def copy_current_pos(self):
        coord_str = f"{self.current_xyz[0]}, {self.current_xyz[1]}, {self.current_xyz[2]}"
        self.txt_pick.setText(coord_str)
        self.log(f"Position {coord_str} in Pick-Feld übernommen.")

    def start_loop(self):
        if self.loop_active or self.busy > 0:
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

        self.loop_active = True
        self.busy += 1
        self.cycle_count = 0
        self._abort.clear()

        def loop_thread():
            try:
                while self.loop_active and not self._abort.is_set():
                    self.cycle_count += 1
                    self.sig_loop_tick.emit(self.cycle_count)
                    self.sig_log.emit(f"--- DURCHLAUF #{self.cycle_count} ---")

                    pos = dict(self.angles)
                    pos["Finger"] = GREIFER_OFFEN
                    self.move_smoothly(pos)

                    ok, hover_pick, pts_h1 = self.calculate_ik(pick_pt[0], pick_pt[1], pick_pt[2] + 4.0)
                    if not ok or not self.loop_active or self._abort.is_set():
                        break
                    hover_pick["Finger"] = GREIFER_OFFEN
                    self.move_smoothly(hover_pick, pts_h1)

                    ik_pick["Finger"] = GREIFER_OFFEN
                    self.move_smoothly(ik_pick, pts_pick)
                    if not self._abortable_sleep(0.3):
                        break
                    ik_pick["Finger"] = GREIFER_ZU
                    self._send_angles(ik_pick)
                    self.sig_pose.emit(dict(ik_pick), None)
                    if not self._abortable_sleep(0.5):
                        break

                    self.move_smoothly(hover_pick, pts_h1)

                    ok, hover_drop, pts_h2 = self.calculate_ik(drop_pt[0], drop_pt[1], drop_pt[2] + 4.0)
                    if not ok or not self.loop_active or self._abort.is_set():
                        break
                    hover_drop["Finger"] = GREIFER_ZU
                    self.move_smoothly(hover_drop, pts_h2)

                    ik_drop["Finger"] = GREIFER_ZU
                    self.move_smoothly(ik_drop, pts_drop)
                    if not self._abortable_sleep(0.3):
                        break
                    ik_drop["Finger"] = GREIFER_OFFEN
                    self._send_angles(ik_drop)
                    self.sig_pose.emit(dict(ik_drop), None)
                    if not self._abortable_sleep(0.5):
                        break

                    self.move_smoothly(hover_drop, pts_h2)
            finally:
                self.loop_active = False
                self.busy = max(0, self.busy - 1)
                self.sig_log.emit("🛑 LOOP BEENDET ODER MANUELL GESTOPPT.")

        threading.Thread(target=loop_thread, daemon=True).start()

    def stop_loop(self):
        self.loop_active = False
        self.log("Stopp-Signal gesendet...")

    def _on_loop_tick(self, count):
        self.lbl_cycles.setText(f"Durchläufe: {count}")

    def open_gripper(self):
        pos = dict(self.angles)
        pos["Finger"] = GREIFER_OFFEN
        self.send_angles(pos)
        self.recalculate_fk_and_update()
        self.log("Greifer Manuell Geöffnet.")

    def close_gripper(self):
        pos = dict(self.angles)
        pos["Finger"] = GREIFER_ZU
        self.send_angles(pos)
        self.recalculate_fk_and_update()
        self.log("Greifer Manuell Geschlossen.")

    def reset_all_servos(self):
        for k in self.angles:
            self.angles[k] = 90
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self.log("Alle Servomotoren auf 90° Reset gesetzt.")

    def go_home(self):
        self.angles = dict(HOME)
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()

    def on_slider_moved(self, joint, val):
        if self.busy > 0:
            self._resync_sliders(joint)
            return
        self.angles[joint] = val
        self.spins[joint].setValue(val)
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self._record_manual()

    def on_spin_changed(self, joint, val):
        if self._syncing or self.busy > 0:
            if self.busy > 0:
                self._resync_sliders(joint)
            return
        self.angles[joint] = val
        self.sliders[joint][0].setValue(val)
        self.sliders[joint][1].setText(f"{val}°")
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self._record_manual()

    def _resync_sliders(self, joint):
        # Gesperrt: Slider zur aktuellen Soll-Position zurücksetzen
        v = self.angles[joint]
        self._syncing = True
        try:
            self.sliders[joint][0].setValue(v)
            self.sliders[joint][1].setText(f"{v}°")
            self.spins[joint].setValue(v)
        finally:
            self._syncing = False

    # ==================================================================
    # TASTATUR
    # ==================================================================
    def _input_has_focus(self):
        w = self.focusWidget()
        return isinstance(w, (QLineEdit, QSpinBox, QTextEdit, QComboBox))

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.estop()
            event.accept()
            return
        if self._input_has_focus() or self.busy > 0:
            event.accept()
            return
        key = (event.text() or '').lower()
        if not key:
            event.ignore()
            return
        for j, mp in KEYMAP.items():
            if key == mp['decrease']:
                self.nudge(j, -1)
                event.accept()
                return
            if key == mp['increase']:
                self.nudge(j, +1)
                event.accept()
                return
        super().keyPressEvent(event)

    def nudge(self, joint, sign):
        if self.busy > 0:
            return
        self.angles[joint] = max(0, min(180, self.angles[joint] + sign * self.step_angle))
        self._syncing = True
        try:
            self.sliders[joint][0].setValue(self.angles[joint])
            self.sliders[joint][1].setText(f"{self.angles[joint]}°")
            self.spins[joint].setValue(self.angles[joint])
        finally:
            self._syncing = False
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self._record_manual()

    # ==================================================================
    # JOYSTICK
    # ==================================================================
    def _on_joy_moved(self, vx, vy):
        self.joystick['active'] = True
        self.joystick['vx'] = vx
        self.joystick['vy'] = vy

    def _on_joy_released(self):
        self.joystick['active'] = False
        self.joystick['vx'] = 0.0
        self.joystick['vy'] = 0.0

    def _set_joy_mode(self, mode):
        self.joy_mode = None if self.joy_mode == mode else mode
        self._render_joy_mode()
        self.log(f"Joystick-Modus: {self._joy_mode_label()}")

    def _joy_mode_label(self):
        if self.joy_mode:
            return f"{self.joy_mode}: " + {
                'A': 'X→Wrist', 'B': 'Y→Elbow', 'C': 'X→Finger', 'D': 'Y→Arm'}[self.joy_mode]
        return 'none: X→Base / Y→Shoulder'

    def _render_joy_mode(self):
        for _id, btn in self.joy_btns.items():
            active = (self.joy_mode == _id)
            btn.setChecked(active)
        self.lbl_joy_mode.setText("Modus: " + self._joy_mode_label())

    def _joy_apply_vel(self):
        vx, vy = self.joystick['vx'], self.joystick['vy']
        if self.joy_mode == 'A':
            jx, jy = 'Wrist', None
        elif self.joy_mode == 'B':
            jx, jy = None, 'Elbow'
        elif self.joy_mode == 'C':
            jx, jy = 'Finger', None
        elif self.joy_mode == 'D':
            jx, jy = None, 'Arm'
        else:
            jx, jy = 'Base', 'Dual'
        if jx and abs(vx) > 0.05:
            self.angles[jx] = max(0, min(180, self.angles[jx] + vx * 2))
        if jy and abs(vy) > 0.05:
            self.angles[jy] = max(0, min(180, self.angles[jy] + vy * 2))
        self._syncing = True
        try:
            for j in (jx, jy):
                if j:
                    self.sliders[j][0].setValue(self.angles[j])
                    self.sliders[j][1].setText(f"{self.angles[j]}°")
                    self.spins[j].setValue(self.angles[j])
        finally:
            self._syncing = False
        self._send_angles(self.angles)
        self._record_manual()

    # ==================================================================
    # SWEEP (MOTOR-TEST)
    # ==================================================================
    def run_sweep(self):
        if self.busy > 0:
            return
        joint = self.combo_sweep.currentText()
        try:
            delay = int(self.txt_sweep_delay.text())
        except ValueError:
            delay = 15
        delay = max(5, min(200, delay))
        self.busy += 1
        self._abort.clear()
        self.sweep_cancel.clear()
        self.lbl_sweep_status.setText(f"Sweep {joint} läuft…")
        base = dict(self.angles)
        self.sweep_progress.setValue(0)

        def worker():
            try:
                targets = list(range(0, 181, 5)) + list(range(180, -1, -5))
                for target in targets:
                    if self.sweep_cancel.is_set() or self._abort.is_set():
                        break
                    p = dict(base)
                    p[joint] = target
                    self._send_angles(p)
                    self.sig_pose.emit(dict(p), None)
                    self.sig_sweep_progress.emit(target)
                    if not self._abortable_sleep(delay / 1000.0):
                        break
            finally:
                self._send_angles(base)
                self.sig_sweep_done.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _on_sweep_progress(self, value):
        self.sweep_progress.setValue(value)

    def _on_sweep_done(self):
        self.busy = max(0, self.busy - 1)
        self.sweep_progress.setValue(0)
        self.lbl_sweep_status.setText("Bereit.")
        self._on_pose(dict(self.angles), None)

    # ==================================================================
    # SEQUENCER
    # ==================================================================
    def record_pose(self):
        self.seq.append(dict(self.angles))
        self._render_seq()

    def del_pose(self):
        row = self.seq_list.currentRow()
        if row >= 0 and row < len(self.seq):
            self.seq.pop(row)
            self._render_seq()

    def _render_seq(self):
        self.seq_list.clear()
        for i, p in enumerate(self.seq):
            self.seq_list.addItem(f"{i:02d}: " + ' '.join(f"{j[:1]}{p[j]}" for j in JOINT_ORDER))
        self.lbl_seq_status.setText(f"{len(self.seq)} Posen")

    def play_seq(self):
        if not self.seq or self.seq_playing or self.busy > 0:
            return
        try:
            delay = int(self.txt_seq_speed.text())
        except ValueError:
            delay = 200
        delay = max(30, min(2000, delay))
        self.seq_playing = True
        self.busy += 1
        self._abort.clear()
        self.seq_cancel.clear()
        self.lbl_seq_status.setText("Wiedergabe läuft…")

        def worker():
            try:
                for p in list(self.seq):
                    if self.seq_cancel.is_set() or self._abort.is_set():
                        break
                    self._send_angles(p)
                    self.sig_seq_step.emit(dict(p))
                    if not self._abortable_sleep(delay / 1000.0):
                        break
            finally:
                self.sig_seq_done.emit()

        threading.Thread(target=worker, daemon=True).start()

    def _on_seq_step(self, p):
        self._on_pose(dict(p), None)

    def _on_seq_done(self):
        self.seq_playing = False
        self.busy = max(0, self.busy - 1)
        self.lbl_seq_status.setText(f"{len(self.seq)} Posen")

    def save_seq(self):
        if not self.seq:
            QMessageBox.information(self, "Sequencer", "Noch keine Posen aufgenommen.")
            return
        os.makedirs(SEQUENCE_DIR, exist_ok=True)
        from PySide6.QtWidgets import QFileDialog
        fname, _ = QFileDialog.getSaveFileName(
            self, "Sequenz speichern", os.path.join(SEQUENCE_DIR, "sequence.json"),
            "JSON (*.json)")
        if fname:
            with open(fname, 'w', encoding='utf-8') as f:
                json.dump(self.seq, f, indent=2)
            self.log(f"Sequenz gespeichert: {os.path.basename(fname)}")

    def load_seq(self):
        from PySide6.QtWidgets import QFileDialog
        fname, _ = QFileDialog.getOpenFileName(
            self, "Sequenz laden", SEQUENCE_DIR, "JSON (*.json)")
        if fname:
            try:
                with open(fname, encoding='utf-8') as f:
                    data = json.load(f)
                self.seq = [{k: int(v) for k, v in p.items() if k in JOINT_ORDER} for p in data]
                self._render_seq()
            except Exception as e:
                QMessageBox.critical(self, "Ladefehler", str(e))

    # ==================================================================
    # MOTION RECORDER
    # ==================================================================
    def _record_manual(self):
        if not self.recording or self.busy != 0:
            return
        if self.motion_log and self.motion_log[-1][1] == self.angles:
            return
        t = (time.time() - self.rec_start) * 1000.0
        self.motion_log.append((t, dict(self.angles)))
        self.sig_recorder_status.emit()

    def toggle_record(self):
        if self.recording:
            self.recording = False
        else:
            self.motion_log = []
            self.rec_start = time.time()
            self.recording = True
        self._render_recorder()

    def _render_recorder(self):
        if self.recording:
            dur = time.time() - self.rec_start
            self.rec_btn.setText("■ AUFNAHME STOPPEN")
            self.rec_btn.setStyleSheet(_REC_STYLE)
            self.lbl_rec_status.setText(f"Aufnahme {dur:.1f} s · {len(self.motion_log)} Punkte")
        else:
            self.rec_btn.setText("● AUFNAHME")
            self.rec_btn.setStyleSheet(_REC_IDLE_STYLE)
            state = f"Bereit · {len(self.motion_log)} Punkte" + \
                    (" · läuft" if self.motion_playing else "")
            self.lbl_rec_status.setText(state)

    def play_motion(self):
        if not self.motion_log or self.motion_playing or self.busy > 0:
            return
        speed = self.rec_speed.value() / 100.0
        loop = self.chk_loop.isChecked()
        self.motion_playing = True
        self.busy += 1
        self._abort.clear()
        self.motion_pause.set()
        self.motion_cancel.clear()
        self.rec_progress.setValue(0)
        self._render_recorder()

        def worker(speed=speed, loop=loop):
            try:
                while True:
                    if self.motion_cancel.is_set() or self._abort.is_set():
                        break
                    total = self.motion_log[-1][0]
                    for n, (t, p) in enumerate(self.motion_log):
                        while not self.motion_pause.is_set():
                            if self.motion_cancel.is_set() or self._abort.is_set():
                                return
                            time.sleep(0.01)
                        if self.motion_cancel.is_set() or self._abort.is_set():
                            return
                        self._send_angles(p)
                        self.sig_motion_step.emit(n, t, dict(p), total)
                        delay = ((self.motion_log[n + 1][0] - t)
                                 if n + 1 < len(self.motion_log) else 0)
                        if not self._abortable_sleep((delay / 1000.0) / speed):
                            return
                    if not loop:
                        break
            finally:
                self.sig_motion_done.emit()

        threading.Thread(target=worker, daemon=True).start()

    def toggle_pause(self):
        if not self.motion_playing:
            return
        if self.motion_pause.is_set():
            self.motion_pause.clear()
            self.log("Wiedergabe pausiert.")
        else:
            self.motion_pause.set()
            self.log("Wiedergabe fortgesetzt.")

    def stop_motion(self):
        self.motion_cancel.set()
        self.motion_pause.set()

    def _on_motion_step(self, n, t, pos, total):
        if total:
            self.rec_progress.setValue(int(t / total * 1000))
        self._on_pose(dict(pos), None)

    def _on_motion_done(self):
        self.motion_playing = False
        self.busy = max(0, self.busy - 1)
        self.rec_progress.setValue(0)
        self._render_recorder()

    def save_motion(self):
        if not self.motion_log:
            QMessageBox.information(self, "Recorder", "Noch keine Bewegung aufgezeichnet.")
            return
        os.makedirs(SEQUENCE_DIR, exist_ok=True)
        from PySide6.QtWidgets import QFileDialog
        fname, _ = QFileDialog.getSaveFileName(
            self, "Bewegung speichern", os.path.join(SEQUENCE_DIR, "motion.json"),
            "JSON (*.json)")
        if fname:
            data = [{'t': round(t, 2), 'pos': {j: int(p[j]) for j in JOINT_ORDER}}
                    for t, p in self.motion_log]
            with open(fname, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            self.log(f"Bewegung gespeichert: {os.path.basename(fname)}")

    def load_motion(self):
        from PySide6.QtWidgets import QFileDialog
        fname, _ = QFileDialog.getOpenFileName(
            self, "Bewegung laden", SEQUENCE_DIR, "JSON (*.json)")
        if not fname:
            return
        try:
            with open(fname, encoding='utf-8') as f:
                data = json.load(f)
            self.motion_log = []
            t0 = data[0]['t'] if data else 0.0
            for item in data:
                p = {k: max(0, min(180, int(v))) for k, v in item['pos'].items()
                     if k in JOINT_ORDER}
                self.motion_log.append((float(item['t']) - t0, p))
            self._render_recorder()
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    # ==================================================================
    # PRESETS
    # ==================================================================
    def apply_preset(self):
        row = self.preset_list.currentRow()
        names = list(self.presets.keys())
        if row < 0 or row >= len(names):
            return
        name = names[row]
        self.angles = dict(self.presets[name])
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self.log(f"Preset angewendet: {name}")

    def save_preset(self):
        name = self.txt_preset_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Preset", "Bitte einen Namen für das Preset eingeben.")
            return
        self.presets[name] = dict(self.angles)
        self._render_presets()
        self._save_presets()
        self.txt_preset_name.setText("Posename")

    def delete_preset(self):
        row = self.preset_list.currentRow()
        names = list(self.presets.keys())
        if row < 0 or row >= len(names):
            return
        name = names[row]
        del self.presets[name]
        self._render_presets()
        self._save_presets()

    def _render_presets(self):
        self.preset_list.clear()
        for name in self.presets:
            self.preset_list.addItem(name)

    # ==================================================================
    # SETTINGS / PRESETS I/O
    # ==================================================================
    def _load_safe_home(self):
        raw = self.cfg.get('safety', 'safe_home', fallback='90,90,90,90,90,90')
        try:
            vals = [max(0, min(180, int(x))) for x in raw.split(',')]
            self.safe_home = {j: v for j, v in zip(JOINT_ORDER, vals)}
        except Exception:
            self.safe_home = dict(HOME)

    def _load_presets(self):
        self.presets = {}
        try:
            if os.path.exists(PRESET_FILE):
                with open(PRESET_FILE, encoding='utf-8') as f:
                    data = json.load(f)
                self.presets = {k: {j: max(0, min(180, int(v))) for j, v in p.items()
                                    if j in JOINT_ORDER} for k, p in data.items()}
        except Exception:
            self.presets = {}

    def _save_presets(self):
        try:
            with open(PRESET_FILE, 'w', encoding='utf-8') as f:
                json.dump(self.presets, f, indent=2)
            self.log(f"Presets gespeichert ({len(self.presets)}).")
        except Exception as e:
            QMessageBox.critical(self, "Presets-Fehler", str(e))

    def _save_safe_home(self):
        raw = self.txt_safe_home.text()
        try:
            vals = [max(0, min(180, int(x))) for x in raw.split(',')]
            self.safe_home = {j: v for j, v in zip(JOINT_ORDER, vals)}
            self.log("Safe-Home-Position gespeichert.")
        except Exception:
            QMessageBox.warning(self, "Fehler", "Ungültige Safe-Home-Werte.")

    def _on_step_angle_changed(self, val):
        self.step_angle = val

    def _on_baud_changed(self, val):
        self.baud = val

    def _apply_recorder_config(self):
        try:
            self.rec_speed.setValue(int(float(self.cfg.get('recorder', 'speed', fallback='1.0')) * 100))
        except Exception:
            pass
        try:
            self.chk_loop.setChecked(self.cfg.get('recorder', 'loop', fallback='false').lower() == 'true')
        except Exception:
            pass

    def _restore_window(self):
        g = self.cfg.get('window', 'geometry', fallback='1400x900')
        try:
            w, h = [int(x) for x in g.split('x')[:2]]
            self.resize(w, h)
        except Exception:
            pass

    def _save_window(self):
        try:
            self.cfg.set('window', 'geometry', f"{self.width()}x{self.height()}")
        except Exception:
            pass
        try:
            self.cfg.set('recorder', 'speed', f"{self.rec_speed.value() / 100.0:.2f}")
        except Exception:
            pass
        try:
            self.cfg.set('recorder', 'loop', 'true' if self.chk_loop.isChecked() else 'false')
        except Exception:
            pass
        try:
            self.cfg.set('serial', 'step_angle', str(self.step_angle))
        except Exception:
            pass
        try:
            self.cfg.set('serial', 'baud', str(self.baud))
        except Exception:
            pass
        try:
            self.cfg.set('serial', 'port', self.port_input.text().strip())
        except Exception:
            pass
        try:
            self.cfg.set('safety', 'safe_home',
                         ','.join(str(self.safe_home[j]) for j in JOINT_ORDER))
        except Exception:
            pass
        save_config(self.cfg)

    # ==================================================================
    # VERBINDUNG / SIMULATION / E-STOP
    # ==================================================================
    def toggle_connection(self):
        if self.manager is not None and not getattr(self.manager, 'sim', False):
            # Hardware-Verbindung trennen
            self.manager.close()
            self.manager = None
            self.btn_connect.setText("⚡ VERBINDEN")
            self.lbl_status.setText("● GETRENNT")
            self.log("Verbindung getrennt.")
            return
        port = self.port_input.text().strip() or 'COM6'
        try:
            self.manager = SerialManager(port, self.baud, parent=self)
        except Exception as e:
            QMessageBox.critical(self, "Verbindungsfehler",
                                 f"Port {port} nicht verfügbar: {e}")
            self.manager = None
            self.chk_sim.blockSignals(True)
            self.chk_sim.setChecked(False)
            self.chk_sim.blockSignals(False)
            return
        self.btn_connect.setText("⛔ TRENNEN")
        self.log(f"Verbindung zu {port} hergestellt.")

    def _on_sim_toggled(self, checked):
        if checked:
            if self.manager is not None and not getattr(self.manager, 'sim', False):
                self.manager.close()
            self.manager = SimulatedManager(parent=self)
            self.btn_connect.setText("⛔ TRENNEN")
            self.log("Simulationsmodus aktiviert (keine Hardware).")
            self.lbl_status.setText("● SIMULATION")
        else:
            # zurück zu Hardware versuchen
            if self.manager is not None and getattr(self.manager, 'sim', False):
                self.manager.close()
            self.manager = None
            self.btn_connect.setText("⚡ VERBINDEN")
            self.lbl_status.setText("● GETRENNT")
            self._try_hardware_after_sim_off()

    def _try_hardware_after_sim_off(self):
        port = self.port_input.text().strip() or 'COM6'
        try:
            self.manager = SerialManager(port, self.baud, parent=self)
            self.btn_connect.setText("⛔ TRENNEN")
            self.log(f"Verbindung zu {port} hergestellt.")
        except Exception as e:
            QMessageBox.critical(self, "Verbindungsfehler",
                                 f"Port {port} nicht verfügbar: {e}")
            self.manager = None
            # Schalter springt zurück in den Simulationsmodus
            self.chk_sim.blockSignals(True)
            self.chk_sim.setChecked(True)
            self.chk_sim.blockSignals(False)
            self.manager = SimulatedManager(parent=self)
            self.lbl_status.setText("● SIMULATION")

    def _update_conn_label(self):
        pass

    def estop(self):
        self._abort.set()
        self.sweep_cancel.set()
        self.seq_cancel.set()
        self.motion_cancel.set()
        self.motion_pause.set()
        self.loop_active = False
        self.busy = 0
        # Safe-Home anfahren
        self.angles = dict(self.safe_home)
        self.send_angles(self.angles)
        self.recalculate_fk_and_update()
        self.log("E-STOP: Bewegung gestoppt, Safe-Home angewendet")
        self.lbl_status.setText("⛔ E-STOP")
        self.lbl_status.setStyleSheet("color: #e0645a; font-weight: bold;")

    # ==================================================================
    # SCHLIESSEN
    # ==================================================================
    def closeEvent(self, event):
        self._save_window()
        # alle Threads stoppen
        self._abort.set()
        self.sweep_cancel.set()
        self.seq_cancel.set()
        self.motion_cancel.set()
        self.motion_pause.set()
        self.loop_active = False
        self._poll_timer.stop()
        if self.manager is not None:
            try:
                self.manager.close()
            except Exception:
                pass
        super().closeEvent(event)

    # ==================================================================
    # SCI-FI STYLESHEET
    # ==================================================================
    def apply_scifi_stylesheet(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #06060c; }
            QGroupBox {
                color: #00ffcc; font-size: 11px; font-weight: bold;
                border: 1px solid #00ffcc; border-radius: 6px;
                margin-top: 8px; padding-top: 12px; background-color: #0a0d18;
            }
            QLabel { color: #a0f0ff; font-weight: bold; font-family: Consolas, monospace; }
            QLineEdit, QComboBox, QTextEdit, QSpinBox {
                background-color: #03050a; color: #00ffcc;
                border: 1px solid #00a8ff; border-radius: 4px; padding: 4px;
                font-family: Consolas, monospace;
            }
            QPushButton {
                background-color: #0f172a; color: #00ffcc; font-weight: bold;
                border: 1px solid #00ffcc; padding: 6px; border-radius: 4px;
                font-family: Consolas, monospace;
            }
            QPushButton:hover { background-color: #00ffcc; color: #06060c; border: 1px solid #ffffff; }
            QPushButton:checked { background-color: #00ffcc; color: #06060c; border: 1px solid #ffffff; }
            QCheckBox { color: #00ffcc; font-weight: bold; }
            QListWidget {
                background-color: #0a0d18; color: #a0f0ff; font-weight: bold;
                border: 1px solid #00ffcc; border-radius: 6px;
                font-family: Consolas, monospace; font-size: 11px;
            }
            QListWidget::item:selected { background-color: #00ffcc; color: #06060c; }
            QListWidget::item:hover { background-color: #12203a; color: #00ffcc; }
            QProgressBar {
                border: 1px solid #00a8ff; border-radius: 4px; background-color: #1a2238;
                text-align: center; color: #00ffcc;
            }
            QProgressBar::chunk { background-color: #00ffcc; border-radius: 3px; }
            QStackedWidget { background-color: #06060c; }
            QSlider::groove:horizontal { height: 6px; background: #1a2238; border-radius: 3px; }
            QSlider::handle:horizontal { background: #ff0055; width: 16px; margin: -5px 0; border-radius: 8px; }
            QScrollBar:vertical { background: #0a0d18; width: 12px; }
            QScrollBar::handle:vertical { background: #00ffcc; border-radius: 5px; min-height: 20px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        """)


HOME = {j: 90 for j in JOINT_ORDER}

_REC_STYLE = ("QPushButton { background-color: #e8b84b; color: #14181d; font-weight: bold;"
              " border: 1px solid #e8b84b; padding: 6px; border-radius: 4px; }")
_REC_IDLE_STYLE = ("QPushButton { background-color: #0f172a; color: #00ffcc; font-weight: bold;"
                   " border: 1px solid #00ffcc; padding: 6px; border-radius: 4px; }")


def main():
    app = QApplication(sys.argv)
    window = SciFiRobotGUI()
    # Automatische Verbindung / Simulation
    port = window.port
    try:
        window.manager = SerialManager(port, window.baud, parent=window)
        window.btn_connect.setText("⛔ TRENNEN")
        window.log(f"Verbindung zu {port} hergestellt.")
    except Exception as e:
        print(f"Arduino nicht auf {port} gefunden ({e}); Simulationsmodus.")
        window.manager = SimulatedManager(parent=window)
        window.chk_sim.blockSignals(True)
        window.chk_sim.setChecked(True)
        window.chk_sim.blockSignals(False)
        window.lbl_status.setText("● SIMULATION")
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()