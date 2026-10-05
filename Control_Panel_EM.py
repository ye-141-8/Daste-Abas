"""
robot_arm_ui.py  --  Desktop control UI for a 6-DoF robot arm (Daste Abas)

Pure Python GUI built on CustomTkinter (a modern Tkinter wrapper). No HTML/CSS/JS.

Talk to the Arduino (firmware: Control_Panel.cpp) over a serial port.

Protocol (defined by Control_Panel.cpp):
    SEND : "P:base,finger,wrist,arm,elbow,dual\n"   (absolute position, 0-180 each)
    RECV : "POS:base,finger,wrist,arm,elbow,dual\n" (live angle echo)
    REQ  : "?\n"                                    (reply with a fresh POS: report)

Features
    * Modern dark UI (CustomTkinter)
    * 2D side (elevation) kinematics view with target/actual ghost
    * Keyboard shortcuts + sliders + a mouse-driven virtual joystick
    * Per-joint motor test sweep (0 -> 180 -> 0)
    * Motion recorder (auto-record manual movement + replay: loop/pause/speed)
    * Teach-in / sequencer with JSON import / export
    * Emergency stop (ESC) + configurable safe-home posture
    * Preset poses (presets.json)
    * On-screen console + persistent settings (config.ini)

Requirements:  pip install customtkinter pyserial
Requires hardware: the app refuses to start if the serial port is unavailable.
"""

import configparser
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, filedialog

import customtkinter as ctk

try:
    import serial
except ImportError:
    serial = None

ctk.set_appearance_mode('dark')
ctk.set_default_color_theme('blue')


# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
ARDUINO_PORT = 'COM6'          # COM port of the Arduino
BAUD_RATE = 9600               # must match Control_Panel.cpp
STEP_ANGLE = 5                 # degrees per keyboard/shortcut press

# Joint names in protocol order (must match Control_Panel.cpp "P:" format).
# key -> (display label, PCA9685 channel)  (channel only informational)
JOINTS = {
    'base':   ('Base',   6),
    'finger': ('Finger', 1),
    'wrist':  ('Wrist',  2),
    'arm':    ('Arm',    4),
    'elbow':  ('Elbow',  3),
    'dual':   ('Shoulder', 10),   # dual opposing servos on ch 10 & 11
}
JOINT_ORDER = list(JOINTS)

# Keyboard layout: joint -> { 'decrease': key, 'increase': key }
KEYMAP = {
    'base':   {'decrease': 'a', 'increase': 'd'},
    'finger': {'decrease': 'q', 'increase': 'e'},
    'wrist':  {'decrease': 'k', 'increase': 'j'},
    'arm':    {'decrease': 'o', 'increase': 'i'},
    'elbow':  {'decrease': 'n', 'increase': 'm'},
    'dual':   {'decrease': 's', 'increase': 'w'},
}

HOME = {j: 90 for j in JOINT_ORDER}

# Emergency-stop posture (configurable via config.ini)
SAFE_HOME = {j: 90 for j in JOINT_ORDER}

# Persistent settings / presets files (next to this script)
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.ini')
PRESET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'presets.json')

# ---------------------------------------------------------------------------
# Muted industrial palette
# ---------------------------------------------------------------------------
BG, PANEL, PANEL2, BORDER = '#171a21', '#20242e', '#2a2f3b', '#323947'
FG, FG_DIM = '#e8ecf3', '#9aa4b2'
ACCENT, ACCENT2 = '#2fc0a8', '#5b8cff'
OK, WARN, DANGER = '#34d399', '#e8b84b', '#e0645a'
PRESET = '#c678dd'                 # violet accent for the Presets tab
FONT = 'Segoe UI'


def load_config():
    cfg = configparser.ConfigParser()
    cfg['serial'] = {'port': ARDUINO_PORT, 'baud': str(BAUD_RATE),
                     'step_angle': str(STEP_ANGLE)}
    cfg['safety'] = {'safe_home': '90,90,90,90,90,90'}
    cfg['recorder'] = {'speed': '1.0', 'loop': 'false'}
    cfg['window'] = {'geometry': ''}
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
# SERIAL LAYER
# ===========================================================================
class SerialManager:
    """Owns the serial port, parses POS: feedback on a background thread."""

    def __init__(self, port, baud):
        if serial is None:
            raise RuntimeError('pyserial is not installed. Run: pip install pyserial')
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self._state = {j: 90 for j in JOINT_ORDER}
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self.connected = True
        self.last_pos_time = time.time()
        self._stop = threading.Event()
        self.log_q = queue.Queue()          # raw serial lines for the console panel
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
            if byte in (0x0A, 0x0D):            # \n or \r
                line = bytes(buf).decode('utf-8', 'ignore').strip()
                buf = bytearray()
                self._parse(line)
                if line:
                    self.log_q.put(line)
            else:
                buf.append(byte)
        self.connected = False

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

    def request_position(self):
        with self._write_lock:
            try:
                self.ser.write(b'?\n')
            except Exception:
                self.connected = False

    def close(self):
        self._stop.set()
        try:
            with self._write_lock:
                self.ser.close()
        except Exception:
            pass


class SimulatedManager:
    """Fake serial backend so the UI starts and runs fully without hardware.

    Mirrors the SerialManager API used by the GUI; commands simply update an
    in-memory position so the arm moves on screen for demo/testing.
    """
    sim = True

    def __init__(self, port='COM7 (no Arduino - simulation)'):
        self.port = port
        self.connected = True
        self.last_pos_time = time.time()
        self._state = {j: 90 for j in JOINT_ORDER}
        self.log_q = queue.Queue()

    def get_state(self):
        return dict(self._state), self.connected

    def send_abs(self, pos):
        self._state.update(pos)
        self.last_pos_time = time.time()

    def request_position(self):
        self.last_pos_time = time.time()

    def close(self):
        pass


# ===========================================================================
# KINEMATICS
# ===========================================================================
def unit(o):
    """Direction unit vector. `o` is an angle measured from +Y (up), clockwise."""
    return (math.sin(o), math.cos(o))


def forward_kinematics(pos):
    """Compute arm joint positions in world units (x right, y up). Returns a
    dict of labelled (x, y) points plus per-segment orientations (radians).

    Schematic side-view model: the physical arm has 6 DoF but only 4 visible
    planar links, so some joints are folded into the link angles.
    """
    L_upper, L_fore, L_hand, L_fing = 1.00, 0.90, 0.50, 0.32

    # Joint angles are relative to straight-up (= 90°). Unit direction is
    # measured clockwise from +Y (up), so 0° -> straight up, 90° -> right.
    base_o   = math.radians(pos['base'])
    upper    = math.radians(90 - pos['dual'])   + math.radians(90 - pos['arm']) * 0.5
    forearm  = upper + math.radians(90 - pos['elbow'])
    hand     = forearm + math.radians(90 - pos['wrist']) * 0.6

    shoulder = (0.0, 0.0)
    elbow    = _add(shoulder, scale(unit(upper), L_upper))
    wrist    = _add(elbow, scale(unit(forearm), L_fore))
    grip     = _add(wrist, scale(unit(hand), L_hand))

    spread = math.radians(-10 + pos['finger'] * (55.0 / 180.0))
    tip_l = _add(grip, scale(unit(hand - spread), L_fing))
    tip_r = _add(grip, scale(unit(hand + spread), L_fing))

    return {
        'shoulder': shoulder, 'elbow': elbow, 'wrist': wrist, 'grip': grip,
        'tip_l': tip_l, 'tip_r': tip_r,
        'base_o': base_o, 'upper': upper, 'forearm': forearm, 'hand': hand,
    }


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def scale(v, s):
    return (v[0] * s, v[1] * s)


# ===========================================================================
# GUI
# ===========================================================================
class RobotArmUI:
    def __init__(self, root, manager):
        self.root = root
        self.mgr = manager
        self.port = getattr(manager, 'port', ARDUINO_PORT)
        self.pos = dict(HOME)          # last commanded target
        self.display = dict(HOME)      # last confirmed (from POS:)
        self.connected = True

        self.busy = 0
        self.sweep_cancel = threading.Event()

        self.seq = []
        self.seq_playing = False
        self.seq_cancel = threading.Event()

        self.motion_log = []
        self.recording = False
        self.rec_start = 0.0
        self.motion_playing = False
        self.motion_pause = threading.Event()
        self.motion_pause.set()
        self.motion_cancel = threading.Event()

        self.joystick = {'active': False, 'vx': 0.0, 'vy': 0.0}
        self.joy_mode = None          # None / 'A' / 'B' / 'C' / 'D' (joystick routing)
        self._syncing = False
        self._gui_q = queue.Queue()

        self.cfg = load_config()
        self._load_safe_home()
        self._load_presets()

        self._build_ui()
        self._apply_recorder_config()
        self._restore_window()

        self.root.bind_all('<Key>', self.on_key)
        self.root.bind_all('<Escape>', lambda e: self._estop())
        self.root.focus_force()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self._poll()

    # ================================================================== UI
    def _build_ui(self):
        self.root.title('Daste Abas — Robot Arm Control')
        self.root.geometry('1180x900')
        self.root.minsize(1000, 680)
        self.root.configure(fg_color=BG)

        # ---- header -----------------------------------------------------
        header = ctk.CTkFrame(self.root, fg_color=PANEL, corner_radius=10)
        header.pack(fill='x', padx=10, pady=(8, 4))
        ctk.CTkLabel(header, text='DASTE ABAS', font=(FONT, 18, 'bold'),
                     text_color=ACCENT).pack(side='left', padx=12)
        ctk.CTkButton(header, text='E-STOP  [ESC]', command=self._estop,
                      fg_color=DANGER, hover_color='#c0392b', text_color='#ffffff',
                      font=(FONT, 13, 'bold'), corner_radius=8, width=130,
                      height=34).pack(side='left', padx=10)
        ctk.CTkButton(header, text='Re-sync', command=self.mgr.request_position,
                      fg_color=PANEL2, hover_color=BORDER, text_color=FG,
                      corner_radius=8).pack(side='left', padx=6)
        self.status = ctk.CTkLabel(header, text='connecting…',
                                   font=(FONT, 13), text_color=WARN)
        self.status.pack(side='right', padx=14)

        # ---- body: fixed left column + main area ----
        body = ctk.CTkFrame(self.root, fg_color=BG, corner_radius=0, bg_color='transparent')
        body.pack(fill='both', expand=True, padx=4, pady=(2, 0))

        col_w = ctk.CTkFrame(body, fg_color=PANEL, corner_radius=10, width=298)
        col_w.pack(side='left', fill='y', padx=(2, 2), pady=2)
        col_w.pack_propagate(False)
        self._build_joint_panel(col_w, 'JOINT CONTROL', accent=ACCENT)

        main = ctk.CTkFrame(body, fg_color=BG, corner_radius=0, bg_color='transparent')
        main.pack(side='left', fill='both', expand=True, padx=2, pady=2)

        # ---- 2D view: permanent, full main width ----
        canvas_frame = ctk.CTkFrame(main, fg_color=PANEL, corner_radius=10)
        canvas_frame.pack(fill='both', expand=True, padx=2, pady=(2, 2))
        self._build_canvas(canvas_frame)

        # ---- tools: one tab per function ----
        tab_frame = ctk.CTkFrame(main, fg_color=PANEL, corner_radius=10)
        tab_frame.pack(fill='both', padx=2, pady=(2, 2))
        self.tabs = ctk.CTkTabview(
            tab_frame, corner_radius=8,
            fg_color=PANEL2,
            segmented_button_fg_color=PANEL2,
            segmented_button_selected_color=ACCENT,          # strong selected fill
            segmented_button_selected_hover_color='#37a893',
            segmented_button_unselected_color=BORDER,
            segmented_button_unselected_hover_color='#3d4757',
            segmented_button_font=(FONT, 12, 'bold'),
            text_color=FG, text_color_disabled=BG,
            command=self._on_tab_change)
        self.tabs.pack(fill='both', expand=True, padx=6, pady=4)

        tabs = [('Motor Test', ACCENT2, '#252a35'),
                ('Motion Recorder', WARN, '#2a2721'),
                ('Sequencer', ACCENT, '#243630'),
                ('Presets', PRESET, '#2b2433')]
        self._tab_specs = {name: ac for name, ac, _ in tabs}
        for name, accent, tint in tabs:
            tab = self.tabs.add(name)
            tab.configure(fg_color=tint)
            self._tab_bar(tab, name, accent)
        self.tabs.set('Motor Test')
        self._build_test_panel(self.tabs._tab_dict['Motor Test'])
        self._build_recorder_panel(self.tabs._tab_dict['Motion Recorder'])
        self._build_seq_panel(self.tabs._tab_dict['Sequencer'])
        self._build_preset_panel(self.tabs._tab_dict['Presets'])

        self._build_console(self.root)

        ctk.CTkLabel(self.root, text='Keys:  A/D Base · Q/E Finger · K/J Wrist · O/I Arm · N/M Elbow · S/W Shoulder   ·   ESC = E-Stop',
                     font=(FONT, 11), text_color=FG_DIM).pack(fill='x', padx=12)

    # -- generic section helpers ------------------------------------------
    @staticmethod
    def _card_title(parent, text, accent=ACCENT):
        ctk.CTkLabel(parent, text=text.upper(), font=(FONT, 12, 'bold'),
                     text_color=accent).pack(anchor='w', padx=12, pady=(10, 4))

    @staticmethod
    def _tab_bar(parent, text, accent):
        """Colored banner inside a tab; gives each tab its own style."""
        bar = ctk.CTkFrame(parent, fg_color=accent, corner_radius=6, height=30)
        bar.pack(fill='x', pady=(6, 2))
        bar.pack_propagate(False)
        ctk.CTkLabel(bar, text=f'  {text.upper()}  ', text_color='#ffffff',
                     font=(FONT, 12, 'bold')).pack()

    def _on_tab_change(self, _name):
        pass

    # -- joint sliders + numeric + + / - ------------------------------------
    def _build_joint_panel(self, parent, title, accent):
        self._card_title(parent, title, accent)
        self.vars = {}
        self.bars = {}
        self.entries = {}
        self.val_lab = {}

        for j in JOINT_ORDER:
            label, ch = JOINTS[j]
            row = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
            row.pack(fill='x', padx=8, pady=3)

            ctk.CTkLabel(row, text=f'{label}', width=58, text_color=FG,
                         font=(FONT, 11)).pack(side='left', padx=4)

            self.vars[j] = tk.IntVar(value=90)
            self.val_lab[j] = ctk.CTkLabel(row, text='90', width=30,
                                           text_color=OK, font=(FONT, 12, 'bold'))
            self.val_lab[j].pack(side='right', padx=2)

            bar = ctk.CTkSlider(row, from_=0, to=180, command=lambda v, k=j: self.on_slider(k, v),
                                fg_color=BORDER, progress_color=accent,
                                button_color=accent, button_hover_color=accent)
            self.bars[j] = bar
            bar.set(90)
            bar.pack(side='right', fill='x', expand=True, padx=(4, 2))

            ent = ctk.CTkEntry(row, width=46, justify='center', text_color=OK,
                               fg_color=PANEL2, border_color=BORDER,
                               font=(FONT, 11))
            ent.insert(0, '90')
            ent.bind('<Return>', lambda e, k=j, w=ent: self.on_entry(k, w))
            self.entries[j] = ent
            ent.pack(side='right', padx=2)

        homestance = ctk.CTkFrame(parent, fg_color='transparent')
        homestance.pack(fill='x', padx=8, pady=(8, 2))
        ctk.CTkButton(homestance, text='HOME (90°)', command=self.go_home,
                      fg_color=PANEL2, hover_color=ACCENT2, text_color=ACCENT2,
                      corner_radius=8).pack(fill='x')

        self._card_title(parent, 'Virtual Joystick', ACCENT2)

        joy_zone = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        joy_zone.pack(fill='x', padx=8, pady=2)
        self.joy_canvas = tk.Canvas(joy_zone, width=210, height=150, bg='#0f1216',
                                    highlightthickness=0)
        self.joy_canvas.pack(side='left', padx=2, pady=2)
        self.joy_canvas.bind('<ButtonPress-1>', self.joy_press)
        self.joy_canvas.bind('<B1-Motion>', self.joy_drag)
        self.joy_canvas.bind('<ButtonRelease-1>', self.joy_release)
        self.joy_center = (105, 75)
        self.joy_radius = 42
        self.joy_canvas.create_oval(*self._joy_circle(self.joy_center, self.joy_radius),
                                    outline='#3a4751', width=2)
        self.joy_canvas.create_oval(self.joy_center[0] - 3, self.joy_center[1] - 3,
                                    self.joy_center[0] + 3, self.joy_center[1] + 3,
                                    fill='#2fc0a8', outline='')
        self.joy_knob = self.joy_canvas.create_oval(*self._joy_circle(self.joy_center, 18),
                                                    fill='#2fc0a8', outline='#2fc0a8')

        # --- buttons A/B/C/D (mirror firmware joystick routing) ---
        btns = ctk.CTkFrame(joy_zone, fg_color='transparent')
        btns.pack(side='left', fill='y', padx=4)
        ctk.CTkLabel(btns, text='JOYSTICK', text_color=FG_DIM, font=(FONT, 9)).pack(pady=(6, 2))
        self.joy_btns = {}
        for _id in ('A', 'B', 'C', 'D'):
            btn = ctk.CTkButton(
                btns, text=_id, width=34, height=30, font=(FONT, 12, 'bold'),
                command=lambda k=_id: self._set_joy_mode(k),
                fg_color=BORDER, hover_color=PANEL2, text_color=FG_DIM)
            btn.pack(pady=3, fill='x')
            self.joy_btns[_id] = btn
        self.joy_btn_lab = {'meta': ctk.CTkLabel(btns, text='', text_color=ACCENT,
                                                 font=(FONT, 8, 'bold'), wraplength=150,
                                                 justify='center')}
        self.joy_btn_lab['meta'].pack(pady=(3, 2))
        self.joy_mode = None
        self._render_joy_mode()

    def _joy_circle(self, c, r):
        x, y = c
        return (x - r, y - r, x + r, y + r)

    def joy_press(self, event):
        self.joystick['active'] = True
        self._joy_move(event)

    def joy_drag(self, event):
        if self.joystick['active']:
            self._joy_move(event)

    def joy_release(self, _):
        self.joystick['active'] = False
        self.joystick['vx'] = 0
        self.joystick['vy'] = 0
        self.joy_canvas.coords(self.joy_knob, *self._joy_circle(self.joy_center, 18))

    def _joy_move(self, event):
        cx, cy = self.joy_center
        dx, dy = event.x - cx, event.y - cy
        dist = math.hypot(dx, dy)
        if dist > self.joy_radius:
            dx, dy = dx / dist * self.joy_radius, dy / dist * self.joy_radius
        self.joystick['vx'] = dx / self.joy_radius
        self.joystick['vy'] = dy / self.joy_radius
        self.joy_canvas.coords(self.joy_knob,
                               *(cx + dx - 18, cy + dy - 18, cx + dx + 18, cy + dy + 18))

    def _set_joy_mode(self, mode):
        self.joy_mode = None if self.joy_mode == mode else mode
        self._render_joy_mode()
        self._log(f'joystick mode: {self._joy_mode_label()}')

    def _joy_mode_label(self):
        if self.joy_mode:
            return f'{self.joy_mode}: ' + {'A': 'X→Wrist', 'B': 'Y→Elbow',
                                           'C': 'X→Finger', 'D': 'Y→Arm'}[self.joy_mode]
        return 'none: X→Base / Y→Shoulder'

    def _render_joy_mode(self):
        for _id, btn in getattr(self, 'joy_btns', {}).items():
            active = (self.joy_mode == _id)
            btn.configure(fg_color=ACCENT if active else BORDER,
                          text_color='#ffffff' if active else FG_DIM)
        # small caption under the buttons block
        lbl = self.joy_btn_lab.get('meta')
        if lbl is not None:
            lbl.configure(text=self._joy_mode_label())

    # ============================================================== CANVAS
    def _build_canvas(self, parent):
        self._card_title(parent, '2D Side View', ACCENT)
        self.canvas = tk.Canvas(parent, bg='#0f1216', highlightthickness=0, height=360)
        self.canvas.pack(fill='both', expand=True, padx=6, pady=2)

    def _draw_arm(self):
        c = self.canvas
        c.delete('all')
        w = c.winfo_width() if c.winfo_width() > 10 else 720
        h = c.winfo_height() if c.winfo_height() > 10 else 420

        tower_h = 0.78
        half_base = 0.40          # half width of tower + rotation disc in world units
        pts_key = ('shoulder', 'elbow', 'wrist', 'grip', 'tip_l', 'tip_r')

        kin_t = forward_kinematics(self.pos)      # target (dim ghost)
        kin_a = forward_kinematics(self.display)  # actual (solid)

        # ---- world-space bounding box (arm + tower + base disc) ----
        xs, ys = [0.0], [0.0, -tower_h, -half_base, half_base]
        for k in pts_key:
            xs.append(kin_a[k][0]); ys.append(kin_a[k][1])
            xs.append(kin_t[k][0]); ys.append(kin_t[k][1])
        minX, maxX = min(xs), max(xs)
        minY, maxY = min(ys), max(ys)
        maxY = max(maxY, 0.0)

        # ---- auto-fit isotropic scale + margins ----
        hpad, top_pad, bot_pad = 34.0, 34.0, 26.0
        sx = (w - 2 * hpad) / (maxX - minX)
        sy = (h - top_pad - bot_pad) / (maxY - minY)
        S = max(2.0, min(sx, sy))

        ground_y = h - bot_pad
        center_x = (minX + maxX) / 2.0
        base_x = w / 2.0 - center_x * S        # screen x of world x = 0
        base_x = max(base_x, hpad * 1.5)

        def P(pt):
            # world x centered; world y up -> screen y down (minY sits on ground)
            return (w / 2.0 + (pt[0] - center_x) * S,
                    ground_y - (pt[1] - minY) * S)

        # ---- subtle grid + ground ----
        for gx in range(int(hpad), w, 40):
            c.create_line(gx, top_pad, gx, ground_y, fill='#141a20', width=1)
        for gy in range(int(top_pad), int(ground_y), 40):
            c.create_line(0, gy, w, gy, fill='#141a20', width=1)
        c.create_line(0, ground_y, w, ground_y, fill='#3a4751', width=2)

        shoulder_y = P((0.0, 0.0))[1]
        self._draw_base(c, base_x, shoulder_y, ground_y, S, kin_t)

        # target ghost first (drawn under)
        self._draw_armature(c, kin_t, P, seg_colors=['#39424f', '#39424f', '#39424f', '#39424f'],
                            width_s=S * 0.11, dash=(3, 4), joints=False, labels=False)

        # actual arm
        seg = ['#6f9ec2', '#4f86ab', '#8a9ab0', '#b6b1a2']
        self._draw_armature(c, kin_a, P, seg_colors=seg, width_s=S * 0.15,
                            joints=True, labels=True, S=S)

        # readout line
        s = self.pos
        c.create_text(w * 0.5, 18, anchor='n',
                      text=f'EL {s["elbow"]}°   WRIST {s["wrist"]}°   ARM {s["arm"]}°   SHLDR {s["dual"]}°   FINGER {s["finger"]}°',
                      fill='#e8ecf3', font=(FONT, 11))

    def _draw_base(self, c, bx, shoulder_y, ground_y, S, kin):
        # tower
        c.create_rectangle(bx - 0.30 * S, shoulder_y, bx + 0.30 * S, ground_y,
                           fill='#2e3542', outline='#4a5568', width=2)
        # base plate
        c.create_rectangle(bx - 0.55 * S, ground_y - 4, bx + 0.55 * S, ground_y,
                           fill='#39424f', outline='#4a5568', width=1)
        # shoulder pivot ball
        r = 0.10 * S
        c.create_oval(bx - r, shoulder_y - r, bx + r, shoulder_y + r,
                      fill='#c7cede', outline='#9fb2c4', width=2)
        # base rotation disc (top-down indicator)
        dr = 0.30 * S
        cy = shoulder_y - 0.34 * S
        c.create_oval(bx - dr, cy - dr, bx + dr, cy + dr, outline='#4a5568',
                      width=2, fill='#20242e')
        ao = -kin['base_o']
        ax = bx + (dr - 4) * math.sin(ao)
        ay = cy - (dr - 4) * math.cos(ao)
        c.create_line(bx, cy, ax, ay, fill='#e8b84b', width=3, arrow='last')
        c.create_text(bx, cy + dr + 14, text=f'BASE {int(self.pos["base"])}°',
                      fill='#e8b84b', font=(FONT, 10, 'bold'))

    def _draw_armature(self, c, kin, P, seg_colors, width_s, dash=None,
                       joints=True, labels=True, S=1.0):
        seq = [('shoulder', 'elbow'), ('elbow', 'wrist'), ('wrist', 'grip'),
               ('grip', 'tip_l')]
        names = ['UPPER', 'FOREARM', 'HAND', 'FINGER']
        for i, (a, b) in enumerate(seq):
            kw = {}
            if dash:
                kw = {'fill': '#6b788a', 'dash': dash}
            else:
                kw = {'fill': seg_colors[i]}
            c.create_line(*P(kin[a]), *P(kin[b]), width=width_s, capstyle='round', **kw)
            if labels and not dash:
                mx, my = (kin[a][0] + kin[b][0]) / 2, (kin[a][1] + kin[b][1]) / 2
                c.create_text(*P((mx, my)), text=names[i], fill='#9aa4b2',
                              font=(FONT, 8, 'italic'))

        # gripper second finger
        c.create_line(*P(kin['grip']), *P(kin['tip_r']), width=width_s * 0.8,
                      capstyle='round', fill=seg_colors[3])

        if joints:
            for jx, lab in (('elbow', f'EL {self.display["elbow"]}°'),
                            ('wrist', f'WR {self.display["wrist"]}°'),
                            ('grip', f'GR {self.display["finger"]}°')):
                x, y = P(kin[jx])
                r = max(5, 0.045 * S)
                c.create_oval(x - r, y - r, x + r, y + r, fill='#d4d9e2',
                              outline='#9fb2c4', width=1.5)
                c.create_rectangle(x - 26, y - 30, x + 26, y - 6, fill='#20242e',
                                   outline='#3a4751')
                c.create_text(x, y - 18, text=lab, fill='#e8ecf3', font=(FONT, 8, 'bold'))

    # ============================================================== TEST
    def _build_test_panel(self, parent):
        row = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        row.pack(fill='x', padx=10, pady=(6, 0))
        ctk.CTkLabel(row, text='Joint', text_color=FG_DIM, font=(FONT, 11)).pack(side='left', padx=6)
        self.sweep_var = tk.StringVar(value='dual')
        ctk.CTkOptionMenu(row, variable=self.sweep_var, values=JOINT_ORDER,
                          fg_color=PANEL2, button_color=BORDER, text_color=FG,
                          font=(FONT, 11), width=100).pack(side='left', padx=6)
        ctk.CTkLabel(row, text='ms', text_color=FG_DIM).pack(side='left', padx=4)
        self.sweep_speed = ctk.CTkEntry(row, width=54, justify='center',
                                        text_color=FG, fg_color=PANEL2,
                                        border_color=BORDER)
        self.sweep_speed.insert(0, '15')
        self.sweep_speed.pack(side='left', padx=4)

        btn_row = ctk.CTkFrame(parent, fg_color='transparent')
        btn_row.pack(fill='x', padx=10, pady=4)
        ctk.CTkButton(btn_row, text='Run Sweep 0→180→0', command=self.run_sweep,
                      fg_color=OK, hover_color='#2aa37d', text_color='#0b0f12',
                      corner_radius=8).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(btn_row, text='Stop', command=self.sweep_cancel.set,
                      fg_color=DANGER, hover_color='#c0392b', text_color='#fff',
                      corner_radius=8).pack(side='left', padx=2)

        self.sweep_progress = ctk.CTkProgressBar(parent, height=12, corner_radius=6,
                                                 fg_color=BORDER, progress_color=ACCENT2)
        self.sweep_progress.pack(fill='x', padx=10, pady=2)
        self.sweep_progress.set(0)
        self.sweep_status = ctk.CTkLabel(parent, text='idle', text_color=FG_DIM,
                                         font=(FONT, 10))
        self.sweep_status.pack(anchor='w', padx=12)

    # ============================================================== SEQ
    def _build_seq_panel(self, parent):
        t = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        t.pack(fill='x', padx=10, pady=(6, 0))
        ctk.CTkButton(t, text='Record Pose', command=self.record_pose,
                      fg_color=WARN, hover_color='#cf9f35', text_color='#14181d',
                      corner_radius=8).pack(side='left', fill='x', expand=True)

        sp = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        sp.pack(fill='x', padx=10, pady=2)
        ctk.CTkLabel(sp, text='Speed (ms)', text_color=FG_DIM).pack(side='left', padx=6)
        self.seq_speed = ctk.CTkEntry(sp, width=54, justify='center', text_color=FG,
                                      fg_color=PANEL2, border_color=BORDER)
        self.seq_speed.insert(0, '200')
        self.seq_speed.pack(side='left', padx=4)

        b2 = ctk.CTkFrame(parent, fg_color='transparent')
        b2.pack(fill='x', padx=10, pady=2)
        ctk.CTkButton(b2, text='Play', command=self.play_seq, fg_color=OK,
                      hover_color='#2aa37d', corner_radius=8).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(b2, text='Stop', command=self.seq_cancel.set, fg_color=DANGER,
                      hover_color='#c0392b').pack(side='left', fill='x', expand=True, padx=2)

        b3 = ctk.CTkFrame(parent, fg_color='transparent')
        b3.pack(fill='x', padx=10, pady=2)
        ctk.CTkButton(b3, text='Save JSON', command=self.save_seq, fg_color=PANEL2,
                      hover_color=BORDER, text_color=FG).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(b3, text='Load JSON', command=self.load_seq, fg_color=PANEL2,
                      hover_color=BORDER, text_color=FG).pack(side='left', fill='x', expand=True, padx=2)

        self._card_title(parent, 'Waypoints', FG_DIM, )
        self.seq_list = self._make_list(parent, 4)
        ctk.CTkButton(parent, text='Delete Selected', command=self.del_pose,
                      fg_color=DANGER, hover_color='#c0392b').pack(fill='x', padx=10, pady=2)
        self.seq_status = ctk.CTkLabel(parent, text='0 poses', text_color=FG_DIM,
                                       font=(FONT, 10))
        self.seq_status.pack(anchor='e', padx=12)

    def _make_list(self, parent, height):
        f = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        f.pack(fill='x', padx=10, pady=(2, 0))
        lb = tk.Listbox(f, height=height, bg='#14181d', fg='#d6dbe3',
                        selectbackground='#2f3a47', selectforeground='#ffffff',
                        highlightthickness=0, relief='flat', font=(FONT, 9))
        sb = ctk.CTkScrollbar(f, command=lb.yview)
        lb.configure(yscrollcommand=sb.set)
        lb.pack(side='left', fill='both', expand=True, padx=(2, 0))
        sb.pack(side='right', fill='y')
        return lb

    # ============================================================== RECORD
    def _build_recorder_panel(self, parent):
        self.rec_btn = ctk.CTkButton(parent, text='● Record', command=self.toggle_record,
                                     fg_color=DANGER, hover_color='#c0392b',
                                     text_color='#fff', font=(FONT, 12, 'bold'))
        self.rec_btn.pack(fill='x', padx=10, pady=(6, 0))
        self.rec_status = ctk.CTkLabel(parent, text='idle', text_color=FG_DIM,
                                       font=(FONT, 10))
        self.rec_status.pack(anchor='w', padx=12)

        pr = ctk.CTkFrame(parent, fg_color='transparent')
        pr.pack(fill='x', padx=10, pady=2)
        ctk.CTkButton(pr, text='Play', command=self.play_motion, fg_color=OK,
                      hover_color='#2aa37d', corner_radius=8).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(pr, text='Pause', command=self.toggle_pause, fg_color=ACCENT2,
                      hover_color=BORDER, corner_radius=8).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(pr, text='Stop', command=self.stop_motion, fg_color=DANGER,
                      hover_color='#c0392b', corner_radius=8).pack(side='left', fill='x', expand=True, padx=2)

        opt = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        opt.pack(fill='x', padx=10, pady=2)
        self.loop_var = tk.BooleanVar(value=False)
        ctk.CTkSwitch(opt, text='Loop', variable=self.loop_var, text_color=FG,
                      progress_color=ACCENT).pack(side='left', padx=6)
        ctk.CTkLabel(opt, text='Speed', text_color=FG_DIM).pack(side='left', padx=8)
        self.speed_var = tk.StringVar(value='1.00×')
        self.rec_speed = ctk.CTkSlider(opt, from_=0.25, to=4.0,
                                       command=lambda v: self.speed_var.set(f'{v:.2f}×'),
                                       progress_color=WARN, button_color=WARN,
                                       button_hover_color=WARN)
        self.rec_speed.set(1.0)
        self.rec_speed.pack(side='left', fill='x', expand=True, padx=(4, 2))
        ctk.CTkLabel(opt, textvariable=self.speed_var, width=50, text_color=OK).pack(side='left')

        self.rec_progress = ctk.CTkProgressBar(parent, height=12, corner_radius=6,
                                               fg_color=BORDER, progress_color=WARN)
        self.rec_progress.pack(fill='x', padx=10, pady=2)
        self.rec_progress.set(0)

        io = ctk.CTkFrame(parent, fg_color='transparent')
        io.pack(fill='x', padx=10, pady=2)
        ctk.CTkButton(io, text='Save JSON', command=self.save_motion, fg_color=PANEL2,
                      hover_color=BORDER, text_color=FG).pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(io, text='Load JSON', command=self.load_motion, fg_color=PANEL2,
                      hover_color=BORDER, text_color=FG).pack(side='left', fill='x', expand=True, padx=2)

    # ============================================================== PRESET
    def _build_preset_panel(self, parent):
        nr = ctk.CTkFrame(parent, fg_color=PANEL2, corner_radius=8)
        nr.pack(fill='x', padx=10, pady=(6, 0))
        self.preset_entry = ctk.CTkEntry(nr, fg_color=PANEL2, text_color=FG,
                                         border_color=BORDER, font=(FONT, 11))
        self.preset_entry.insert(0, 'pose name')
        self.preset_entry.pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(nr, text='Save', command=self.save_preset, fg_color=ACCENT2,
                      hover_color=BORDER).pack(side='left', padx=2)
        self.preset_list = self._make_list(parent, 3)
        self._render_presets()
        br = ctk.CTkFrame(parent, fg_color='transparent')
        br.pack(fill='x', padx=10, pady=2)
        ctk.CTkButton(br, text='Apply', command=self.apply_preset, fg_color=OK,
                      hover_color='#2aa37d').pack(side='left', fill='x', expand=True, padx=2)
        ctk.CTkButton(br, text='Delete', command=self.delete_preset, fg_color=DANGER,
                      hover_color='#c0392b').pack(side='left', fill='x', expand=True, padx=2)

    # ============================================================== CONSOLE
    def _build_console(self, parent):
        wrap = ctk.CTkFrame(parent, fg_color=PANEL, corner_radius=10)
        wrap.pack(fill='x', padx=10, pady=2, side='bottom')
        ctk.CTkLabel(wrap, text='CONSOLE', font=(FONT, 11, 'bold'),
                     text_color=ACCENT2).pack(anchor='w', padx=10, pady=(2, 0))
        self.console = ctk.CTkTextbox(wrap, height=90, fg_color='#0f1216',
                                      text_color='#c3cad6', font=(FONT, 10),
                                      border_width=0, activate_scrollbars=True)
        self.console.pack(fill='both', expand=True, padx=6, pady=(2, 2))
        self.console.insert('1.0', 'System ready — move the arm or open serial console.\n')

    # ============================================================== LOGIC
    def _update_joint_ui(self, j, value):
        self.vars[j].set(value)
        self._syncing = True
        try:
            self.bars[j].set(value)
        finally:
            self._syncing = False
        self.val_lab[j].configure(text=str(value))

    def _apply_pos(self):
        if self.busy == 0:
            self.mgr.send_abs(self.pos)
        for j in JOINT_ORDER:
            self._update_joint_ui(j, int(self.pos[j]))
        self._record_manual()

    def _record_manual(self):
        if not self.recording or self.busy != 0:
            return
        if self.motion_log and self.motion_log[-1][1] == self.pos:
            return
        t = (time.time() - self.rec_start) * 1000.0
        self.motion_log.append((t, dict(self.pos)))
        self._render_recorder()

    def on_entry(self, joint, widget):
        try:
            v = int(widget.get())
        except ValueError:
            return
        self.pos[joint] = max(0, min(180, v))
        self._apply_pos()

    def nudge(self, joint, sign):
        self.pos[joint] = max(0, min(180, self.pos[joint] + sign * STEP_ANGLE))
        self._apply_pos()

    def on_slider(self, joint, value):
        if self._syncing:
            return
        self.pos[joint] = max(0, min(180, int(float(value))))
        self.val_lab[joint].configure(text=str(self.pos[joint]))
        if self.busy == 0:
            self.mgr.send_abs(self.pos)
        self._record_manual()

    def go_home(self):
        self.pos = dict(HOME)
        self._apply_pos()

    def on_key(self, event):
        key = (event.char or '').lower()
        if not key or self.busy > 0:
            return
        w = self.root.focus_get()
        if isinstance(w, ctk.CTkEntry):
            return
        for j, mp in KEYMAP.items():
            if key == mp['decrease']:
                self.nudge(j, -1); return
            if key == mp['increase']:
                self.nudge(j, +1); return

    # ---- sweep ----
    def run_sweep(self):
        if self.busy > 0:
            return
        joint = self.sweep_var.get()
        try:
            delay = int(self.sweep_speed.get())
        except ValueError:
            delay = 15
        delay = max(5, min(200, delay))
        self.busy += 1
        self.sweep_status.configure(text=f'sweeping {joint}…', text_color=WARN)
        self.sweep_cancel.clear()

        def worker():
            base = dict(self.pos)
            for target in list(range(0, 181, 5)) + list(range(180, -1, -5)):
                if self.sweep_cancel.is_set():
                    break
                p = dict(base); p[joint] = target
                self.mgr.send_abs(p)
                self.pos[joint] = target
                self._emit(self._ui_sweep_update, target)
                time.sleep(delay / 1000.0)
            self.mgr.send_abs(base)
            self._emit(self._ui_sweep_done)

        threading.Thread(target=worker, daemon=True).start()

    def _ui_sweep_update(self, value):
        self.sweep_progress.set(value / 180.0)
        self._update_joint_ui(self.sweep_var.get(), value)

    def _ui_sweep_done(self):
        self.busy = max(0, self.busy - 1)
        self.sweep_progress.set(0)
        self.sweep_status.configure(text='idle', text_color=FG_DIM)
        self._apply_pos()

    # ---- sequencer ----
    def record_pose(self):
        self.seq.append(dict(self.pos))
        self._render_seq()

    def del_pose(self):
        sel = self.seq_list.curselection()
        if sel:
            self.seq.pop(sel[0])
            self._render_seq()

    def _render_seq(self):
        self.seq_list.delete(0, 'end')
        for i, p in enumerate(self.seq):
            self.seq_list.insert('end', f'{i:02d}: ' + ' '.join(f'{j[:1]}{p[j]}' for j in JOINT_ORDER))
        self.seq_status.configure(text=f'{len(self.seq)} poses')

    def play_seq(self):
        if not self.seq or self.seq_playing or self.busy > 0:
            return
        try:
            delay = int(self.seq_speed.get())
        except ValueError:
            delay = 200
        delay = max(30, min(2000, delay))
        self.seq_playing = True
        self.busy += 1
        self.seq_cancel.clear()
        self.seq_status.configure(text='playing…', text_color=WARN)

        def worker():
            try:
                for p in self.seq:
                    if self.seq_cancel.is_set():
                        break
                    self.mgr.send_abs(p)
                    self.pos = dict(p)
                    self._emit(self._ui_seq_step, dict(p))
                    time.sleep(delay / 1000.0)
            finally:
                self._emit(self._ui_seq_done)

        threading.Thread(target=worker, daemon=True).start()

    def _ui_seq_step(self, p):
        for j in JOINT_ORDER:
            self._update_joint_ui(j, p[j])

    def _ui_seq_done(self):
        self.seq_playing = False
        self.busy = max(0, self.busy - 1)
        self.seq_status.configure(text=f'{len(self.seq)} poses', text_color=FG_DIM)

    def save_seq(self):
        if not self.seq:
            messagebox.showinfo('Sequencer', 'No poses recorded yet.')
            return
        os.makedirs('sequences', exist_ok=True)
        fname = filedialog.asksaveasfilename(
            defaultextension='.json', initialdir='sequences',
            filetypes=[('JSON', '*.json')], initialfile='sequence.json')
        if fname:
            with open(fname, 'w', encoding='utf-8') as f:
                json.dump(self.seq, f, indent=2)
            self.seq_status.configure(text=f'saved: {os.path.basename(fname)}')

    def load_seq(self):
        fname = filedialog.askopenfilename(
            defaultextension='.json', initialdir='sequences', filetypes=[('JSON', '*.json')])
        if fname:
            try:
                with open(fname, encoding='utf-8') as f:
                    data = json.load(f)
                self.seq = [{k: int(v) for k, v in p.items() if k in JOINTS} for p in data]
                self._render_seq()
            except Exception as e:
                messagebox.showerror('Load error', str(e))

    # ---- recorder ----
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
            self.rec_btn.configure(text='■ Stop Recording', fg_color=WARN, hover_color='#cf9f35')
            self.rec_status.configure(text=f'recording {dur:.1f} s · {len(self.motion_log)} pts',
                                      text_color=WARN)
        else:
            self.rec_btn.configure(text='● Record', fg_color=DANGER, hover_color='#c0392b')
            state = f'idle · {len(self.motion_log)} pts' + (' · playing' if self.motion_playing else '')
            self.rec_status.configure(text=state, text_color=FG_DIM)

    def play_motion(self):
        if not self.motion_log or self.motion_playing or self.busy > 0:
            return
        speed = float(self.rec_speed.get())
        loop = bool(self.loop_var.get())
        self.motion_playing = True
        self.busy += 1
        self.motion_pause.set()
        self.motion_cancel.clear()
        self.rec_progress.set(0)
        self._render_recorder()

        def worker(speed=speed, loop=loop):
            try:
                while True:
                    if self.motion_cancel.is_set():
                        break
                    total = self.motion_log[-1][0]
                    for n, (t, p) in enumerate(self.motion_log):
                        while not self.motion_pause.is_set():
                            if self.motion_cancel.is_set():
                                return
                            time.sleep(0.01)
                        if self.motion_cancel.is_set():
                            return
                        self.mgr.send_abs(p)
                        self.pos = dict(p)
                        self._emit(self._ui_motion_step, n, t, dict(p), total)
                        delay = ((self.motion_log[n + 1][0] - t)
                                 if n + 1 < len(self.motion_log) else 0)
                        self._sleep((delay / 1000.0) / speed)
                    if not loop:
                        break
            finally:
                self._emit(self._ui_motion_done)

        threading.Thread(target=worker, daemon=True).start()

    def _sleep(self, seconds):
        # Always yield at least a short frame so a fast/looping playback can
        # never starve the Tk main thread.
        seconds = max(0.002, seconds)
        end = time.time() + seconds
        while time.time() < end:
            if self.motion_cancel.is_set():
                return
            time.sleep(min(0.01, max(0.001, end - time.time())))

    def toggle_pause(self):
        if not self.motion_playing:
            return
        if self.motion_pause.is_set():
            self.motion_pause.clear()
        else:
            self.motion_pause.set()

    def stop_motion(self):
        self.motion_cancel.set()
        self.motion_pause.set()

    def _ui_motion_step(self, n, t, pos, total):
        if total:
            self.rec_progress.set(t / total)
        for j in JOINT_ORDER:
            self._update_joint_ui(j, pos[j])

    def _ui_motion_done(self):
        self.motion_playing = False
        self.busy = max(0, self.busy - 1)
        self.rec_progress.set(0)
        self._render_recorder()

    def save_motion(self):
        if not self.motion_log:
            messagebox.showinfo('Recorder', 'No motion recorded yet.')
            return
        os.makedirs('sequences', exist_ok=True)
        fname = filedialog.asksaveasfilename(
            defaultextension='.json', initialdir='sequences',
            filetypes=[('JSON', '*.json')], initialfile='motion.json')
        if fname:
            data = [{'t': round(t, 2), 'pos': {j: int(p[j]) for j in JOINT_ORDER}}
                    for t, p in self.motion_log]
            with open(fname, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            self.rec_status.configure(text=f'saved: {os.path.basename(fname)}')

    def load_motion(self):
        fname = filedialog.askopenfilename(
            defaultextension='.json', initialdir='sequences', filetypes=[('JSON', '*.json')])
        if not fname:
            return
        try:
            with open(fname, encoding='utf-8') as f:
                data = json.load(f)
            self.motion_log = []
            t0 = data[0]['t'] if data else 0.0
            for item in data:
                p = {k: max(0, min(180, int(v))) for k, v in item['pos'].items()
                     if k in JOINTS}
                self.motion_log.append((float(item['t']) - t0, p))
            self._render_recorder()
        except Exception as e:
            messagebox.showerror('Load error', str(e))

    # ---- presets ----
    def apply_preset(self):
        sel = self.preset_list.curselection()
        if not sel:
            return
        name = self.preset_list.get(sel[0])
        if name in self.presets:
            self.pos = dict(self.presets[name])
            self._apply_pos()
            self._log(f'preset applied: {name}')

    def save_preset(self):
        name = self.preset_entry.get().strip()
        if not name:
            messagebox.showwarning('Preset', 'Enter a name for the preset.')
            return
        self.presets[name] = dict(self.pos)
        self._render_presets()
        self._save_presets()
        self.preset_entry.delete(0, 'end')

    def delete_preset(self):
        sel = self.preset_list.curselection()
        if not sel:
            return
        name = self.preset_list.get(sel[0])
        if name in self.presets:
            del self.presets[name]
            self._render_presets()
            self._save_presets()

    def _render_presets(self):
        self.preset_list.delete(0, 'end')
        for name in self.presets:
            self.preset_list.insert('end', name)

    # ---- settings / presets io ----
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
                    self.presets = {k: {j: max(0, min(180, int(v))) for j, v in p.items()
                                        if j in JOINTS} for k, p in json.load(f).items()}
        except Exception:
            self.presets = {}

    def _save_presets(self):
        try:
            with open(PRESET_FILE, 'w', encoding='utf-8') as f:
                json.dump(self.presets, f, indent=2)
            self._log(f'presets saved ({len(self.presets)})')
        except Exception as e:
            messagebox.showerror('Presets error', str(e))

    def _restore_window(self):
        g = self.cfg.get('window', 'geometry', fallback='')
        if g:
            self.root.geometry(g)

    def _save_window(self):
        try:
            self.cfg.set('window', 'geometry', self.root.geometry()
                         if self.root.winfo_exists() else '')
        except Exception:
            pass
        try:
            self.cfg.set('recorder', 'speed', f'{self.rec_speed.get():.2f}')
        except Exception:
            pass
        try:
            self.cfg.set('recorder', 'loop', 'true' if self.loop_var.get() else 'false')
        except Exception:
            pass
        save_config(self.cfg)

    def _apply_recorder_config(self):
        try:
            self.rec_speed.set(float(self.cfg.get('recorder', 'speed', fallback='1.0')))
        except Exception:
            pass
        try:
            self.loop_var.set(self.cfg.get('recorder', 'loop', fallback='false').lower() == 'true')
        except Exception:
            pass

    # ============================================================== LOOP
    def _emit(self, fn, *args):
        self._gui_q.put((fn, args))

    def _log(self, msg):
        stamps = time.strftime('%H:%M:%S')
        try:
            self.console.configure(state='normal')
            self.console.insert('end', f'[{stamps}] {msg}\n')
            self.console.see('end')
            while int(self.console.index('end-1c').split('.')[0]) > 500:
                self.console.delete('1.0', '2.0')
            self.console.configure(state='disabled')
        except Exception:
            pass

    def _poll(self):
        drained = 0
        while drained < 200:                       # bound work per tick
            try:
                fn, args = self._gui_q.get_nowait()
            except queue.Empty:
                break
            try:
                fn(*args)
            except Exception:
                pass
            drained += 1
        try:
            self.display, self.connected = self.mgr.get_state()
            if getattr(self.mgr, 'sim', False):
                # No hardware: reflect the commanded position so the arm moves
                self.status.configure(text='● SIMULATION (no Arduino)', text_color=ACCENT2)
            else:
                live = (time.time() - self.mgr.last_pos_time) < 1.5
                if not self.connected:
                    text, col = '● LOST', DANGER
                elif live:
                    text, col = '● CONNECTED', OK
                else:
                    text, col = '● NO DATA', WARN
                self.status.configure(text=f'{self.port} — {text}', text_color=col)
                if self.connected and not live:
                    self.mgr.request_position()
            self._draw_arm()
            if self.joystick['active'] and self.busy == 0:
                self._joy_apply_vel()
        except Exception:
            pass
        while True:
            try:
                line = self.mgr.log_q.get_nowait()
            except queue.Empty:
                break
            self._log('< ' + line)
        self.root.after(30, self._poll)

    def _joy_apply_vel(self):
        vx, vy = self.joystick['vx'], self.joystick['vy']
        # Routing mirrors firmware (Control_Panel.cpp): default X->Base, Y->Shoulder;
        # A: X->Wrist, B: Y->Elbow, C: X->Finger, D: Y->Arm.
        if self.joy_mode == 'A':
            jx = 'wrist'; jy = None
        elif self.joy_mode == 'B':
            jx = None; jy = 'elbow'
        elif self.joy_mode == 'C':
            jx = 'finger'; jy = None
        elif self.joy_mode == 'D':
            jx = None; jy = 'arm'
        else:
            jx, jy = 'base', 'dual'
        if jx and abs(vx) > 0.05:
            self.pos[jx] = max(0, min(180, self.pos[jx] + vx * 2))
        if jy and abs(vy) > 0.05:
            self.pos[jy] = max(0, min(180, self.pos[jy] + vy * 2))
        self._apply_pos()

    def _estop(self):
        self.sweep_cancel.set()
        self.seq_cancel.set()
        self.motion_cancel.set()
        self.motion_pause.set()
        self.busy = 0
        self.pos = dict(self.safe_home)
        self._apply_pos()
        self._log('E-STOP: movement stopped, safe home applied')
        self.status.configure(text='E-STOP', text_color=DANGER)

    def on_close(self):
        try:
            self._save_window()
        except Exception:
            pass
        try:
            self.mgr.close()
        finally:
            self.root.destroy()


# ===========================================================================
# ENTRY
# ===========================================================================
def main():
    # Auto-detect: start with real hardware if the port is available,
    # otherwise run in silent simulation mode (no dialog).
    try:
        manager = SerialManager(ARDUINO_PORT, BAUD_RATE)
    except Exception as e:
        print(f'Arduino not found on {ARDUINO_PORT} ({e}); running in simulation mode.')
        manager = SimulatedManager()
    root = ctk.CTk()
    app = RobotArmUI(root, manager)
    app.port = getattr(manager, 'port', ARDUINO_PORT)
    root.mainloop()


if __name__ == '__main__':
    main()