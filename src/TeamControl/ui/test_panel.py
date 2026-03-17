"""
Hardware Test Console — direct robot connectivity testing & manual control.

Tabbed layout:
  Connect & Drive  — pick a robot, set exact velocities, send commands
  Quick Tests      — one-click preset movements
  Robots           — full ipconfig.yaml table
  Packet Log       — raw packet inspector + scrolling log
"""

import time
import math
import socket

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QGroupBox, QComboBox, QLineEdit, QSpinBox,
    QDoubleSpinBox, QSlider, QPlainTextEdit, QCheckBox,
    QTabWidget, QFrame, QSplitter, QSizePolicy, QHeaderView,
    QTableWidget, QTableWidgetItem, QAbstractItemView,
    QScrollArea, QSpacerItem,
)
from PySide6.QtCore import Qt, QTimer, QTime, Signal
from PySide6.QtGui import QFont, QColor

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, DANGER, WARNING,
    YELLOW_TEAM, BLUE_TEAM, BG_DARK, BG_MID, BORDER,
)

import json
import math
import os

from TeamControl.network.robot_command import RobotCommand
from TeamControl.network.sender import Sender
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.grSimPacketFactory import grSimPacketFactory
from TeamControl.utils.yaml_config import Config
from TeamControl.robot.constants import (
    _TUNING_PATH, _load_tuning, MAX_W,
    HALF_LEN, HALF_WID, FIELD_MARGIN,
)
from TeamControl.world.transform_cords import world2robot

# Safe boundary — stop if the robot gets this close to the field edge
_SAFE_X = HALF_LEN - FIELD_MARGIN   # 2200 mm from center
_SAFE_Y = HALF_WID - FIELD_MARGIN   # 1200 mm from center


def _ts():
    return QTime.currentTime().toString("HH:mm:ss.zzz")


class _LogView(QPlainTextEdit):
    MAX_LINES = 2000

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setMaximumBlockCount(self.MAX_LINES)
        self.setFont(QFont("Cascadia Code", 10))

    def ok(self, msg):
        self.appendHtml(
            f'<span style="color:{TEXT_DIM}">{_ts()}</span> '
            f'<span style="color:{SUCCESS}">{msg}</span>')

    def err(self, msg):
        self.appendHtml(
            f'<span style="color:{TEXT_DIM}">{_ts()}</span> '
            f'<span style="color:{DANGER}">{msg}</span>')

    def info(self, msg):
        self.appendHtml(
            f'<span style="color:{TEXT_DIM}">{_ts()}</span> '
            f'<span style="color:#eaeaea">{msg}</span>')


class _RobotRow:
    __slots__ = ("label", "team", "shell_id", "grsim_id", "ip", "port")

    def __init__(self, label, team, shell_id, grsim_id, ip, port):
        self.label = label
        self.team = team
        self.shell_id = shell_id
        self.grsim_id = grsim_id
        self.ip = ip
        self.port = port


def _heading(text):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"font-size:13px; font-weight:bold; color:{ACCENT}; padding:2px 0;")
    return lbl


def _sep():
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setStyleSheet(f"color:{BORDER};")
    return line


class TestPanel(QWidget):
    """Hardware testing & manual robot control console."""

    def __init__(self, engine=None, field=None, parent=None):
        super().__init__(parent)
        self._engine = engine
        self._field = field
        self._sender = Sender()
        self._grsim: grSimSender | None = None
        self._continuous_timer = QTimer(self)
        self._continuous_timer.setInterval(50)
        self._continuous_timer.timeout.connect(self._send_continuous)
        self._robots: list[_RobotRow] = []
        self._n_sent = 0
        self._n_err = 0

        # Action test state
        self._action_timer = QTimer(self)
        self._action_timer.setInterval(50)  # 20 Hz
        self._action_timer.timeout.connect(self._action_tick)
        self._action_mode = None  # "go_to_ball", "go_to_ball_kick", "draw_square", "go_to_point"
        self._last_ball_dist = None  # track last known ball distance for kick-on-occlude
        self._square_step = 0
        self._square_step_ticks = 0
        self._goto_target = None  # (x_mm, y_mm) for go_to_point

        self._build_ui()
        self._load_robots()

    # ══════════════════════════════════════════════════════════════
    #  UI
    # ══════════════════════════════════════════════════════════════

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        tabs = QTabWidget()
        tabs.setDocumentMode(True)
        tabs.addTab(self._build_drive_tab(), "Connect && Drive")
        tabs.addTab(self._build_tuning_tab(), "Tuning")
        tabs.addTab(self._build_robots_tab(), "Robots")
        tabs.addTab(self._build_log_tab(), "Packet Log")
        root.addWidget(tabs)

    # ── Tab 1: Connect & Drive ────────────────────────────────────

    def _build_drive_tab(self):
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(16)

        # ── Left column: target ───────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(12)

        left.addWidget(_heading("Robot Target"))

        tg = QGridLayout()
        tg.setSpacing(8)
        tg.setColumnStretch(1, 1)

        tg.addWidget(QLabel("Preset:"), 0, 0)
        self._robot_combo = QComboBox()
        self._robot_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._robot_combo.currentIndexChanged.connect(self._on_robot_selected)
        tg.addWidget(self._robot_combo, 0, 1, 1, 3)

        tg.addWidget(QLabel("IP Address:"), 1, 0)
        self._ip_edit = QLineEdit("127.0.0.1")
        tg.addWidget(self._ip_edit, 1, 1, 1, 3)

        tg.addWidget(QLabel("Port:"), 2, 0)
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1025, 65534)
        self._port_spin.setValue(50514)
        tg.addWidget(self._port_spin, 2, 1)

        tg.addWidget(QLabel("Shell ID:"), 3, 0)
        self._id_spin = QSpinBox()
        self._id_spin.setRange(0, 15)
        tg.addWidget(self._id_spin, 3, 1)

        tg.addWidget(QLabel("Team:"), 4, 0)
        self._team_combo = QComboBox()
        self._team_combo.addItems(["Yellow", "Blue"])
        tg.addWidget(self._team_combo, 4, 1)

        left.addLayout(tg)

        ping_btn = QPushButton("Test Connection")
        ping_btn.setMinimumHeight(36)
        ping_btn.setStyleSheet(f"font-weight:bold; color:{ACCENT};")
        ping_btn.clicked.connect(self._test_connection)
        left.addWidget(ping_btn)

        left.addWidget(_sep())
        left.addWidget(_heading("grSim"))

        gsl = QGridLayout()
        gsl.setSpacing(8)
        gsl.addWidget(QLabel("IP:"), 0, 0)
        self._grsim_ip = QLineEdit("127.0.0.1")
        gsl.addWidget(self._grsim_ip, 0, 1)
        gsl.addWidget(QLabel("Port:"), 1, 0)
        self._grsim_port = QSpinBox()
        self._grsim_port.setRange(1025, 65534)
        self._grsim_port.setValue(20011)
        gsl.addWidget(self._grsim_port, 1, 1)
        left.addLayout(gsl)

        self._grsim_also = QCheckBox("Mirror every command to grSim")
        self._grsim_also.setStyleSheet("font-weight:bold;")
        left.addWidget(self._grsim_also)

        grsim_test = QPushButton("Test grSim (spin robot 0)")
        grsim_test.setMinimumHeight(34)
        grsim_test.clicked.connect(self._test_grsim)
        left.addWidget(grsim_test)

        left.addStretch()

        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(320)
        outer.addWidget(left_w)

        # Vertical divider
        div = QFrame()
        div.setFrameShape(QFrame.VLine)
        div.setStyleSheet(f"color:{BORDER};")
        outer.addWidget(div)

        # ── Right column: velocity + send ─────────────────────────
        right = QVBoxLayout()
        right.setSpacing(12)

        right.addWidget(_heading("Velocity Command"))

        vel_grid = QGridLayout()
        vel_grid.setSpacing(10)
        vel_grid.setColumnStretch(1, 1)

        self._vx_spin, self._vx_slider = self._make_vel_row(
            "VX  (tangent)", -5.0, 5.0, 0.0, "m/s", vel_grid, 0)
        self._vy_spin, self._vy_slider = self._make_vel_row(
            "VY  (normal)", -5.0, 5.0, 0.0, "m/s", vel_grid, 1)
        self._w_spin, self._w_slider = self._make_vel_row(
            "W   (angular)", -2.0, 2.0, 0.0, "rad/s", vel_grid, 2)

        right.addLayout(vel_grid)

        right.addWidget(_sep())
        right.addWidget(_heading("Kick & Dribble"))

        kd_grid = QGridLayout()
        kd_grid.setSpacing(10)

        self._kick_btn = QPushButton("Kick")
        self._kick_btn.setStyleSheet("font-size:14px; font-weight:bold;")
        self._kick_btn.setCheckable(True)
        self._dribble_btn = QPushButton("Dribble / Spinner")
        self._dribble_btn.setStyleSheet("font-size:14px; font-weight:bold;")
        self._dribble_btn.setCheckable(True)

        kd_grid.addWidget(self._kick_btn, 0, 0)
        kd_grid.addWidget(self._dribble_btn, 0, 1)

        self._kick_speed_spin = QDoubleSpinBox()
        self._kick_speed_spin.setRange(0, 20)
        self._kick_speed_spin.setValue(10.0)
        self._kick_speed_spin.setSingleStep(0.5)
        self._kick_speed_spin.setSuffix(" m/s")
        self._kick_speed_spin.setPrefix("Kick speed: ")
        kd_grid.addWidget(self._kick_speed_spin, 1, 0, 1, 2)

        right.addLayout(kd_grid)

        right.addWidget(_sep())
        right.addWidget(_heading("Send"))

        send_grid = QGridLayout()
        send_grid.setSpacing(8)

        send_once = QPushButton("Send Once")
        send_once.setObjectName("startBtn")
        send_once.setMinimumHeight(44)
        send_once.setStyleSheet("font-size:14px;")
        send_once.clicked.connect(self._send_once)
        send_grid.addWidget(send_once, 0, 0)

        self._cont_btn = QPushButton("Start Continuous (20 Hz)")
        self._cont_btn.setMinimumHeight(44)
        self._cont_btn.setStyleSheet("font-size:14px;")
        self._cont_btn.clicked.connect(self._toggle_continuous)
        send_grid.addWidget(self._cont_btn, 0, 1)

        stop_btn = QPushButton("STOP")
        stop_btn.setObjectName("stopBtn")
        stop_btn.setMinimumHeight(44)
        stop_btn.setStyleSheet("font-size:14px;")
        stop_btn.clicked.connect(self._send_stop)
        send_grid.addWidget(stop_btn, 1, 0)

        zero_btn = QPushButton("Zero All Inputs")
        zero_btn.setMinimumHeight(44)
        zero_btn.setStyleSheet("font-size:14px;")
        zero_btn.clicked.connect(self._zero_inputs)
        send_grid.addWidget(zero_btn, 1, 1)

        right.addLayout(send_grid)

        # Raw packet preview (inline)
        right.addWidget(_sep())
        right.addWidget(_heading("Last Packet"))
        self._raw_label = QLabel("No packets sent yet")
        self._raw_label.setFont(QFont("Cascadia Code", 11))
        self._raw_label.setWordWrap(True)
        self._raw_label.setMinimumHeight(60)
        self._raw_label.setStyleSheet(
            f"background:{BG_DARK}; padding:10px; border:1px solid {BORDER}; "
            f"border-radius:6px; color:{ACCENT};")
        self._raw_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        right.addWidget(self._raw_label)

        # ── Actions (right-click on field triggers these) ────────────
        right.addWidget(_sep())
        right.addWidget(_heading("Actions"))

        act_grid = QGridLayout()
        act_grid.setSpacing(8)

        act_grid.addWidget(QLabel("Go-to velocity:"), 0, 0)
        self._goto_vel_spin = QDoubleSpinBox()
        self._goto_vel_spin.setRange(0.05, 2.0)
        self._goto_vel_spin.setValue(0.2)
        self._goto_vel_spin.setSingleStep(0.05)
        self._goto_vel_spin.setSuffix("  m/s")
        self._goto_vel_spin.setMinimumWidth(120)
        act_grid.addWidget(self._goto_vel_spin, 0, 1)

        right.addLayout(act_grid)

        self._action_status = QLabel("")
        self._action_status.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px; padding:4px;")
        right.addWidget(self._action_status)

        self._goto_status = QLabel("")
        self._goto_status.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px; padding:4px;")
        right.addWidget(self._goto_status)

        stop_row = QHBoxLayout()
        stop_row.setSpacing(8)

        stop_btn = QPushButton("STOP")
        stop_btn.setObjectName("stopBtn")
        stop_btn.setMinimumHeight(40)
        stop_btn.setStyleSheet("font-size:14px;")
        stop_btn.clicked.connect(self._send_stop)
        stop_row.addWidget(stop_btn)

        stop_all = QPushButton("STOP ALL")
        stop_all.setObjectName("stopBtn")
        stop_all.setMinimumHeight(40)
        stop_all.setStyleSheet("font-size:14px;")
        stop_all.clicked.connect(self._stop_all)
        stop_row.addWidget(stop_all)

        right.addLayout(stop_row)

        right.addStretch()
        outer.addLayout(right, 1)

        return page

    # ── Tab: Tuning ────────────────────────────────────────────────

    def _build_tuning_tab(self):
        page = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(
            f"QScrollArea {{ background: {BG_DARK}; border: none; }}")

        inner = QWidget()
        inner.setStyleSheet(f"background: {BG_DARK};")
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(12)

        lay.addWidget(_heading("Angular Velocity Tuning"))
        lay.addWidget(QLabel(
            "Adjust parameters below then click Save. "
            "Values are written to tuning.json and apply when the model restarts."))

        t = _load_tuning()

        grid = QGridLayout()
        grid.setSpacing(10)
        grid.setColumnStretch(1, 1)

        self._tuning_spins = {}
        params = [
            ("max_w_raw",              "MAX_W (raw cap)",       0.05, 3.0,  t["max_w_raw"],              "rad/s"),
            ("w_clamp_pct",            "Clamp %",               0.10, 1.0,  t["w_clamp_pct"],            ""),
            ("turn_gain",              "Turn Gain",             0.1,  5.0,  t["turn_gain"],              ""),
            ("face_ball_gain",         "Face-Ball Gain",        0.1,  5.0,  t["face_ball_gain"],         ""),
            ("path_planner_gain",      "Path Planner Gain",     0.1,  5.0,  t["path_planner_gain"],      ""),
            ("path_planner_min_impulse","Min Impulse",          0.0,  1.0,  t["path_planner_min_impulse"],"rad/s"),
            ("angular_slow_speed",     "Angular Slow Speed",    0.01, 2.0,  t["angular_slow_speed"],     "rad/s"),
            ("angular_normal_speed",   "Angular Normal Speed",  0.01, 2.0,  t["angular_normal_speed"],   "rad/s"),
            ("angular_fast_speed",     "Angular Fast Speed",    0.01, 3.0,  t["angular_fast_speed"],     "rad/s"),
        ]

        for row, (key, label, lo, hi, default, suffix) in enumerate(params):
            lbl = QLabel(label)
            lbl.setStyleSheet("font-weight:bold; font-size:13px;")
            lbl.setMinimumWidth(160)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(lo * 100), int(hi * 100))
            slider.setValue(int(default * 100))
            slider.setMinimumWidth(200)

            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setValue(default)
            spin.setDecimals(2)
            spin.setSingleStep(0.05)
            if suffix:
                spin.setSuffix(f"  {suffix}")
            spin.setMinimumWidth(130)
            spin.setMinimumHeight(30)
            spin.setStyleSheet("font-size:13px;")

            slider.valueChanged.connect(lambda v, s=spin: s.setValue(v / 100.0))
            spin.valueChanged.connect(lambda v, sl=slider: sl.setValue(int(v * 100)))

            grid.addWidget(lbl, row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)

            self._tuning_spins[key] = spin

        lay.addLayout(grid)

        # Effective MAX_W readout
        lay.addWidget(_sep())
        self._effective_w_label = QLabel()
        self._effective_w_label.setStyleSheet(
            f"font-size:14px; font-weight:bold; color:{ACCENT}; padding:6px;")
        self._update_effective_w_label()
        lay.addWidget(self._effective_w_label)

        self._tuning_spins["max_w_raw"].valueChanged.connect(
            lambda _: self._update_effective_w_label())
        self._tuning_spins["w_clamp_pct"].valueChanged.connect(
            lambda _: self._update_effective_w_label())

        # Buttons
        lay.addWidget(_sep())
        btn_row = QHBoxLayout()

        save_btn = QPushButton("Save to tuning.json")
        save_btn.setObjectName("startBtn")
        save_btn.setMinimumHeight(44)
        save_btn.setStyleSheet("font-size:14px; font-weight:bold;")
        save_btn.clicked.connect(self._save_tuning)
        btn_row.addWidget(save_btn)

        reload_btn = QPushButton("Reload from file")
        reload_btn.setMinimumHeight(44)
        reload_btn.setStyleSheet("font-size:14px;")
        reload_btn.clicked.connect(self._reload_tuning)
        btn_row.addWidget(reload_btn)

        reset_btn = QPushButton("Reset to defaults")
        reset_btn.setMinimumHeight(44)
        reset_btn.setStyleSheet(f"font-size:14px; color:{WARNING};")
        reset_btn.clicked.connect(self._reset_tuning)
        btn_row.addWidget(reset_btn)

        lay.addLayout(btn_row)

        self._tuning_status = QLabel("")
        self._tuning_status.setStyleSheet(f"color:{SUCCESS}; font-size:12px; padding:4px;")
        lay.addWidget(self._tuning_status)

        lay.addStretch()
        scroll.setWidget(inner)

        wrapper = QVBoxLayout(page)
        wrapper.setContentsMargins(0, 0, 0, 0)
        wrapper.addWidget(scroll)
        return page

    def _update_effective_w_label(self):
        raw = self._tuning_spins["max_w_raw"].value()
        pct = self._tuning_spins["w_clamp_pct"].value()
        eff = raw * pct
        self._effective_w_label.setText(
            f"Effective MAX_W = {raw:.2f} x {pct:.0%} = {eff:.3f} rad/s")

    def _save_tuning(self):
        data = {k: spin.value() for k, spin in self._tuning_spins.items()}
        try:
            with open(_TUNING_PATH, "w") as f:
                json.dump(data, f, indent=4)
            self._tuning_status.setStyleSheet(f"color:{SUCCESS}; font-size:12px; padding:4px;")
            self._tuning_status.setText(
                f"Saved to {_TUNING_PATH}  —  restart the model to apply")
        except Exception as e:
            self._tuning_status.setStyleSheet(f"color:{DANGER}; font-size:12px; padding:4px;")
            self._tuning_status.setText(f"Save failed: {e}")

    def _reload_tuning(self):
        t = _load_tuning()
        for key, spin in self._tuning_spins.items():
            if key in t:
                spin.setValue(t[key])
        self._tuning_status.setStyleSheet(f"color:{SUCCESS}; font-size:12px; padding:4px;")
        self._tuning_status.setText("Reloaded from tuning.json")

    def _reset_tuning(self):
        defaults = {
            "max_w_raw": 0.5,
            "w_clamp_pct": 0.60,
            "turn_gain": 0.8,
            "face_ball_gain": 0.8,
            "path_planner_gain": 0.8,
            "path_planner_min_impulse": 0.15,
            "angular_slow_speed": 0.25,
            "angular_normal_speed": 0.5,
            "angular_fast_speed": 0.6,
        }
        for key, spin in self._tuning_spins.items():
            spin.setValue(defaults.get(key, spin.value()))
        self._tuning_status.setStyleSheet(f"color:{WARNING}; font-size:12px; padding:4px;")
        self._tuning_status.setText("Reset to defaults — click Save to persist")

    # ── Tab 3: Robots table ───────────────────────────────────────

    def _build_robots_tab(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(12)

        lay.addWidget(_heading("All Configured Robots (ipconfig.yaml)"))
        lay.addWidget(QLabel(
            "Click a row to select that robot as the target on the Connect & Drive tab."))

        self._robot_table = QTableWidget(0, 6)
        self._robot_table.setHorizontalHeaderLabels(
            ["Team", "Label", "Shell ID", "grSim ID", "IP Address", "Port"])
        self._robot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._robot_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._robot_table.setAlternatingRowColors(True)
        self._robot_table.verticalHeader().setVisible(False)
        hh = self._robot_table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setMinimumSectionSize(60)
        self._robot_table.currentCellChanged.connect(self._on_table_row)
        lay.addWidget(self._robot_table)

        btn_row = QHBoxLayout()
        reload_btn = QPushButton("Reload ipconfig.yaml")
        reload_btn.setMinimumHeight(36)
        reload_btn.clicked.connect(self._load_robots)
        btn_row.addWidget(reload_btn)
        btn_row.addStretch()
        lay.addLayout(btn_row)

        return page

    # ── Tab 4: Packet Log ─────────────────────────────────────────

    def _build_log_tab(self):
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(16, 16, 16, 16)
        lay.setSpacing(8)

        lay.addWidget(_heading("Packet Inspector"))

        bar = QHBoxLayout()
        self._send_count = QLabel("0 sent")
        self._send_count.setStyleSheet(f"color:{TEXT_DIM}; font-size:13px;")
        self._err_count = QLabel("0 errors")
        self._err_count.setStyleSheet(f"color:{DANGER}; font-size:13px;")
        clear_btn = QPushButton("Clear Log")
        clear_btn.setFixedWidth(80)
        clear_btn.clicked.connect(lambda: (self._log.clear(),
                                           self._reset_counts()))
        bar.addWidget(self._send_count)
        bar.addWidget(QLabel("  "))
        bar.addWidget(self._err_count)
        bar.addStretch()
        bar.addWidget(clear_btn)
        lay.addLayout(bar)

        self._log = _LogView()
        lay.addWidget(self._log)

        return page

    # ── Velocity row factory ──────────────────────────────────────

    def _make_vel_row(self, label, lo, hi, default, suffix, grid, row):
        lbl = QLabel(label)
        lbl.setStyleSheet("font-weight:bold; font-size:13px;")
        lbl.setMinimumWidth(110)

        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(lo * 100), int(hi * 100))
        slider.setValue(int(default * 100))
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int((hi - lo) * 10))
        slider.setMinimumWidth(200)

        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setValue(default)
        spin.setDecimals(2)
        spin.setSingleStep(0.1)
        spin.setSuffix(f"  {suffix}")
        spin.setMinimumWidth(130)
        spin.setMinimumHeight(30)
        spin.setStyleSheet("font-size:13px;")

        reset = QPushButton("0")
        reset.setFixedSize(30, 30)
        reset.setToolTip("Reset to zero")

        slider.valueChanged.connect(lambda v: spin.setValue(v / 100.0))
        spin.valueChanged.connect(lambda v: slider.setValue(int(v * 100)))
        reset.clicked.connect(lambda: (spin.setValue(0), slider.setValue(0)))

        grid.addWidget(lbl, row, 0)
        grid.addWidget(slider, row, 1)
        grid.addWidget(spin, row, 2)
        grid.addWidget(reset, row, 3)

        return spin, slider

    # ── Robot loading ─────────────────────────────────────────────

    def _load_robots(self):
        self._robots.clear()
        self._robot_combo.clear()
        try:
            cfg = Config()
        except Exception as e:
            self._log.err(f"Failed to load config: {e}")
            return

        for team_name, team_data in [("Yellow", cfg.yellow), ("Blue", cfg.blue)]:
            if not team_data:
                continue
            for key, rd in team_data.items():
                r = _RobotRow(
                    label=f"{team_name} {key}",
                    team=team_name,
                    shell_id=rd.get("shellID", 0),
                    grsim_id=rd.get("grSimID", 0),
                    ip=rd.get("ip", "127.0.0.1"),
                    port=rd.get("port", 50514),
                )
                self._robots.append(r)
                self._robot_combo.addItem(
                    f"{r.label}  —  shell {r.shell_id}  →  {r.ip}:{r.port}")

        self._robot_combo.addItem("— Custom —")

        self._robot_table.setRowCount(len(self._robots))
        for i, r in enumerate(self._robots):
            color = QColor(YELLOW_TEAM) if r.team == "Yellow" else QColor(BLUE_TEAM)
            vals = [r.team, r.label, str(r.shell_id),
                    str(r.grsim_id), r.ip, str(r.port)]
            for j, v in enumerate(vals):
                item = QTableWidgetItem(v)
                item.setTextAlignment(Qt.AlignCenter)
                if j == 0:
                    item.setForeground(color)
                    item.setFont(QFont("Segoe UI", 11, QFont.Bold))
                self._robot_table.setItem(i, j, item)

        self._log.info(f"Loaded {len(self._robots)} robots from ipconfig.yaml")

        try:
            self._grsim_ip.setText(cfg.grSim_addr[0])
            self._grsim_port.setValue(cfg.grSim_addr[1])
        except Exception:
            pass

    def _on_robot_selected(self, idx):
        if 0 <= idx < len(self._robots):
            r = self._robots[idx]
            self._ip_edit.setText(r.ip)
            self._port_spin.setValue(r.port)
            self._id_spin.setValue(r.shell_id)
            self._team_combo.setCurrentText(r.team)

    def _on_table_row(self, row, col, prev_row, prev_col):
        if 0 <= row < len(self._robots):
            self._robot_combo.setCurrentIndex(row)

    # ── Command building ──────────────────────────────────────────

    def _build_cmd(self, vx=None, vy=None, w=None, kick=None, dribble=None):
        return RobotCommand(
            robot_id=self._id_spin.value(),
            vx=vx if vx is not None else self._vx_spin.value(),
            vy=vy if vy is not None else self._vy_spin.value(),
            w=w if w is not None else self._w_spin.value(),
            kick=kick if kick is not None else int(self._kick_btn.isChecked()),
            dribble=dribble if dribble is not None else int(self._dribble_btn.isChecked()),
            isYellow=(self._team_combo.currentText() == "Yellow"),
        )

    # ── Sending ───────────────────────────────────────────────────

    def _do_send(self, cmd: RobotCommand):
        ip = self._ip_edit.text().strip()
        port = self._port_spin.value()
        raw_text = str(cmd)

        self._raw_label.setText(
            f"<b>To:</b> {ip}:{port}<br>"
            f"<b>Raw:</b> {raw_text}<br>"
            f"<b>Bytes:</b> {cmd.encode()!r}")

        try:
            self._sender.send(cmd, ip, port)
            self._log.ok(
                f"→ {ip}:{port}  |  "
                f"id={cmd.robot_id} vx={cmd.vx:.2f} vy={cmd.vy:.2f} "
                f"w={cmd.w:.2f} kick={cmd.kick} drib={cmd.dribble}")
            self._n_sent += 1
        except Exception as e:
            self._log.err(f"SEND FAILED to {ip}:{port} — {e}")
            self._n_err += 1

        if self._grsim_also.isChecked():
            self._do_grsim_send(cmd)

        self._update_counts()

    def _do_grsim_send(self, cmd: RobotCommand):
        try:
            gs = self._get_grsim()
            gs.send_robot_command(cmd)
            self._n_sent += 1
        except Exception as e:
            self._log.err(f"grSim SEND FAILED — {e}")
            self._n_err += 1

    def _get_grsim(self) -> grSimSender:
        ip = self._grsim_ip.text().strip()
        port = self._grsim_port.value()
        if (self._grsim is None
                or self._grsim._destination_ip != ip
                or self._grsim.destination_port != port):
            self._grsim = grSimSender(ip, port)
        return self._grsim

    # ── Button handlers ───────────────────────────────────────────

    def _send_once(self):
        self._do_send(self._build_cmd())

    def _send_stop(self):
        self._stop_action()
        cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
        self._do_send(cmd)
        self._zero_inputs()
        self._log.info("STOP sent")

    def _zero_inputs(self):
        self._vx_spin.setValue(0)
        self._vy_spin.setValue(0)
        self._w_spin.setValue(0)
        self._kick_btn.setChecked(False)
        self._dribble_btn.setChecked(False)

    def _send_test(self, vx, vy, w, kick, dribble):
        cmd = self._build_cmd(vx=vx, vy=vy, w=w, kick=kick, dribble=dribble)
        self._do_send(cmd)

    def _toggle_continuous(self):
        if self._continuous_timer.isActive():
            self._continuous_timer.stop()
            self._cont_btn.setText("Start Continuous (20 Hz)")
            self._cont_btn.setStyleSheet("font-size:14px;")
            self._log.info("Continuous send stopped")
        else:
            self._continuous_timer.start()
            self._cont_btn.setText("STOP Continuous")
            self._cont_btn.setStyleSheet(
                f"background:{DANGER}; color:#fff; font-weight:bold; font-size:14px;")
            self._log.info("Continuous send started at 20 Hz")

    def _send_continuous(self):
        self._do_send(self._build_cmd())

    def _stop_all(self):
        if self._continuous_timer.isActive():
            self._continuous_timer.stop()
            self._cont_btn.setText("Start Continuous (20 Hz)")
            self._cont_btn.setStyleSheet("font-size:14px;")

        for r in self._robots:
            cmd = RobotCommand(
                robot_id=r.shell_id, vx=0, vy=0, w=0,
                kick=0, dribble=0,
                isYellow=(r.team == "Yellow"))
            try:
                self._sender.send(cmd, r.ip, r.port)
                self._log.ok(f"STOP → {r.label} ({r.ip}:{r.port})")
                self._n_sent += 1
            except Exception as e:
                self._log.err(f"STOP FAILED → {r.label} — {e}")
                self._n_err += 1

        self._update_counts()
        self._log.info(f"STOP ALL sent to {len(self._robots)} robots")

    # ── Action tests ────────────────────────────────────────────

    def _start_action(self, mode):
        self._stop_action()
        self._action_mode = mode
        self._square_step = 0
        self._square_step_ticks = 0
        self._last_ball_dist = None
        self._action_timer.start()

        labels = {
            "go_to_ball": "Go to Ball",
            "go_to_ball_kick": "Go to Ball & Kick",
            "draw_square": "Draw Square",
        }
        self._action_status.setStyleSheet(
            f"color:{SUCCESS}; font-size:12px; padding:4px;")
        self._action_status.setText(
            f"Running: {labels.get(mode, mode)} — click STOP to cancel")
        self._log.info(f"Action test started: {labels.get(mode, mode)}")

    def _stop_action(self):
        if self._action_timer.isActive():
            self._action_timer.stop()
            self._action_mode = None
            self._action_status.setText("")
            # Send a stop command
            cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
            self._do_send(cmd)

    def _get_ball_and_robot(self):
        """Get ball and robot positions from the engine's world model."""
        if not self._engine:
            return None, None
        wm = self._engine._wm
        if wm is None:
            return None, None
        frame = wm.get_latest_frame()
        if frame is None:
            return None, None

        ball = frame.ball
        if ball is None:
            return None, None
        ball_pos = (float(ball.x), float(ball.y))

        rid = self._id_spin.value()
        is_yellow = self._team_combo.currentText() == "Yellow"
        team = frame.robots_yellow if is_yellow else frame.robots_blue
        try:
            robot = team[rid]
        except (IndexError, TypeError):
            return ball_pos, None

        from TeamControl.SSL.vision.robots import Robot
        if not isinstance(robot, Robot):
            return ball_pos, None

        robot_pose = (float(robot.x), float(robot.y), float(robot.o))
        return ball_pos, robot_pose

    def field_action(self, action_name):
        """Called from the field right-click menu."""
        if action_name == "stop":
            self._send_stop()
        else:
            self._start_action(action_name)

    def _pick_goto_point(self):
        if self._field is None:
            self._goto_status.setStyleSheet(
                f"color:{DANGER}; font-size:12px; padding:4px;")
            self._goto_status.setText("No field canvas available")
            return
        self._field.set_place_mode("go_to_point")
        self._goto_status.setStyleSheet(
            f"color:{WARNING}; font-size:12px; padding:4px;")
        self._goto_status.setText("Click a point on the Dashboard field view...")

    def go_to_point(self, x_mm, y_mm):
        """Called when the user clicks on the field after picking go-to-point."""
        # Clamp target inside safe bounds
        x_mm = max(-_SAFE_X, min(_SAFE_X, x_mm))
        y_mm = max(-_SAFE_Y, min(_SAFE_Y, y_mm))
        self._goto_target = (x_mm, y_mm)
        self._stop_action()
        self._action_mode = "go_to_point"
        self._action_timer.start()
        self._goto_status.setStyleSheet(
            f"color:{SUCCESS}; font-size:12px; padding:4px;")
        self._goto_status.setText(
            f"Going to ({x_mm:.0f}, {y_mm:.0f}) at "
            f"{self._goto_vel_spin.value():.2f} m/s — click STOP to cancel")
        self._action_status.setStyleSheet(
            f"color:{SUCCESS}; font-size:12px; padding:4px;")
        self._action_status.setText("Running: Go to Point — click STOP to cancel")
        self._log.info(f"Go to point ({x_mm:.0f}, {y_mm:.0f})")

    def _is_near_boundary(self, robot_pose):
        """True if the robot is close to the field edge."""
        if robot_pose is None:
            return False
        rx, ry = abs(robot_pose[0]), abs(robot_pose[1])
        return rx > _SAFE_X or ry > _SAFE_Y

    def _get_robot_pose(self):
        """Get just the robot pose (no ball needed)."""
        if not self._engine:
            return None
        wm = self._engine._wm
        if wm is None:
            return None
        frame = wm.get_latest_frame()
        if frame is None:
            return None
        rid = self._id_spin.value()
        is_yellow = self._team_combo.currentText() == "Yellow"
        team = frame.robots_yellow if is_yellow else frame.robots_blue
        try:
            robot = team[rid]
        except (IndexError, TypeError):
            return None
        from TeamControl.SSL.vision.robots import Robot
        if not isinstance(robot, Robot):
            return None
        return (float(robot.x), float(robot.y), float(robot.o))

    def _action_tick(self):
        if self._action_mode == "draw_square":
            self._tick_draw_square()
            return

        if self._action_mode == "go_to_point":
            self._tick_go_to_point()
            return

        # go_to_ball or go_to_ball_kick
        ball_pos, robot_pose = self._get_ball_and_robot()
        if robot_pose is None:
            return

        # Ball disappeared — if we were close, assume we're on it
        if ball_pos is None:
            if (self._action_mode == "go_to_ball_kick"
                    and self._last_ball_dist is not None
                    and self._last_ball_dist < 200):
                # Ball occluded by robot — fire the kick
                cmd = self._build_cmd(vx=0.3, vy=0, w=0, kick=1, dribble=0)
                self._do_send(cmd)
            return

        # Safety — stop if near field boundary
        if self._is_near_boundary(robot_pose):
            self._stop_action()
            self._action_status.setStyleSheet(
                f"color:{WARNING}; font-size:12px; padding:4px;")
            self._action_status.setText("Stopped — robot near field boundary")
            return

        rel_ball = world2robot(robot_pose, ball_pos)
        dist = math.hypot(rel_ball[0], rel_ball[1])
        angle = math.atan2(rel_ball[1], rel_ball[0])
        self._last_ball_dist = dist

        kick = 0
        dribble = 0
        vx = 0.0
        vy = 0.0
        w = 0.0

        arrive_dist = 70  # mm — close enough to touch/kick

        if dist < arrive_dist:
            # Right on the ball
            if self._action_mode == "go_to_ball_kick":
                if abs(angle) < 0.15:
                    vx = 0.3
                    kick = 1
                else:
                    w = max(-0.3, min(0.3, angle * 0.5))
            else:
                # go_to_ball — just stop, we're there
                pass
        elif abs(angle) > 0.2:
            # Turn to face ball first
            w = max(-0.4, min(0.4, angle * 0.5))
        else:
            # Facing the ball — drive forward with decel ramp
            speed = min(0.5, max(0.05, (dist - arrive_dist) / 1000.0))
            vx = speed

        cmd = self._build_cmd(vx=vx, vy=vy, w=w, kick=kick, dribble=dribble)
        self._do_send(cmd)

    def _tick_go_to_point(self):
        if self._goto_target is None:
            self._stop_action()
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        # Safety — stop if near field boundary
        if self._is_near_boundary(robot_pose):
            self._stop_action()
            self._goto_status.setStyleSheet(
                f"color:{WARNING}; font-size:12px; padding:4px;")
            self._goto_status.setText("Stopped — robot near field boundary")
            return

        rel = world2robot(robot_pose, self._goto_target)
        dist = math.hypot(rel[0], rel[1])
        angle = math.atan2(rel[1], rel[0])

        if dist < 150:
            # Arrived — send explicit stop
            cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
            self._do_send(cmd)
            self._stop_action()
            self._goto_status.setStyleSheet(
                f"color:{SUCCESS}; font-size:12px; padding:4px;")
            self._goto_status.setText("Arrived at target point")
            return

        max_speed = self._goto_vel_spin.value()
        vx = 0.0
        w = 0.0

        if abs(angle) > 0.2:
            # Turn to face target first
            w = max(-0.4, min(0.4, angle * 0.5))
        else:
            # Drive forward with decel ramp
            ramp_dist = max(0.0, dist - 150)
            speed = min(max_speed, ramp_dist / 1000.0)
            if speed < 0.05:
                speed = 0.05
            vx = speed

        cmd = self._build_cmd(vx=vx, vy=0, w=w, kick=0, dribble=0)
        self._do_send(cmd)

    # Square waypoints — small square near center of field (500mm sides)
    _SQUARE_WAYPOINTS = [
        ( 250,  250),   # front-right
        ( 250, -250),   # back-right
        (-250, -250),   # back-left
        (-250,  250),   # front-left
    ]
    _SQUARE_ARRIVE_DIST = 150  # mm — close enough to move to next waypoint

    def _tick_draw_square(self):
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        # Safety — stop if near field boundary
        if self._is_near_boundary(robot_pose):
            cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
            self._do_send(cmd)
            self._stop_action()
            self._action_status.setStyleSheet(
                f"color:{WARNING}; font-size:12px; padding:4px;")
            self._action_status.setText("Stopped — robot near field boundary")
            return

        if self._square_step >= len(self._SQUARE_WAYPOINTS):
            cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
            self._do_send(cmd)
            self._stop_action()
            self._action_status.setStyleSheet(
                f"color:{SUCCESS}; font-size:12px; padding:4px;")
            self._action_status.setText("Draw Square completed")
            return

        # Navigate to current waypoint
        target = self._SQUARE_WAYPOINTS[self._square_step]
        rel = world2robot(robot_pose, target)
        dist = math.hypot(rel[0], rel[1])
        angle = math.atan2(rel[1], rel[0])

        if dist < self._SQUARE_ARRIVE_DIST:
            # Stop at waypoint, then move to next
            cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
            self._do_send(cmd)
            self._square_step += 1
            self._action_status.setText(
                f"Draw Square — waypoint {self._square_step}/{len(self._SQUARE_WAYPOINTS)}")
            return

        vx = 0.0
        w = 0.0

        if abs(angle) > 0.2:
            # Turn to face waypoint first
            w = max(-0.4, min(0.4, angle * 0.5))
        else:
            # Drive toward waypoint with decel ramp
            ramp_dist = max(0.0, dist - 150)
            speed = min(0.5, ramp_dist / 1000.0)
            if speed < 0.05:
                speed = 0.05
            vx = speed

        cmd = self._build_cmd(vx=vx, vy=0, w=w, kick=0, dribble=0)
        self._do_send(cmd)

    def _test_connection(self):
        ip = self._ip_edit.text().strip()
        port = self._port_spin.value()
        rid = self._id_spin.value()
        self._log.info(f"Testing connection to {ip}:{port} (robot {rid})…")

        cmd = RobotCommand(robot_id=rid, vx=0, vy=0, w=0, kick=0, dribble=0,
                           isYellow=(self._team_combo.currentText() == "Yellow"))
        try:
            self._sender.send(cmd, ip, port)
            self._log.ok(
                f"Packet sent to {ip}:{port} — "
                f"UDP is fire-and-forget so no receipt confirmation")
            self._n_sent += 1
        except socket.error as e:
            self._log.err(f"SOCKET ERROR: {e}")
            self._n_err += 1
        except Exception as e:
            self._log.err(f"ERROR: {type(e).__name__}: {e}")
            self._n_err += 1
        self._update_counts()

    def _test_grsim(self):
        self._log.info("Testing grSim…")
        try:
            gs = self._get_grsim()
            cmd = RobotCommand(robot_id=0, vx=0, vy=0, w=1.5,
                               kick=0, dribble=0, isYellow=True)
            gs.send_robot_command(cmd)
            self._log.ok(
                f"grSim test sent — Yellow robot 0 should spin for 1s")
            self._n_sent += 1
            QTimer.singleShot(1000, lambda: self._grsim_stop_test(gs))
        except Exception as e:
            self._log.err(f"grSim test FAILED: {e}")
            self._n_err += 1
        self._update_counts()

    def _grsim_stop_test(self, gs):
        try:
            cmd = RobotCommand(robot_id=0, vx=0, vy=0, w=0,
                               kick=0, dribble=0, isYellow=True)
            gs.send_robot_command(cmd)
            self._log.ok("grSim stop sent")
        except Exception:
            pass

    def _update_counts(self):
        self._send_count.setText(f"{self._n_sent} sent")
        self._err_count.setText(f"{self._n_err} errors")

    def _reset_counts(self):
        self._n_sent = 0
        self._n_err = 0
        self._update_counts()
