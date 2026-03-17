"""
Calibration Page — comprehensive motor / drive calibration tools.

Layout:
  +------------------------------+-----------------------+
  |                              |  Calibration controls |
  |        Field Canvas          |  Test selector        |
  |        (own instance)        |  Results / history    |
  |                              |                       |
  +------------------------------+-----------------------+

Tests:
  Straight Line   Drive forward, correct lateral veer, measure drift
  Rectangle       Drive a rectangle, measure return-to-start error
  Rotation        Spin N turns, measure angular drift
  Velocity Sweep  Auto-run straight lines at increasing speeds
  Auto Calibrate  Continuously run + auto-adjust stopping offsets
"""

import json
import math
import os
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QLabel,
    QPushButton, QFrame, QGridLayout, QDoubleSpinBox,
    QSpinBox, QComboBox, QGroupBox, QPlainTextEdit,
    QSizePolicy, QCheckBox, QProgressBar, QScrollArea,
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QColor

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, TEXT, SUCCESS, WARNING, DANGER,
    BORDER, BG_DARK, BG_CARD, BG_MID,
)
from TeamControl.ui.field_canvas import FieldCanvas
from TeamControl.network.robot_command import RobotCommand
from TeamControl.network.sender import Sender
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.robot.constants import (
    HALF_LEN, HALF_WID, FIELD_MARGIN, _TUNING_PATH,
)
from TeamControl.world.transform_cords import world2robot


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _card(title_text):
    card = QFrame()
    card.setObjectName("card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(12, 10, 12, 10)
    lay.setSpacing(6)
    title = QLabel(title_text)
    title.setStyleSheet(
        f"font-size:13px; font-weight:bold; color:{ACCENT}; padding:0;")
    lay.addWidget(title)
    return card, lay


_SAFE_MARGIN = 400          # mm from field edge for waypoints
_ARRIVE = 120               # mm — close enough to a waypoint
_TURN_THRESH = 0.20         # rad — turn-in-place above this
_TURN_W = 0.15              # rad/s max turning speed
_SETTLE_MS = 1500           # ms wait after stop to measure drift
_RECT_W = 1800              # mm rectangle width
_RECT_H = 1200              # mm rectangle height
_LATERAL_GAIN = 0.0003      # vy correction per mm of lateral error

# Auto-cal saves results here
_CAL_DATA_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir,
    "calibration.json"))


class CalibrationPage(QWidget):
    """Full calibration tab with its own field and multiple test routines."""

    def __init__(self, parent=None, engine=None, test_panel=None):
        super().__init__(parent)
        self._engine = engine
        self._test_panel = test_panel
        self._our_id_spin = None          # set by MainWindow
        self._sender = Sender()
        self._grsim: grSimSender | None = None

        # --- Layout ---
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        splitter = QSplitter(Qt.Horizontal)

        # Left: own field canvas
        self._field = FieldCanvas()
        splitter.addWidget(self._field)

        # Right: scrollable control sidebar
        sidebar = QWidget()
        sidebar.setMinimumWidth(360)
        sidebar.setMaximumWidth(520)
        sb_lay = QVBoxLayout(sidebar)
        sb_lay.setContentsMargins(0, 0, 0, 0)
        sb_lay.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"QScrollArea {{ background: {BG_DARK}; border: none; }}")
        scroll_inner = QWidget()
        scroll_inner.setStyleSheet(f"background: {BG_DARK};")
        inner_lay = QVBoxLayout(scroll_inner)
        inner_lay.setContentsMargins(8, 8, 8, 8)
        inner_lay.setSpacing(8)

        self._build_test_selector(inner_lay)
        self._build_auto_card(inner_lay)
        self._build_straight_card(inner_lay)
        self._build_rect_card(inner_lay)
        self._build_rotation_card(inner_lay)
        self._build_sweep_card(inner_lay)
        self._build_results_card(inner_lay)

        inner_lay.addStretch()
        scroll.setWidget(scroll_inner)
        sb_lay.addWidget(scroll)

        splitter.addWidget(sidebar)

        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1100, 400])
        root.addWidget(splitter)

        # --- Timers ---
        self._tick_timer = QTimer(self)
        self._tick_timer.setInterval(50)   # 20 Hz
        self._tick_timer.timeout.connect(self._tick)

        self._settle_timer = QTimer(self)
        self._settle_timer.setSingleShot(True)
        self._settle_timer.timeout.connect(self._on_settle)

        # --- State ---
        self._phase = None       # current test phase
        self._test_type = None   # "straight", "rect", "rotation", "sweep", "auto"
        self._waypoints = []     # list of (x, y) targets
        self._wp_idx = 0
        self._start_pose = None  # (x, y, o) at test start
        self._stop_pose = None   # (x, y, o) when we stopped
        self._leg_starts = []    # per-leg start positions for rectangle
        self._line_y = 0.0       # ideal Y for straight line correction
        self._history: dict[str, list] = {
            "straight": [], "rect": [], "rotation": [], "sweep": [],
        }
        self._sweep_speeds = []
        self._sweep_idx = 0
        self._continuous = False

        # Auto-calibration state
        self._auto_run_count = 0
        self._auto_direction = 1    # 1 = left-to-right, -1 = right-to-left
        self._cal_data = self._load_cal_data()

        # Feed engine frames to our field
        self._frame_timer = QTimer(self)
        self._frame_timer.setInterval(50)
        self._frame_timer.timeout.connect(self._poll_frame)
        self._frame_timer.start()

    # ==================================================================
    # Public API
    # ==================================================================

    def set_our_bot_spin(self, spin):
        self._our_id_spin = spin

    # ==================================================================
    # UI builders
    # ==================================================================

    def _build_test_selector(self, parent_lay):
        card, lay = _card("Calibration")
        lay.addWidget(QLabel(
            "Run motor calibration tests. The robot drives patterns "
            "and measures drift, straightness, and accuracy."))

        row = QHBoxLayout()
        row.addWidget(QLabel("Speed:"))
        self._speed_spin = QDoubleSpinBox()
        self._speed_spin.setRange(0.05, 2.0)
        self._speed_spin.setValue(0.20)
        self._speed_spin.setSingleStep(0.05)
        self._speed_spin.setSuffix(" m/s")
        row.addWidget(self._speed_spin)
        lay.addLayout(row)

        self._cont_cb = QCheckBox("Continuous (repeat until stopped)")
        lay.addWidget(self._cont_cb)

        self._status_lbl = QLabel("Ready")
        self._status_lbl.setStyleSheet(f"color:{TEXT_DIM}; font-size:12px;")
        self._status_lbl.setWordWrap(True)
        lay.addWidget(self._status_lbl)

        self._progress = QProgressBar()
        self._progress.setRange(0, 100)
        self._progress.setValue(0)
        self._progress.setFixedHeight(8)
        self._progress.setTextVisible(False)
        lay.addWidget(self._progress)

        # Big stop button
        self._stop_btn = QPushButton("STOP ALL")
        self._stop_btn.setObjectName("stopBtn")
        self._stop_btn.setMinimumHeight(36)
        self._stop_btn.clicked.connect(self._stop_test)
        lay.addWidget(self._stop_btn)

        parent_lay.addWidget(card)

    # --- Auto Calibrate ---
    def _build_auto_card(self, parent_lay):
        card, lay = _card("Auto Calibrate")
        lay.addWidget(QLabel(
            "Continuously drives back and forth across the field, "
            "corrects lateral veer, measures stopping drift at each speed, "
            "and builds a calibration profile. Saves to calibration.json."))

        row = QHBoxLayout()
        row.addWidget(QLabel("From:"))
        self._auto_lo = QDoubleSpinBox()
        self._auto_lo.setRange(0.05, 1.0)
        self._auto_lo.setValue(0.10)
        self._auto_lo.setSingleStep(0.05)
        self._auto_lo.setSuffix(" m/s")
        row.addWidget(self._auto_lo)
        row.addWidget(QLabel("To:"))
        self._auto_hi = QDoubleSpinBox()
        self._auto_hi.setRange(0.10, 2.0)
        self._auto_hi.setValue(0.50)
        self._auto_hi.setSingleStep(0.05)
        self._auto_hi.setSuffix(" m/s")
        row.addWidget(self._auto_hi)
        row.addWidget(QLabel("Step:"))
        self._auto_step = QDoubleSpinBox()
        self._auto_step.setRange(0.05, 0.5)
        self._auto_step.setValue(0.10)
        self._auto_step.setSingleStep(0.05)
        self._auto_step.setSuffix(" m/s")
        row.addWidget(self._auto_step)
        lay.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Runs per speed:"))
        self._auto_reps = QSpinBox()
        self._auto_reps.setRange(1, 50)
        self._auto_reps.setValue(3)
        row2.addWidget(self._auto_reps)
        lay.addLayout(row2)

        btn = QPushButton("Start Auto Calibration")
        btn.setObjectName("startBtn")
        btn.setMinimumHeight(36)
        btn.clicked.connect(self._start_auto)
        lay.addWidget(btn)
        self._auto_btn = btn

        self._auto_result = QLabel("")
        self._auto_result.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._auto_result.setWordWrap(True)
        lay.addWidget(self._auto_result)

        self._auto_profile = QLabel(self._format_profile())
        self._auto_profile.setStyleSheet(f"color:{TEXT_DIM}; font-size:10px;")
        self._auto_profile.setWordWrap(True)
        lay.addWidget(self._auto_profile)

        parent_lay.addWidget(card)

    # --- Straight Line ---
    def _build_straight_card(self, parent_lay):
        card, lay = _card("Straight Line Test")
        lay.addWidget(QLabel(
            "Drives left-to-right across the field. Corrects lateral veer. "
            "Measures lateral drift and stopping distance."))
        btn = QPushButton("Run Straight Line")
        btn.setObjectName("startBtn")
        btn.setMinimumHeight(32)
        btn.clicked.connect(lambda: self._start_test("straight"))
        lay.addWidget(btn)
        self._straight_btn = btn
        self._straight_result = QLabel("")
        self._straight_result.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._straight_result.setWordWrap(True)
        lay.addWidget(self._straight_result)
        parent_lay.addWidget(card)

    # --- Rectangle ---
    def _build_rect_card(self, parent_lay):
        card, lay = _card("Rectangle Test")
        lay.addWidget(QLabel(
            "Drives a rectangle and returns to start. Measures "
            "accumulated position error and per-leg straightness."))

        size_row = QHBoxLayout()
        size_row.addWidget(QLabel("W:"))
        self._rect_w_spin = QSpinBox()
        self._rect_w_spin.setRange(400, 4000)
        self._rect_w_spin.setValue(_RECT_W)
        self._rect_w_spin.setSuffix(" mm")
        self._rect_w_spin.setSingleStep(200)
        size_row.addWidget(self._rect_w_spin)
        size_row.addWidget(QLabel("H:"))
        self._rect_h_spin = QSpinBox()
        self._rect_h_spin.setRange(400, 2400)
        self._rect_h_spin.setValue(_RECT_H)
        self._rect_h_spin.setSuffix(" mm")
        self._rect_h_spin.setSingleStep(200)
        size_row.addWidget(self._rect_h_spin)
        lay.addLayout(size_row)

        btn = QPushButton("Run Rectangle")
        btn.setObjectName("startBtn")
        btn.setMinimumHeight(32)
        btn.clicked.connect(lambda: self._start_test("rect"))
        lay.addWidget(btn)
        self._rect_btn = btn
        self._rect_result = QLabel("")
        self._rect_result.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._rect_result.setWordWrap(True)
        lay.addWidget(self._rect_result)
        parent_lay.addWidget(card)

    # --- Rotation ---
    def _build_rotation_card(self, parent_lay):
        card, lay = _card("Rotation Test")
        lay.addWidget(QLabel(
            "Spins the robot N full turns and checks if orientation "
            "returns to the start angle."))

        row = QHBoxLayout()
        row.addWidget(QLabel("Turns:"))
        self._rot_turns = QSpinBox()
        self._rot_turns.setRange(1, 20)
        self._rot_turns.setValue(4)
        row.addWidget(self._rot_turns)
        row.addWidget(QLabel("Speed:"))
        self._rot_speed = QDoubleSpinBox()
        self._rot_speed.setRange(0.05, 1.0)
        self._rot_speed.setValue(0.3)
        self._rot_speed.setSingleStep(0.05)
        self._rot_speed.setSuffix(" rad/s")
        row.addWidget(self._rot_speed)
        lay.addLayout(row)

        btn = QPushButton("Run Rotation")
        btn.setObjectName("startBtn")
        btn.setMinimumHeight(32)
        btn.clicked.connect(lambda: self._start_test("rotation"))
        lay.addWidget(btn)
        self._rot_btn = btn
        self._rot_result = QLabel("")
        self._rot_result.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._rot_result.setWordWrap(True)
        lay.addWidget(self._rot_result)
        parent_lay.addWidget(card)

    # --- Velocity Sweep ---
    def _build_sweep_card(self, parent_lay):
        card, lay = _card("Velocity Sweep")
        lay.addWidget(QLabel(
            "Runs straight-line tests at increasing speeds to build "
            "a stopping-distance profile."))

        row = QHBoxLayout()
        row.addWidget(QLabel("From:"))
        self._sweep_lo = QDoubleSpinBox()
        self._sweep_lo.setRange(0.05, 1.0)
        self._sweep_lo.setValue(0.10)
        self._sweep_lo.setSingleStep(0.05)
        self._sweep_lo.setSuffix(" m/s")
        row.addWidget(self._sweep_lo)
        row.addWidget(QLabel("To:"))
        self._sweep_hi = QDoubleSpinBox()
        self._sweep_hi.setRange(0.10, 2.0)
        self._sweep_hi.setValue(0.50)
        self._sweep_hi.setSingleStep(0.05)
        self._sweep_hi.setSuffix(" m/s")
        row.addWidget(self._sweep_hi)
        row.addWidget(QLabel("Step:"))
        self._sweep_step = QDoubleSpinBox()
        self._sweep_step.setRange(0.05, 0.5)
        self._sweep_step.setValue(0.10)
        self._sweep_step.setSingleStep(0.05)
        self._sweep_step.setSuffix(" m/s")
        row.addWidget(self._sweep_step)
        lay.addLayout(row)

        btn = QPushButton("Run Sweep")
        btn.setObjectName("startBtn")
        btn.setMinimumHeight(32)
        btn.clicked.connect(lambda: self._start_test("sweep"))
        lay.addWidget(btn)
        self._sweep_btn = btn
        self._sweep_result = QLabel("")
        self._sweep_result.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._sweep_result.setWordWrap(True)
        lay.addWidget(self._sweep_result)
        parent_lay.addWidget(card)

    # --- Results log ---
    def _build_results_card(self, parent_lay):
        card, lay = _card("Results Log")
        self._results_log = QPlainTextEdit()
        self._results_log.setReadOnly(True)
        self._results_log.setMaximumHeight(200)
        self._results_log.setPlaceholderText("Run a test to see results here...")
        lay.addWidget(self._results_log)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self._results_log.clear)
        lay.addWidget(clear_btn)

        parent_lay.addWidget(card)

    # ==================================================================
    # Frame polling — keep our field in sync with engine
    # ==================================================================

    def _poll_frame(self):
        if not self._engine or not self._engine._wm:
            return
        try:
            frame = self._engine._wm.get_latest_frame()
            if frame is None:
                return
            snap = self._engine._extract_snapshot(frame)
            self._field.set_frame(snap)
        except Exception:
            pass

    # ==================================================================
    # Robot helpers
    # ==================================================================

    def _get_rid_and_yellow(self):
        rid = self._our_id_spin.value() if self._our_id_spin else 0
        cfg = self._engine.config if self._engine else None
        is_yellow = cfg.us_yellow if cfg else True
        return rid, is_yellow

    def _get_robot_pose(self):
        if not self._engine or not self._engine._wm:
            return None
        frame = self._engine._wm.get_latest_frame()
        if frame is None:
            return None
        rid, is_yellow = self._get_rid_and_yellow()
        team = frame.robots_yellow if is_yellow else frame.robots_blue
        try:
            robot = team[rid]
        except (IndexError, TypeError):
            return None
        from TeamControl.SSL.vision.robots import Robot
        if not isinstance(robot, Robot):
            return None
        return (float(robot.x), float(robot.y), float(robot.o))

    def _send_cmd(self, vx=0.0, vy=0.0, w=0.0, kick=0, dribble=0):
        rid, is_yellow = self._get_rid_and_yellow()
        cmd = RobotCommand(
            robot_id=rid, vx=vx, vy=vy, w=w,
            kick=kick, dribble=dribble, isYellow=is_yellow)
        if self._test_panel:
            self._test_panel._do_send(cmd)
        else:
            try:
                if self._engine and self._engine._grsim_sender:
                    self._engine._grsim_sender.send_robot_command(cmd)
            except Exception:
                pass

    def _send_stop(self):
        self._send_cmd(vx=0.0, vy=0.0, w=0.0)

    # ==================================================================
    # Test lifecycle
    # ==================================================================

    def _start_test(self, test_type):
        if self._phase is not None:
            self._stop_test()

        pose = self._get_robot_pose()
        if pose is None:
            self._set_status("No robot position — is the engine running?", DANGER)
            return

        self._test_type = test_type
        self._continuous = self._cont_cb.isChecked()
        self._start_pose = pose
        self._leg_starts = []

        if test_type == "straight":
            self._setup_straight()
        elif test_type == "rect":
            self._setup_rect()
        elif test_type == "rotation":
            self._setup_rotation()
        elif test_type == "sweep":
            self._setup_sweep()

    def _stop_test(self):
        self._tick_timer.stop()
        self._settle_timer.stop()
        self._send_stop()
        self._phase = None
        self._test_type = None
        self._continuous = False
        self._field.set_targets([])
        self._field.set_paths([])
        self._progress.setValue(0)
        self._set_status("Stopped", TEXT_DIM)

    def _set_status(self, text, color=TEXT_DIM):
        self._status_lbl.setText(text)
        self._status_lbl.setStyleSheet(f"color:{color}; font-size:12px;")

    def _log_result(self, line):
        self._results_log.appendPlainText(line)

    # ==================================================================
    # Setup routines
    # ==================================================================

    def _setup_straight(self, direction=1):
        m = _SAFE_MARGIN
        if direction == 1:
            self._waypoints = [
                (-HALF_LEN + m, 0),
                (HALF_LEN - m, 0),
            ]
        else:
            self._waypoints = [
                (HALF_LEN - m, 0),
                (-HALF_LEN + m, 0),
            ]
        self._wp_idx = 0
        self._phase = "nav"
        self._line_y = 0.0  # will be set when we start the measured run
        self._field.set_targets(self._waypoints)
        self._field.set_paths([self._waypoints])
        self._set_status("Navigating to start position...", WARNING)
        self._progress.setValue(5)
        self._tick_timer.start()

    def _setup_rect(self):
        rw = self._rect_w_spin.value()
        rh = self._rect_h_spin.value()
        hw, hh = rw / 2, rh / 2

        max_x = HALF_LEN - _SAFE_MARGIN
        max_y = HALF_WID - _SAFE_MARGIN
        hw = min(hw, max_x)
        hh = min(hh, max_y)

        self._waypoints = [
            (-hw, -hh),
            (hw, -hh),
            (hw, hh),
            (-hw, hh),
            (-hw, -hh),
        ]
        self._wp_idx = 0
        self._phase = "nav"
        self._field.set_targets(self._waypoints)
        self._field.set_paths([self._waypoints])
        self._set_status("Navigating to start corner...", WARNING)
        self._progress.setValue(5)
        self._tick_timer.start()

    def _setup_rotation(self):
        self._waypoints = []
        self._wp_idx = 0
        self._rot_accumulated = 0.0
        self._rot_last_angle = None
        n_turns = self._rot_turns.value()
        self._rot_target_rad = n_turns * 2 * math.pi
        self._phase = "rotating"
        self._field.set_targets([])
        self._field.set_paths([])
        self._set_status(
            f"Spinning {n_turns} turns at "
            f"{self._rot_speed.value():.2f} rad/s...", SUCCESS)
        self._progress.setValue(10)
        self._tick_timer.start()

    def _setup_sweep(self):
        lo = self._sweep_lo.value()
        hi = self._sweep_hi.value()
        step = self._sweep_step.value()
        self._sweep_speeds = []
        v = lo
        while v <= hi + 0.001:
            self._sweep_speeds.append(round(v, 3))
            v += step
        if not self._sweep_speeds:
            self._set_status("Invalid sweep range", DANGER)
            return
        self._sweep_idx = 0
        self._sweep_result.setText("")
        self._log_result(f"--- Velocity Sweep: "
                         f"{self._sweep_speeds[0]:.2f} -> "
                         f"{self._sweep_speeds[-1]:.2f} m/s ---")
        self._speed_spin.setValue(self._sweep_speeds[0])
        self._setup_straight()
        self._test_type = "sweep"

    # ==================================================================
    # Main tick
    # ==================================================================

    def _tick(self):
        pose = self._get_robot_pose()
        if pose is None:
            return

        if self._test_type == "rotation":
            self._tick_rotation(pose)
            return

        if self._phase == "nav":
            self._tick_nav_to_start(pose)
        elif self._phase == "running":
            self._tick_running(pose)

    def _tick_nav_to_start(self, pose):
        """Navigate to the first waypoint (start position)."""
        target = self._waypoints[self._wp_idx]
        rel = world2robot(pose, target)
        dist = math.hypot(rel[0], rel[1])
        angle = math.atan2(rel[1], rel[0])

        if dist < _ARRIVE:
            self._send_stop()
            self._start_pose = pose
            self._leg_starts = [pose]
            # Record the Y we're starting at for lateral correction
            self._line_y = pose[1]
            self._wp_idx += 1
            if self._wp_idx >= len(self._waypoints):
                self._finish_test(pose)
                return
            self._phase = "running"
            wp_total = len(self._waypoints)
            self._set_status(
                f"Running — waypoint {self._wp_idx}/{wp_total - 1}...",
                SUCCESS)
            self._progress.setValue(20)
            return

        if abs(angle) > _TURN_THRESH:
            w = max(-_TURN_W, min(_TURN_W, angle * 0.3))
            self._send_cmd(w=w)
        else:
            speed = min(0.25, max(0.03, (dist - _ARRIVE) / 2000.0))
            self._send_cmd(vx=speed)

    def _tick_running(self, pose):
        """Drive through waypoints at calibration speed with lateral correction."""
        target = self._waypoints[self._wp_idx]
        rel = world2robot(pose, target)
        dist = math.hypot(rel[0], rel[1])
        angle = math.atan2(rel[1], rel[0])

        wp_total = len(self._waypoints)
        pct = 20 + int(80 * self._wp_idx / max(1, wp_total - 1))
        self._progress.setValue(min(pct, 95))

        if dist < _ARRIVE:
            self._leg_starts.append(pose)
            self._wp_idx += 1
            if self._wp_idx >= wp_total:
                self._send_stop()
                self._stop_pose = pose
                self._phase = "settling"
                self._set_status(
                    f"Stopped — settling for "
                    f"{_SETTLE_MS / 1000:.1f}s...", WARNING)
                self._settle_timer.start(_SETTLE_MS)
                return

            # Update line_y for the next leg (use current Y as baseline)
            self._line_y = pose[1]
            self._set_status(
                f"Running — waypoint {self._wp_idx}/{wp_total - 1}...",
                SUCCESS)
            return

        speed = self._speed_spin.value()

        # Lateral correction: if veering off the ideal line, apply vy
        lateral_err = pose[1] - self._line_y  # positive = drifted up
        # Convert to robot frame: need to correct sideways
        vy_correction = -lateral_err * _LATERAL_GAIN
        vy_correction = max(-0.15, min(0.15, vy_correction))

        if abs(angle) > _TURN_THRESH:
            w = max(-_TURN_W, min(_TURN_W, angle * 0.3))
            self._send_cmd(w=w, vy=vy_correction)
        else:
            ramp_speed = min(speed, max(0.03, (dist - _ARRIVE) / 1500.0))
            self._send_cmd(vx=ramp_speed, vy=vy_correction)

    def _tick_rotation(self, pose):
        """Spin and track accumulated rotation."""
        current_angle = pose[2]

        if self._rot_last_angle is not None:
            delta = current_angle - self._rot_last_angle
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            self._rot_accumulated += delta

        self._rot_last_angle = current_angle

        pct = min(95, int(100 * abs(self._rot_accumulated) /
                          self._rot_target_rad))
        self._progress.setValue(pct)

        if abs(self._rot_accumulated) >= self._rot_target_rad:
            self._send_stop()
            self._stop_pose = pose
            self._phase = "settling"
            self._set_status(
                f"Stopped — settling for {_SETTLE_MS / 1000:.1f}s...",
                WARNING)
            self._settle_timer.start(_SETTLE_MS)
            return

        w = self._rot_speed.value()
        self._send_cmd(w=w)
        self._set_status(
            f"Rotating: {math.degrees(self._rot_accumulated):.0f} deg / "
            f"{math.degrees(self._rot_target_rad):.0f} deg", SUCCESS)

    # ==================================================================
    # Settle + measurement
    # ==================================================================

    def _on_settle(self):
        """Called after the settle timer — measure final pose."""
        final = self._get_robot_pose()
        self._tick_timer.stop()
        self._progress.setValue(100)

        if final is None:
            self._set_status("Lost robot position during settle", DANGER)
            self._phase = None
            return

        self._field.set_targets([])
        self._field.set_paths([])

        if self._test_type in ("straight", "sweep", "auto"):
            self._measure_straight(final)
        elif self._test_type == "rect":
            self._measure_rect(final)
        elif self._test_type == "rotation":
            self._measure_rotation(final)

    def _measure_straight(self, final):
        start = self._start_pose
        stop = self._stop_pose

        stop_drift = math.hypot(
            final[0] - stop[0], final[1] - stop[1])
        lateral_drift = abs(final[1] - start[1])
        run_dist = math.hypot(
            stop[0] - start[0], stop[1] - start[1])

        speed = self._speed_spin.value()
        result = {
            "speed": speed,
            "run_mm": run_dist,
            "stop_drift_mm": stop_drift,
            "lateral_drift_mm": lateral_drift,
        }
        self._history["straight"].append(result)

        txt = (f"Speed: {speed:.2f} m/s\n"
               f"Run: {run_dist:.0f} mm\n"
               f"Stopping drift: {stop_drift:.0f} mm\n"
               f"Lateral drift: {lateral_drift:.0f} mm")
        self._straight_result.setText(txt)
        self._log_result(
            f"[straight] {speed:.2f} m/s | run={run_dist:.0f} | "
            f"stop_drift={stop_drift:.0f} | lateral={lateral_drift:.0f}")

        if self._test_type == "sweep":
            self._sweep_record(result)
            return
        if self._test_type == "auto":
            self._auto_record(result)
            return

        self._set_status("Done", SUCCESS)
        self._phase = None
        self._maybe_repeat()

    def _measure_rect(self, final):
        start = self._start_pose

        return_err = math.hypot(
            final[0] - start[0], final[1] - start[1])

        angle_err = final[2] - start[2]
        while angle_err > math.pi:
            angle_err -= 2 * math.pi
        while angle_err < -math.pi:
            angle_err += 2 * math.pi
        angle_err = abs(angle_err)

        leg_drifts = []
        for i in range(len(self._leg_starts) - 1):
            s = self._leg_starts[i]
            e = self._leg_starts[i + 1]
            leg_len = math.hypot(e[0] - s[0], e[1] - s[1])
            if leg_len < 10:
                continue
            wp = self._waypoints[i + 1] if i + 1 < len(self._waypoints) else e
            wp_err = math.hypot(e[0] - wp[0], e[1] - wp[1])
            leg_drifts.append(wp_err)

        avg_leg_err = (sum(leg_drifts) / len(leg_drifts)) if leg_drifts else 0

        speed = self._speed_spin.value()
        result = {
            "speed": speed,
            "return_err_mm": return_err,
            "angle_err_deg": math.degrees(angle_err),
            "avg_leg_err_mm": avg_leg_err,
        }
        self._history["rect"].append(result)

        txt = (f"Speed: {speed:.2f} m/s\n"
               f"Return-to-start error: {return_err:.0f} mm\n"
               f"Orientation drift: {math.degrees(angle_err):.1f} deg\n"
               f"Avg per-leg error: {avg_leg_err:.0f} mm")
        self._rect_result.setText(txt)
        self._log_result(
            f"[rect] {speed:.2f} m/s | return={return_err:.0f} | "
            f"angle={math.degrees(angle_err):.1f} deg | "
            f"leg_err={avg_leg_err:.0f}")

        self._set_status("Done", SUCCESS)
        self._phase = None
        self._maybe_repeat()

    def _measure_rotation(self, final):
        start = self._start_pose

        pos_drift = math.hypot(
            final[0] - start[0], final[1] - start[1])

        angle_err = final[2] - start[2]
        while angle_err > math.pi:
            angle_err -= 2 * math.pi
        while angle_err < -math.pi:
            angle_err += 2 * math.pi

        n_turns = self._rot_turns.value()
        w_speed = self._rot_speed.value()
        result = {
            "turns": n_turns,
            "w_speed": w_speed,
            "pos_drift_mm": pos_drift,
            "angle_err_deg": math.degrees(angle_err),
        }
        self._history["rotation"].append(result)

        txt = (f"{n_turns} turns at {w_speed:.2f} rad/s\n"
               f"Position drift: {pos_drift:.0f} mm\n"
               f"Angle error: {math.degrees(angle_err):.1f} deg")
        self._rot_result.setText(txt)
        self._log_result(
            f"[rotation] {n_turns}T @ {w_speed:.2f} | "
            f"pos_drift={pos_drift:.0f} | "
            f"angle_err={math.degrees(angle_err):.1f} deg")

        self._set_status("Done", SUCCESS)
        self._phase = None
        self._maybe_repeat()

    # ==================================================================
    # Sweep logic
    # ==================================================================

    def _sweep_record(self, result):
        self._sweep_idx += 1
        n = len(self._sweep_speeds)
        pct = int(100 * self._sweep_idx / n)
        self._progress.setValue(pct)

        if self._sweep_idx >= n:
            self._sweep_summarize()
            self._phase = None
            self._test_type = None
            self._set_status("Sweep complete", SUCCESS)
            return

        next_speed = self._sweep_speeds[self._sweep_idx]
        self._speed_spin.setValue(next_speed)
        self._set_status(
            f"Sweep {self._sweep_idx + 1}/{n}: {next_speed:.2f} m/s...",
            WARNING)
        self._setup_straight()
        self._test_type = "sweep"

    def _sweep_summarize(self):
        lines = ["--- Sweep Results ---"]
        lines.append(f"{'Speed':>8s}  {'Stop Drift':>10s}  {'Lateral':>8s}")
        for r in self._history["straight"][-len(self._sweep_speeds):]:
            lines.append(
                f"{r['speed']:8.2f}  {r['stop_drift_mm']:10.0f}  "
                f"{r['lateral_drift_mm']:8.0f}")
        txt = "\n".join(lines)
        self._sweep_result.setText(txt)
        self._log_result(txt)

    # ==================================================================
    # Auto-calibration
    # ==================================================================

    def _start_auto(self):
        if self._phase is not None:
            self._stop_test()

        pose = self._get_robot_pose()
        if pose is None:
            self._set_status("No robot position — is the engine running?", DANGER)
            return

        # Build speed list
        lo = self._auto_lo.value()
        hi = self._auto_hi.value()
        step = self._auto_step.value()
        self._sweep_speeds = []
        v = lo
        while v <= hi + 0.001:
            self._sweep_speeds.append(round(v, 3))
            v += step
        if not self._sweep_speeds:
            self._set_status("Invalid speed range", DANGER)
            return

        self._auto_run_count = 0
        self._auto_direction = 1
        self._sweep_idx = 0
        self._auto_reps_done = 0
        self._test_type = "auto"
        self._continuous = False  # auto handles its own repeat

        self._log_result(f"--- Auto Calibration: "
                         f"{self._sweep_speeds[0]:.2f} -> "
                         f"{self._sweep_speeds[-1]:.2f} m/s, "
                         f"{self._auto_reps.value()} reps each ---")

        # Start the first run
        self._speed_spin.setValue(self._sweep_speeds[0])
        self._start_pose = pose
        self._setup_straight(direction=self._auto_direction)
        self._test_type = "auto"

    def _auto_record(self, result):
        """Record one auto-cal data point, then decide what's next."""
        speed_key = f"{result['speed']:.2f}"

        # Store in cal_data
        if speed_key not in self._cal_data:
            self._cal_data[speed_key] = {
                "stop_drift": [], "lateral_drift": [],
            }
        self._cal_data[speed_key]["stop_drift"].append(result["stop_drift_mm"])
        self._cal_data[speed_key]["lateral_drift"].append(result["lateral_drift_mm"])

        self._auto_run_count += 1
        self._auto_reps_done += 1
        reps_needed = self._auto_reps.value()

        # Update display
        avg_stop = (sum(self._cal_data[speed_key]["stop_drift"])
                    / len(self._cal_data[speed_key]["stop_drift"]))
        avg_lat = (sum(self._cal_data[speed_key]["lateral_drift"])
                   / len(self._cal_data[speed_key]["lateral_drift"]))
        n = len(self._cal_data[speed_key]["stop_drift"])

        self._auto_result.setText(
            f"Run #{self._auto_run_count} | {result['speed']:.2f} m/s\n"
            f"Stop drift: {result['stop_drift_mm']:.0f} mm "
            f"(avg: {avg_stop:.0f}, n={n})\n"
            f"Lateral: {result['lateral_drift_mm']:.0f} mm "
            f"(avg: {avg_lat:.0f})")

        self._auto_profile.setText(self._format_profile())

        # Save after every run
        self._save_cal_data()

        # Check if we need more reps at this speed
        if self._auto_reps_done < reps_needed:
            # Same speed, flip direction
            self._auto_direction *= -1
            self._set_status(
                f"Auto: {result['speed']:.2f} m/s — "
                f"rep {self._auto_reps_done + 1}/{reps_needed}...", WARNING)
            self._setup_straight(direction=self._auto_direction)
            self._test_type = "auto"
            return

        # Move to next speed
        self._sweep_idx += 1
        self._auto_reps_done = 0

        if self._sweep_idx >= len(self._sweep_speeds):
            # All speeds done
            self._save_cal_data()
            self._phase = None
            self._test_type = None
            self._set_status(
                f"Auto calibration complete — {self._auto_run_count} runs, "
                f"saved to calibration.json", SUCCESS)
            self._log_result(
                f"[auto] Complete: {self._auto_run_count} runs across "
                f"{len(self._sweep_speeds)} speeds")
            self._auto_profile.setText(self._format_profile())
            return

        # Next speed
        next_speed = self._sweep_speeds[self._sweep_idx]
        self._speed_spin.setValue(next_speed)
        self._auto_direction *= -1
        n_total = len(self._sweep_speeds) * reps_needed
        self._set_status(
            f"Auto: {next_speed:.2f} m/s — "
            f"run {self._auto_run_count + 1}/{n_total}...", WARNING)
        self._setup_straight(direction=self._auto_direction)
        self._test_type = "auto"

    # ==================================================================
    # Calibration data persistence
    # ==================================================================

    def _load_cal_data(self) -> dict:
        try:
            with open(_CAL_DATA_PATH, "r") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def _save_cal_data(self):
        try:
            with open(_CAL_DATA_PATH, "w") as f:
                json.dump(self._cal_data, f, indent=2)
        except Exception:
            pass

    def _format_profile(self):
        if not self._cal_data:
            return "No calibration data yet. Run Auto Calibrate to build a profile."
        lines = ["Calibration Profile (from calibration.json):"]
        lines.append(f"{'Speed':>8s}  {'Avg Stop':>9s}  {'Avg Lat':>8s}  {'Runs':>5s}")
        for key in sorted(self._cal_data.keys(), key=float):
            d = self._cal_data[key]
            stops = d.get("stop_drift", [])
            lats = d.get("lateral_drift", [])
            avg_s = sum(stops) / len(stops) if stops else 0
            avg_l = sum(lats) / len(lats) if lats else 0
            n = len(stops)
            lines.append(f"{key:>8s}  {avg_s:9.0f}  {avg_l:8.0f}  {n:5d}")
        return "\n".join(lines)

    # ==================================================================
    # Continuous repeat
    # ==================================================================

    def _maybe_repeat(self):
        if self._continuous and self._test_type is not None:
            QTimer.singleShot(500, lambda: self._start_test(self._test_type))
        else:
            self._test_type = None

    # ==================================================================
    # Finish
    # ==================================================================

    def _finish_test(self, pose):
        self._send_stop()
        self._stop_pose = pose
        self._phase = "settling"
        self._settle_timer.start(_SETTLE_MS)
