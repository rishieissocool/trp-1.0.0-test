"""
Dashboard page — the main operational view.

Layout:
  ┌────────────────────────────────────┬──────────────────┐
  │                                    │  Robots table    │
  │           Field Canvas             │  Game state      │
  │           (fills space)            │  Network info    │
  │                                    │                  │
  └────────────────────────────────────┴──────────────────┘
"""

import math
import time

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QLabel,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QGroupBox, QGridLayout, QProgressBar, QFrame, QPushButton,
    QSizePolicy, QDoubleSpinBox, QComboBox,
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QColor, QFont

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, WARNING, DANGER, BORDER,
    YELLOW_TEAM, BLUE_TEAM, BG_DARK, BG_CARD, BG_MID,
)


def _card(title_text):
    """Create a styled card frame with a title."""
    card = QFrame()
    card.setObjectName("card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(12, 10, 12, 10)
    lay.setSpacing(6)
    title = QLabel(title_text)
    title.setStyleSheet(f"font-size:13px; font-weight:bold; color:{ACCENT}; padding:0;")
    lay.addWidget(title)
    return card, lay


def _val_label(text="—", size=12, bold=True):
    lbl = QLabel(text)
    weight = QFont.Bold if bold else QFont.Normal
    lbl.setFont(QFont("Segoe UI", size, weight))
    return lbl


class _StatusDot(QLabel):
    def __init__(self):
        super().__init__("●")
        self.setFixedWidth(18)
        self.setAlignment(Qt.AlignCenter)
        self.set_ok(False)

    def set_ok(self, ok):
        c = SUCCESS if ok else DANGER
        self.setStyleSheet(f"color:{c}; font-size:14px;")


class DashboardPage(QWidget):
    """Field + sidebar with robots, game state, and network."""

    coordinate_hover = Signal(float, float)

    def __init__(self, field_canvas, parent=None, engine=None, test_panel=None):
        super().__init__(parent)
        self._field = field_canvas
        self._engine = engine
        self._test_panel = test_panel

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        splitter = QSplitter(Qt.Horizontal)

        # ── Left: Field canvas (stretches) ────────────────────────
        splitter.addWidget(self._field)

        # ── Right: sidebar ────────────────────────────────────────
        sidebar = QWidget()
        sidebar.setMinimumWidth(320)
        sidebar.setMaximumWidth(480)
        sb_lay = QVBoxLayout(sidebar)
        sb_lay.setContentsMargins(8, 8, 8, 8)
        sb_lay.setSpacing(8)

        # --- Robot table card ---
        self._build_robot_card(sb_lay)

        # --- Game state card ---
        self._build_game_card(sb_lay)

        # --- Network card ---
        self._build_network_card(sb_lay)

        # --- Calibration card ---
        self._build_calibration_card(sb_lay)

        sb_lay.addStretch()
        splitter.addWidget(sidebar)

        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1100, 360])

        root.addWidget(splitter)

        self._field.coordinate_hover.connect(self.coordinate_hover.emit)

        # FPS tracking
        self._frame_times: list[float] = []
        self._last_frame_time = 0.0

    # ── Robot table ───────────────────────────────────────────────

    def _build_robot_card(self, parent_lay):
        card, lay = _card("Robots")
        self._robot_summary = QLabel("Waiting for data…")
        self._robot_summary.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        lay.addWidget(self._robot_summary)

        cols = ["Team", "ID", "X", "Y", "θ°", "Conf"]
        self._robot_table = QTableWidget(0, len(cols))
        self._robot_table.setHorizontalHeaderLabels(cols)
        self._robot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._robot_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._robot_table.setAlternatingRowColors(True)
        self._robot_table.verticalHeader().setVisible(False)
        self._robot_table.setShowGrid(False)
        self._robot_table.setMaximumHeight(260)
        hh = self._robot_table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setMinimumSectionSize(40)
        lay.addWidget(self._robot_table)
        parent_lay.addWidget(card)

    # ── Game state ────────────────────────────────────────────────

    def _build_game_card(self, parent_lay):
        card, lay = _card("Game State")

        grid = QGridLayout()
        grid.setSpacing(6)

        grid.addWidget(QLabel("State:"), 0, 0)
        self._gs_state = _val_label("WAITING")
        grid.addWidget(self._gs_state, 0, 1)

        grid.addWidget(QLabel("Mode:"), 1, 0)
        self._gs_mode = _val_label("—")
        grid.addWidget(self._gs_mode, 1, 1)

        # Score
        grid.addWidget(QLabel("Score:"), 2, 0)
        score_row = QHBoxLayout()
        self._score_y = QLabel("0")
        self._score_y.setStyleSheet(
            f"font-size:22px; font-weight:bold; color:{YELLOW_TEAM};")
        self._score_vs = QLabel(" vs ")
        self._score_vs.setStyleSheet("font-size:14px; font-weight:bold;")
        self._score_b = QLabel("0")
        self._score_b.setStyleSheet(
            f"font-size:22px; font-weight:bold; color:{BLUE_TEAM};")
        score_row.addWidget(self._score_y)
        score_row.addWidget(self._score_vs)
        score_row.addWidget(self._score_b)
        score_row.addStretch()
        grid.addLayout(score_row, 2, 1)

        lay.addLayout(grid)
        parent_lay.addWidget(card)

    # ── Network ───────────────────────────────────────────────────

    def _build_network_card(self, parent_lay):
        card, lay = _card("Network")

        grid = QGridLayout()
        grid.setSpacing(5)

        self._vis_dot = _StatusDot()
        self._gc_dot = _StatusDot()
        self._grsim_dot = _StatusDot()

        for i, (name, dot) in enumerate([
            ("Vision", self._vis_dot),
            ("Game Controller", self._gc_dot),
            ("grSim", self._grsim_dot),
        ]):
            grid.addWidget(dot, i, 0)
            grid.addWidget(QLabel(name), i, 1)

        lay.addLayout(grid)

        fps_row = QHBoxLayout()
        fps_row.setSpacing(8)
        self._fps_lbl = QLabel("0 fps")
        self._fps_lbl.setFont(QFont("Segoe UI", 14, QFont.Bold))
        self._fps_bar = QProgressBar()
        self._fps_bar.setRange(0, 60)
        self._fps_bar.setValue(0)
        self._fps_bar.setTextVisible(False)
        self._fps_bar.setFixedHeight(8)
        fps_row.addWidget(self._fps_lbl)
        fps_row.addWidget(self._fps_bar, 1)
        lay.addLayout(fps_row)

        parent_lay.addWidget(card)

    # ── Calibration ─────────────────────────────────────────────

    _CAL_MARGIN = 500  # mm from field edge
    _CAL_STOP_ZONE = 400  # mm — start braking this far from target
    _CAL_ARRIVE = 150  # mm — close enough = arrived

    def _build_calibration_card(self, parent_lay):
        card, lay = _card("Velocity Calibration")

        lay.addWidget(QLabel(
            "Drives the robot across the field (left→right), "
            "stops, and measures how far it drifts."))

        grid = QGridLayout()
        grid.setSpacing(6)

        grid.addWidget(QLabel("Speed:"), 0, 0)
        self._cal_speed = QDoubleSpinBox()
        self._cal_speed.setRange(0.05, 2.0)
        self._cal_speed.setValue(0.2)
        self._cal_speed.setSingleStep(0.05)
        self._cal_speed.setSuffix(" m/s")
        grid.addWidget(self._cal_speed, 0, 1)

        lay.addLayout(grid)

        self._cal_run_btn = QPushButton("Run Calibration")
        self._cal_run_btn.setObjectName("startBtn")
        self._cal_run_btn.setMinimumHeight(34)
        self._cal_run_btn.clicked.connect(self._cal_start)
        lay.addWidget(self._cal_run_btn)

        self._cal_status = QLabel("Ready")
        self._cal_status.setStyleSheet(f"color:{TEXT_DIM}; font-size:11px;")
        self._cal_status.setWordWrap(True)
        lay.addWidget(self._cal_status)

        self._cal_results = QLabel("")
        self._cal_results.setStyleSheet(f"color:{ACCENT}; font-size:11px;")
        self._cal_results.setWordWrap(True)
        lay.addWidget(self._cal_results)

        self._cal_estimates = QLabel(self._format_estimates({}))
        self._cal_estimates.setStyleSheet(f"color:{TEXT_DIM}; font-size:10px;")
        self._cal_estimates.setWordWrap(True)
        lay.addWidget(self._cal_estimates)

        parent_lay.addWidget(card)

        # Calibration state
        self._cal_tick_timer = QTimer(self)
        self._cal_tick_timer.setInterval(50)  # 20 Hz
        self._cal_tick_timer.timeout.connect(self._cal_tick)

        self._cal_settle_timer = QTimer(self)
        self._cal_settle_timer.setSingleShot(True)
        self._cal_settle_timer.timeout.connect(self._cal_measure)

        self._cal_phase = None  # "go_start", "running", "settling"
        self._cal_target = None  # (x, y) — the stop target
        self._cal_start_side = None  # (x, y) — the starting side
        self._cal_stop_pos = None
        self._cal_history = {}

    def _get_robot_pos_cal(self):
        if not self._engine:
            return None
        wm = self._engine._wm
        if wm is None:
            return None
        frame = wm.get_latest_frame()
        if frame is None:
            return None
        if self._test_panel:
            rid = self._test_panel._id_spin.value()
            is_yellow = self._test_panel._team_combo.currentText() == "Yellow"
        else:
            rid = 0
            is_yellow = True
        team = frame.robots_yellow if is_yellow else frame.robots_blue
        try:
            robot = team[rid]
        except (IndexError, TypeError):
            return None
        from TeamControl.SSL.vision.robots import Robot
        if not isinstance(robot, Robot):
            return None
        return (float(robot.x), float(robot.y), float(robot.o))

    def _cal_send_cmd(self, vx=0.0, vy=0.0, w=0.0):
        if not self._test_panel:
            return
        cmd = self._test_panel._build_cmd(vx=vx, vy=vy, w=w, kick=0, dribble=0)
        self._test_panel._do_send(cmd)

    def _cal_start(self):
        from TeamControl.robot.constants import HALF_LEN
        pos = self._get_robot_pos_cal()
        if pos is None:
            self._cal_status.setText("No robot position — is the engine running?")
            self._cal_status.setStyleSheet(f"color:{DANGER}; font-size:11px;")
            return

        m = self._CAL_MARGIN
        # Start on left, target on right (y=0 to stay centered)
        self._cal_start_side = (-HALF_LEN + m, 0)
        self._cal_target = (HALF_LEN - m, 0)

        self._cal_run_btn.setEnabled(False)
        self._cal_phase = "go_start"
        self._cal_status.setText("Navigating to start position (left side)...")
        self._cal_status.setStyleSheet(f"color:{WARNING}; font-size:11px;")
        self._cal_results.setText("")

        # Show the two points on the field
        self._field.set_targets([self._cal_start_side, self._cal_target])

        self._cal_tick_timer.start()

    def _cal_tick(self):
        import math
        from TeamControl.world.transform_cords import world2robot

        pose = self._get_robot_pos_cal()
        if pose is None:
            return

        if self._cal_phase == "go_start":
            # Navigate to the left starting position
            target = self._cal_start_side
            rel = world2robot(pose, target)
            dist = math.hypot(rel[0], rel[1])
            angle = math.atan2(rel[1], rel[0])

            if dist < self._CAL_ARRIVE:
                # At start — now face right (toward target)
                self._cal_phase = "running"
                self._cal_stop_pos = None
                self._cal_status.setText(
                    f"Running across field at {self._cal_speed.value():.2f} m/s...")
                self._cal_status.setStyleSheet(f"color:{SUCCESS}; font-size:11px;")
                return

            if abs(angle) > 0.2:
                self._cal_send_cmd(w=max(-0.15, min(0.15, angle * 0.2)))
            else:
                speed = min(0.3, max(0.03, (dist - 150) / 2000.0))
                self._cal_send_cmd(vx=speed)

        elif self._cal_phase == "running":
            # Drive toward right side target at calibration speed
            target = self._cal_target
            rel = world2robot(pose, target)
            dist = math.hypot(rel[0], rel[1])
            angle = math.atan2(rel[1], rel[0])

            if dist < self._CAL_STOP_ZONE:
                # Hit the stop zone — slam on brakes and measure drift
                self._cal_send_cmd(vx=0.0, vy=0.0, w=0.0)
                self._cal_stop_pos = (pose[0], pose[1])
                self._cal_phase = "settling"
                self._cal_tick_timer.stop()
                self._cal_status.setText(
                    f"Stopped at {self._CAL_STOP_ZONE}mm from target — "
                    f"waiting 1.5s for robot to settle...")
                self._cal_status.setStyleSheet(f"color:{WARNING}; font-size:11px;")
                self._cal_settle_timer.start(1500)
                return

            speed = self._cal_speed.value()
            if abs(angle) > 0.2:
                self._cal_send_cmd(w=max(-0.15, min(0.15, angle * 0.2)))
            else:
                self._cal_send_cmd(vx=speed)

    def _cal_measure(self):
        import math

        final = self._get_robot_pos_cal()
        self._cal_phase = None
        self._cal_run_btn.setEnabled(True)
        self._field.set_targets([])

        if final is None or self._cal_stop_pos is None:
            self._cal_status.setText("Failed — lost robot position")
            self._cal_status.setStyleSheet(f"color:{DANGER}; font-size:11px;")
            self._field.set_targets([])
            return

        drift = math.hypot(
            final[0] - self._cal_stop_pos[0],
            final[1] - self._cal_stop_pos[1])

        # Total run distance (start side to stop position)
        run_dist = math.hypot(
            self._cal_stop_pos[0] - self._cal_start_side[0],
            self._cal_stop_pos[1] - self._cal_start_side[1])

        speed = self._cal_speed.value()
        key = f"{speed:.2f}"
        if key not in self._cal_history:
            self._cal_history[key] = []
        self._cal_history[key].append(drift)

        avg = sum(self._cal_history[key]) / len(self._cal_history[key])
        n = len(self._cal_history[key])

        self._cal_status.setText("Done")
        self._cal_status.setStyleSheet(f"color:{SUCCESS}; font-size:11px;")

        self._cal_results.setText(
            f"Speed: {speed:.2f} m/s\n"
            f"Run distance: {run_dist:.0f} mm\n"
            f"Drift after stop: {drift:.0f} mm\n"
            f"Avg drift @ {speed:.2f}: {avg:.0f} mm ({n} runs)")

        self._cal_estimates.setText(self._format_estimates(self._cal_history))

    def _format_estimates(self, history):
        if not history:
            return (
                "Estimates (pre-calibration):\n"
                "  0.10 m/s → ~15 mm drift\n"
                "  0.20 m/s → ~40 mm drift\n"
                "  0.50 m/s → ~120 mm drift\n"
                "  1.00 m/s → ~300 mm drift\n"
                "Run tests to get actual values.")
        lines = ["Measured stopping drift:"]
        for speed_key in sorted(history.keys()):
            drifts = history[speed_key]
            avg = sum(drifts) / len(drifts)
            lines.append(f"  {speed_key} m/s → {avg:.0f} mm ({len(drifts)} runs)")
        return "\n".join(lines)

    # ── Public update API ─────────────────────────────────────────

    def update_frame(self, snap):
        self._field.set_frame(snap)
        self._update_robot_table(snap)
        self._update_fps()

    def update_game_state(self, state):
        if state is None:
            self._gs_state.setText("NO DATA")
            self._gs_state.setStyleSheet(f"color:{TEXT_DIM};")
            return
        name = state.name if hasattr(state, "name") else str(state)
        color_map = {"HALTED": DANGER, "STOPPED": WARNING, "RUNNING": SUCCESS}
        c = color_map.get(name, ACCENT)
        self._gs_state.setText(name)
        self._gs_state.setStyleSheet(f"color:{c}; font-weight:bold;")

    def set_mode(self, mode):
        self._gs_mode.setText(mode.upper())

    def set_engine_running(self, running):
        self._vis_dot.set_ok(running)
        self._gc_dot.set_ok(running)
        self._grsim_dot.set_ok(running)
        if not running:
            self._fps_lbl.setText("0 fps")
            self._fps_bar.setValue(0)

    def get_fps(self):
        return len(self._frame_times)

    # ── Internal ──────────────────────────────────────────────────

    def _update_robot_table(self, snap):
        robots = [(YELLOW_TEAM, "Y", r) for r in snap.yellow] + \
                 [(BLUE_TEAM, "B", r) for r in snap.blue]

        self._robot_table.setRowCount(len(robots))
        for row, (color_hex, team_short, r) in enumerate(robots):
            vals = [team_short, str(r.id), f"{r.x:.0f}",
                    f"{r.y:.0f}", f"{math.degrees(r.o):.1f}",
                    f"{r.confidence:.2f}"]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 0:
                    item.setForeground(QColor(color_hex))
                    item.setFont(QFont("Segoe UI", 10, QFont.Bold))
                self._robot_table.setItem(row, col, item)

        ny, nb = len(snap.yellow), len(snap.blue)
        ball = f"Ball ({snap.ball.x:.0f}, {snap.ball.y:.0f})" if snap.ball else "No ball"
        self._robot_summary.setText(
            f"Y:{ny}  B:{nb}  |  {ball}  |  Frame #{snap.frame_number}")

    def _update_fps(self):
        now = time.time()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if t > now - 1.0]
        fps = len(self._frame_times)
        self._fps_lbl.setText(f"{fps} fps")
        self._fps_bar.setValue(min(fps, 60))
        self._vis_dot.set_ok(fps > 0)
