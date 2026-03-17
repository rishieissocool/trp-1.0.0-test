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
    QGridLayout, QProgressBar, QFrame,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QFont

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, WARNING, DANGER,
    YELLOW_TEAM, BLUE_TEAM,
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
