"""
Main Window — assembles every panel into a dockable, professional dashboard.

Layout:
  ┌──────────────────────────────────────────────────────┐
  │  Toolbar: mode selector │ Start │ Stop │ status      │
  ├────────────┬───────────────────────┬─────────────────┤
  │  Robot     │                       │  Game State     │
  │  Dashboard │     Field Canvas      │  Behavior Tree  │
  │            │     (central)         │  (tabbed)       │
  ├────────────┴───────────────────────┴─────────────────┤
  │  Logs  │  Network  │  Config  │  Sim Controls        │
  └──────────────────────────────────────────────────────┘
"""

from PySide6.QtWidgets import (QMainWindow, QDockWidget, QToolBar,
                                QLabel, QComboBox, QPushButton,
                                QTabWidget, QStatusBar, QWidget,
                                QVBoxLayout, QHBoxLayout, QSizePolicy,
                                QApplication)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QFont, QIcon

from TeamControl.ui.theme import QSS, ACCENT, TEXT_DIM, SUCCESS, DANGER, WARNING
from TeamControl.ui.engine import SimEngine
from TeamControl.ui.field_canvas import FieldCanvas
from TeamControl.ui.robot_panel import RobotPanel
from TeamControl.ui.game_panel import GamePanel
from TeamControl.ui.behavior_tree import BehaviorTreePanel
from TeamControl.ui.config_panel import ConfigPanel
from TeamControl.ui.log_panel import LogPanel
from TeamControl.ui.sim_panel import SimPanel
from TeamControl.ui.network_panel import NetworkPanel


class MainWindow(QMainWindow):
    """TurtleRabbit SSL Command Center."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleRabbit — SSL Command Center")
        self.setMinimumSize(1400, 900)
        self.resize(1800, 1050)
        self.setStyleSheet(QSS)

        # ── Engine ────────────────────────────────────────────────
        self._engine = SimEngine(self)

        # ── Create all panels ─────────────────────────────────────
        self._field = FieldCanvas()
        self._robot_panel = RobotPanel()
        self._game_panel = GamePanel()
        self._bt_panel = BehaviorTreePanel()
        self._config_panel = ConfigPanel()
        self._log_panel = LogPanel()
        self._sim_panel = SimPanel()
        self._network_panel = NetworkPanel()

        # ── Top-level tabs: Field view vs Behavior Tree ──────────
        self._central_tabs = QTabWidget()
        self._central_tabs.addTab(self._field, "Field")
        self._central_tabs.addTab(self._bt_panel, "Behavior Tree")
        self.setCentralWidget(self._central_tabs)

        # ── Toolbar ───────────────────────────────────────────────
        self._build_toolbar()

        # ── Menu bar ──────────────────────────────────────────────
        self._build_menu()

        # ── Dock: Left — Robot Dashboard ──────────────────────────
        left_dock = QDockWidget("Robots", self)
        left_dock.setWidget(self._robot_panel)
        left_dock.setMinimumWidth(340)
        self.addDockWidget(Qt.LeftDockWidgetArea, left_dock)

        # ── Dock: Right — Game State + Network (tabbed) ──────────
        right_tabs = QTabWidget()
        right_tabs.addTab(self._game_panel, "Game State")
        right_tabs.addTab(self._network_panel, "Network")

        right_dock = QDockWidget("Intelligence", self)
        right_dock.setWidget(right_tabs)
        right_dock.setMinimumWidth(360)
        self.addDockWidget(Qt.RightDockWidgetArea, right_dock)

        # ── Dock: Bottom — Logs + Config + Sim (tabbed) ──────────
        bottom_tabs = QTabWidget()
        bottom_tabs.addTab(self._log_panel, "Console")
        bottom_tabs.addTab(self._config_panel, "Config")
        bottom_tabs.addTab(self._sim_panel, "Simulation")

        bottom_dock = QDockWidget("Operations", self)
        bottom_dock.setWidget(bottom_tabs)
        bottom_dock.setMinimumHeight(200)
        self.addDockWidget(Qt.BottomDockWidgetArea, bottom_dock)

        # ── Status bar ────────────────────────────────────────────
        self._status_mode = QLabel("Mode: —")
        self._status_mode.setStyleSheet(f"color:{TEXT_DIM}; padding:0 12px;")
        self._status_coords = QLabel("(—, —)")
        self._status_coords.setStyleSheet(f"color:{TEXT_DIM}; padding:0 12px;")
        self._status_fps = QLabel("0 fps")
        self._status_fps.setStyleSheet(f"color:{TEXT_DIM}; padding:0 12px;")

        sb = QStatusBar()
        sb.addWidget(self._status_mode)
        sb.addWidget(self._status_coords)
        sb.addPermanentWidget(self._status_fps)
        self.setStatusBar(sb)

        # ── Wire signals ──────────────────────────────────────────
        self._wire_signals()

        # ── Initial log ───────────────────────────────────────────
        self._log_panel.append("[engine] TurtleRabbit Command Center ready")
        self._log_panel.append("[engine] Select a mode and click Start")

    # ── Toolbar ───────────────────────────────────────────────────

    def _build_toolbar(self):
        tb = QToolBar("Control")
        tb.setMovable(False)
        tb.setIconSize(tb.iconSize())
        self.addToolBar(tb)

        logo = QLabel("  TurtleRabbit  ")
        logo.setFont(QFont("Segoe UI", 14, QFont.Bold))
        logo.setStyleSheet(f"color:{ACCENT};")
        tb.addWidget(logo)

        tb.addSeparator()

        tb.addWidget(QLabel("  Mode: "))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(SimEngine.MODES)
        self._mode_combo.setMinimumWidth(120)
        tb.addWidget(self._mode_combo)

        tb.addSeparator()

        self._start_btn = QPushButton("  Start  ")
        self._start_btn.setObjectName("startBtn")
        self._start_btn.clicked.connect(self._on_start)
        tb.addWidget(self._start_btn)

        self._stop_btn = QPushButton("  Stop  ")
        self._stop_btn.setObjectName("stopBtn")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        tb.addWidget(self._stop_btn)

        tb.addSeparator()

        self._state_label = QLabel("  IDLE  ")
        self._state_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self._state_label.setStyleSheet(f"color:{TEXT_DIM};")
        tb.addWidget(self._state_label)

        tb.addSeparator()

        # Field zoom controls
        zoom_in = QPushButton("+")
        zoom_in.setFixedSize(28, 28)
        zoom_in.setToolTip("Zoom In")
        zoom_in.clicked.connect(lambda: self._zoom(1.2))
        zoom_out = QPushButton("-")
        zoom_out.setFixedSize(28, 28)
        zoom_out.setToolTip("Zoom Out")
        zoom_out.clicked.connect(lambda: self._zoom(0.8))
        zoom_reset = QPushButton("Fit")
        zoom_reset.setFixedSize(40, 28)
        zoom_reset.setToolTip("Reset View")
        zoom_reset.clicked.connect(self._reset_field_view)

        tb.addWidget(QLabel(" View: "))
        tb.addWidget(zoom_in)
        tb.addWidget(zoom_out)
        tb.addWidget(zoom_reset)

    def _zoom(self, factor):
        self._field._scale = max(0.2, min(5.0, self._field._scale * factor))
        self._field.update()

    def _reset_field_view(self):
        from PySide6.QtCore import QPointF
        self._field._scale = 1.0
        self._field._offset = QPointF(0, 0)
        self._field.update()

    # ── Menu bar ──────────────────────────────────────────────────

    def _build_menu(self):
        mb = self.menuBar()

        file_menu = mb.addMenu("File")
        file_menu.addAction("Reload Config", self._config_panel.load)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        view_menu = mb.addMenu("View")
        view_menu.addAction("Reset Field View", self._reset_field_view)

        sim_menu = mb.addMenu("Simulation")
        sim_menu.addAction("Center Ball",
                           lambda: self._engine.place_ball(0, 0))
        sim_menu.addAction("Kickoff Formation",
                           self._sim_panel._kickoff_formation)

        mode_menu = mb.addMenu("Mode")
        for m in SimEngine.MODES:
            mode_menu.addAction(m.capitalize(),
                                lambda checked=False, mode=m: self._switch_mode(mode))

        help_menu = mb.addMenu("Help")
        help_menu.addAction("About", self._show_about)

    def _show_about(self):
        from PySide6.QtWidgets import QMessageBox
        QMessageBox.about(self, "TurtleRabbit",
                          "WSU TurtleRabbit SSL Command Center\n\n"
                          "RoboCup Small Size League\n"
                          "Team Control Dashboard v1.0")

    # ── Signal wiring ─────────────────────────────────────────────

    def _wire_signals(self):
        eng = self._engine

        # Engine → panels
        eng.frame_ready.connect(self._on_frame)
        eng.game_state_ready.connect(self._game_panel.update_game_state)
        eng.engine_started.connect(self._on_engine_started)
        eng.engine_stopped.connect(self._on_engine_stopped)
        eng.log_message.connect(self._log_panel.append)

        # Field canvas coordinates
        self._field.coordinate_hover.connect(self._on_coord_hover)

        # Sim panel → engine
        self._sim_panel.place_ball_requested.connect(
            lambda x, y, vx, vy: eng.place_ball(x, y, vx, vy))
        self._sim_panel.place_robot_requested.connect(
            lambda rid, yl, x, y, o: eng.place_robot(rid, yl, x, y, o))

        # Sim panel → field canvas (click-to-place)
        self._sim_panel.field_place_ball.connect(
            lambda: self._field.set_place_mode("ball"))
        self._sim_panel.field_place_robot.connect(
            lambda rid, yl: self._field.set_place_mode(("robot", rid, yl)))

        # Field canvas click → engine
        self._field.ball_placed.connect(
            lambda x, y: eng.place_ball(x, y))
        self._field.robot_placed.connect(
            lambda rid, yl, x, y: eng.place_robot(rid, yl, x, y))

        # Config changed
        self._config_panel.config_changed.connect(
            lambda: self._log_panel.append("[config] Configuration saved"))

    # ── Handlers ──────────────────────────────────────────────────

    def _on_start(self):
        mode = self._mode_combo.currentText()
        self._log_panel.append(f"[engine] Starting {mode}…")
        try:
            self._engine.start(mode)
        except Exception as e:
            self._log_panel.append(f"[error] Failed to start: {e}")

    def _on_stop(self):
        self._log_panel.append("[engine] Stopping…")
        self._engine.stop()

    def _switch_mode(self, mode):
        idx = SimEngine.MODES.index(mode)
        self._mode_combo.setCurrentIndex(idx)
        if self._engine.is_running:
            self._engine.stop()
        self._engine.start(mode)

    def _on_engine_started(self, mode):
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._mode_combo.setEnabled(False)
        self._state_label.setText(f"  RUNNING — {mode.upper()}  ")
        self._state_label.setStyleSheet(f"color:{SUCCESS}; font-weight:bold;")
        self._status_mode.setText(f"Mode: {mode}")
        self._game_panel.set_mode(mode)
        self._network_panel.set_engine_running(True)

    def _on_engine_stopped(self):
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._mode_combo.setEnabled(True)
        self._state_label.setText("  IDLE  ")
        self._state_label.setStyleSheet(f"color:{TEXT_DIM};")
        self._status_mode.setText("Mode: —")
        self._network_panel.set_engine_running(False)

    def _on_frame(self, snap):
        self._field.set_frame(snap)
        self._robot_panel.update_frame(snap)
        self._game_panel.update_frame(snap)
        self._network_panel.on_frame()

    def _on_coord_hover(self, x, y):
        self._status_coords.setText(f"({x:.0f}, {y:.0f}) mm")

    # ── Cleanup ───────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._engine.is_running:
            self._engine.stop()
        event.accept()
