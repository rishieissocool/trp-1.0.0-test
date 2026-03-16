"""
Hardware Test Console — direct robot connectivity testing & manual control.

Lets you:
  - Pick any robot from ipconfig or type a custom IP:port
  - Send individual test commands (spin, drive, kick, stop)
  - Use sliders for real-time manual control at 20 Hz
  - See the exact raw packet being sent
  - Verify grSim connectivity separately
  - View a live packet log with timestamps and error highlighting
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
)
from PySide6.QtCore import Qt, QTimer, QTime, Signal
from PySide6.QtGui import QFont, QColor

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, DANGER, WARNING,
    YELLOW_TEAM, BLUE_TEAM, BG_DARK, BG_MID,
)

from TeamControl.network.robot_command import RobotCommand
from TeamControl.network.sender import Sender
from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.grSimPacketFactory import grSimPacketFactory
from TeamControl.utils.yaml_config import Config


# ── Helpers ──────────────────────────────────────────────────────────

def _ts():
    return QTime.currentTime().toString("HH:mm:ss.zzz")


class _LogView(QPlainTextEdit):
    """Packet log with coloured entries."""

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


# ── Robot test card (one per robot in ipconfig) ──────────────────────

class _RobotRow:
    """Data for one row in the robot table."""
    __slots__ = ("label", "team", "shell_id", "grsim_id", "ip", "port")

    def __init__(self, label, team, shell_id, grsim_id, ip, port):
        self.label = label
        self.team = team
        self.shell_id = shell_id
        self.grsim_id = grsim_id
        self.ip = ip
        self.port = port


# ── Main Panel ───────────────────────────────────────────────────────

class TestPanel(QWidget):
    """Hardware testing & manual robot control console."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sender = Sender()
        self._grsim: grSimSender | None = None
        self._continuous_timer = QTimer(self)
        self._continuous_timer.setInterval(50)  # 20 Hz
        self._continuous_timer.timeout.connect(self._send_continuous)
        self._robots: list[_RobotRow] = []

        self._build_ui()
        self._load_robots()

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        title = QLabel("Hardware Test Console")
        title.setStyleSheet(f"font-size:16px; font-weight:bold; color:{ACCENT};")
        root.addWidget(title)

        splitter = QSplitter(Qt.Horizontal)

        # ── Left: target + commands ───────────────────────────────
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)

        # Target selection
        tgt = QGroupBox("Target")
        tgl = QGridLayout(tgt)

        tgl.addWidget(QLabel("Robot:"), 0, 0)
        self._robot_combo = QComboBox()
        self._robot_combo.currentIndexChanged.connect(self._on_robot_selected)
        tgl.addWidget(self._robot_combo, 0, 1, 1, 3)

        tgl.addWidget(QLabel("IP:"), 1, 0)
        self._ip_edit = QLineEdit("127.0.0.1")
        self._ip_edit.setMinimumWidth(140)
        tgl.addWidget(self._ip_edit, 1, 1)
        tgl.addWidget(QLabel("Port:"), 1, 2)
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1025, 65534)
        self._port_spin.setValue(50514)
        tgl.addWidget(self._port_spin, 1, 3)

        tgl.addWidget(QLabel("Shell ID:"), 2, 0)
        self._id_spin = QSpinBox()
        self._id_spin.setRange(0, 15)
        tgl.addWidget(self._id_spin, 2, 1)
        tgl.addWidget(QLabel("Team:"), 2, 2)
        self._team_combo = QComboBox()
        self._team_combo.addItems(["Yellow", "Blue"])
        tgl.addWidget(self._team_combo, 2, 3)

        ping_btn = QPushButton("Test Connection")
        ping_btn.setStyleSheet(f"font-weight:bold; color:{ACCENT};")
        ping_btn.clicked.connect(self._test_connection)
        tgl.addWidget(ping_btn, 3, 0, 1, 4)

        ll.addWidget(tgt)

        # Manual command sliders
        cmd = QGroupBox("Manual Command")
        cl = QGridLayout(cmd)

        self._vx_slider = self._make_slider("VX (m/s)", -3.0, 3.0, cl, 0)
        self._vy_slider = self._make_slider("VY (m/s)", -3.0, 3.0, cl, 1)
        self._w_slider = self._make_slider("W (rad/s)", -10.0, 10.0, cl, 2)

        self._kick_cb = QCheckBox("Kick")
        self._kick_cb.setStyleSheet("font-weight:bold;")
        self._dribble_cb = QCheckBox("Dribble")
        self._dribble_cb.setStyleSheet("font-weight:bold;")
        cl.addWidget(self._kick_cb, 3, 0)
        cl.addWidget(self._dribble_cb, 3, 1)

        ll.addWidget(cmd)

        # Send buttons
        send_box = QGroupBox("Send")
        sl = QGridLayout(send_box)

        send_once = QPushButton("Send Once")
        send_once.setObjectName("startBtn")
        send_once.clicked.connect(self._send_once)
        sl.addWidget(send_once, 0, 0)

        self._cont_btn = QPushButton("Start Continuous (20 Hz)")
        self._cont_btn.clicked.connect(self._toggle_continuous)
        sl.addWidget(self._cont_btn, 0, 1)

        stop_btn = QPushButton("STOP Robot")
        stop_btn.setObjectName("stopBtn")
        stop_btn.clicked.connect(self._send_stop)
        sl.addWidget(stop_btn, 0, 2)

        ll.addWidget(send_box)

        # Quick tests
        qt = QGroupBox("Quick Tests")
        ql = QGridLayout(qt)

        tests = [
            ("Spin CW",       0.0,  0.0,  5.0, 0, 0),
            ("Spin CCW",      0.0,  0.0, -5.0, 0, 0),
            ("Drive Forward", 1.0,  0.0,  0.0, 0, 0),
            ("Drive Back",   -1.0,  0.0,  0.0, 0, 0),
            ("Strafe Left",   0.0,  1.0,  0.0, 0, 0),
            ("Strafe Right",  0.0, -1.0,  0.0, 0, 0),
            ("Kick",          0.0,  0.0,  0.0, 1, 0),
            ("Dribble",       0.0,  0.0,  0.0, 0, 1),
            ("Diagonal FL",   1.0,  1.0,  0.0, 0, 0),
            ("Full Send",     2.0,  0.0,  0.0, 1, 1),
        ]
        for i, (name, vx, vy, w, k, d) in enumerate(tests):
            btn = QPushButton(name)
            btn.clicked.connect(
                lambda checked=False, vx=vx, vy=vy, w=w, k=k, d=d:
                    self._send_test(vx, vy, w, k, d))
            ql.addWidget(btn, i // 3, i % 3)

        stop_all = QPushButton("STOP ALL")
        stop_all.setObjectName("stopBtn")
        stop_all.clicked.connect(self._stop_all)
        ql.addWidget(stop_all, (len(tests) // 3) + 1, 0, 1, 3)

        ll.addWidget(qt)

        # grSim test
        gs = QGroupBox("grSim Test")
        gsl = QGridLayout(gs)

        gsl.addWidget(QLabel("grSim IP:"), 0, 0)
        self._grsim_ip = QLineEdit("127.0.0.1")
        gsl.addWidget(self._grsim_ip, 0, 1)
        gsl.addWidget(QLabel("Port:"), 0, 2)
        self._grsim_port = QSpinBox()
        self._grsim_port.setRange(1025, 65534)
        self._grsim_port.setValue(20011)
        gsl.addWidget(self._grsim_port, 0, 3)

        self._grsim_also = QCheckBox("Also send to grSim")
        self._grsim_also.setStyleSheet("font-weight:bold;")
        gsl.addWidget(self._grsim_also, 1, 0, 1, 2)

        grsim_test = QPushButton("Test grSim (spin robot 0)")
        grsim_test.clicked.connect(self._test_grsim)
        gsl.addWidget(grsim_test, 1, 2, 1, 2)

        ll.addWidget(gs)
        ll.addStretch()

        splitter.addWidget(left)

        # ── Right: packet log + raw view ──────────────────────────
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)

        # Raw packet preview
        raw_box = QGroupBox("Last Packet (raw)")
        rbl = QVBoxLayout(raw_box)
        self._raw_label = QLabel("—")
        self._raw_label.setFont(QFont("Cascadia Code", 11))
        self._raw_label.setWordWrap(True)
        self._raw_label.setStyleSheet(
            f"background:{BG_DARK}; padding:8px; border:1px solid #2a2a4a; "
            f"border-radius:4px; color:{ACCENT};")
        self._raw_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        rbl.addWidget(self._raw_label)
        rl.addWidget(raw_box)

        # Known robots table
        rt_box = QGroupBox("Configured Robots (ipconfig.yaml)")
        rtl = QVBoxLayout(rt_box)
        self._robot_table = QTableWidget(0, 6)
        self._robot_table.setHorizontalHeaderLabels(
            ["Team", "Label", "Shell ID", "grSim ID", "IP", "Port"])
        self._robot_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._robot_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._robot_table.setAlternatingRowColors(True)
        self._robot_table.verticalHeader().setVisible(False)
        hh = self._robot_table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        self._robot_table.setMaximumHeight(220)
        self._robot_table.currentCellChanged.connect(self._on_table_row)
        rtl.addWidget(self._robot_table)

        reload_btn = QPushButton("Reload ipconfig.yaml")
        reload_btn.clicked.connect(self._load_robots)
        rtl.addWidget(reload_btn)
        rl.addWidget(rt_box)

        # Packet log
        log_box = QGroupBox("Packet Log")
        lgl = QVBoxLayout(log_box)
        log_bar = QHBoxLayout()
        self._send_count = QLabel("0 sent")
        self._send_count.setStyleSheet(f"color:{TEXT_DIM};")
        self._err_count = QLabel("0 errors")
        self._err_count.setStyleSheet(f"color:{DANGER};")
        clear_btn = QPushButton("Clear")
        clear_btn.setFixedWidth(60)
        clear_btn.clicked.connect(lambda: (self._log.clear(),
                                           self._reset_counts()))
        log_bar.addWidget(self._send_count)
        log_bar.addWidget(self._err_count)
        log_bar.addStretch()
        log_bar.addWidget(clear_btn)
        lgl.addLayout(log_bar)

        self._log = _LogView()
        lgl.addWidget(self._log)
        rl.addWidget(log_box)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        root.addWidget(splitter)

        self._n_sent = 0
        self._n_err = 0

    # ── Slider factory ────────────────────────────────────────────

    def _make_slider(self, label, lo, hi, grid, row):
        lbl = QLabel(label)
        lbl.setStyleSheet("font-weight:bold;")
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(lo * 100), int(hi * 100))
        slider.setValue(0)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(int((hi - lo) * 10))
        val_lbl = QLabel("0.00")
        val_lbl.setFixedWidth(50)
        val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        slider.valueChanged.connect(
            lambda v, l=val_lbl: l.setText(f"{v / 100:.2f}"))

        reset = QPushButton("0")
        reset.setFixedSize(24, 24)
        reset.clicked.connect(lambda: slider.setValue(0))

        grid.addWidget(lbl, row, 0)
        grid.addWidget(slider, row, 1)
        grid.addWidget(val_lbl, row, 2)
        grid.addWidget(reset, row, 3)
        return slider

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
                    f"{r.label}  (shell {r.shell_id} → {r.ip}:{r.port})")

        self._robot_combo.addItem("— Custom —")

        # Populate table
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
                    item.setFont(QFont("Segoe UI", 10, QFont.Bold))
                self._robot_table.setItem(i, j, item)

        self._log.info(f"Loaded {len(self._robots)} robots from ipconfig.yaml")

        # Set grSim addr from config
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
            vx=vx if vx is not None else self._vx_slider.value() / 100.0,
            vy=vy if vy is not None else self._vy_slider.value() / 100.0,
            w=w if w is not None else self._w_slider.value() / 100.0,
            kick=kick if kick is not None else int(self._kick_cb.isChecked()),
            dribble=dribble if dribble is not None else int(self._dribble_cb.isChecked()),
            isYellow=(self._team_combo.currentText() == "Yellow"),
        )

    # ── Sending ───────────────────────────────────────────────────

    def _do_send(self, cmd: RobotCommand):
        ip = self._ip_edit.text().strip()
        port = self._port_spin.value()
        raw_text = str(cmd)

        self._raw_label.setText(
            f"To {ip}:{port}\n"
            f"Raw: {raw_text}\n"
            f"Bytes: {cmd.encode()!r}")

        # Send to physical robot
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

        # Also send to grSim if checked
        if self._grsim_also.isChecked():
            self._do_grsim_send(cmd)

        self._update_counts()

    def _do_grsim_send(self, cmd: RobotCommand):
        try:
            gs = self._get_grsim()
            gs.send_robot_command(cmd)
            self._log.ok(f"→ grSim  id={cmd.robot_id}")
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
        cmd = self._build_cmd(vx=0, vy=0, w=0, kick=0, dribble=0)
        self._do_send(cmd)
        self._vx_slider.setValue(0)
        self._vy_slider.setValue(0)
        self._w_slider.setValue(0)
        self._kick_cb.setChecked(False)
        self._dribble_cb.setChecked(False)
        self._log.info("STOP sent")

    def _send_test(self, vx, vy, w, kick, dribble):
        cmd = self._build_cmd(vx=vx, vy=vy, w=w, kick=kick, dribble=dribble)
        self._do_send(cmd)

    def _toggle_continuous(self):
        if self._continuous_timer.isActive():
            self._continuous_timer.stop()
            self._cont_btn.setText("Start Continuous (20 Hz)")
            self._cont_btn.setStyleSheet("")
            self._log.info("Continuous send stopped")
        else:
            self._continuous_timer.start()
            self._cont_btn.setText("STOP Continuous")
            self._cont_btn.setStyleSheet(
                f"background:{DANGER}; color:#fff; font-weight:bold;")
            self._log.info("Continuous send started at 20 Hz")

    def _send_continuous(self):
        self._do_send(self._build_cmd())

    def _stop_all(self):
        """Send stop to every robot in the config."""
        if self._continuous_timer.isActive():
            self._continuous_timer.stop()
            self._cont_btn.setText("Start Continuous (20 Hz)")
            self._cont_btn.setStyleSheet("")

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

    def _test_connection(self):
        """Send a zero-velocity command to verify the path works."""
        ip = self._ip_edit.text().strip()
        port = self._port_spin.value()
        rid = self._id_spin.value()

        self._log.info(f"Testing connection to {ip}:{port} (robot {rid})…")

        cmd = RobotCommand(robot_id=rid, vx=0, vy=0, w=0, kick=0, dribble=0,
                           isYellow=(self._team_combo.currentText() == "Yellow"))

        try:
            self._sender.send(cmd, ip, port)
            self._log.ok(
                f"Packet sent successfully to {ip}:{port}\n"
                f"   Raw: {str(cmd)}\n"
                f"   (no error does NOT confirm robot received it — "
                f"UDP is fire-and-forget)")
            self._n_sent += 1
        except socket.error as e:
            self._log.err(f"SOCKET ERROR: {e}")
            self._n_err += 1
        except Exception as e:
            self._log.err(f"ERROR: {type(e).__name__}: {e}")
            self._n_err += 1

        self._update_counts()

    def _test_grsim(self):
        """Send a spin command to grSim robot 0 yellow."""
        self._log.info("Testing grSim…")
        try:
            gs = self._get_grsim()
            cmd = RobotCommand(robot_id=0, vx=0, vy=0, w=5.0,
                               kick=0, dribble=0, isYellow=True)
            gs.send_robot_command(cmd)
            self._log.ok(
                f"grSim test sent to "
                f"{self._grsim_ip.text()}:{self._grsim_port.value()} — "
                f"Yellow robot 0 should spin")
            self._n_sent += 1

            # Send stop after 1s
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
            self._log.ok("grSim test stop sent")
        except Exception:
            pass

    # ── Counts ────────────────────────────────────────────────────

    def _update_counts(self):
        self._send_count.setText(f"{self._n_sent} sent")
        self._err_count.setText(f"{self._n_err} errors")

    def _reset_counts(self):
        self._n_sent = 0
        self._n_err = 0
        self._update_counts()
