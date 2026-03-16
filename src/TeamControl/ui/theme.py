"""Dark theme stylesheet and color palette for the TurtleRabbit UI."""

from PySide6.QtGui import QColor

# ── Palette ──────────────────────────────────────────────────────────
BG_DARK      = "#1a1a2e"
BG_MID       = "#16213e"
BG_PANEL     = "#0f3460"
BG_INPUT     = "#1a1a2e"
ACCENT       = "#e94560"
ACCENT_HOVER = "#ff6b81"
TEXT         = "#eaeaea"
TEXT_DIM     = "#8a8a9a"
BORDER       = "#2a2a4a"
SUCCESS      = "#2ecc71"
WARNING      = "#f39c12"
DANGER       = "#e74c3c"

YELLOW_TEAM  = "#ffd700"
BLUE_TEAM    = "#4a90d9"
BALL_COLOR   = "#ff8c00"
FIELD_GREEN  = "#1b5e20"
FIELD_LINE   = "#ffffff"

# ── Role colors ──────────────────────────────────────────────────────
ROLE_GOALIE   = "#e74c3c"
ROLE_ATTACKER = "#e67e22"
ROLE_SUPPORT  = "#2ecc71"
ROLE_DEFENDER = "#3498db"

QSS = f"""
QMainWindow {{
    background-color: {BG_DARK};
}}

QWidget {{
    color: {TEXT};
    font-family: "Segoe UI", "Inter", sans-serif;
    font-size: 13px;
}}

QMenuBar {{
    background-color: {BG_MID};
    border-bottom: 1px solid {BORDER};
    padding: 2px;
}}
QMenuBar::item:selected {{
    background-color: {ACCENT};
    border-radius: 4px;
}}
QMenu {{
    background-color: {BG_MID};
    border: 1px solid {BORDER};
    padding: 4px;
}}
QMenu::item:selected {{
    background-color: {ACCENT};
}}

QToolBar {{
    background-color: {BG_MID};
    border-bottom: 1px solid {BORDER};
    spacing: 6px;
    padding: 4px 8px;
}}
QToolBar QLabel {{
    font-weight: bold;
    padding: 0 4px;
}}

QDockWidget {{
    titlebar-close-icon: none;
    font-weight: bold;
    color: {TEXT};
}}
QDockWidget::title {{
    background-color: {BG_MID};
    border: 1px solid {BORDER};
    padding: 6px 10px;
    text-align: left;
}}

QTabWidget::pane {{
    border: 1px solid {BORDER};
    background: {BG_DARK};
}}
QTabBar::tab {{
    background: {BG_MID};
    color: {TEXT_DIM};
    padding: 8px 16px;
    border: 1px solid {BORDER};
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    margin-right: 2px;
}}
QTabBar::tab:selected {{
    background: {BG_DARK};
    color: {TEXT};
    border-bottom: 2px solid {ACCENT};
}}
QTabBar::tab:hover {{
    color: {ACCENT_HOVER};
}}

QPushButton {{
    background-color: {BG_PANEL};
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 6px 16px;
    font-weight: 600;
}}
QPushButton:hover {{
    background-color: {ACCENT};
    border-color: {ACCENT};
}}
QPushButton:pressed {{
    background-color: {ACCENT_HOVER};
}}
QPushButton:disabled {{
    background-color: {BG_MID};
    color: {TEXT_DIM};
}}
QPushButton#startBtn {{
    background-color: {SUCCESS};
    color: #111;
    font-weight: bold;
}}
QPushButton#startBtn:hover {{
    background-color: #27ae60;
}}
QPushButton#stopBtn {{
    background-color: {DANGER};
    color: #fff;
    font-weight: bold;
}}
QPushButton#stopBtn:hover {{
    background-color: #c0392b;
}}

QComboBox {{
    background-color: {BG_INPUT};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    min-width: 100px;
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background-color: {BG_MID};
    border: 1px solid {BORDER};
    selection-background-color: {ACCENT};
}}

QLineEdit, QSpinBox, QDoubleSpinBox {{
    background-color: {BG_INPUT};
    border: 1px solid {BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    color: {TEXT};
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{
    border-color: {ACCENT};
}}

QTreeWidget, QTableWidget {{
    background-color: {BG_DARK};
    border: 1px solid {BORDER};
    alternate-background-color: {BG_MID};
    gridline-color: {BORDER};
}}
QTreeWidget::item:selected, QTableWidget::item:selected {{
    background-color: {ACCENT};
}}
QHeaderView::section {{
    background-color: {BG_MID};
    border: 1px solid {BORDER};
    padding: 4px 8px;
    font-weight: bold;
}}

QPlainTextEdit {{
    background-color: #0d0d1a;
    border: 1px solid {BORDER};
    font-family: "Cascadia Code", "Consolas", monospace;
    font-size: 12px;
    color: {TEXT};
}}

QScrollBar:vertical {{
    background: {BG_DARK};
    width: 10px;
    border: none;
}}
QScrollBar::handle:vertical {{
    background: {BORDER};
    border-radius: 5px;
    min-height: 20px;
}}
QScrollBar::handle:vertical:hover {{
    background: {ACCENT};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}

QGroupBox {{
    border: 1px solid {BORDER};
    border-radius: 6px;
    margin-top: 8px;
    padding-top: 16px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 6px;
    color: {ACCENT};
}}

QSplitter::handle {{
    background: {BORDER};
}}

QStatusBar {{
    background: {BG_MID};
    border-top: 1px solid {BORDER};
    color: {TEXT_DIM};
}}

QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {BORDER};
    border-radius: 3px;
    background: {BG_INPUT};
}}
QCheckBox::indicator:checked {{
    background: {ACCENT};
    border-color: {ACCENT};
}}

QSlider::groove:horizontal {{
    height: 4px;
    background: {BORDER};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    width: 14px;
    height: 14px;
    margin: -5px 0;
    background: {ACCENT};
    border-radius: 7px;
}}
"""
