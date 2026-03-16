"""
Behavior-Tree Visualiser — maps out the decision logic for each robot role.

Draws an interactive tree graph with QPainter.  Nodes light up based on the
current game state so you can see exactly which branch each robot is executing.

Editable parameters are shown in a side panel when a node is selected.
"""

import math
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                                QComboBox, QScrollArea, QGroupBox,
                                QGridLayout, QDoubleSpinBox, QFrame,
                                QSplitter, QSizePolicy, QToolTip)
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QSize
from PySide6.QtGui import (QPainter, QPen, QBrush, QColor, QFont,
                           QPainterPath, QMouseEvent, QPaintEvent)

from TeamControl.ui.theme import (ACCENT, BG_DARK, BG_MID, BG_PANEL,
                                   BORDER, TEXT, TEXT_DIM, SUCCESS,
                                   WARNING, DANGER, ROLE_GOALIE,
                                   ROLE_ATTACKER, ROLE_SUPPORT, ROLE_DEFENDER)


# ── Tree data model ──────────────────────────────────────────────────

class BTNode:
    """Single behaviour-tree node."""
    __slots__ = ("name", "kind", "children", "params", "description",
                 "active", "rect")

    def __init__(self, name, kind="action", children=None, params=None,
                 description=""):
        self.name = name
        self.kind = kind  # "selector", "sequence", "condition", "action"
        self.children = children or []
        self.params = params or {}
        self.description = description
        self.active = False
        self.rect = QRectF()


# ── Static tree definitions (derived from robot/*.py) ────────────────

def _goalie_tree():
    return BTNode("Goalie", "selector", [
        BTNode("Clear / Distribute", "sequence", [
            BTNode("Ball in box?", "condition",
                   description="ball_in_box AND ball_slow AND d_ball < CLEAR_DIST"),
            BTNode("Ball in kick range?", "condition",
                   description="d_ball < KICK_DIST AND rel_ball.x > 0"),
            BTNode("Find pass target", "action",
                   description="Scan teammates for clear passing lane"),
            BTNode("Smart distribute", "selector", [
                BTNode("Pass to mate", "action",
                       params={"PASS_CLEAR": 400},
                       description="Kick to teammate with best lane"),
                BTNode("Clear to sideline", "action",
                       description="Emergency clear toward nearest sideline"),
            ]),
        ]),
        BTNode("Save Shot", "sequence", [
            BTNode("Shot incoming?", "condition",
                   params={"SHOT_SPEED": 500},
                   description="Ball velocity toward goal > threshold"),
            BTNode("Predict crossing", "action",
                   description="Compute predicted Y at goal line (incl. wall bounces)"),
            BTNode("Dive to intercept", "action",
                   params={"SAVE_SPEED": 2.5},
                   description="Sprint to predicted crossing point"),
        ]),
        BTNode("Position", "sequence", [
            BTNode("Compute angle", "action",
                   description="Narrow shooting angle based on ball position"),
            BTNode("Detect danger", "action",
                   description="Find most dangerous attacker, bias positioning"),
            BTNode("Move to arc", "action",
                   params={"MAX_ADVANCE": 1100, "POSITION_SPEED": 1.6},
                   description="Position on arc between ball and goal center"),
        ]),
    ])


def _striker_tree():
    return BTNode("Striker", "selector", [
        BTNode("Wait outside box", "sequence", [
            BTNode("Ball in defense area?", "condition",
                   description="Ball inside opponent penalty box"),
            BTNode("Hold position", "action",
                   description="Wait just outside defense area"),
        ]),
        BTNode("One-Touch Redirect", "sequence", [
            BTNode("Ball moving toward me?", "condition",
                   params={"ONETOUCH_MIN_SPEED": 300},
                   description="Ball velocity component toward robot"),
            BTNode("Close enough?", "condition",
                   description="d_ball < BALL_NEAR * 1.5"),
            BTNode("Aligned with goal?", "condition",
                   params={"ONETOUCH_ANGLE": 0.8},
                   description="Aim angle within tolerance"),
            BTNode("Intercept & redirect", "action",
                   params={"ONETOUCH_SPEED": 1.6},
                   description="Move into ball path, kick toward goal"),
        ]),
        BTNode("Possess Ball", "sequence", [
            BTNode("In kick range?", "condition",
                   params={"KICK_RANGE": 175},
                   description="d_ball < KICK_RANGE AND front-facing"),
            BTNode("Decide action", "selector", [
                BTNode("Emergency flick", "action",
                       params={"PRESSURE_DIST": 500},
                       description="Under pressure → blast toward goal"),
                BTNode("Shoot on goal", "action",
                       description="Aligned → kick"),
                BTNode("Dribble", "action",
                       params={"DRIBBLE_SPEED": 1.0},
                       description="Hold ball, turn toward aim"),
            ]),
        ]),
        BTNode("Charge", "sequence", [
            BTNode("Lined up?", "condition",
                   description="Behind ball, facing aim, close range"),
            BTNode("Drive through", "action",
                   params={"CHARGE_SPEED": 1.4},
                   description="Sprint forward through ball"),
        ]),
        BTNode("Recover", "sequence", [
            BTNode("Close but wrong angle?", "condition",
                   description="d_ball < BALL_NEAR but not aligned"),
            BTNode("Go behind ball", "action",
                   description="Navigate to behind-ball point"),
        ]),
        BTNode("Position / Approach", "sequence", [
            BTNode("Navigate to behind", "action",
                   description="Go to point behind ball toward goal"),
            BTNode("Curve if wrong side", "action",
                   params={"AVOID_RADIUS": 400, "BEHIND_DIST": 300},
                   description="Swing around ball if on wrong side"),
        ]),
    ])


def _team_attacker_tree():
    return BTNode("Attacker (Team)", "selector", [
        BTNode("Ball in box → wait", "sequence", [
            BTNode("Ball in defense area?", "condition"),
            BTNode("Wait outside", "action"),
        ]),
        BTNode("Possess Ball", "sequence", [
            BTNode("In kick range?", "condition",
                   params={"KICK_RANGE": 175}),
            BTNode("Evaluate options", "action",
                   description="Score shot vs pass vs through-ball"),
            BTNode("Execute", "selector", [
                BTNode("Shoot", "action",
                       params={"SHOOT_THRESH": 0.16},
                       description="High shot score → kick at goal"),
                BTNode("Pass", "action",
                       params={"PASS_THRESH": 0.05},
                       description="Best pass score → kick to teammate"),
                BTNode("Through-ball", "action",
                       params={"THROUGH_LEAD": 800},
                       description="Pass into space ahead of runner"),
                BTNode("Emergency flick", "action",
                       description="Under pressure → blast away"),
                BTNode("Dribble forward", "action",
                       params={"SPD_DRIBBLE": 1.1}),
            ]),
        ]),
        BTNode("Approach ball", "sequence", [
            BTNode("Intercept point", "action",
                   description="Friction-aware optimal intercept"),
            BTNode("Line up behind", "action",
                   params={"BEHIND_DIST": 270}),
            BTNode("Sprint / cruise / approach", "action",
                   params={"SPD_SPRINT": 2.5, "SPD_CRUISE": 1.8}),
        ]),
    ])


def _support_tree():
    return BTNode("Support", "sequence", [
        BTNode("Compute target", "action",
               description="Scored potential-field candidates"),
        BTNode("Check pass lane", "condition",
               params={"PASS_LANE_CLR": 320},
               description="Lane to ball must be clear of opponents"),
        BTNode("Evaluate position", "action",
               description="Score: advance, shot potential, spacing, lane clarity"),
        BTNode("Move to target", "action",
               params={"SPD_POSITION": 1.5, "SPD_CRUISE": 1.8},
               description="Navigate to best support position, face ball"),
    ])


def _defender_tree():
    return BTNode("Defender", "selector", [
        BTNode("Press (when attacking half)", "sequence", [
            BTNode("Ball in opp territory?", "condition",
                   params={"PRESS_ZONE": 0.3}),
            BTNode("First presser → ball carrier", "action",
                   params={"SPD_PRESS": 2.0}),
            BTNode("Second presser → cut lane", "action",
                   params={"PRESS_DIST": 600}),
        ]),
        BTNode("Mark opponent", "sequence", [
            BTNode("Find nearest threat", "action",
                   description="Sort opponents by distance to our goal"),
            BTNode("Position between opp & goal", "action",
                   params={"DEFEND_MARK_RATIO": 0.48}),
        ]),
        BTNode("Hold shape", "action",
               params={"DEFEND_LINE": 1800, "DEFEND_SPREAD": 1400},
               description="Fall back to defensive line, spread across width"),
    ])


def _goalie_team_tree():
    return BTNode("Goalie (Team)", "selector", [
        BTNode("Clear / Distribute", "sequence", [
            BTNode("Ball in box & slow?", "condition"),
            BTNode("Smart pass or clear", "action",
                   description="Find teammate or clear to sideline"),
        ]),
        BTNode("Save", "sequence", [
            BTNode("Shot with wall bounce?", "condition",
                   params={"GK_SHOT_SPEED": 480}),
            BTNode("Dive to predicted Y", "action",
                   params={"SPD_SAVE": 2.5}),
        ]),
        BTNode("Position on arc", "action",
               params={"GK_MAX_ADV": 1100},
               description="Angle narrowing with danger-weighted bias"),
    ])


ROLE_TREES = {
    "Goalie": _goalie_tree,
    "Striker": _striker_tree,
    "Team Attacker": _team_attacker_tree,
    "Support": _support_tree,
    "Defender": _defender_tree,
    "Team Goalie": _goalie_team_tree,
}

# ── Kind → colors ────────────────────────────────────────────────────
KIND_COLORS = {
    "selector": "#e67e22",
    "sequence": "#2ecc71",
    "condition": "#3498db",
    "action": "#9b59b6",
}

KIND_SYMBOLS = {
    "selector": "?",
    "sequence": "→",
    "condition": "◆",
    "action": "●",
}


# ── Tree rendering widget ────────────────────────────────────────────

NODE_W = 180
NODE_H = 44
H_GAP  = 24
V_GAP  = 60


def _layout_tree(node: BTNode, x=0, depth=0) -> float:
    """Recursive layout — returns total width consumed."""
    if not node.children:
        node.rect = QRectF(x, depth * (NODE_H + V_GAP), NODE_W, NODE_H)
        return NODE_W + H_GAP

    child_x = x
    total = 0
    for child in node.children:
        w = _layout_tree(child, child_x, depth + 1)
        child_x += w
        total += w

    left = node.children[0].rect.center().x()
    right = node.children[-1].rect.center().x()
    cx = (left + right) / 2
    node.rect = QRectF(cx - NODE_W / 2, depth * (NODE_H + V_GAP),
                       NODE_W, NODE_H)
    return total


class _TreeCanvas(QWidget):
    """Draws a single behaviour tree."""

    node_selected = Signal(object)  # BTNode

    def __init__(self, parent=None):
        super().__init__(parent)
        self._root: BTNode | None = None
        self._hovered: BTNode | None = None
        self.setMouseTracking(True)

    def set_tree(self, root: BTNode):
        self._root = root
        if root:
            _layout_tree(root, 20, 0)
            br = self._bounding_rect(root)
            self.setMinimumSize(int(br.width()) + 40,
                                int(br.height()) + 40)
        self.update()

    def _bounding_rect(self, node: BTNode) -> QRectF:
        r = QRectF(node.rect)
        for c in node.children:
            r = r.united(self._bounding_rect(c))
        return r

    def paintEvent(self, ev: QPaintEvent):
        if not self._root:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(BG_DARK))
        self._draw_node(p, self._root)
        p.end()

    def _draw_node(self, p: QPainter, node: BTNode):
        r = node.rect
        color = QColor(KIND_COLORS.get(node.kind, "#888"))

        if node.active:
            color = QColor(SUCCESS)
        if node is self._hovered:
            color = color.lighter(130)

        # Connections to children
        pen = QPen(QColor(BORDER), 2)
        p.setPen(pen)
        for child in node.children:
            p.drawLine(r.center().x(), r.bottom(),
                       child.rect.center().x(), child.rect.top())

        # Node rectangle
        path = QPainterPath()
        path.addRoundedRect(r, 8, 8)
        p.fillPath(path, QBrush(color))
        p.setPen(QPen(color.darker(150), 2))
        p.drawPath(path)

        # Symbol + text
        p.setPen(QPen(QColor("#ffffff"), 1))
        sym = KIND_SYMBOLS.get(node.kind, "")
        font = QFont("Segoe UI", 9, QFont.Bold)
        p.setFont(font)
        text = f"{sym} {node.name}"
        p.drawText(r.adjusted(6, 0, -6, 0), Qt.AlignVCenter | Qt.AlignLeft,
                   text)

        # Param indicator
        if node.params:
            p.setPen(QPen(QColor(WARNING), 1))
            p.setFont(QFont("Segoe UI", 7))
            p.drawText(r.adjusted(0, 0, -6, -2), Qt.AlignBottom | Qt.AlignRight,
                       f"[{len(node.params)} params]")

        for child in node.children:
            self._draw_node(p, child)

    def _hit_test(self, pos, node: BTNode) -> BTNode | None:
        if node.rect.contains(pos):
            return node
        for c in node.children:
            hit = self._hit_test(pos, c)
            if hit:
                return hit
        return None

    def mouseMoveEvent(self, ev: QMouseEvent):
        if not self._root:
            return
        pt = QPointF(ev.position())
        hit = self._hit_test(pt, self._root)
        if hit != self._hovered:
            self._hovered = hit
            self.update()
        if hit and hit.description:
            QToolTip.showText(ev.globalPosition().toPoint(), hit.description)

    def mousePressEvent(self, ev: QMouseEvent):
        if not self._root or ev.button() != Qt.LeftButton:
            return
        hit = self._hit_test(QPointF(ev.position()), self._root)
        if hit:
            self.node_selected.emit(hit)


# ── Parameter editor ─────────────────────────────────────────────────

class _ParamEditor(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)
        self._header = QLabel("Select a node to edit parameters")
        self._header.setWordWrap(True)
        self._header.setStyleSheet(f"color:{TEXT_DIM}; padding:8px;")
        self._layout.addWidget(self._header)
        self._grid_widget = QWidget()
        self._grid = QGridLayout(self._grid_widget)
        self._layout.addWidget(self._grid_widget)
        self._layout.addStretch()
        self._spinners = {}

    def show_node(self, node: BTNode):
        # Clear old
        while self._grid.count():
            w = self._grid.takeAt(0).widget()
            if w:
                w.deleteLater()
        self._spinners.clear()

        desc = node.description or ""
        self._header.setText(
            f"<b style='color:{ACCENT}'>{node.name}</b> "
            f"<span style='color:{TEXT_DIM}'>({node.kind})</span><br>"
            f"<span style='color:{TEXT_DIM}'>{desc}</span>")

        if not node.params:
            return

        row = 0
        for key, val in node.params.items():
            lbl = QLabel(key)
            lbl.setStyleSheet("font-weight:bold;")
            spin = QDoubleSpinBox()
            spin.setRange(-99999, 99999)
            spin.setDecimals(3)
            spin.setValue(float(val))
            spin.setSingleStep(0.1 if val < 10 else 10)
            self._grid.addWidget(lbl, row, 0)
            self._grid.addWidget(spin, row, 1)
            self._spinners[key] = spin
            row += 1


# ── Composite widget ─────────────────────────────────────────────────

class BehaviorTreePanel(QWidget):
    """Full behavior-tree panel with role selector, tree view, and param editor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("Behavior Tree")
        title.setStyleSheet(f"font-size:15px; font-weight:bold; color:{ACCENT};")
        hdr.addWidget(title)
        hdr.addStretch()

        hdr.addWidget(QLabel("Role:"))
        self._role_combo = QComboBox()
        self._role_combo.addItems(list(ROLE_TREES.keys()))
        self._role_combo.currentTextChanged.connect(self._on_role_changed)
        hdr.addWidget(self._role_combo)
        lay.addLayout(hdr)

        # Splitter: tree canvas | param editor
        splitter = QSplitter(Qt.Horizontal)

        # Scrollable tree
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self._canvas = _TreeCanvas()
        scroll.setWidget(self._canvas)
        splitter.addWidget(scroll)

        # Param editor
        self._editor = _ParamEditor()
        self._editor.setMinimumWidth(220)
        self._editor.setMaximumWidth(320)
        splitter.addWidget(self._editor)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        lay.addWidget(splitter)

        self._canvas.node_selected.connect(self._editor.show_node)

        # Legend
        legend = QHBoxLayout()
        for kind, col in KIND_COLORS.items():
            sym = KIND_SYMBOLS[kind]
            legend.addWidget(QLabel(
                f'<span style="color:{col}; font-weight:bold;">{sym}</span> '
                f'{kind.capitalize()}'))
        legend.addStretch()
        lay.addLayout(legend)

        # Load default
        self._on_role_changed(self._role_combo.currentText())

    def _on_role_changed(self, role_name):
        factory = ROLE_TREES.get(role_name)
        if factory:
            tree = factory()
            self._canvas.set_tree(tree)
