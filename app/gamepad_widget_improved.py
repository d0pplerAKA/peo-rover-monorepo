"""Xbox Series wireless controller — polished, vector-only PySide6 widget.

Drop-in replacement for app.gamepad_widget.GamepadWidget.

- Keeps: GamepadWidget(parent), set_state(GamepadState), state().
- Uses the same rover_comm.protocol button constants and GamepadState fields.
- Preserves live ABXY / D-pad / LB / RB / View / Menu / Share / LS / RS,
  thumbstick offsets, LT / RT levels, battery and connection indication.
- Design is based on the black Xbox Series controller supplied by the user.
- No external images/SVG/assets required: all geometry is vector QPainter.
- Static background/chassis are cached at 2x design resolution for 50 Hz updates.

Optional customization:
    widget = GamepadWidget(colorway="black", background="#eef1f4")
    widget.set_colorway("velocity_green")
    widget.set_background(None)          # transparent: inherit parent's background

Standalone demo (requires PySide6):
    python gamepad_widget_improved.py

Notes:
    The source protocol uses 0..65535 sticks (center=32768), 0..1023 triggers.
    Controller diagrams necessarily use less physical throw than real sticks;
    offsets are exaggerated slightly for readable telemetry.
"""

from __future__ import annotations

import math
from typing import Any

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QBrush, QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPen,
    QPixmap, QRadialGradient,
)
from PySide6.QtWidgets import QWidget

# Keep the original application's public imports. The small fallback is only for
# launching THIS file directly to inspect its appearance outside the project.
try:
    from rover_comm.protocol import (
        GamepadState,
        BTN_A, BTN_B, BTN_X, BTN_Y,
        BTN_LB, BTN_RB, BTN_MENU, BTN_VIEW, BTN_SHARE,
        BTN_LS, BTN_RS,
        BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT,
    )
except ModuleNotFoundError:
    if __name__ != "__main__":
        raise
    BTN_A, BTN_B, BTN_X, BTN_Y = "A", "B", "X", "Y"
    BTN_LB, BTN_RB = "LB", "RB"
    BTN_MENU, BTN_VIEW, BTN_SHARE = "MENU", "VIEW", "SHARE"
    BTN_LS, BTN_RS = "LS", "RS"
    BTN_DPAD_UP, BTN_DPAD_DOWN = "UP", "DOWN"
    BTN_DPAD_LEFT, BTN_DPAD_RIGHT = "LEFT", "RIGHT"

    class GamepadState:  # pragma: no cover - standalone demo only
        connected = False
        joy_l = (32768, 32768)
        joy_r = (32768, 32768)
        triggers = (0, 0)
        battery = 0

        def pressed(self, button: Any) -> bool:
            return False

try:
    from app.i18n import tr
except ModuleNotFoundError:
    if __name__ != "__main__":
        raise

    def tr(key: str) -> str:  # pragma: no cover - standalone demo only
        return {"gamepad_off": "Disconnected", "gamepad_batt": "Battery"}.get(key, key)


# The 690x520 viewBox leaves a little breathing room around the entire shell.
DW, DH = 690.0, 520.0
# Content crop excludes unnecessary outer whitespace in compact application panels.
_VIEWPORT = QRectF(28.0, 46.0, 634.0, 458.0)
_STICK_L = (193.0, 176.0)
_STICK_R = (417.0, 260.0)
_DPAD = (266.0, 263.0)
_LOGO = (345.0, 113.0)
_VIEW = (301.0, 174.0)
_MENU = (387.0, 174.0)
_SHARE = (345.0, 210.0)
_ABXY = {
    BTN_X: (452.0, 172.0, "X", "#3aa4f5"),
    BTN_Y: (494.0, 133.0, "Y", "#f2ce35"),
    BTN_B: (536.0, 172.0, "B", "#f05a4e"),
    BTN_A: (494.0, 211.0, "A", "#6ad55e"),
}

_COLORWAYS = {
    "black": {
        "shell_top": "#393b40", "shell_mid": "#292c31",
        "shell_lower": "#202328", "shell_bottom": "#1a1c20",
        "shell_edge": "#101216", "rim": "#52565c",
        "grip_light": "#757982", "grip_dark": "#07090b",
        "accent": "#7ac6ff", "mark": "#e6e8eb",
    },
    "velocity_green": {
        "shell_top": "#9bd64a", "shell_mid": "#82c335",
        "shell_lower": "#6bad2b", "shell_bottom": "#609526",
        "shell_edge": "#456a20", "rim": "#d2f290",
        "grip_light": "#e1f5b4", "grip_dark": "#365a19",
        "accent": "#b6f7ff", "mark": "#f7ffec",
    },
}


def _c(value: str | QColor, alpha: int | None = None) -> QColor:
    color = QColor(value)
    if alpha is not None:
        color.setAlpha(max(0, min(255, alpha)))
    return color


def _pen(color: str | QColor, width: float = 1.0) -> QPen:
    pen = QPen(_c(color))
    pen.setWidthF(width)
    pen.setJoinStyle(Qt.RoundJoin)
    pen.setCapStyle(Qt.RoundCap)
    return pen


def _ellipse(p: QPainter, x: float, y: float, rx: float, ry: float | None = None):
    p.drawEllipse(QPointF(x, y), rx, rx if ry is None else ry)


def _font(px: float, weight: QFont.Weight = QFont.Normal) -> QFont:
    f = QFont("Segoe UI", int(round(px)))
    f.setPixelSize(int(round(px)))
    f.setWeight(weight)
    return f


def _polygon(points: list[tuple[float, float]]) -> QPainterPath:
    path = QPainterPath()
    for i, (x, y) in enumerate(points):
        if i == 0:
            path.moveTo(x, y)
        else:
            path.lineTo(x, y)
    path.closeSubpath()
    return path


def _body_path() -> QPainterPath:
    """Smooth Xbox Series silhouette, including the deep central palm cutout."""
    path = QPainterPath()
    path.moveTo(105, 128)
    path.cubicTo(119, 97, 145, 79, 176, 76)
    path.cubicTo(232, 68, 290, 79, 345, 79)
    path.cubicTo(400, 79, 457, 68, 514, 76)
    path.cubicTo(552, 80, 568, 101, 583, 138)
    path.cubicTo(611, 208, 629, 303, 636, 361)
    path.cubicTo(644, 406, 631, 445, 608, 458)
    path.cubicTo(593, 471, 577, 468, 561, 451)
    path.cubicTo(537, 427, 510, 381, 481, 361)
    path.cubicTo(468, 352, 454, 348, 430, 348)
    path.lineTo(260, 348)
    path.cubicTo(238, 348, 224, 353, 208, 366)
    path.cubicTo(182, 390, 152, 433, 128, 454)
    path.cubicTo(107, 474, 90, 467, 75, 451)
    path.cubicTo(51, 423, 50, 391, 57, 349)
    path.cubicTo(66, 269, 83, 182, 105, 128)
    path.closeSubpath()
    return path


class GamepadWidget(QWidget):
    """Realtime, scalable Xbox Series front view (all rendering in QPainter)."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        colorway: str = "black",
        background: str | QColor | None = "#eef1f4",
    ):
        super().__init__(parent)
        if colorway not in _COLORWAYS:
            raise ValueError(f"Unknown colorway {colorway!r}; choose {tuple(_COLORWAYS)}")
        self._state = GamepadState()
        self._colorway = colorway
        self._colors = _COLORWAYS[colorway]
        self._background = _c(background) if background is not None else None
        self._static_layer: QPixmap | None = None
        self._body = _body_path()
        self.setMinimumSize(280, 160)
        self.setAttribute(Qt.WA_TranslucentBackground, True)

    def set_state(self, s: GamepadState) -> None:
        """Original interface: supply an immutable/readonly protocol snapshot."""
        self._state = s
        self.update()

    def state(self) -> GamepadState:
        return self._state

    def set_colorway(self, colorway: str) -> None:
        if colorway not in _COLORWAYS:
            raise ValueError(f"Unknown colorway {colorway!r}; choose {tuple(_COLORWAYS)}")
        if colorway != self._colorway:
            self._colorway = colorway
            self._colors = _COLORWAYS[colorway]
            self._static_layer = None
            self.update()

    def set_background(self, color: str | QColor | None) -> None:
        """Set an opaque background color, or None for transparency."""
        self._background = _c(color) if color is not None else None
        self._static_layer = None
        self.update()

    def sizeHint(self):
        from PySide6.QtCore import QSize
        return QSize(690, 520)

    def paintEvent(self, event) -> None:
        if self._static_layer is None:
            self._make_static_layer()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self._draw_background(p)
        scale = min(self.width() / _VIEWPORT.width(),
                    self.height() / _VIEWPORT.height())
        p.translate((self.width() - _VIEWPORT.width() * scale) / 2
                    - _VIEWPORT.x() * scale,
                    (self.height() - _VIEWPORT.height() * scale) / 2
                    - _VIEWPORT.y() * scale)
        p.scale(scale, scale)
        if self._static_layer is not None:
            p.drawPixmap(QRectF(0, 0, DW, DH), self._static_layer,
                         QRectF(0, 0, self._static_layer.width(), self._static_layer.height()))
        self._draw_trigger_feedback(p)
        self._draw_bumper_feedback(p)
        self._draw_stick(p, _STICK_L, self._joy("joy_l"), BTN_LS)
        self._draw_stick(p, _STICK_R, self._joy("joy_r"), BTN_RS)
        self._draw_dpad(p)
        self._draw_face_buttons(p)
        self._draw_xbox_button(p)
        self._draw_utility_button(p, *_VIEW, "VIEW")
        self._draw_utility_button(p, *_MENU, "MENU")
        self._draw_utility_button(p, *_SHARE, "SHARE")
        self._draw_status(p)
        p.end()

    def _make_static_layer(self) -> None:
        """One-time 2x rasterization of static 2.5D chassis for smooth 50 Hz paint."""
        ratio = 2
        img = QPixmap(int(DW * ratio), int(DH * ratio))
        img.fill(Qt.transparent)
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.scale(ratio, ratio)
        self._draw_floor_shadow(p)
        self._draw_rear_shell(p)
        self._draw_trigger_molds(p)
        self._draw_bumpers(p)
        self._draw_body(p)
        p.end()
        self._static_layer = img

    def _draw_background(self, p: QPainter) -> None:
        if self._background is None:
            return
        base = self._background
        grad = QLinearGradient(0, 0, self.width(), self.height())
        grad.setColorAt(0, base.lighter(105))
        grad.setColorAt(0.55, base)
        grad.setColorAt(1, base.darker(105))
        p.fillRect(self.rect(), QBrush(grad))

    def _draw_floor_shadow(self, p: QPainter) -> None:
        # Soft pooled shadow; not a rectangular QGraphicsDropShadowEffect.
        p.setPen(Qt.NoPen)
        for i in range(22, 0, -1):
            alpha = int(4 + 30 * (1 - i / 22) ** 2)
            p.setBrush(QColor(15, 18, 21, alpha))
            _ellipse(p, 347, 456, 250 + i * 1.35, 20 + i * 1.35)
        for i in range(14, 0, -1):
            p.save()
            p.translate(0, 4 + i * 0.9)
            p.setBrush(QColor(15, 18, 22, max(1, int(13 - i * .7))))
            p.drawPath(self._body)
            p.restore()

    def _draw_rear_shell(self, p: QPainter) -> None:
        p.save()
        p.translate(0, 6)
        back = QLinearGradient(0, 80, 0, 467)
        back.setColorAt(0, _c("#1c2025"))
        back.setColorAt(1, _c("#090a0c"))
        p.setPen(_pen("#08090a", 2.6))
        p.setBrush(QBrush(back))
        p.drawPath(self._body)
        p.restore()

    def _draw_trigger_molds(self, p: QPainter) -> None:
        # Left and right trigger tips appear behind the shoulders.
        for mirror in (False, True):
            p.save()
            if mirror:
                p.translate(DW, 0)
                p.scale(-1, 1)
            path = QPainterPath()
            path.moveTo(137, 88)
            path.cubicTo(149, 63, 182, 56, 213, 58)
            path.cubicTo(235, 57, 249, 66, 250, 82)
            path.lineTo(139, 104)
            path.closeSubpath()
            grad = QLinearGradient(170, 54, 196, 105)
            grad.setColorAt(0, _c("#4b4d50"))
            grad.setColorAt(0.5, _c("#202328"))
            grad.setColorAt(1, _c("#0a0c10"))
            p.setPen(_pen("#101115", 1.6))
            p.setBrush(QBrush(grad))
            p.drawPath(path)
            p.restore()

    def _draw_bumpers(self, p: QPainter) -> None:
        for mirror in (False, True):
            p.save()
            if mirror:
                p.translate(DW, 0)
                p.scale(-1, 1)
            path = QPainterPath()
            path.moveTo(122, 113)
            path.cubicTo(130, 86, 158, 74, 197, 72)
            path.cubicTo(225, 69, 240, 77, 252, 87)
            path.cubicTo(209, 89, 155, 105, 122, 113)
            path.closeSubpath()
            grad = QLinearGradient(158, 72, 190, 117)
            grad.setColorAt(0, _c("#3e4145"))
            grad.setColorAt(0.53, _c("#25282b"))
            grad.setColorAt(1, _c("#0d0e11"))
            p.setBrush(QBrush(grad))
            p.setPen(_pen("#141518", 1.8))
            p.drawPath(path)
            gleam = QPainterPath()
            gleam.moveTo(140, 90)
            gleam.cubicTo(166, 76, 201, 75, 228, 80)
            p.setBrush(Qt.NoBrush)
            p.setPen(_pen(QColor(220, 226, 234, 40), 1.6))
            p.drawPath(gleam)
            p.restore()

    def _draw_body(self, p: QPainter) -> None:
        c = self._colors
        # Main, dark, slightly metallic polycarbonate finish.
        shell = QLinearGradient(85, 73, 580, 452)
        shell.setColorAt(0.0, _c(c["shell_top"]))
        shell.setColorAt(0.37, _c(c["shell_mid"]))
        shell.setColorAt(0.72, _c(c["shell_lower"]))
        shell.setColorAt(1.0, _c(c["shell_bottom"]))
        p.setBrush(QBrush(shell))
        p.setPen(_pen(c["shell_edge"], 2.2))
        p.drawPath(self._body)
        p.save()
        p.setClipPath(self._body)

        # Large, very restrained matte highlights give the shell its 3D volume.
        for cx, cy, radius, opacity in ((182, 124, 249, 48), (506, 135, 251, 35)):
            grad = QRadialGradient(cx, cy, radius)
            grad.setColorAt(0, QColor(240, 244, 248, opacity))
            grad.setColorAt(0.48, QColor(210, 215, 223, int(opacity / 3)))
            grad.setColorAt(1, QColor(210, 215, 223, 0))
            p.fillRect(QRectF(45, 75, 600, 390), QBrush(grad))

        # Curved, specular side reflections on both handles.
        for mirror in (False, True):
            p.save()
            if mirror:
                p.translate(DW, 0)
                p.scale(-1, 1)
            haze = QRadialGradient(81, 260, 176)
            hc = _c(c["grip_light"])
            haze.setColorAt(0.0, _c(hc, 56))
            haze.setColorAt(0.56, _c(hc, 18))
            haze.setColorAt(1.0, _c(hc, 0))
            p.fillRect(QRectF(45, 90, 200, 380), QBrush(haze))
            side = QPainterPath()
            side.moveTo(102, 135)
            side.cubicTo(80, 192, 69, 272, 61, 355)
            side.cubicTo(60, 392, 63, 422, 82, 447)
            p.setBrush(Qt.NoBrush)
            p.setPen(_pen(QColor(231, 236, 240, 22), 5))
            p.drawPath(side)
            p.restore()

        # Extremely fine molded-surface sweep, not bright cartoon outlines.
        top = QPainterPath()
        top.moveTo(175, 88)
        top.cubicTo(251, 76, 299, 90, 345, 89)
        top.cubicTo(401, 88, 452, 77, 514, 88)
        p.setPen(_pen(QColor(245, 248, 252, 43), 1.6))
        p.setBrush(Qt.NoBrush)
        p.drawPath(top)
        center_arch = QPainterPath()
        center_arch.moveTo(210, 370)
        center_arch.cubicTo(237, 345, 246, 344, 274, 344)
        center_arch.lineTo(426, 344)
        center_arch.cubicTo(451, 344, 474, 354, 490, 375)
        p.setPen(_pen(QColor(0, 0, 0, 56), 6))
        p.drawPath(center_arch)
        p.setPen(_pen(QColor(243, 247, 252, 14), 1.2))
        p.drawPath(center_arch)
        p.restore()

        # An almost imperceptible line at the top edge suggests the housing seam.
        p.setPen(_pen(QColor(0, 0, 0, 75), 1.3))
        p.drawPath(self._body)

    def _connected(self) -> bool:
        return bool(getattr(self._state, "connected", False))

    def _pressed(self, key: Any) -> bool:
        return self._connected() and bool(self._state.pressed(key))

    def _joy(self, name: str) -> tuple[float, float]:
        value = getattr(self._state, name, (32768, 32768))
        try:
            x, y = value
            return float(x), float(y)
        except (TypeError, ValueError):
            return (32768.0, 32768.0)

    def _draw_trigger_feedback(self, p: QPainter) -> None:
        left, right = getattr(self._state, "triggers", (0, 0))
        for side, value in enumerate((left, right)):
            level = max(0.0, min(1.0, float(value) / 1023.0)) if self._connected() else 0.0
            # Tiny shoulder-top readout; zero = no artificial stripe in photo mode.
            if level <= 0.001:
                continue
            x = 158 if side == 0 else 468
            y = 73
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(4, 7, 12, 160))
            p.drawRoundedRect(QRectF(x, y, 62, 4.2), 2, 2)
            grad = QLinearGradient(x, y, x, y + 5)
            grad.setColorAt(0, _c("#e0f6ff"))
            grad.setColorAt(1, _c(self._colors["accent"]))
            p.setBrush(QBrush(grad))
            p.drawRoundedRect(QRectF(x, y, max(3, 62 * level), 4.2), 2, 2)

    def _draw_bumper_feedback(self, p: QPainter) -> None:
        for side, button in ((0, BTN_LB), (1, BTN_RB)):
            if not self._pressed(button):
                continue
            p.save()
            if side == 1:
                p.translate(DW, 0)
                p.scale(-1, 1)
            p.setPen(_pen(_c(self._colors["accent"], 215), 2.8))
            p.setBrush(Qt.NoBrush)
            bp = QPainterPath()
            bp.moveTo(137, 94)
            bp.cubicTo(166, 77, 207, 74, 238, 84)
            p.drawPath(bp)
            p.restore()

    def _draw_stick(
        self, p: QPainter, center: tuple[float, float],
        joy: tuple[float, float], click_button: Any,
    ) -> None:
        cx, cy = center
        engaged = self._pressed(click_button)
        r = 45.0
        # Inner socket well, including ring chamfer and deeper center shadow.
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 62))
        _ellipse(p, cx + 1.1, cy + 3, r + 4)
        socket = QRadialGradient(cx - 7, cy - 10, r * 1.8)
        socket.setColorAt(0, _c("#050608"))
        socket.setColorAt(0.64, _c("#0f1114"))
        socket.setColorAt(0.82, _c("#393c42"))
        socket.setColorAt(1, _c("#101216"))
        p.setBrush(QBrush(socket))
        p.setPen(_pen(QColor(5, 5, 7, 220), 2))
        _ellipse(p, cx, cy, r)
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(QColor(231, 235, 242, 43), 2.0))
        p.drawArc(QRectF(cx-r+3, cy-r+3, 2*(r-3), 2*(r-3)), 19*16, 133*16)
        p.setPen(_pen(QColor(0, 0, 0, 175), 2.3))
        p.drawArc(QRectF(cx-r+3, cy-r+3, 2*(r-3), 2*(r-3)), 198*16, 141*16)

        # Center 32768 from original GamepadState protocol. Bound to a circle.
        dx, dy = 0.0, 0.0
        if self._connected():
            dx = max(-1.0, min(1.0, (joy[0] - 32768.0) / 32768.0))
            dy = max(-1.0, min(1.0, (joy[1] - 32768.0) / 32768.0))
            mag = math.hypot(dx, dy)
            if mag > 1:
                dx, dy = dx / mag, dy / mag
        # Slight exaggeration makes telemetry perceptible without leaving well.
        hx, hy = cx + dx * 13, cy + dy * 13 + (1.5 if engaged else 0)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 110))
        _ellipse(p, hx + 2.4, hy + 4, 35)
        if engaged:
            p.setBrush(_c(self._colors["accent"], 48))
            _ellipse(p, hx, hy, 41)

        # A broad raised stick top matches the 3D Xbox cap (vs tiny round dot).
        rim = QRadialGradient(hx-8, hy-9, 53)
        rim.setColorAt(0.0, _c("#50545a"))
        rim.setColorAt(0.6, _c("#292c30"))
        rim.setColorAt(0.83, _c("#0e1012"))
        rim.setColorAt(1, _c("#050608"))
        p.setBrush(QBrush(rim))
        p.setPen(_pen("#08090b", 1.3))
        _ellipse(p, hx, hy, 34)
        # Narrow thumb grip lip (highlight top-left, shadow bottom-right).
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(QColor(240, 245, 250, 58), 2))
        p.drawArc(QRectF(hx-31, hy-31, 62, 62), 25*16, 140*16)
        p.setPen(_pen(QColor(0, 0, 0, 160), 2.8))
        p.drawArc(QRectF(hx-30, hy-30, 60, 60), 191*16, 163*16)

        # Fine rubber knurling around perimeter, deliberately not fluorescent.
        p.setPen(_pen(QColor(10, 12, 14, 104), 1))
        for k in range(64):
            a = 2 * math.pi * k / 64
            x1, y1 = hx + math.cos(a)*30.1, hy + math.sin(a)*30.1
            x2, y2 = hx + math.cos(a)*32.6, hy + math.sin(a)*32.6
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Slightly sunken concave center and soft edge transition.
        face = QRadialGradient(hx - 8, hy - 9, 42)
        face.setColorAt(0.0, _c("#33363b"))
        face.setColorAt(0.65, _c("#25282d"))
        face.setColorAt(0.9, _c("#131519"))
        face.setColorAt(1, _c("#0b0c0e"))
        p.setPen(_pen(QColor(6, 7, 9, 140), 1.2))
        p.setBrush(QBrush(face))
        _ellipse(p, hx, hy, 25.6)
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(QColor(227, 232, 237, 31), 1.4))
        p.drawArc(QRectF(hx-24.7, hy-24.7, 49.4, 49.4), 20*16, 132*16)
        if engaged:
            p.setPen(_pen(_c(self._colors["accent"], 210), 2.4))
            _ellipse(p, hx, hy, 35)

    def _draw_dpad(self, p: QPainter) -> None:
        cx, cy = _DPAD
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 78))
        _ellipse(p, cx + 1, cy + 3, 48)
        base = QRadialGradient(cx - 9, cy - 13, 83)
        base.setColorAt(0, _c("#41454a"))
        base.setColorAt(0.5, _c("#25282d"))
        base.setColorAt(0.8, _c("#0b0d10"))
        base.setColorAt(1, _c("#0a0b0e"))
        p.setPen(_pen("#090a0c", 2.1))
        p.setBrush(QBrush(base))
        _ellipse(p, cx, cy, 47)
        p.setPen(_pen(QColor(220, 224, 230, 31), 1.7))
        p.setBrush(Qt.NoBrush)
        p.drawArc(QRectF(cx-43, cy-43, 86, 86), 27*16, 134*16)

        # Central circular dish below the Xbox hybrid cross design.
        disk = QLinearGradient(cx, cy - 39, cx, cy + 39)
        disk.setColorAt(0, _c("#34383d"))
        disk.setColorAt(0.5, _c("#2c3035"))
        disk.setColorAt(1, _c("#1b1e22"))
        p.setBrush(QBrush(disk))
        p.setPen(_pen("#14171a", 1.5))
        _ellipse(p, cx, cy, 39.2)

        # Hybrid faceted cross: four broad, distinct buttons meeting centrally.
        buttons = (
            (BTN_DPAD_UP, [(cx-12,cy-11),(cx-12,cy-37),(cx+12,cy-37),(cx+12,cy-11)]),
            (BTN_DPAD_DOWN, [(cx-12,cy+11),(cx+12,cy+11),(cx+12,cy+37),(cx-12,cy+37)]),
            (BTN_DPAD_LEFT, [(cx-37,cy-12),(cx-11,cy-12),(cx-11,cy+12),(cx-37,cy+12)]),
            (BTN_DPAD_RIGHT, [(cx+11,cy-12),(cx+37,cy-12),(cx+37,cy+12),(cx+11,cy+12)]),
        )
        for btn, pts in buttons:
            pressed = self._pressed(btn)
            p.setPen(_pen("#141619", 1.05))
            g = QLinearGradient(cx-24, cy-38, cx+30, cy+39)
            g.setColorAt(0, _c("#50545a" if pressed else "#41454b"))
            g.setColorAt(1, _c("#2e4555" if pressed else "#272b30"))
            p.setBrush(QBrush(g))
            p.drawPath(_polygon(pts))
            if pressed:
                p.setBrush(_c(self._colors["accent"], 72))
                p.setPen(Qt.NoPen)
                p.drawPath(_polygon(pts))
        # Center hub is a slightly raised square; fine seams create four facets.
        p.setPen(_pen("#151719", 1))
        mid = QLinearGradient(cx-10, cy-10, cx+10, cy+10)
        mid.setColorAt(0, _c("#40454a"))
        mid.setColorAt(1, _c("#30343a"))
        p.setBrush(QBrush(mid))
        p.drawRoundedRect(QRectF(cx-12, cy-12, 24, 24), 2.4, 2.4)
        p.setPen(_pen(QColor(230, 233, 238, 30), 1))
        p.drawLine(QPointF(cx-10,cy-11),QPointF(cx+10,cy-11))
        # Four grooves visually divide the dish without neon dividers.
        p.setPen(_pen(QColor(6, 7, 8, 110), 1.2))
        for dx, dy in ((0,-1),(0,1),(-1,0),(1,0)):
            p.drawLine(QPointF(cx+dx*13,cy+dy*13), QPointF(cx+dx*36,cy+dy*36))

    def _draw_face_buttons(self, p: QPainter) -> None:
        for button, (cx, cy, char, color) in _ABXY.items():
            pressed = self._pressed(button)
            yy = cy + (1.8 if pressed else 0.0)
            if pressed:
                p.setPen(Qt.NoPen)
                p.setBrush(_c(color, 55))
                _ellipse(p, cx, yy, 27)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 90))
            _ellipse(p, cx+1, yy+2, 22.4)
            bezel = QLinearGradient(cx, yy-23, cx, yy+23)
            bezel.setColorAt(0, _c("#42464b"))
            bezel.setColorAt(.5, _c("#202328"))
            bezel.setColorAt(1, _c("#08090a"))
            p.setPen(_pen("#101215", 1.1))
            p.setBrush(QBrush(bezel))
            _ellipse(p, cx, yy, 21.4)
            face = QRadialGradient(cx-6, yy-7, 31)
            face.setColorAt(0, _c("#24282c" if pressed else "#23272b"))
            face.setColorAt(.75, _c("#121417"))
            face.setColorAt(1, _c("#080a0c"))
            p.setBrush(QBrush(face))
            p.setPen(_pen("#292d31", 0.85))
            _ellipse(p, cx, yy, 18.3)
            p.setPen(_pen(QColor(255, 255, 255, 31), 1.1))
            p.setBrush(Qt.NoBrush)
            p.drawArc(QRectF(cx-18, yy-18, 36, 36), 20*16, 135*16)
            p.setFont(_font(24, QFont.Bold))
            p.setPen(_c(color if pressed else color))
            p.drawText(QRectF(cx-21, yy-21.5, 42, 43), Qt.AlignCenter, char)

    def _draw_xbox_button(self, p: QPainter) -> None:
        cx, cy = _LOGO
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 92))
        _ellipse(p, cx+0.7, cy+2.1, 25.3)
        lip = QLinearGradient(cx, cy-25, cx, cy+25)
        lip.setColorAt(0, _c("#60636a"))
        lip.setColorAt(1, _c("#131619"))
        p.setBrush(QBrush(lip))
        _ellipse(p, cx, cy, 24.2)
        lit = self._connected()
        sphere = QRadialGradient(cx-7, cy-10, 44)
        sphere.setColorAt(0, _c("#ffffff" if lit else "#e5e7eb"))
        sphere.setColorAt(0.7, _c("#f2f4f6" if lit else "#ccd0d4"))
        sphere.setColorAt(1, _c("#acb2b9"))
        p.setPen(_pen("#30343a", 0.9))
        p.setBrush(QBrush(sphere))
        _ellipse(p, cx, cy, 21.6)

        # Engraved Xbox 'X' ribbons, clipped to the white ball.
        p.save()
        mask = QPainterPath()
        mask.addEllipse(QPointF(cx, cy), 20.3, 20.3)
        p.setClipPath(mask)
        p.translate(cx, cy)
        p.setPen(Qt.NoPen)
        ink = _c("#4a4d52" if lit else "#666a70")
        p.setBrush(ink)
        shapes = (
            [(-25,-23),(-9,-20),(-3,-11),(-10,-1),(-19,2),(-28,-3)],
            [(25,-23),(9,-20),(3,-11),(10,-1),(19,2),(28,-3)],
            [(-24,24),(-9,20),(-2,9),(-9,1),(-20,-4),(-29,4)],
            [(24,24),(9,20),(2,9),(9,1),(20,-4),(29,4)],
        )
        for shape in shapes:
            p.drawPath(_polygon(shape))
        p.restore()
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(QColor(255, 255, 255, 115), 1.2))
        p.drawArc(QRectF(cx-20,cy-20,40,40), 20*16, 110*16)

    def _draw_utility_button(self, p: QPainter, cx: float, cy: float, name: str) -> None:
        on = self._pressed({"VIEW": BTN_VIEW, "MENU": BTN_MENU, "SHARE": BTN_SHARE}[name])
        r = 13.5 if name != "SHARE" else 11.2
        cy += 1.3 if on else 0
        if on:
            p.setPen(Qt.NoPen)
            p.setBrush(_c(self._colors["accent"], 50))
            _ellipse(p, cx, cy, r + 4)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 62))
        _ellipse(p, cx+0.5, cy+1.6, r+1.8)
        grad = QLinearGradient(cx,cy-r,cx,cy+r)
        grad.setColorAt(0, _c("#5a5d63" if on else "#43474d"))
        grad.setColorAt(1, _c("#181a1e"))
        p.setBrush(QBrush(grad))
        p.setPen(_pen("#111316", 1.2))
        _ellipse(p, cx, cy, r)
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(QColor(210, 216, 221, 46), 1.0))
        p.drawArc(QRectF(cx-r+2,cy-r+2,2*r-4,2*r-4), 22*16, 135*16)
        ico = _c("#dfe4ea" if on else "#9ea4aa")
        p.setPen(_pen(ico, 1.65))
        if name == "VIEW":
            p.drawRoundedRect(QRectF(cx-5.8,cy-4.1,7.0,7.2),.6,.6)
            p.drawRoundedRect(QRectF(cx-1.4,cy-2.3,7.2,7.2),.6,.6)
        elif name == "MENU":
            for offset in (-3.6, 0, 3.6):
                p.drawLine(QPointF(cx-5.5,cy+offset),QPointF(cx+5.5,cy+offset))
        else:
            p.drawRoundedRect(QRectF(cx-5.0,cy-2.0,10,7),1.2,1.2)
            p.drawLine(QPointF(cx,cy+0.2),QPointF(cx,cy-6.5))
            p.drawLine(QPointF(cx,cy-6.5),QPointF(cx-2.8,cy-3.4))
            p.drawLine(QPointF(cx,cy-6.5),QPointF(cx+2.8,cy-3.4))

    def _draw_status(self, p: QPainter) -> None:
        """Quiet status strip OUTSIDE the shell, never an opaque overlay."""
        connected = self._connected()
        # Dark text on white backing, light text if host requests dark backing.
        light_background = (self._background is not None and
                            self._background.lightness() >= 128)
        label = _c("#626a75" if light_background else "#d3dbe4")
        subdued = _c("#9299a3" if light_background else "#9ca5b1")
        x, y = 89, 489
        p.setPen(Qt.NoPen)
        p.setBrush(_c("#399c6a" if connected else "#8b94a0"))
        _ellipse(p, x, y, 4.7)
        p.setPen(label)
        p.setFont(_font(12, QFont.DemiBold))
        status = tr("gamepad_on") if connected else tr("gamepad_off")
        p.drawText(QRectF(x+12, y-10, 165, 20), Qt.AlignLeft | Qt.AlignVCenter, status)
        level = getattr(self._state, "battery", 0)
        try:
            level = int(max(0, min(100, float(level))))
        except (TypeError, ValueError):
            level = 0
        bx, by = 541, 482
        p.setBrush(Qt.NoBrush)
        p.setPen(_pen(subdued, 1.3))
        p.drawRoundedRect(QRectF(bx,by,27,14),2.6,2.6)
        p.drawRect(QRectF(bx+27.6,by+4.1,2.3,5.8))
        if connected and level > 0:
            p.setPen(Qt.NoPen)
            p.setBrush(_c("#4aa775" if level>20 else "#d49d4d"))
            p.drawRoundedRect(QRectF(bx+2.5,by+2.5,22*level/100,9),1.2,1.2)
        p.setPen(label)
        p.setFont(_font(12, QFont.DemiBold))
        p.drawText(QRectF(577, by-3, 80, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   f"{level}%" if connected else "--")


if __name__ == "__main__":  # pragma: no cover - demo, not used by project
    import sys
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication, QVBoxLayout, QWidget as Panel

    class PreviewState:
        connected = True
        battery = 87
        joy_l = (32768, 32768)
        joy_r = (32768, 32768)
        triggers = (0, 0)
        active: set[Any] = set()

        def pressed(self, button: Any) -> bool:
            return button in self.active

    app = QApplication(sys.argv)
    window = Panel()
    window.setWindowTitle("Xbox Series — Vector Gamepad Preview")
    layout = QVBoxLayout(window)
    layout.setContentsMargins(0, 0, 0, 0)
    gamepad = GamepadWidget(background="#edf0f3")
    layout.addWidget(gamepad)
    window.resize(920, 690)

    preview_state = PreviewState()
    tick = [0]

    def animate() -> None:
        t = tick[0] * .045
        tick[0] += 1
        preview_state.joy_l = (
            int(32768 + math.cos(t) * 23000),
            int(32768 + math.sin(t) * 23000),
        )
        preview_state.joy_r = (32768, 32768)
        preview_state.triggers = (int((math.sin(t)+1)*511.5), 0)
        preview_state.active = {BTN_A} if math.sin(t*1.4) > 0.65 else set()
        gamepad.set_state(preview_state)

    timer = QTimer(window)
    timer.timeout.connect(animate)
    timer.start(20)
    animate()
    window.show()
    sys.exit(app.exec())
