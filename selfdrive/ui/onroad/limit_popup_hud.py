"""Onroad widget for the LIMIT-plate long-press popup (model in limit_popup.py)."""
from __future__ import annotations

import time

import pyray as rl
from openpilot.selfdrive.mapd.limit_source import PARAM_PREFER_OSM
from openpilot.selfdrive.ui.onroad.limit_popup import LimitPopupModel, hit_name, popup_rects
from openpilot.system.ui.lib.application import FontWeight, MousePos, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PANEL_BG = rl.Color(20, 20, 20, 235)
BTN_BG = rl.Color(60, 60, 60, 255)
BTN_HOT = rl.Color(40, 120, 70, 255)
INK = rl.WHITE
DIM = rl.Color(170, 170, 170, 255)
PARAM_POLL_S = 2.0


def _r(t) -> rl.Rectangle:
  return rl.Rectangle(t[0], t[1], t[2], t[3])


class LimitPopupHud(Widget):
  def __init__(self, params=None):
    super().__init__()
    self.model = LimitPopupModel()
    self._sign: tuple[float, float, float, float] | None = None
    self._screen = (0.0, 0.0, 0.0, 0.0)
    self._rects = popup_rects(0, 0, 2160, 1080)
    self._params = params
    self._prefer_osm = False
    self._poll_t = -1e9
    self.current_mph = 0
    self.tesla_mph = 0
    self.osm_mph = 0
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font = gui_app.font(FontWeight.SEMI_BOLD)

  def set_inputs(self, sign_rect: rl.Rectangle | None, current_mph: float, tesla_mph: int, osm_mph: int) -> None:
    self._sign = (sign_rect.x, sign_rect.y, sign_rect.width, sign_rect.height) if sign_rect is not None else None
    self.current_mph = int(round(current_mph))
    self.tesla_mph = int(tesla_mph)
    self.osm_mph = int(osm_mph)

  @property
  def is_open(self) -> bool:
    return self.model.is_open

  def owns_touch(self, x: float, y: float) -> bool:
    if self.model.is_open:
      return True
    s = self._sign
    return s is not None and s[0] <= x <= s[0] + s[2] and s[1] <= y <= s[1] + s[3]

  @property
  def _hit_rect(self) -> rl.Rectangle:
    if self.model.is_open:
      return self._rect
    return _r(self._sign) if self._sign is not None else rl.Rectangle(0, 0, 0, 0)

  def _update_state(self) -> None:
    now = time.monotonic()
    if now - self._poll_t > PARAM_POLL_S:
      self._poll_t = now
      try:
        if self._params is None:
          from openpilot.common.params import Params
          self._params = Params()
        self._prefer_osm = bool(self._params.get_bool(PARAM_PREFER_OSM))
      except Exception:
        pass
    if self._sign is None and not self.model.is_open:
      self.model.release()
    self.model.tick(self.is_pressed and not self.model.is_open, self.current_mph, self.tesla_mph, self.osm_mph,
                    self.model.prefer_osm if self.model.is_open else self._prefer_osm)

  def _layout(self) -> None:
    r = self._rect
    self._rects = popup_rects(r.x, r.y, r.width, r.height)

  def _handle_mouse_press(self, mouse_pos: MousePos) -> None:
    if self.model.is_open:
      act = self.model.tap(hit_name(mouse_pos.x, mouse_pos.y, self._rects))
      if act == "source":
        self._prefer_osm = self.model.prefer_osm
        self._poll_t = time.monotonic()
    else:
      self.model.press_sign()

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    self.model.release()

  def _btn(self, key: str, label: str, size: int, sub: str = "", hot: bool = False) -> None:
    r = _r(self._rects[key])
    rl.draw_rectangle_rounded(r, 0.2, 8, BTN_HOT if hot else BTN_BG)
    w = measure_text_cached(self._font_bold, label, size).x
    ty = r.y + (r.height - size) / 2 - (14 if sub else 0)
    rl.draw_text_ex(self._font_bold, label, rl.Vector2(r.x + (r.width - w) / 2, ty), size, 0, INK)
    if sub:
      sw = measure_text_cached(self._font, sub, 26).x
      rl.draw_text_ex(self._font, sub, rl.Vector2(r.x + (r.width - sw) / 2, ty + size + 2), 26, 0, DIM)

  def _render(self, rect: rl.Rectangle) -> None:
    m = self.model
    if not m.is_open:
      return
    rl.draw_rectangle_rounded(_r(self._rects["panel"]), 0.06, 10, PANEL_BG)
    p = self._rects["panel"]
    title = "Speed limit for this road  (tap a map to use it)"
    rl.draw_text_ex(self._font, title, rl.Vector2(p[0] + 30, p[1] + 22), 34, 0, DIM)
    self._btn("tesla", str(m.tesla_mph) if m.tesla_mph > 0 else "--", 64, "TESLA")
    self._btn("osm", str(m.osm_mph) if m.osm_mph > 0 else "--", 64, "OSM")
    self._btn("down", "-5", 54)
    self._btn("value", f"{m.value} mph", 64)
    self._btn("up", "+5", 54)
    self._btn("save", "Save", 40, hot=True)
    self._btn("clear", "Clear", 40)
    self._btn("source", "OSM" if m.prefer_osm else "TESLA", 36, "source")
    self._btn("close", "Close", 40)
    if m.note:
      rl.draw_text_ex(self._font, m.note, rl.Vector2(p[0] + 30, p[1] + p[3] - 40), 28, 0, rl.Color(255, 190, 120, 255))
