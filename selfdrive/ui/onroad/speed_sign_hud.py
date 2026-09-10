"""On-road camera speed-sign HUD (display-only).

Large opaque MUTCD-style plate. TICI: top-right, left of the experimental
button (opposite MAX / OSM LIMIT). Mici: top-right.

When the logger is On and ONNX weights failed to load, the plate shows
NO WT instead of staying blank (and never a fake mph).
"""
from __future__ import annotations

import pyray as rl
from openpilot.selfdrive.speedsignd.hud import HUD_LABEL, LiveSignView, live_sign_from_event, plate_digit_size
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.text_measure import measure_text_cached

# TICI 2160x1080 — bigger than OSM LIMIT (64pt), smaller than live speed (176pt).
TICI_W, TICI_H = 200, 248
TICI_DIGIT = 120
TICI_LABEL = 36
# Mici 536x240 — still a solid plate, not a toast.
MICI_W, MICI_H = 108, 128
MICI_DIGIT = 72
MICI_LABEL = 18

_SIGN_BG = rl.WHITE
_SIGN_BORDER = rl.Color(180, 40, 40, 255)
_SIGN_INK = rl.BLACK
_SIGN_LABEL = rl.Color(40, 40, 40, 255)
_SIGN_MISSING_INK = rl.Color(160, 40, 40, 255)


def live_sign_from_sm(sm, enabled: bool) -> LiveSignView:
  if not enabled:
    return LiveSignView()
  try:
    d = sm["liveSpeedSignNAP"]
  except Exception:
    return LiveSignView()
  msg_valid = False
  try:
    msg_valid = bool(sm.valid["liveSpeedSignNAP"])
  except Exception:
    msg_valid = False
  return live_sign_from_event(True, d, msg_valid)


def logger_enabled() -> bool:
  return bool(getattr(ui_state, "speed_sign_log", False))


def tici_sign_rect(rect: rl.Rectangle) -> rl.Rectangle:
  """Left of the experimental button, same row as MAX."""
  button_x = rect.x + rect.width - 30 - 192
  x = button_x - 18 - TICI_W
  y = rect.y + 45
  return rl.Rectangle(x, y, TICI_W, TICI_H)


def mici_sign_rect(rect: rl.Rectangle) -> rl.Rectangle:
  x = rect.x + rect.width - 8 - MICI_W
  y = rect.y + 6
  return rl.Rectangle(x, y, MICI_W, MICI_H)


def draw_speed_sign_plate(
  sign: rl.Rectangle,
  text: str,
  font_bold,
  font_label,
  digit_size: int,
  label_size: int,
  *,
  ink=None,
) -> None:
  rl.draw_rectangle_rounded(sign, 0.10, 8, _SIGN_BG)
  rl.draw_rectangle_rounded_lines_ex(sign, 0.10, 8, 6, _SIGN_BORDER)
  label_w = measure_text_cached(font_label, HUD_LABEL, label_size).x
  rl.draw_text_ex(
    font_label, HUD_LABEL,
    rl.Vector2(sign.x + (sign.width - label_w) / 2, sign.y + max(6, sign.height * 0.06)),
    label_size, 0, _SIGN_LABEL,
  )
  num = str(text)
  size = plate_digit_size(num, digit_size)
  num_w = measure_text_cached(font_bold, num, size).x
  num_h = measure_text_cached(font_bold, num, size).y
  color = _SIGN_INK if ink is None else ink
  rl.draw_text_ex(
    font_bold, num,
    rl.Vector2(sign.x + (sign.width - num_w) / 2, sign.y + sign.height * 0.38 - num_h / 4),
    size, 0, color,
  )


def _draw_live_sign(rect_fn, rect: rl.Rectangle, font_bold, font_label, digit_size: int, label_size: int) -> None:
  view = live_sign_from_sm(ui_state.sm, logger_enabled())
  if not view.show:
    return
  ink = _SIGN_MISSING_INK if view.show_missing_weights else _SIGN_INK
  draw_speed_sign_plate(
    rect_fn(rect), view.plate_text, font_bold, font_label, digit_size, label_size, ink=ink,
  )


def draw_tici_speed_sign(rect: rl.Rectangle, font_bold, font_label) -> None:
  _draw_live_sign(tici_sign_rect, rect, font_bold, font_label, TICI_DIGIT, TICI_LABEL)


def draw_mici_speed_sign(rect: rl.Rectangle, font_bold, font_label) -> None:
  _draw_live_sign(mici_sign_rect, rect, font_bold, font_label, MICI_DIGIT, MICI_LABEL)
