"""Live HUD hold for the camera speed-sign readout.

Display-only. Does not write sqlite or change cruise.
"""
from __future__ import annotations

from dataclasses import dataclass

# Hold long enough that a skipped 1 Hz cycle does not flicker the plate.
HUD_HOLD_S = 3.0
HUD_LABEL = "SIGN"
# Logger On + ONNX missing: show this instead of a blank plate or a fake mph.
HUD_MISSING_WEIGHTS_TEXT = "NO WT"
HUD_CONFIRM_PROMPT = "accurate?"
HUD_CONFIRM_YES = "Yes"
HUD_CONFIRM_NO = "No"

# TICI 2160x1080 — driver-left, below the MAX box (hud_renderer UI_CONFIG).
TICI_SIGN_W = 200
TICI_SIGN_H = 248
TICI_SIGN_LEFT = 60
TICI_MAX_TOP = 45
TICI_MAX_H = 204
TICI_BELOW_MAX_GAP = 16
TICI_CONFIRM_BTN_H = 112
TICI_CONFIRM_GAP = 10
TICI_CONFIRM_PROMPT_H = 32

# Mici 536x240 — same left / driver-side idea.
MICI_SIGN_W = 108
MICI_SIGN_H = 128
MICI_SIGN_LEFT = 8
MICI_SIGN_TOP = 6
MICI_CONFIRM_BTN_W = 68
MICI_CONFIRM_GAP = 6


def plate_digit_size(text: str, default: int) -> int:
  """Shrink 'NO WT' so it fits the mph plate; keep 2–3 digit mph large."""
  if len(text) > 3:
    return max(22, int(default * 0.42))
  return default


def hud_should_show(enabled: bool, valid: bool, mph: int) -> bool:
  return bool(enabled) and bool(valid) and int(mph) > 0


def hud_should_show_missing_weights(enabled: bool, weights_missing: bool) -> bool:
  return bool(enabled) and bool(weights_missing)


def hud_confirm_visible(view: LiveSignView | None = None, *, show_mph: bool = False, mph: int = 0,
                        show_missing_weights: bool = False) -> bool:
  """Yes/No accuracy overlay: only when a real mph is on the plate, never for NO WT."""
  if view is not None:
    show_mph = view.show_mph
    mph = view.mph
    show_missing_weights = view.show_missing_weights
  return bool(show_mph) and int(mph) > 0 and not bool(show_missing_weights)


def tici_sign_origin(rect_x: float = 0.0, rect_y: float = 0.0) -> tuple[float, float]:
  """Left / driver side, below MAX so the plate does not cover MAX or the center path."""
  return rect_x + TICI_SIGN_LEFT, rect_y + TICI_MAX_TOP + TICI_MAX_H + TICI_BELOW_MAX_GAP


def mici_sign_origin(rect_x: float = 0.0, rect_y: float = 0.0) -> tuple[float, float]:
  return rect_x + MICI_SIGN_LEFT, rect_y + MICI_SIGN_TOP


def accuracy_button_rects(
  sign_x: float,
  sign_y: float,
  sign_w: float,
  sign_h: float,
  *,
  beside: bool = False,
  btn_w: float | None = None,
  btn_h: float = TICI_CONFIRM_BTN_H,
  gap: float = TICI_CONFIRM_GAP,
  prompt_h: float = 0.0,
) -> tuple[tuple[float, float, float, float], tuple[float, float, float, float]]:
  """Return (yes, no) as (x, y, w, h). TICI stacks under the plate; mici sits beside."""
  if beside:
    w = MICI_CONFIRM_BTN_W if btn_w is None else btn_w
    h = (sign_h - gap) / 2.0
    yes = (sign_x + sign_w + gap, sign_y, w, h)
    no = (sign_x + sign_w + gap, sign_y + h + gap, w, h)
    return yes, no
  y0 = sign_y + sign_h + gap + prompt_h
  yes = (sign_x, y0, sign_w, btn_h)
  no = (sign_x, y0 + btn_h + gap, sign_w, btn_h)
  return yes, no


def point_in_rect(px: float, py: float, r: tuple[float, float, float, float]) -> bool:
  x, y, w, h = r
  return x <= px <= x + w and y <= py <= y + h


def hit_accuracy_button(
  px: float,
  py: float,
  yes: tuple[float, float, float, float],
  no: tuple[float, float, float, float],
) -> bool | None:
  """True = Yes, False = No, None = miss."""
  if point_in_rect(px, py, yes):
    return True
  if point_in_rect(px, py, no):
    return False
  return None


@dataclass(frozen=True)
class LiveSignView:
  show_mph: bool = False
  mph: int = 0
  show_missing_weights: bool = False

  @property
  def show(self) -> bool:
    return self.show_mph or self.show_missing_weights

  @property
  def plate_text(self) -> str:
    if self.show_missing_weights:
      return HUD_MISSING_WEIGHTS_TEXT
    if self.show_mph:
      return str(int(self.mph))
    return ""

  @property
  def confirm_visible(self) -> bool:
    return hud_confirm_visible(self)


def live_sign_from_event(enabled: bool, d, msg_valid: bool) -> LiveSignView:
  """Parse a liveSpeedSignNAP-like object for the on-road plate."""
  weights_missing = bool(getattr(d, "weightsMissing", False))
  return live_sign_view(
    enabled=bool(enabled),
    msg_valid=bool(msg_valid) or weights_missing,
    mph=int(getattr(d, "mph", 0) or 0),
    valid=bool(getattr(d, "valid", False)),
    weights_missing=weights_missing,
  )


def live_sign_view(
  *,
  enabled: bool,
  msg_valid: bool,
  mph: int,
  valid: bool,
  weights_missing: bool = False,
) -> LiveSignView:
  """What the on-road SIGN plate should show. No false mph when weights are missing."""
  if not enabled:
    return LiveSignView()
  if hud_should_show_missing_weights(True, weights_missing):
    return LiveSignView(show_missing_weights=True)
  if not msg_valid:
    return LiveSignView()
  if hud_should_show(True, valid, mph):
    return LiveSignView(show_mph=True, mph=int(mph))
  return LiveSignView()


def apply_live_sign(dst, *, mph: int, conf: float, valid: bool, weights_missing: bool = False) -> None:
  dst.mph = int(mph) if valid else 0
  dst.conf = float(conf) if valid else 0.0
  dst.valid = bool(valid)
  dst.weightsMissing = bool(weights_missing)


@dataclass
class LiveSignHold:
  hold_s: float = HUD_HOLD_S
  mph: int = 0
  conf: float = 0.0
  until: float = 0.0

  def update(self, signs, now: float) -> tuple[bool, int, float]:
    if signs:
      best = max(signs, key=lambda s: s.conf)
      self.mph = int(best.mph)
      self.conf = float(best.conf)
      self.until = now + self.hold_s
    live = now < self.until and self.mph > 0
    if not live:
      return False, 0, 0.0
    return True, self.mph, self.conf
