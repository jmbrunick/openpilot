"""Live HUD hold for the camera speed-sign readout.

Display-only. Does not write sqlite or change cruise.
"""
from __future__ import annotations

from dataclasses import dataclass

# Keep the last mph on screen so 4 Hz detect does not flicker. Hide after this.
HUD_HOLD_S = 1.5
HUD_LABEL = "SIGN"
# Logger On + ONNX missing: show this instead of a blank plate or a fake mph.
HUD_MISSING_WEIGHTS_TEXT = "NO WT"


def hud_should_show(enabled: bool, valid: bool, mph: int) -> bool:
  return bool(enabled) and bool(valid) and int(mph) > 0


def hud_should_show_missing_weights(enabled: bool, weights_missing: bool) -> bool:
  return bool(enabled) and bool(weights_missing)


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
