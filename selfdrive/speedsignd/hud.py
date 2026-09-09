"""Live HUD hold for the camera speed-sign readout.

Display-only. Does not write sqlite or change cruise.
"""
from __future__ import annotations

from dataclasses import dataclass

# Keep the last mph on screen so 4 Hz detect does not flicker. Hide after this.
HUD_HOLD_S = 1.5
HUD_LABEL = "SIGN"


def hud_should_show(enabled: bool, valid: bool, mph: int) -> bool:
  return bool(enabled) and bool(valid) and int(mph) > 0


def apply_live_sign(dst, *, mph: int, conf: float, valid: bool) -> None:
  dst.mph = int(mph) if valid else 0
  dst.conf = float(conf) if valid else 0.0
  dst.valid = bool(valid)


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
