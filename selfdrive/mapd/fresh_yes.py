"""A fresh driver Yes beats an older confirmed section once the car is past its sign (Nov 1 2026, 1e0_041 / 1ea_108 US 12).

nap-dev port of FreshYes from nap-alpha's short_segment.py (nap-dev has no ShortSegmentHold)."""
from __future__ import annotations

import math

from openpilot.selfdrive.mapd import speed_overrides as so

YES_FRESH_S = 30.0             # a driver Yes stays fresh this long
YES_PASSED_M = -5.0            # car at / past the Yes sign point along the heading (GNSS slack)


class FreshYes:
  """The last driver Yes (Nov 1 2026, 1e0_041 / 1ea_108 on US 12).

  For YES_FRESH_S after a Yes, once the car is at or past the sign point the Yes reported, the
  Yes value beats an older confirmed section still covering the road there (1ea: an older 50
  covered the start of way 1557241351 for ~3 s after a Yes 60), and ShortSegmentHold never
  holds it back (1e0: the confirmed 60 was held as a "short" jump for ~95 m). Before the sign
  point the section rules are unchanged (the limit still starts at the sign)."""

  def __init__(self) -> None:
    self.clear()

  def clear(self) -> None:
    self.ms: float | None = None
    self.t = 0.0
    self.sign_ll: tuple[float, float] | None = None
    self.heading: float | None = None

  def start(self, mph: int, now: float, sign_lat=None, sign_lon=None, heading=None) -> None:
    self.ms = int(mph) * so.MPH_TO_MS
    self.t = float(now)
    self.sign_ll = (float(sign_lat), float(sign_lon)) if sign_lat is not None and sign_lon is not None else None
    self.heading = float(heading) if heading is not None else None

  def fresh(self, now: float) -> bool:
    if self.ms is None:
      return False
    if now - self.t > YES_FRESH_S:
      self.clear()
      return False
    return True

  def passed_sign(self, lat, lon) -> bool:
    """Car at / past the sign point (unknown sign point or heading: treated as passed)."""
    if self.sign_ll is None or self.heading is None or lat is None or lon is None:
      return True
    x, y = so._xy(float(lat), float(lon), self.sign_ll[0], self.sign_ll[1])
    h = math.radians(self.heading)
    return x * math.sin(h) + y * math.cos(h) >= YES_PASSED_M

  def current(self, now: float, lat, lon) -> float | None:
    """Yes limit (m/s) that should be published now, else None."""
    if not self.fresh(now) or not self.passed_sign(lat, lon):
      return None
    return self.ms
