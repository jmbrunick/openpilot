"""Clear the on-ring state as soon as the car is driving out of the ring (NAP, longitudinal only).

mapd calls a ring "on" while the car is within 25 m of its polyline, so
after the exit the ring cap (18 mph MAX, planner decel) stayed ~3 s on the
exit road: Oct 14 7:04:07-7:04:11 PM CT, 30th St SW (way 758182390), the
plan asked -0.6 to -1.0 m/s² at 21-27 mph after the car had straightened.

Leaving = outside the ring's own radius by EXIT_OUT_M and heading outward
(the travel direction points away from the ring centre within
EXIT_MAX_ANGLE_DEG). Entering heads inward, circulating heads along the
tangent, so neither trips it. Pure geometry, called by mapd only.
"""
from __future__ import annotations

import math

EXIT_OUT_M = 3.0
EXIT_MAX_ANGLE_DEG = 50.0
_EARTH_R = 6371000.0


def ring_center_radius(coords) -> tuple[float, float, float] | None:
  """(lat, lon, mean radius m) of a ring polyline."""
  pts = [(float(a), float(b)) for a, b in coords or ()]
  if len(pts) >= 2 and pts[0] == pts[-1]:
    pts = pts[:-1]
  if len(pts) < 3:
    return None
  lat0 = sum(p[0] for p in pts) / len(pts)
  lon0 = sum(p[1] for p in pts) / len(pts)
  k = math.cos(math.radians(lat0))
  r = sum(math.hypot(math.radians(b - lon0) * _EARTH_R * k, math.radians(a - lat0) * _EARTH_R) for a, b in pts) / len(pts)
  return lat0, lon0, r


def leaving_ring(lat: float, lon: float, bearing_deg: float | None, coords) -> bool:
  if bearing_deg is None:
    return False
  c = ring_center_radius(coords)
  if c is None:
    return False
  lat0, lon0, r = c
  east = math.radians(float(lon) - lon0) * _EARTH_R * math.cos(math.radians(lat0))
  north = math.radians(float(lat) - lat0) * _EARTH_R
  d = math.hypot(east, north)
  if d < r + EXIT_OUT_M or d <= 1e-3:
    return False
  b = math.radians(float(bearing_deg))
  cos_out = (math.sin(b) * east + math.cos(b) * north) / d
  return cos_out >= math.cos(math.radians(EXIT_MAX_ANGLE_DEG))


# ---- planner side: no braking once the car is driving out of the ring ----
# The map "on ring" flag lags the real exit (GPS + 25 m match). The steering
# wheel does not. Right-hand traffic circulates counter-clockwise, so
# circulating = steering LEFT (steeringAngleDeg > 0) by at least
# HOLD_CIRC_MIN_DEG for HOLD_CIRC_S. Entry and exit deflections are right
# turns and never count. After circulating, the wheel unwinding below
# HOLD_UNWIND_FRAC of the left peak for HOLD_S means the car is leaving.
# From then until the ring hint clears the ring may not brake: the planner
# holds at least the current speed. MAX is not raised here; the exit
# restore (right blinker + leaving, RingExitGate) and the restore once off
# the ring are unchanged.
# Oct 14 30th St SW: left 67-148 deg from 7:03:59, 32 deg at 7:04:07, 2 at 7:04:08.
HOLD_CIRC_MIN_DEG = 40.0
HOLD_CIRC_S = 1.0
HOLD_UNWIND_FRAC = 0.4
HOLD_S = 0.3


class RingExitHold:
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.holding = False
    self._left_s = 0.0
    self._run_peak = 0.0
    self._peak = 0.0
    self._unwind_s = 0.0

  def update(self, on_ring: bool, ring_active: bool, steer_left_deg: float | None, dt: float) -> bool:
    """on_ring: map says on the ring. ring_active: any ring hint. steer_left_deg: carState.steeringAngleDeg.

    True = the car is leaving the ring: do not brake for it.
    """
    if not ring_active:
      self.reset()
      return False
    if self.holding:
      return True
    dt = max(0.0, float(dt))
    try:
      a = float(steer_left_deg) if steer_left_deg is not None else 0.0
    except (TypeError, ValueError):
      a = 0.0
    if not math.isfinite(a):
      a = 0.0
    if on_ring and a >= HOLD_CIRC_MIN_DEG:
      self._left_s += dt
      self._run_peak = max(self._run_peak, a)
      if self._left_s >= HOLD_CIRC_S:
        self._peak = max(self._peak, self._run_peak)
    else:
      self._left_s, self._run_peak = 0.0, 0.0
    if not on_ring or self._peak <= 0.0:
      self._unwind_s = 0.0
      return False
    if a < HOLD_UNWIND_FRAC * self._peak:
      self._unwind_s += dt
    else:
      self._unwind_s = 0.0
    if self._unwind_s + 1e-9 >= HOLD_S:
      self.holding = True
    return self.holding
