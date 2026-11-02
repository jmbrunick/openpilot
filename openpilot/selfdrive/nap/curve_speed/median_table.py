"""Justin's median corner speed by radius, capped by the Pre-AP steering limit (NAP, longitudinal only).

Fitted Oct 14 2026 to the 52 curves Justin drove himself (Oct 4-10 logs),
binned by radius at the tightest point (<60, 60-95, 95-130, 130-160,
160-215, >=215 m). Raw bin medians: 22.4, 26.7, 29.6, 42.5, 41.5, 41.3 mph.
They are not monotone above 130 m (he holds ~42 mph on anything from about
140 to 230 m), so those bins are pooled to one plateau (a tighter bend is
never allowed to be faster than a gentler one: otherwise every gentle point
on a road caps the car below the median of tighter bins). The speeds were
then calibrated in closed-loop replay so the replayed apex speed's median
in each bin lands on Justin's (all within 1.3 mph). Between rows speed is
linear in radius; past the end rows the end row's lateral accel is held.

Safety first: no speed may need more lateral accel than LAT_CAP_MS2. That is
openpilot's lateral-control limit (drive_helpers.MAX_LATERAL_ACCEL_NO_ROLL,
3.0 m/s²) less a margin, so the car never has to turn harder than its
steering can, whatever the median says.
"""
from __future__ import annotations

import math

# (median radius m, median apex speed m/s, curves in bin)
MEDIAN_ROWS = (
  (48.0, 9.50, 5),
  (85.0, 10.23, 5),
  (107.0, 13.65, 5),
  (142.0, 18.85, 7),
  (184.0, 18.85, 26),
  (227.0, 18.85, 5),
  (260.0, 18.85, 0),
)
LAT_CAP_MS2 = 2.8
# Past the last row (gentle, fast bends Justin's 52 curves do not cover) the
# speed blends over BLEND_M into curve follow's previous lateral-accel table,
# so highway sweepers are not held to the town plateau.
BLEND_M = 140.0
FAST_LAT_BP_MS = (8.0, 13.0, 18.0, 22.0, 27.0)
FAST_LAT_MS2 = (1.45, 1.55, 1.85, 2.05, 2.20)
URGENT_DECEL_MS2 = 2.5   # firmest the curve term may brake when the bend is close (Justin: OK)
URGENT_JERK_MS3 = 2.0


def _interp(x, xp, fp):
  if x <= xp[0]:
    return float(fp[0])
  for i in range(1, len(xp)):
    if x <= xp[i]:
      return float(fp[i - 1] + (x - xp[i - 1]) / (xp[i] - xp[i - 1]) * (fp[i] - fp[i - 1]))
  return float(fp[-1])


def fast_speed_for_radius(r_m: float) -> float:
  """Lowest v with v² / r >= FAST_LAT(v) (the previous table)."""
  r = max(1.0, float(r_m))
  v = 0.5
  while v < 60.0 and v * v / r < _interp(v, FAST_LAT_BP_MS, FAST_LAT_MS2):
    v += 0.25
  return v


def median_speed_for_radius(r_m: float) -> float:
  r = max(1.0, float(r_m))
  R = [row[0] for row in MEDIAN_ROWS]
  V = [row[1] for row in MEDIAN_ROWS]
  if r <= R[0]:
    return math.sqrt(V[0] ** 2 / R[0] * r)
  if r >= R[-1]:
    f = min(1.0, (r - R[-1]) / BLEND_M)
    return V[-1] + f * max(0.0, fast_speed_for_radius(r) - V[-1])
  return _interp(r, R, V)


def steering_cap_speed(r_m: float) -> float:
  return math.sqrt(LAT_CAP_MS2 * max(1.0, float(r_m)))


def corner_speed(kappa: float) -> float | None:
  """Target speed at the tightest point for |kappa| (1/m); None on a straight."""
  k = abs(float(kappa))
  if not math.isfinite(k) or k <= 1e-6:
    return None
  r = 1.0 / k
  return min(median_speed_for_radius(r), steering_cap_speed(r))
