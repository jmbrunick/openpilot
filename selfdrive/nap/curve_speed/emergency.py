"""Emergency braking tier for a late, sharp bend (NAP, longitudinal only).

Normal curve braking is capped at the urgent 2.5 m/s². This tier exists only
for the bend that shows up too late for that: when braking at the normal cap
from here would still reach the tightest point fast enough to need more
lateral accel than the steering can give (STEER_LAT_MS2, openpilot's 3.0
m/s² lateral limit), curve follow may brake up to EMERGENCY_DECEL_MS2.

3.4 m/s² sits inside the stack floor (opendbc ACCEL_MIN = -3.5) and the
panda Tesla longitudinal limit (min_accel raw 288 = -3.48 m/s²). Its build
jerk is limited (EMERGENCY_JERK_MS3): firm, never a wall.
"""
from __future__ import annotations

import math

NORMAL_DECEL_MS2 = 2.5
EMERGENCY_DECEL_MS2 = 3.4
EMERGENCY_JERK_MS3 = 4.0
STEER_LAT_MS2 = 3.0


def apex_speed_at_normal_cap(v_ego: float, room_m: float) -> float:
  return math.sqrt(max(0.0, float(v_ego) ** 2 - 2.0 * NORMAL_DECEL_MS2 * max(0.0, float(room_m))))


def needs_emergency(v_ego: float, room_m: float | None, kappa: float | None) -> bool:
  """True when the normal cap cannot keep the bend's apex under the steering limit."""
  if room_m is None or kappa is None:
    return False
  v_apex = apex_speed_at_normal_cap(v_ego, room_m)
  return v_apex * v_apex * abs(float(kappa)) > STEER_LAT_MS2
