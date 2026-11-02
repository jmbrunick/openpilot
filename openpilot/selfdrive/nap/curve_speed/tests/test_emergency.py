import math

from openpilot.selfdrive.nap.curve_speed.emergency import (
  EMERGENCY_DECEL_MS2, EMERGENCY_JERK_MS3, NORMAL_DECEL_MS2, STEER_LAT_MS2, apex_speed_at_normal_cap, needs_emergency,
)


def test_limits_inside_stack_and_panda():
  assert NORMAL_DECEL_MS2 < EMERGENCY_DECEL_MS2 <= 3.45  # opendbc -3.5, panda -3.48
  assert EMERGENCY_JERK_MS3 <= 5.0
  assert 2.8 <= STEER_LAT_MS2 <= 3.0


def test_apex_speed():
  assert math.isclose(apex_speed_at_normal_cap(20.0, 40.0), math.sqrt(400 - 200))
  assert apex_speed_at_normal_cap(10.0, 100.0) == 0.0


def test_only_late_sharp_bends():
  # plenty of room: normal cap reaches a safe speed
  assert not needs_emergency(25.0, 150.0, 1 / 50)
  # gentle bend, little room: still under steering limit
  assert not needs_emergency(25.0, 10.0, 1 / 400)
  # sharp bend, too close for 2.5 m/s²
  assert needs_emergency(25.0, 40.0, 1 / 50)
  # sign of curvature does not matter; missing data never triggers
  assert needs_emergency(25.0, 40.0, -1 / 50)
  assert not needs_emergency(25.0, None, 1 / 50) and not needs_emergency(25.0, 40.0, None)
