from openpilot.selfdrive.nap.curve_speed.median_table import (
  LAT_CAP_MS2, MEDIAN_ROWS, corner_speed, median_speed_for_radius, steering_cap_speed,
)


def test_rows_are_the_table():
  for r, v, _ in MEDIAN_ROWS:
    assert abs(median_speed_for_radius(r) - v) < 1e-9


def test_monotone_in_radius():
  prev = 0.0
  for r in range(5, 1500, 3):
    v = corner_speed(1.0 / r)
    assert v >= prev - 1e-9
    prev = v


def test_never_over_steering_cap():
  for r in range(10, 2000, 7):
    v = corner_speed(1.0 / r)
    assert v * v / r <= LAT_CAP_MS2 + 1e-9
    assert v <= steering_cap_speed(r) + 1e-9


def test_straight_is_none():
  assert corner_speed(0.0) is None
