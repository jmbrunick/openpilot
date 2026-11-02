"""A fresh driver Yes beats an older section once past its sign (Nov 1 2026: 1e0_041 and 1ea_108 on US 12, Benson)."""
from openpilot.selfdrive.mapd.fresh_yes import YES_FRESH_S, FreshYes
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import pt

EB = 90.0
V = 20.0   # m/s, ~45 mph


def mph(ms):
  return round(ms / MPH_TO_MS)


def test_1ea_108_yes_60_beats_older_50_once_past_sign():
  # Oct 31 4:37 PM: Yes 60 (sign at x=500 eastbound); an older 50 section still covers the next way.
  y = FreshYes()
  y.start(60, now=100.0, sign_lat=pt(500)[0], sign_lon=pt(500)[1], heading=EB)
  assert y.current(101.0, *pt(450)) is None              # before the sign: section rules unchanged
  assert mph(y.current(102.0, *pt(500)) or 0) == 60      # at the sign: 60 right away
  assert mph(y.current(104.0, *pt(560)) or 0) == 60      # onto the next way, not the old 50
  assert y.current(100.0 + YES_FRESH_S + 1, *pt(900)) is None


def test_fresh_yes_without_sign_point_applies_now():
  y = FreshYes()
  y.start(50, now=0.0)
  assert mph(y.current(0.5, *pt(0)) or 0) == 50
