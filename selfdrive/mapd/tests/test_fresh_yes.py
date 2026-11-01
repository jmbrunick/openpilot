"""A fresh driver Yes is never held back (Nov 1 2026: 1e0_041 and 1ea_108 on US 12, Benson)."""
from openpilot.selfdrive.mapd.short_segment import YES_FRESH_S, FreshYes, ShortSegmentHold
from openpilot.selfdrive.mapd.speed_overrides import MPH_TO_MS
from openpilot.selfdrive.mapd.tests.test_speed_override_sections import pt

EB = 90.0
V = 20.0   # m/s, ~45 mph


def mph(ms):
  return round(ms / MPH_TO_MS)


def short_60_profile(x, start=500.0, end=560.0):
  """1e0 shape: confirmed 60 from the sign, the profile ahead shows it for under 100 yd (an older 30
  section still starting past it), so the plain filter writes it out."""
  if x < start:
    return [(0.0, 30 * MPH_TO_MS), (start - x, 60 * MPH_TO_MS), (end - x, 30 * MPH_TO_MS)]
  if x < end:
    return [(0.0, 60 * MPH_TO_MS), (end - x, 30 * MPH_TO_MS)]
  return [(0.0, 30 * MPH_TO_MS)]


def run_1e0(yes_at_t):
  """Drive east at V from x=300; Yes 60 at time yes_at_t; raw 60 from x=500. Returns [(t, x, pub_mph)]."""
  h, out = ShortSegmentHold(), []
  t, x = 0.0, 300.0
  while x <= 540.0:
    if yes_at_t is not None and abs(t - yes_at_t) < 0.025:
      h.note_yes(60, t)
    prof = short_60_profile(x)
    out.append((t, x, mph(h.update(prof[0][1], "sign", x, prof, 400.0, now=t))))
    t, x = t + 0.05, x + V * 0.05
  return out


def test_1e0_without_yes_the_short_60_is_written_out():
  assert all(p == 30 for _t, _x, p in run_1e0(None))   # the Oct 26 rule still works without a Yes


def test_1e0_041_confirmed_60_within_1s_of_sign():
  tr = run_1e0(yes_at_t=8.0)                             # Yes 2 s before the sign (x=460)
  t_sign = min(t for t, x, _p in tr if x >= 500.0)
  t60 = min(t for t, _x, p in tr if p == 60)
  assert t60 - t_sign <= 1.0


def test_yes_goes_stale_after_30s():
  h = ShortSegmentHold()
  h.note_yes(60, 0.0)
  h.update(30 * MPH_TO_MS, "sign", 0.0, [(0.0, 30 * MPH_TO_MS)], 400.0, now=1.0)
  prof = [(0.0, 60 * MPH_TO_MS), (40.0, 30 * MPH_TO_MS)]
  assert mph(h.update(60 * MPH_TO_MS, "sign", 10.0, prof, 400.0, now=YES_FRESH_S + 5.0)) == 30


def test_new_yes_resets_a_running_hold():
  h = ShortSegmentHold()
  h.update(30 * MPH_TO_MS, "sign", 0.0, [(0.0, 30 * MPH_TO_MS)], 400.0, now=0.0)
  prof = [(0.0, 60 * MPH_TO_MS), (40.0, 30 * MPH_TO_MS)]
  assert mph(h.update(60 * MPH_TO_MS, "sign", 5.0, prof, 400.0, now=0.1)) == 30 and h.held
  h.note_yes(60, 0.2)
  assert mph(h.update(60 * MPH_TO_MS, "sign", 6.0, prof, 400.0, now=0.25)) == 60 and not h.held


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
