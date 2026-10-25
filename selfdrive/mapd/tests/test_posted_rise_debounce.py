"""Oct 25 2026: a higher posted limit must hold RISE_HOLD_S before MAX rises; drops stay immediate.

Route 000001cd 08:51:55 CDT, DeGraff Atlantic Ave: a stale SIGN 60 replaced Tesla / OSM 30 for
~4 s and MAX seeded 30 -> 59.4 at once, then eased down for ~20 s."""
from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import ACCEL_DEFAULT, LOOKAHEAD_NORMAL, MODE_CAP, MODE_FOLLOW
from openpilot.selfdrive.mapd.map_speed_policy import (
  RISE_HOLD_S, MapCruiseHold, PostedRiseDebounce, apply_map_speed_kph, decide_map_cruise,
  effective_map_limit_ms, map_slew_a_ms2, slew_map_speed_ms,
)

MPH = CV.MPH_TO_MS
DT = 0.01


def feed(db, seq, t0=0.0):
  """seq: [(mph or None, seconds)] at 100 Hz -> list of (t, debounced mph or None)."""
  out, t = [], t0
  for mph, secs in seq:
    for _ in range(int(round(secs / DT))):
      v = db.update(None if mph is None else mph * MPH, t)
      out.append((t, None if v is None else round(v / MPH)))
      t += DT
  return out


def test_first_limit_passes_at_once():
  db = PostedRiseDebounce()
  assert db.update(30 * MPH, 0.0) == 30 * MPH


def test_drop_is_immediate():
  db = PostedRiseDebounce()
  out = feed(db, [(60, 5.0), (55, 0.02)])
  assert out[-1][1] == 55          # a 5 mph drop (the #386 case) lands on the next cycle
  out = feed(db, [(30, 0.01)], t0=6.0)
  assert out[-1][1] == 30


def test_rise_waits_hold_then_lands():
  db = PostedRiseDebounce()
  out = feed(db, [(30, 2.0), (60, RISE_HOLD_S + 0.5)])
  rise = [t for t, v in out if v == 60]
  assert rise and abs(rise[0] - (2.0 + RISE_HOLD_S)) < 0.03
  assert all(v == 30 for t, v in out if t < 2.0 + RISE_HOLD_S - 0.02)


def test_short_blip_never_raises():
  db = PostedRiseDebounce()
  out = feed(db, [(30, 2.0), (60, RISE_HOLD_S - 0.2), (30, 3.0)])
  assert max(v for _, v in out) == 30


def test_flicker_restarts_hold():
  db = PostedRiseDebounce()
  out = feed(db, [(30, 1.0)] + [(60, 1.0), (30, 0.1)] * 5 + [(30, 1.0)])
  assert max(v for _, v in out) == 30


def test_rise_adopts_lowest_value_seen_during_hold():
  db = PostedRiseDebounce()
  out = feed(db, [(30, 1.0), (60, 1.0), (45, RISE_HOLD_S)])
  assert out[-1][1] == 45


def test_short_dropout_keeps_value_long_gap_resets():
  db = PostedRiseDebounce()
  feed(db, [(30, 1.0), (None, 1.0)])
  out = feed(db, [(60, 0.5)], t0=2.0)
  assert out[-1][1] == 30          # a dropout is not a free pass for a rise
  db2 = PostedRiseDebounce()
  feed(db2, [(30, 1.0), (None, 10.0)])
  assert db2.update(60 * MPH, 11.0) == 60 * MPH   # long gap: new limit as is


def _max_trace(seq, mode, debounce=True):
  """seq: [(posted mph, seconds)]; Pre-AP pedal at 60 mph. Returns MAX mph per cycle."""
  h, slew, db = MapCruiseHold(), None, PostedRiseDebounce()
  out, now, first = [], 0.0, True
  for mph, secs in seq:
    for _ in range(int(round(secs / DT))):
      lim_ms = db.update(mph * MPH, now) if debounce else mph * MPH
      dec = decide_map_cruise(h, engaged=True, mode=mode, raw_kph=60 * CV.MPH_TO_KPH, posted_kph=lim_ms * CV.MS_TO_KPH,
                              engage_rising=first, now=now, traveled_kph=33 * CV.MPH_TO_KPH, long_active=True)
      first = False
      lim = None if dec.sticky else effective_map_limit_ms(lim_ms, 0.0, 0.0, 15.0, LOOKAHEAD_NORMAL, ACCEL_DEFAULT)
      mk = None
      if lim:
        slew = lim if slew is None else slew
        slew = slew_map_speed_ms(slew, lim, DT, map_slew_a_ms2(slew, lim, LOOKAHEAD_NORMAL, ACCEL_DEFAULT))
        mk = slew * CV.MS_TO_KPH
      if dec.seed_kph is not None:
        v = dec.seed_kph
        slew = dec.seed_kph * CV.KPH_TO_MS
      else:
        v = apply_map_speed_kph(dec.driver_kph, mk, mode=mode, engaged=True, op_long_software_cruise=True,
                                driver_override=dec.follow_override)
      out.append(v / CV.MPH_TO_KPH)
      now += DT
  return out


def test_degraff_blip_max_without_vs_with_debounce():
  # 30 for 10 s, a stale 60 blip shorter than the hold, then 30 again
  seq = [(30, 10.0), (60, RISE_HOLD_S - 0.5), (30, 25.0)]
  for mode in (MODE_FOLLOW, MODE_CAP):
    before = _max_trace(seq, mode, debounce=False)
    after = _max_trace(seq, mode, debounce=True)
    assert max(before) > 55, mode          # the Oct 25 jump (59.4 logged)
    assert max(after) < 30.5, mode         # MAX never leaves 30


def test_real_rise_still_lands_and_eases_down_unchanged():
  seq = [(30, 10.0), (60, 10.0), (30, 30.0)]
  b = _max_trace(seq, MODE_FOLLOW, debounce=False)
  a = _max_trace(seq, MODE_FOLLOW, debounce=True)
  i_rise = int((10.0 + RISE_HOLD_S + 0.1) / DT)
  assert a[i_rise] > 55 and a[int((10.0 + RISE_HOLD_S - 0.1) / DT)] < 30.5
  # the drop at 20 s is not debounced: ease-down from 60 matches the old path cycle for cycle
  i_drop = int(20.0 / DT)
  assert all(abs(x - y) < 0.05 for x, y in zip(a[i_drop:], b[i_drop:]))
