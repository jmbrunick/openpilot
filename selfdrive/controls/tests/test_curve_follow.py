"""Curve-follow: the continuous curve term in the longitudinal planner (Pre-AP).

The road ahead is a speed envelope (curve follow's own lateral table, a little
under how Justin corners)
and its most limiting point is a virtual lead. The term is a ceiling on
acceleration: exactly FREE on a straight, lower only when the path ahead
asks, released in proportion as the path opens, with no set speed and no latch.
"""
import json
import math
import time

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.curve_follow import (
  CONF_FALL_1_S,
  FREE_A_MS2,
  LOG_PREFIX,
  MODE_ACTIVE,
  MODE_OFF,
  MODE_SHADOW,
  CurveFollow,
  curve_follow_raw,
  curve_speed_for_curvature,
  decel_limit_ms2,
  format_log_line,
  read_curve_follow_mode,
)
def _bend_kappa(s, start, length, kappa, ramp=15.0):
  """Curvature at arc length s: clothoid ramps in/out around a constant arc."""
  if s < start - ramp or s > start + length + ramp:
    return 0.0
  if s < start:
    return kappa * (s - (start - ramp)) / ramp
  if s <= start + length:
    return kappa
  return kappa * (start + length + ramp - s) / ramp

from openpilot.selfdrive.modeld.constants import ModelConstants

DT = 0.05
T = np.array(ModelConstants.T_IDXS)
TL = T.tolist()


def _road_k(s, bends):
  """Signed curvature at arc length s (left +, right -). bends = (start, length, kappa)."""
  return sum(_bend_kappa(s, *b) for b in bends)


def _path(v, s, bends):
  xs = (v * T).tolist()
  return TL, xs, [_road_k(s + x, bends) for x in xs]


def _step(cf, v, s, bends, **kw):
  ts, xs, ks = _path(v, s, bends)
  return cf.step(dt=DT, v_ego=v, ts=ts, ds=xs, ks=ks, **kw)


def _drive(bends, v0, seconds, v_max=40.0, cruise_gain=0.25, a_cruise_max=1.2, cf=None, **kw):
  """Kinematic car, P cruise toward v_max, curve term as a ceiling (perfect path)."""
  cf = CurveFollow() if cf is None else cf
  s, v = 0.0, float(v0)
  rows = []
  for k in range(int(round(seconds / DT))):
    a_cf = _step(cf, v, s, bends, **kw)
    a_cruise = max(-0.5, min(a_cruise_max, cruise_gain * (v_max - v)))
    a = min(a_cruise, a_cf)
    rows.append((k * DT, s, v, a, a_cf, cf))
    v = max(0.0, v + a * DT)
    s += v * DT
  return rows


def _brake_jerk(a_list):
  """Largest |da/dt| where the ceiling is braking or just released. Above ~0 it is only a bound on +a."""
  worst = 0.0
  for a0, a1 in zip(a_list, a_list[1:], strict=False):
    if a0 <= 0.05 and a1 <= 0.05:
      worst = max(worst, abs(a1 - a0) / DT)
  return worst


def _speed_at(rows, s_target):
  return next(v for _, s, v, *_ in rows if s >= s_target)


def _time_at(rows, s_target):
  return next(t for t, s, *_ in rows if s >= s_target)


# 1 ---------------------------------------------------------------------
@pytest.mark.parametrize("v", [5.0, 8.0, 15.0, 25.0, 35.0])
def test_straight_returns_exactly_free(v):
  cf = CurveFollow()
  rng = np.random.default_rng(7)
  for _ in range(200):
    ks = (rng.normal(0.0, 0.0003, len(TL))).tolist()
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=(v * T).tolist(), ks=ks, y_std=(0.3 + 0.2 * T).tolist())
    assert a == FREE_A_MS2
  assert cf.raw == FREE_A_MS2 and cf.limit_idx == -1


# 2 ---------------------------------------------------------------------
@pytest.mark.parametrize("radius, v0", [(60.0, 15.0), (100.0, 20.0), (40.0, 12.0), (80.0, 20.0)])
def test_constant_radius_bend_slows_early_gently_and_to_the_table(radius, v0):
  start = 160.0
  bends = [(start, 80.0, 1.0 / radius)]
  rows = _drive(bends, v0, 26.0, v_max=v0)
  v_env = curve_speed_for_curvature(1.0 / radius)
  t_entry = _time_at(rows, start)
  t_on = next(t for t, _, _, a, *_ in rows if a < -0.05)
  assert t_entry - t_on >= 3.0
  assert _speed_at(rows, start) <= v_env * 1.15
  assert min(a for _, _, _, a, *_ in rows) >= -1.05
  assert _brake_jerk([r[4] for r in rows]) <= 3.1
  # Inside the arc the car holds the table speed, it is not pushed below it.
  mid = _speed_at(rows, start + 50.0)
  assert v_env * 0.85 <= mid <= v_env * 1.15


# 3 ---------------------------------------------------------------------
def test_small_deficit_is_a_gentle_lift_not_a_brake():
  v_env = curve_speed_for_curvature(1.0 / 150.0)
  v0 = v_env + 1.2
  rows = _drive([(150.0, 200.0, 1.0 / 150.0)], v0, 14.0, v_max=v0)
  a_cf = [r[4] for r in rows]
  assert min(a_cf) >= -0.45
  assert min(a_cf) < -0.02
  assert _brake_jerk(a_cf) <= 1.2


# 4 ---------------------------------------------------------------------
def test_tightening_spiral_keeps_deepening_within_the_jerk_limit():
  # Radius shrinks 200 -> 35 m over the road: each later point asks for less speed.
  bends = [(120.0 + 45.0 * i, 45.0, 1.0 / r) for i, r in enumerate((200.0, 140.0, 95.0, 65.0, 45.0, 35.0))]
  rows = _drive(bends, 17.0, 45.0, v_max=17.0)
  a = [r[3] for r in rows]
  assert min(a) < -0.3
  k_min = int(np.argmin(a))
  # No release while the road is still tightening.
  rising = [a[i + 1] - a[i] for i in range(k_min)]
  assert max(rising) <= 0.12
  assert _brake_jerk([r[4] for r in rows]) <= 3.1
  # Cornering stays on the table at the tightest arc.
  assert _speed_at(rows, 120.0 + 45.0 * 5 + 20.0) <= curve_speed_for_curvature(1.0 / 35.0) * 1.2


# 5 ---------------------------------------------------------------------
def test_release_after_the_apex_lifts_and_never_brakes_into_the_exit():
  start, length = 130.0, 70.0
  rows = _drive([(start, length, 1.0 / 40.0)], 14.0, 32.0, v_max=14.0)
  end = start + length
  after = [a_cf for _, s, _, _, a_cf, _ in rows if s >= end + 8.0]
  assert after and min(after) >= -0.1
  # By the end of the exit ramp the term does not hold the cruise back.
  last = [a_cf for _, s, _, _, a_cf, _ in rows if s >= end + 40.0]
  assert last and min(last) >= 1.0


def test_proportional_release_rate_is_bounded():
  cf = CurveFollow()
  v = 14.0
  bends = [(60.0, 50.0, 1.0 / 35.0)]
  s = 0.0
  prev = None
  worst_rise = 0.0
  for _ in range(int(14.0 / DT)):
    a = _step(cf, v, s, bends)
    if prev is not None and prev < 0.0:
      worst_rise = max(worst_rise, (a - prev) / DT)
    prev = a
    s += v * DT
  # The release blends back at the comfort jerk (~0.45 m/s^3), never a snap.
  assert worst_rise <= 0.55


# 6 ---------------------------------------------------------------------
def _sbend(gap_s, v=14.0, radius=50.0):
  """Left arc then a right arc whose apex is gap_s later at constant speed."""
  a1 = 90.0
  a2 = a1 + gap_s * v
  return [(a1, 40.0, 1.0 / radius), (a2, 40.0, -1.0 / radius)]


@pytest.mark.parametrize("gap_s", [4.0, 6.0, 8.0, 10.0, 14.0, 20.0])
def test_s_curve_family_regens_back_down_without_chatter_or_latch(gap_s):
  v0 = 11.0
  bends = _sbend(gap_s, v=v0)
  v_env = curve_speed_for_curvature(1.0 / 50.0)
  rows = _drive(bends, v0, 14.0 + gap_s + 8.0, v_max=18.0)
  arc1_end = bends[0][0] + bends[0][1]
  arc2_start = bends[1][0]
  between = [r for r in rows if arc1_end + 10.0 <= r[1] <= arc2_start - 20.0]
  a_cf = [r[4] for r in rows if arc1_end <= r[1] <= arc2_start + 10.0]
  # Speed at each tight arc stays on the table.
  assert _speed_at(rows, bends[0][0] + 20.0) <= v_env * 1.2
  assert _speed_at(rows, bends[1][0] + 20.0) <= v_env * 1.2
  # One event (or a coast between equal-radius bends): no brake-accel-brake pulse.
  pulses = _sign_flips(a_cf)
  if gap_s <= 10.0:
    assert pulses == 0
    assert not between or max(r[4] for r in between) <= 0.15
  else:
    # A far second bend may appear only after the first event ends; at most
    # one re-entry after that exit, never chatter.
    assert pulses <= 1


def test_s_curve_has_no_state_a_fresh_filter_agrees_mid_scene():
  """The demand is a function of the path and speed: a filter started mid-scene agrees after a moment."""
  v0 = 11.0
  bends = _sbend(12.0, v=v0)
  cf = CurveFollow()
  rows = _drive(bends, v0, 12.0, v_max=18.0, cf=cf)
  s_mid, v_mid = rows[-1][1], rows[-1][2]
  fresh = CurveFollow()
  worst = 0.0
  for k in range(int(3.0 / DT)):
    _step(fresh, v_mid, s_mid, bends)
    _step(cf, v_mid, s_mid, bends)
    # The curve-event latch is deliberate state (it follows a bend that slowed the car).
    if k * DT >= 0.5 and (not fresh.event) and (not cf.event):
      worst = max(worst, abs(fresh.raw - cf.raw))
    s_mid += v_mid * DT
  assert worst <= 0.2


# 7 ---------------------------------------------------------------------
def test_one_frame_spike_barely_moves_the_command():
  v = 15.0
  xs = (v * T).tolist()
  quiet = [0.0] * len(TL)
  spike = list(quiet)
  spike[25] = 0.08
  cf = CurveFollow()
  lo = FREE_A_MS2
  for i in range(40):
    ks = spike if i == 20 else quiet
    lo = min(lo, cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks))
  # Sustained, the same reading is a real bend and asks for much more.
  cf2 = CurveFollow()
  hi = FREE_A_MS2
  sustained = [0.0] * 25 + [0.08] * 8
  for _ in range(40):
    hi = min(hi, cf2.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=sustained))
  assert lo > hi
  assert FREE_A_MS2 - lo <= 0.25 * (FREE_A_MS2 - hi)


def test_strong_reading_ramps_confidence_quickly_after_a_gate():
  cf = CurveFollow()
  v = 15.0
  bend = [(60.0, 80.0, 1.0 / 30.0)]
  for _ in range(30):
    _step(cf, v, 0.0, bend, lane_change=True)
  assert cf.conf < 0.1
  for _ in range(10):  # 0.5 s
    _step(cf, v, 0.0, bend, lane_change=False)
  assert cf.conf >= 0.6


def test_high_position_std_silences_far_points_low_std_trusts_them():
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 90.0 else 1.0 / 45.0 for x in xs]
  noisy, clean = CurveFollow(), CurveFollow()
  for _ in range(40):
    a_noisy = noisy.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=[40.0] * len(TL))
    a_clean = clean.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=[0.25] * len(TL))
  assert a_noisy == FREE_A_MS2
  assert a_clean < -0.2


def test_lane_change_fades_the_term_out_without_a_step():
  v = 15.0
  bend = [(70.0, 80.0, 1.0 / 40.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  seq, confs = [], []
  for _ in range(30):
    seq.append(_step(cf, v, 0.0, bend, lane_change=True))
    confs.append(cf.conf)
  assert seq[-1] > a0 + 0.5
  assert _brake_jerk([a0] + seq) <= 2.1
  # The confidence itself fades (a ramp, not a step to zero) ...
  assert confs[0] > 0.8
  assert all(c1 <= c0 for c0, c1 in zip(confs, confs[1:], strict=False))
  # ... and is gone within the 1.5 s window.
  assert cf.conf < 0.1


def test_dropout_while_braking_releases_without_a_surge():
  v = 15.0
  bend = [(70.0, 80.0, 1.0 / 40.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  seq = [a0]
  for _ in range(30):
    seq.append(cf.step(dt=DT, v_ego=v, ts=None, ds=None, ks=None, valid=False))
  assert _brake_jerk(seq) <= 2.6
  assert cf.conf <= math.exp(-CONF_FALL_1_S * 0.5) + 0.35
  cf.reset()
  assert cf.conf == 1.0 and cf.a == FREE_A_MS2


# 8 ---------------------------------------------------------------------
@pytest.mark.parametrize("radius, v_max", [(35.0, 14.0), (70.0, 20.0)])
def test_max_stays_a_hard_ceiling_and_the_term_never_raises_the_command(radius, v_max):
  rows = _drive([(150.0, 90.0, 1.0 / radius)], 10.0, 24.0, v_max=v_max)
  for _, _, v, _a, a_cf, _ in rows:
    assert a_cf <= FREE_A_MS2
    assert v <= v_max + 0.6
  # The ceiling only ever lowers: the applied accel is never above the cruise accel.
  assert all(a <= 1.2 + 1e-9 for _, _, _, a, *_ in rows)


# 10 --------------------------------------------------------------------
def test_active_roundabout_funnel_fades_the_term_out():
  v = 12.0
  bend = [(60.0, 90.0, 1.0 / 28.0)]
  cf = CurveFollow()
  for _ in range(40):
    a0 = _step(cf, v, 0.0, bend)
  assert a0 < -0.2
  for _ in range(40):
    a_rb = _step(cf, v, 0.0, bend, rb_active=True)
  assert a_rb > a0 + 0.8 and cf.conf < 0.05


# 11 --------------------------------------------------------------------
def test_low_speed_rules():
  bend_near = [(14.0, 40.0, 1.0 / 15.0)]
  bend_far = [(40.0, 40.0, 1.0 / 15.0)]
  # Below 5 m/s: exactly FREE.
  cf = CurveFollow()
  assert all(_step(cf, 4.0, 0.0, bend_near, blinker=True) == FREE_A_MS2 for _ in range(20))
  # 6 m/s, no blinker: no anticipation at an intersection.
  cf = CurveFollow()
  assert all(_step(cf, 6.0, 0.0, bend_near) == FREE_A_MS2 for _ in range(20))
  # Still under the 6.5 m/s gate (and above the 6 m/s corner-speed floor, so
  # the turn really would ask to slow): no blinker, no anticipation.
  cf = CurveFollow()
  assert all(_step(cf, 6.4, 0.0, bend_near) == FREE_A_MS2 for _ in range(40))
  # With the blinker a near turn is anticipated ...
  cf = CurveFollow()
  v = 7.5
  out = [_step(cf, v, 0.0, [(10.0, 40.0, 1.0 / 12.0)], blinker=True) for _ in range(40)]
  assert min(out) < -0.2
  # ... but only out to ~3 s: a turn 5 s away is not.
  cf = CurveFollow()
  v = 6.5
  out = [_step(cf, v, 0.0, [(v * 7.0, 40.0, 1.0 / 12.0)], blinker=True) for _ in range(40)]
  assert min(out) == FREE_A_MS2
  assert bend_far


# 12 --------------------------------------------------------------------
def test_mode_param_reader_defaults_to_shadow_and_clamps():
  class _P:
    def __init__(self, value):
      self.value = value

    def get(self, key, return_default=False):
      assert key == "NAPCurveFollow"
      if isinstance(self.value, Exception):
        raise self.value
      return self.value

  assert read_curve_follow_mode(_P(0)) == MODE_OFF
  assert read_curve_follow_mode(_P(1)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(2)) == MODE_ACTIVE
  assert read_curve_follow_mode(_P(7)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(None)) == MODE_SHADOW
  assert read_curve_follow_mode(_P(AssertionError("unknown key"))) == MODE_SHADOW


# 13 --------------------------------------------------------------------
def test_full_distance_demand_is_milder_than_a_shortened_room():
  """The raw law uses the full remaining distance: shrinking the room (the old
  time-based finish-early buffer) makes the same bend ask for a harder bite."""
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 70.0 else 1.0 / 40.0 for x in xs]
  w = [1.0] * len(TL)
  a_full = curve_follow_raw(v, TL, xs, ks, w)[0]
  assert a_full < 0.0
  lead_s = 2.4
  xs_short = [max(2.0, x - lead_s * v) for x in xs]
  a_short = curve_follow_raw(v, TL, xs_short, ks, w)[0]
  assert a_short < a_full < 0.0


def test_raw_law_spreads_the_slowdown_over_the_full_distance():
  """a = (v_safe^2 - v^2) / (2 d) with d the full distance to the binding
  point: no time-based finish-before-apex buffer shortens it."""
  for v, start, radius in ((20.0, 150.0, 60.0), (15.0, 110.0, 40.0), (25.0, 190.0, 120.0)):
    xs = (v * T).tolist()
    ks = [0.0 if x < start else 1.0 / radius for x in xs]
    a, idx, v_env, _near = curve_follow_raw(v, TL, xs, ks, [1.0] * len(TL))
    assert idx >= 0 and v_env is not None and a < 0.0
    expected = (v_env * v_env - v * v) / (2.0 * xs[idx])
    assert a == pytest.approx(expected, rel=0.03)


def test_cycle_time_stays_small():
  cf = CurveFollow()
  v = 15.0
  xs = (v * T).tolist()
  ks = [0.0 if x < 70.0 else 1.0 / 40.0 for x in xs]
  ys = (0.3 + 0.2 * T).tolist()
  for _ in range(50):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=ys)
  t0 = time.perf_counter()
  worst = 0.0
  n = 400
  for _ in range(n):
    t1 = time.perf_counter()
    cf.step(dt=DT, v_ego=v, ts=TL, ds=xs, ks=ks, y_std=ys)
    worst = max(worst, time.perf_counter() - t1)
  mean = (time.perf_counter() - t0) / n
  assert mean < 0.002
  assert worst < 0.01


# 14 --------------------------------------------------------------------
def test_log_line_carries_the_path_summary_and_the_full_arrays():
  cf = CurveFollow()
  v = 15.0
  ts, xs, ks = _path(v, 0.0, [(70.0, 80.0, 1.0 / 40.0)])
  ys = (0.3 + 0.2 * T).tolist()
  for _ in range(30):
    cf.step(dt=DT, v_ego=v, ts=ts, ds=xs, ks=ks, y_std=ys)
  meta = {"bl": 0, "lc": "off", "df": 0.0, "cn": "green", "rb": 0, "ds": [0, 0.97]}
  line = format_log_line(mode=1, t=123.456, v=v, a_ego=0.1, v_cruise_kph=64.4, a_target=-0.2, cf=cf, old_a=-0.3,
                         ts=ts, ds=xs, ks=ks, y_std=ys, meta=meta, arrays=True)
  assert line.startswith(LOG_PREFIX)
  rec = json.loads(line[len(LOG_PREFIX):])
  assert rec["m"] == 1 and rec["mx"] == pytest.approx(64.4) and rec["old"] == pytest.approx(-0.3)
  assert rec["cf"] == pytest.approx(cf.a, abs=1e-3) and rec["cfr"] == pytest.approx(cf.raw, abs=1e-3)
  assert len(rec["k"]) == 9 and len(rec["ys"]) == 3
  assert len(rec["K"]) == len(rec["D"]) == len(rec["T"]) == len(rec["Y"]) == 33
  assert rec["li"] == cf.limit_idx and rec["w"] == pytest.approx(cf.conf, abs=1e-3)
  quiet = format_log_line(mode=1, t=1.0, v=v, a_ego=0.0, v_cruise_kph=64.4, a_target=0.0, cf=cf, old_a=2.0,
                          ts=ts, ds=xs, ks=ks, y_std=None, meta=meta, arrays=False)
  assert "\"K\"" not in quiet and "\"k\"" in quiet


# 15 --------------------------------------------------------------------
def test_zero_mean_curvature_noise_does_not_accumulate_into_a_bend():
  """Signed accumulation: noise around zero walks, it does not build a displacement."""
  cf = CurveFollow()
  rng = np.random.default_rng(7)
  lo = FREE_A_MS2
  for _ in range(200):
    ks = rng.normal(0.0, 0.0015, len(TL)).tolist()
    lo = min(lo, cf.step(dt=DT, v_ego=20.0, ts=TL, ds=(20.0 * T).tolist(), ks=ks, y_std=(0.3 + 0.2 * T).tolist()))
  # Never brakes; the median table's 42 mph plateau may trim the +a ceiling a
  # little (1.6 here), unsigned accumulation would build a bend (1.2).
  assert lo >= 1.4
  # The reliability weights themselves: zero-mean noise mostly stays inside
  # the path's own std (signed: median mean weight ~0.16); summing |k| would
  # build a displacement and trust every point (~1.0).
  v = 20.0
  mean_w = []
  for seed in range(20):
    ks = np.random.default_rng(seed).normal(0.0, 0.0015, len(TL)).tolist()
    w = CurveFollow._snr_weights(TL, (v * T).tolist(), ks, (0.3 + 0.2 * T).tolist())
    mean_w.append(float(np.mean(w)))
  assert float(np.median(mean_w)) < 0.5


def test_curvature_filter_clips_a_spike_and_opens_slowly():
  n = len(TL)
  steady = [0.0] * 10 + [0.02] * (n - 10)
  cf = CurveFollow()
  for _ in range(20):
    cf._filter_k(steady, DT)
  spike = list(steady)
  spike[25] = 0.10
  assert cf._filter_k(spike, DT)[25] < 0.03  # clipped to twice its neighbours, then filtered
  cf = CurveFollow()
  for _ in range(20):
    cf._filter_k(steady, DT)
  gone = [0.0] * n
  assert cf._filter_k(gone, DT)[25] >= 0.017  # one frame of "it opens" barely moves it
  assert cf._filter_k([0.0] * n, DT)[25] > 0.014


def test_dropped_frames_fade_the_term_out_and_light_drops_do_not():
  v = 15.0
  bend = [(60.0, 80.0, 1.0 / 30.0)]
  cf = CurveFollow()
  for _ in range(40):
    _step(cf, v, 0.0, bend, frame_drop_pct=30.0)
  assert cf.conf < 0.1 and not cf.trusted
  cf = CurveFollow()
  for _ in range(40):
    _step(cf, v, 0.0, bend, frame_drop_pct=10.0)
  assert cf.conf > 0.99 and cf.trusted


def test_near_field_term_is_a_lift_not_a_brake():
  n = len(TL)
  ts, ds = TL, (25.0 * T).tolist()
  a, idx, _, near = curve_follow_raw(25.0, ts, ds, [0.05] * n, [1.0] * n, 0.0)
  assert near is not None and near < 0.0
  assert near >= -0.35 - 1e-9


def test_apex_detection_reads_the_path_peaking():
  v = 15.0
  n = len(TL)
  ds = (v * T).tolist()
  cf = CurveFollow()
  peaking = [0.03 * max(0.0, 1.0 - float(t) / 2.0) for t in TL]  # k falls away within 2 s
  for _ in range(5):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=peaking)
  assert cf.passed > 0.9
  cf = CurveFollow()
  for _ in range(5):
    cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=[0.03] * n)
  assert cf.passed == 0.0


def test_braking_never_exceeds_the_speed_dependent_decel_limit():
  """Speed table 1.2 @10 / 0.55 @25 m/s; an urgent demand may grow it to 2.5 (Justin: firmer is OK), never more."""
  assert decel_limit_ms2(10.0) == pytest.approx(1.2)
  assert decel_limit_ms2(25.0) == pytest.approx(0.55)
  assert decel_limit_ms2(25.0, -0.4) == pytest.approx(0.55)   # not urgent
  assert decel_limit_ms2(25.0, -5.0) == pytest.approx(2.5)    # urgent, capped
  v = 30.0
  cf = CurveFollow()
  ds = (v * T).tolist()
  a_prev = FREE_A_MS2
  worst_jerk = 0.0
  saw_emergency = False
  for _ in range(80):
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=[0.0] * 3 + [0.06] * (len(TL) - 3))
    lim = 3.4 if cf.emergency else 2.5   # emergency tier: late sharp bend only
    assert cf.raw >= -lim - 1e-9 and a >= -lim - 1e-9
    saw_emergency = saw_emergency or cf.emergency
    if a < 0.0 and a_prev < 0.0:
      worst_jerk = max(worst_jerk, (a_prev - a) / DT)
    a_prev = a
  assert cf.raw == pytest.approx(-3.4)  # the road asks for far more; the emergency cap binds
  assert saw_emergency                  # a 0.06 wall at 30 m/s is the late sharp bend the tier is for
  assert a_prev == pytest.approx(-3.4)  # ... and it brakes to the emergency cap, no further
  assert worst_jerk <= 4.0 + 1e-6       # jerk limited even in emergency: not a wall


def test_emergency_tier_stays_off_for_a_normal_curve():
  """A bend seen early enough for 2.5 m/s² never uses the emergency tier."""
  v = 25.0
  cf = CurveFollow()
  ds = (v * T).tolist()
  ks = [0.0 if d < 140.0 else 1 / 60 for d in ds]
  for _ in range(40):
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=ks)
    assert not cf.emergency and a >= -2.5 - 1e-9


def test_mild_demand_at_speed_stays_within_the_lift_limit():
  """A sweeper a little under speed at 25 m/s: a lift, never past 0.8 (the urgent ramp starts at 0.5)."""
  v = 25.0
  v_env = curve_speed_for_curvature(1.0 / 250.0)
  assert v_env is not None and v_env < v
  rows = _drive([(250.0, 300.0, 1.0 / 250.0)], v, 20.0, v_max=v)
  assert min(r[4] for r in rows) >= -0.8 - 1e-6


# ---------------------------------------------------------------------------
# Gradual glide law: no pulsing, comfort jerk, earlier onset, exit only opening
# ---------------------------------------------------------------------------
def _sign_flips(a_list, band=0.15):
  """Pulses: returning to a brake after the ceiling has already released to +a.

  Entering a curve (free -> brake) and leaving it once (brake -> free) is
  not a pulse. brake-accel-brake is.
  """
  state, seen_brake, pulses = 0, False, 0
  for a in a_list:
    new = 1 if a > band else (-1 if a < -band else state)
    if new == -1 and state == 1 and seen_brake:
      pulses += 1
    if new == -1:
      seen_brake = True
    if new:
      state = new
  return pulses


def test_curve_event_does_not_pulse_the_ceiling():
  """Once a curve event starts, the ceiling never flips to +a and back to braking."""
  rows = _drive([(150.0, 80.0, 1.0 / 50.0)], 16.0, 28.0, v_max=16.0)
  a_cf = [r[4] for r in rows]
  assert _sign_flips(a_cf) == 0
  # After the first ease, the command may deepen or relax toward zero, not chatter.
  started = next(i for i, a in enumerate(a_cf) if a < -0.05)
  event = a_cf[started:]
  # Until the ceiling is clearly free again, it stays <= a small +a deadband.
  free_i = next((i for i, a in enumerate(event) if a > 0.15), len(event))
  assert all(a <= 0.15 for a in event[:free_i])
  assert _brake_jerk(a_cf) <= 0.55  # comfort jerk on a bend seen early


def test_s_curve_holds_speed_between_close_bends():
  """Equal-radius S with a 6 s gap: one event, coast/hold between, no re-accel."""
  v0 = 11.0
  bends = _sbend(6.0, v=v0)
  rows = _drive(bends, v0, 14.0 + 6.0 + 8.0, v_max=18.0)
  arc1_end = bends[0][0] + bends[0][1]
  arc2_start = bends[1][0]
  between = [r[4] for r in rows if arc1_end + 8.0 <= r[1] <= arc2_start - 10.0]
  a_cf = [r[4] for r in rows]
  assert _sign_flips(a_cf) == 0
  assert between and max(between) <= 0.15
  v_env = curve_speed_for_curvature(1.0 / 50.0)
  assert _speed_at(rows, bends[0][0] + 20.0) <= v_env * 1.2
  assert _speed_at(rows, bends[1][0] + 20.0) <= v_env * 1.2


def test_comfort_jerk_is_bounded_and_late_emergency_may_use_the_firm_tier():
  # Early, gentle sweeper: comfort jerk only.
  rows = _drive([(250.0, 300.0, 1.0 / 250.0)], 25.0, 20.0, v_max=25.0)
  assert _brake_jerk([r[4] for r in rows]) <= 0.55
  # Late sharp wall: emergency cap, jerk still bounded at the emergency limit.
  cf = CurveFollow()
  v = 30.0
  ds = (v * T).tolist()
  a_prev = FREE_A_MS2
  worst = 0.0
  for _ in range(80):
    a = cf.step(dt=DT, v_ego=v, ts=TL, ds=ds, ks=[0.0] * 3 + [0.06] * (len(TL) - 3))
    if a < 0.0 and a_prev < 0.0:
      worst = max(worst, (a_prev - a) / DT)
    a_prev = a
  assert cf.emergency and a_prev == pytest.approx(-3.4)
  assert worst <= 4.0 + 1e-6


def test_onset_starts_as_soon_as_the_bend_is_visible():
  """A 50 m radius bend 160 m out at 15 m/s must start easing several seconds before entry."""
  start = 160.0
  rows = _drive([(start, 80.0, 1.0 / 50.0)], 15.0, 26.0, v_max=15.0)
  t_entry = _time_at(rows, start)
  t_on = next(t for t, _, _, a, *_ in rows if a < -0.05)
  assert t_entry - t_on >= 5.0
  v_env = curve_speed_for_curvature(1.0 / 50.0)
  mid = _speed_at(rows, start + 40.0)
  assert v_env * 0.80 <= mid <= v_env * 1.15  # ~2 mph band at these speeds is ~10%


def test_exit_reaccel_only_while_the_path_is_opening():
  start, length = 130.0, 70.0
  rows = _drive([(start, length, 1.0 / 40.0)], 14.0, 32.0, v_max=14.0)
  # Once the event has started, no +a until the path is clearly opening.
  started = next(i for i, r in enumerate(rows) if r[4] < -0.05)
  mid_event = []
  for r in rows[started:]:
    if r[4] > 0.15:
      break
    mid_event.append(r)
  assert mid_event
  in_entry = [r[4] for r in mid_event if r[1] <= start + 15.0]
  assert in_entry and all(a <= 0.15 for a in in_entry)
  # After the exit, the ceiling eventually lets cruise back in, without a new bite.
  after = [r[4] for r in rows if r[1] >= start + length + 20.0]
  assert after and min(after) >= -0.1
  assert _sign_flips([r[4] for r in rows]) == 0


# Nov 18 2026 drive (ef0c30ca): a 55 mph highway sweeper (|k| peak ~0.0043,
# |k| under the car never above PEAK_MIN_K) latched an event that never
# released; the ceiling sat at 0 for 25 min and cruise held 50-54 under MAX 60.
@pytest.mark.parametrize("kappa", [0.0030, 0.0038, 0.0043])
def test_gentle_sweeper_event_releases_on_the_straight_after(kappa):
  start, length = 300.0, 250.0
  rows = _drive([(start, length, kappa)], 25.0, 70.0, v_max=26.8)
  assert any(cf_a < -0.05 for *_, cf_a, _ in rows), "sweeper should have slowed the car"
  end = start + length + 15.0
  straight = [(s, v, a_cf) for _, s, v, _, a_cf, _ in rows if s >= end + 150.0]
  assert straight, "drive long enough to reach the straight"
  assert min(a for *_, a in straight) >= 1.0, "ceiling must be free again on the straight"
  assert not rows[-1][5].event
  assert rows[-1][2] > 26.0, "cruise climbs back to MAX"


def _latched_sweeper(kappa=0.0043, v=25.0):
  """Hold v into a gentle sweeper until the event latches; return (cf, s)."""
  cf = CurveFollow()
  bends = [(300.0, 250.0, kappa)]
  s = 0.0
  for _ in range(int(20.0 / DT)):
    _step(cf, v, s, bends)
    s += v * DT
    if cf.event and cf.a < -0.2:
      return cf, s, bends
  raise AssertionError("event never latched")


@pytest.mark.parametrize("why", ["disengage", "gas_override"])
def test_not_in_control_ends_a_latched_event(why):
  # Disengage and the driver's gas override both reach the term as
  # long_active=False (the planner's reset); no event or brake survives it.
  cf, s, bends = _latched_sweeper()
  v = 25.0
  for _ in range(int(1.0 / DT)):
    a = _step(cf, v, s, bends, long_active=False)
    s += v * DT
    assert a == FREE_A_MS2
    assert not cf.event and cf.raw == FREE_A_MS2 and not cf.emergency
  # Filters keep tracking the path while inactive.
  assert cf._kf is not None and cf.conf > 0.9


def test_reengage_mid_approach_starts_fresh_and_still_brakes_to_the_table():
  kappa = 1.0 / 120.0
  bends = [(250.0, 120.0, kappa)]
  v_table = curve_speed_for_curvature(kappa)
  cf = CurveFollow()
  s, v = 0.0, 27.0
  rows = []
  for k in range(int(30.0 / DT)):
    t = k * DT
    active = not (3.0 <= t < 5.0)  # a 2 s gas override on the approach
    a_cf = _step(cf, v, s, bends, long_active=active)
    a = min(max(-0.5, min(1.2, 0.25 * (27.0 - v))), a_cf) if active else 0.3
    rows.append((t, s, v, a_cf, active, cf.event))
    v = max(0.0, v + a * DT)
    s += v * DT
  during = [r for r in rows if not r[4]]
  assert all(r[3] == FREE_A_MS2 and not r[5] for r in during)
  after = [r for r in rows if r[0] >= 5.0]
  # Starts from free (no step into a stale brake), then brakes for the bend.
  assert after[0][3] >= -0.05
  assert min(r[3] for r in after) < -0.3
  assert _brake_jerk([r[3] for r in after]) <= 3.0
  # Inside the arc it holds the table, same as a car never overridden.
  mid = _speed_at(rows, 250.0 + 60.0)
  assert v_table * 0.85 <= mid <= v_table * 1.15
  ref = _drive(bends, 27.0, 30.0, v_max=27.0)
  assert abs(mid - _speed_at(ref, 250.0 + 60.0)) <= 0.5


def _kappa_for_speed(v_target):
  lo, hi = 1e-4, 0.2
  for _ in range(60):
    mid = 0.5 * (lo + hi)
    v = curve_speed_for_curvature(mid)
    if v is None or v > v_target:
      lo = mid
    else:
      hi = mid
  return 0.5 * (lo + hi)


def test_near_envelope_sweeper_stays_one_event_then_releases_on_the_straight():
  # Oct 10 2026 17:42 CDT shape: a tighter entry, then a long sweeper whose
  # corner speed sits ~1 m/s above the car. Clearing only the hold margin
  # must not release (that pulses brake / +a / brake on a noisy path); the
  # straight after it must.
  k1 = _kappa_for_speed(22.0)
  k2 = _kappa_for_speed(23.0)
  bends = [(200.0, 60.0, k1), (275.0, 500.0, k2)]
  rows = _drive(bends, 26.0, 60.0, v_max=27.0)
  assert min(r[4] for r in rows) < -0.3, "the tighter entry should brake"
  sweep = [r[4] for r in rows if 320.0 <= r[1] <= 740.0]
  assert sweep and max(sweep) <= 0.05, "no +a mid-sweeper"
  end = 775.0 + 15.0
  after = [r[4] for r in rows if r[1] >= end + 150.0]
  assert after and min(after) >= 1.0
