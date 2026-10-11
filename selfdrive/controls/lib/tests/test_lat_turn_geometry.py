"""Turn geometry correction (NAPLatTurnGeom / NAPLatRefOffset).

Pins: default ON, OFF == stock lookahead (1119c0f) bit-for-bit, the low-speed
delay table, rear reference offset, speed floor/fade, 0.2 s cap and 0.2 s
floor, 250 deg/s rate-limit hold-off, lateralManeuverPlan untouched, and the
#223 roundabout outer bias stripped while the correction is active.

Turn-in delay trim (NAPTurnInDelay, steps -2..+3, 30 ms each, low-speed band
only): default 0 is bit-identical, >= 10 mph untouched, bounded, slewed.

Low-speed reach (Sep 28 PM): 1.5-2.5 m/s fade-in, 0.25 s cap and direct plan
sampling down to 0.2 s below 8 mph, reduction frozen (not slewed to stock)
while rate-limited below 10 mph, and >= 10 mph identical to 7c49a8e.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import lat_turn_geometry as tg
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_STABLE_DELAY, curv_from_psis, get_curvature_from_plan
from openpilot.selfdrive.mapd.roundabout import (
  RoundaboutHint, roundabout_lateral_curvature_bias, roundabout_outer_curvature_bias,
  roundabout_outer_path_offset_m,
)

ROOT = Path(__file__).resolve().parents[4]
DT_MDL = 0.05
STOCK = 0.40 + 0.05 + 0.025   # lagd 0.40 + frame + action delay (LAT_SMOOTH 0)


class _CP:
  def __init__(self, brand="tesla", fp="TESLA_MODEL_S_PREAP"):
    self.brand = brand
    self.carFingerprint = fp


# ---------- params / toggle ----------

def test_params_default_on_and_offset_035():
  keys = (ROOT / "common/params_keys.h").read_text()
  assert '{"NAPLatTurnGeom", {PERSISTENT, BOOL, "1"}}' in keys
  assert '{"NAPLatRefOffset", {PERSISTENT, FLOAT, "0.35"}}' in keys
  assert tg.REF_OFFSET_DEFAULT_M == 0.35


def test_active_only_preap_and_toggle_on():
  assert tg.turn_geometry_active(tg.is_preap_car(_CP()), True) is True
  assert tg.turn_geometry_active(tg.is_preap_car(_CP()), False) is False
  assert tg.turn_geometry_active(tg.is_preap_car(_CP(fp="TESLA_MODEL_S_RAVEN")), True) is False
  assert tg.turn_geometry_active(tg.is_preap_car(_CP(brand="toyota")), True) is False


def test_ref_offset_clamped_to_menu_range():
  assert tg.clamp_ref_offset(-1) == 0.0
  assert tg.clamp_ref_offset(1.9) == 1.0
  assert tg.clamp_ref_offset(None) == 0.35
  assert tg.clamp_ref_offset(float("nan")) == 0.35
  assert tg.clamp_ref_offset(0.5) == 0.5


# ---------- formula ----------

def test_off_is_stock_bit_for_bit():
  c = tg.TurnGeometryCorrection(DT_MDL)
  for v in np.linspace(0.0, 40.0, 81):
    for stock in (0.12, 0.2, 0.3, STOCK, 0.6):
      # even with a pinned rate limiter the OFF path is the plain stock value
      out = c.update(enabled=False, stock_lookahead_s=stock, v_ego=float(v), ref_offset_m=1.0,
                     lat_active=True, cmd_angle_deg=90.0, out_angle_deg=float(v))
      assert out is stock
  assert c.reduction_s == 0.0


def test_on_default_reduces_lookahead_at_turn_speed():
  c = tg.TurnGeometryCorrection(DT_MDL)
  out = STOCK
  for _ in range(40):
    out = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=10.0, ref_offset_m=0.35)
  assert out == pytest.approx(STOCK - (0.10 + 0.035), abs=1e-6)


def test_delay_table_values():
  # measured desired->yaw lag minus lagd, per speed
  assert tg.target_reduction_s(10.0, 0.0) == pytest.approx(0.10)
  assert tg.target_reduction_s(14.0, 0.0) == pytest.approx(0.08)
  assert tg.target_reduction_s(19.0, 0.0) == pytest.approx(0.05)
  assert tg.target_reduction_s(7.5, 0.0) == pytest.approx(0.115)


def test_ref_offset_is_used():
  assert tg.target_reduction_s(10.0, 0.35) == pytest.approx(0.135)
  assert tg.target_reduction_s(10.0, 0.7) == pytest.approx(0.17)
  assert tg.target_reduction_s(10.0, 0.35) > tg.target_reduction_s(10.0, 0.0)


def test_ref_offset_speed_floor():
  assert tg.ref_offset_s(1.0, 0.35) == pytest.approx(0.35 / 3.0)
  assert tg.ref_offset_s(0.5, 0.9) == pytest.approx(0.3)
  assert tg.ref_offset_s(7.0, 0.35) == pytest.approx(0.05)


def test_highway_only_reference_offset():
  for v in (25.0, 30.0, 35.0):
    assert tg.target_reduction_s(v, 0.35) == pytest.approx(0.35 / v)
    assert tg.target_reduction_s(v, 0.35) < 0.015
    assert tg.target_reduction_s(v, 0.0) == 0.0


def test_no_effect_below_1p5ms_and_continuous_fade():
  for v in (0.0, 1.0, 1.49, 1.5):
    assert tg.target_reduction_s(v, 0.35) == 0.0
  # half faded at 2.0 m/s: 0.5 * (0.13 + 0.35 / 3)
  assert tg.target_reduction_s(2.0, 0.35) == pytest.approx(0.5 * (0.13 + 0.35 / 3.0))
  assert tg.target_reduction_s(2.5, 0.35) == pytest.approx(0.13 + 0.35 / 3.0)
  vs = np.linspace(0.0, 40.0, 4001)
  r = np.array([tg.target_reduction_s(float(v), 0.35) for v in vs])
  assert np.max(np.abs(np.diff(r))) <= 0.0026         # no jumps (fade slope 0.25 s per m/s)
  assert np.all(r <= tg.LOW_SPEED_MAX_REDUCTION_S + 1e-12)
  assert np.all(r[vs >= tg.LOW_SPEED_REACH_END_MS] <= tg.MAX_REDUCTION_S + 1e-12)


def test_reduction_capped_at_02():
  assert tg.target_reduction_s(5.0, 1.0) == pytest.approx(0.2)
  assert tg.apply_reduction(STOCK, 0.5) == pytest.approx(STOCK - 0.2)


def test_min_lookahead_floor_never_raises():
  assert tg.apply_reduction(0.30, 0.2) == pytest.approx(0.20)
  assert tg.apply_reduction(0.25, 0.2) == pytest.approx(0.20)
  assert tg.apply_reduction(0.15, 0.2) == pytest.approx(0.15)   # already below floor: unchanged
  assert tg.apply_reduction(0.15, 0.0) == 0.15


def test_rate_limit_holds_stock_then_slews_back():
  c = tg.TurnGeometryCorrection(DT_MDL)
  out_angle = 0.0
  for _ in range(40):
    c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0, ref_offset_m=0.35,
             lat_active=True, cmd_angle_deg=0.0, out_angle_deg=out_angle)
  full = c.reduction_s
  assert full == pytest.approx(tg.target_reduction_s(6.0, 0.35))
  # turn-in from low speed: output moving 12.5 deg per 50 ms (250 deg/s), 80 deg short
  outs = []
  for _ in range(8):
    out_angle += 12.5
    outs.append(c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0, ref_offset_m=0.35,
                         lat_active=True, cmd_angle_deg=out_angle + 80.0, out_angle_deg=out_angle))
  assert c.rate_limited
  assert c.reduction_s == pytest.approx(0.0, abs=1e-9)
  assert outs[-1] == pytest.approx(STOCK)
  # slewed, not stepped
  assert np.max(np.abs(np.diff(outs))) <= tg.REDUCTION_SLEW_S_PER_S * DT_MDL + 1e-9
  # limiter releases: held 0.5 s, then back to full correction
  for i in range(int(tg.RATE_LIMIT_HOLD_S / DT_MDL) - 1):
    c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0, ref_offset_m=0.35,
             lat_active=True, cmd_angle_deg=out_angle, out_angle_deg=out_angle)
    assert c.reduction_s == 0.0
  for _ in range(40):
    c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0, ref_offset_m=0.35,
             lat_active=True, cmd_angle_deg=out_angle, out_angle_deg=out_angle)
  assert c.reduction_s == pytest.approx(full)


def test_small_gap_or_slow_rate_is_not_rate_limited():
  assert not tg.steer_rate_limited(10.0, 9.0, 0.0, DT_MDL, True)       # fast but caught up
  assert not tg.steer_rate_limited(50.0, 1.0, 0.5, DT_MDL, True)       # far but slow
  assert not tg.steer_rate_limited(50.0, 12.5, 0.0, DT_MDL, False)     # lat off
  assert tg.steer_rate_limited(50.0, 12.5, 0.0, DT_MDL, True)


# ---------- plan sampling: onset later ----------

def _turn_plan(v, t_turn, kappa):
  t = np.array([10.0 * (i / 32) ** 2 for i in range(33)])   # ModelConstants.T_IDXS
  yaw_rate = np.where(t >= t_turn, v * kappa, 0.0)
  yaw = np.concatenate([[0.0], np.cumsum(0.5 * (yaw_rate[1:] + yaw_rate[:-1]) * np.diff(t))])
  return t, yaw, yaw_rate


def test_plan_turn_in_later_steady_state_same():
  v, kappa = 6.0, 1.0 / 15.0
  on_t = tg.apply_reduction(STOCK, tg.target_reduction_s(v, 0.35))
  assert on_t == pytest.approx(STOCK - 0.182, abs=0.002)
  # turn 0.40 s ahead: stock already steers into it, corrected waits
  t, yaw, yr = _turn_plan(v, 0.40, kappa)
  assert get_curvature_from_plan(yaw, yr, t, v, STOCK) > 0.01
  assert get_curvature_from_plan(yaw, yr, t, v, on_t) == pytest.approx(0.0, abs=1e-9)
  # on the arc: same curvature (steady radius unaffected by timing)
  t, yaw, yr = _turn_plan(v, -1.0, kappa)
  off_k = get_curvature_from_plan(yaw, yr, t, v, STOCK)
  on_k = get_curvature_from_plan(yaw, yr, t, v, on_t)
  assert on_k == pytest.approx(off_k, rel=1e-6)
  assert on_k == pytest.approx(kappa, rel=0.05)


# ---------- closed-loop kinematic ----------

def _plant_lag(v):
  return float(np.interp(v, [5, 10, 14, 19, 25], [0.26, 0.29, 0.31, 0.34, 0.39])) + 0.075


def _sim_turn(R, v, lookahead, d=1.90, d_t=1.55, angle=math.pi / 2, dt=0.01):
  """Rear-axle deviation (+inside / -outside) from the lane-centre arc."""
  L1, La, ds = 40.0, R * angle, 0.02
  ss = np.arange(0.0, L1 + La + 40.0, ds)
  k = np.where((ss >= L1) & (ss < L1 + La), 1.0 / R, 0.0)
  psi = np.cumsum(k * ds)
  X, Y = np.cumsum(np.cos(psi) * ds), np.cumsum(np.sin(psi) * ds)
  TX, TY = X + d_t * np.cos(psi), Y + d_t * np.sin(psi)
  i0 = int(20.0 / ds)
  x, y, h = X[i0], Y[i0], psi[i0]
  buf = [0.0] * int(round(_plant_lag(v) / dt))
  Ld = max(6.0, 1.5 * v)
  jr = jc = i0
  out = []
  while True:
    lo, hi = max(0, jr - 100), min(len(ss), jr + 100)
    jr = lo + int(np.argmin((X[lo:hi] - x) ** 2 + (Y[lo:hi] - y) ** 2))
    if ss[jr] > L1 + La + 25.0:
      break
    px, py = x + d * math.cos(h), y + d * math.sin(h)
    lo, hi = max(0, jc - 100), min(len(ss), jc + 150)
    jc = lo + int(np.argmin((TX[lo:hi] - px) ** 2 + (TY[lo:hi] - py) ** 2))
    e = (px - TX[jc]) * -math.sin(psi[jc]) + (py - TY[jc]) * math.cos(psi[jc])
    buf.append(float(np.interp(ss[jc] + v * lookahead, ss, k)) - 2.0 * e / (Ld * Ld))
    kk = buf.pop(0)
    x += v * math.cos(h) * dt
    y += v * math.sin(h) * dt
    h += v * kk * dt
    out.append((x - X[jr]) * -math.sin(psi[jr]) + (y - Y[jr]) * math.cos(psi[jr]))
  o = np.array(out)
  return float(o.max()), float(o.min())


@pytest.mark.parametrize("R,v", [(10.0, 4.5), (15.0, 5.5), (20.0, 6.5)])
def test_closed_loop_corner_cut_removed_without_running_wide(R, v):
  on_t = tg.apply_reduction(STOCK, tg.target_reduction_s(v, tg.REF_OFFSET_DEFAULT_M))
  off_in, off_out = _sim_turn(R, v, STOCK)
  on_in, on_out = _sim_turn(R, v, on_t)
  assert off_in > 0.45                   # today: early turn-in clips the corner
  assert on_in < 0.2 and on_in < 0.4 * off_in
  assert on_out > -0.2                   # and does not run wide


def test_closed_loop_highway_unchanged():
  v = 30.0
  on_t = tg.apply_reduction(STOCK, tg.target_reduction_s(v, 0.35))
  assert STOCK - on_t < 0.015


# ---------- wiring pins ----------

def test_modeld_applies_only_to_model_action_lookahead():
  src = (ROOT / "selfdrive/modeld/modeld.py").read_text()
  assert "lat_action_t = lat_delay + frame_delay + action_delay\n" in src
  call = "action = get_action_from_model(model_output, prev_action, lat_action_t, long_delay + frame_delay + action_delay, v_ego,"
  assert call + "\n                                     lat_sample_floor_s)" in src
  # stock sample floor unless the Pre-AP correction supplies one
  assert "      lat_sample_floor_s = MIN_STABLE_DELAY\n      if turn_geom_preap:" in src
  assert "        lat_sample_floor_s = turn_geom.sample_floor_s\n" in src
  assert "desired_curvature = plan_curvature(" in src
  assert "get_curvature_from_plan(" not in src
  assert "if turn_geom_preap:" in src
  assert "enabled=turn_geom_on, stock_lookahead_s=lat_action_t" in src
  # no schema change: modeld writes no new capnp fields
  assert "napLat" not in src


def test_lateral_maneuver_plan_bypasses_correction():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  assert ("    if self.sm.valid['lateralManeuverPlan']:\n"
          "      model_or_plan_curvature = self.sm['lateralManeuverPlan'].desiredCurvature\n") in src
  assert "turn_geom.update" not in src
  assert "napLat" not in src


# ---------- roundabout: no double compensation ----------

def _hint(on=True, approaching=False, d=0.0):
  return RoundaboutHint(on_roundabout=on, approaching=approaching, distance_m=d, speed_limit_ms=0.0, way_id=1)


def test_roundabout_outer_bias_stripped_when_correction_active():
  for h in (_hint(on=True), _hint(on=False, approaching=True, d=30.0), _hint(on=False, approaching=True, d=150.0), None):
    for rhd in (False, True):
      assert roundabout_lateral_curvature_bias(h, is_rhd=rhd, turn_geometry_active=True) == 0.0


def test_roundabout_outer_bias_off_path_is_legacy():
  for h in (_hint(on=True), _hint(on=False, approaching=True, d=30.0), _hint(on=False, approaching=True, d=150.0), None):
    for rhd in (False, True):
      legacy = roundabout_outer_curvature_bias(roundabout_outer_path_offset_m(
        on_roundabout=bool(h.on_roundabout) if h is not None else False,
        approaching=bool(h.approaching) if h is not None else False,
        distance_m=float(h.distance_m) if h is not None else 0.0,
        is_rhd=rhd))
      assert roundabout_lateral_curvature_bias(h, is_rhd=rhd, turn_geometry_active=False) == legacy
  assert roundabout_lateral_curvature_bias(_hint(on=True), turn_geometry_active=False) == pytest.approx(-2 * 3.2 / 18.0 ** 2)


def test_controlsd_gates_roundabout_bias_on_turn_geometry():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  assert "roundabout_lateral_curvature_bias(\n      rb_hint, is_rhd=is_rhd, turn_geometry_active=self._turn_geom_active," in src
  assert "self._turn_geom_preap, bool(self.params.get_bool(PARAM_TURN_GEOMETRY)))" in src
  assert "roundabout_outer_curvature_bias(" not in src


# ---------- low-speed reach (Sep 28 PM: 5-8 mph turns still clip the inside) ----------

MPH = 0.44704


def test_low_speed_reach_band_and_values():
  for mph in (1.0, 3.0, 5.0, 8.0):
    assert tg.low_speed_reach_weight(mph * MPH) == pytest.approx(1.0)
  assert tg.low_speed_reach_weight(9.0 * MPH) == pytest.approx(0.5)
  for mph in (10.0, 12.0, 20.0, 70.0):
    assert tg.low_speed_reach_weight(mph * MPH) == 0.0
    assert tg.reduction_cap_s(mph * MPH) == tg.MAX_REDUCTION_S
    assert tg.sample_floor_s(mph * MPH) == MIN_STABLE_DELAY
  assert tg.reduction_cap_s(6.0 * MPH) == pytest.approx(0.25)
  assert tg.sample_floor_s(6.0 * MPH) == pytest.approx(0.20)
  assert tg.reduction_cap_s(9.0 * MPH) == pytest.approx(0.225)
  assert tg.sample_floor_s(9.0 * MPH) == pytest.approx(0.25)


def _settle(c, v, n=60, **kw):
  out = None
  for _ in range(n):
    out = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=v, ref_offset_m=0.35, **kw)
  return out


@pytest.mark.parametrize("mph", [6.0, 7.6])
def test_low_speed_extra_reach_at_5_to_8_mph(mph):
  v = mph * MPH
  c = tg.TurnGeometryCorrection(DT_MDL)
  la = _settle(c, v)
  red = min(0.25, 0.13 + 0.35 / max(v, 3.0))
  assert la == pytest.approx(STOCK - red, abs=1e-6)       # ~0.23 s, was 0.475 (6 mph) / 0.40 (7.6 mph)
  assert la < MIN_STABLE_DELAY
  assert c.sample_floor_s == pytest.approx(0.20)
  # a turn starting 0.26 s ahead: 7c49a8e (0.3 s floor) already steers, the new sampling waits
  t, yaw, yr = _turn_plan(v, 0.26, 1.0 / 8.0)
  assert get_curvature_from_plan(yaw, yr, t, v, la) > 0.01
  assert tg.plan_curvature(yaw, yr, t, v, la, c.sample_floor_s) == pytest.approx(0.0, abs=1e-9)
  # steady arc: same radius either way
  t, yaw, yr = _turn_plan(v, -1.0, 1.0 / 8.0)
  assert tg.plan_curvature(yaw, yr, t, v, la, c.sample_floor_s) == pytest.approx(
    get_curvature_from_plan(yaw, yr, t, v, la), rel=1e-6)


def test_plan_curvature_stock_floor_is_get_curvature_from_plan():
  rng = np.random.default_rng(7)
  t = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
  for _ in range(300):
    yr = np.cumsum(rng.normal(0, 0.05, 33))
    yaw = np.concatenate([[0.0], np.cumsum(0.5 * (yr[1:] + yr[:-1]) * np.diff(t))])
    v = float(rng.uniform(0.0, 40.0))
    a = float(rng.uniform(0.05, 0.8))
    assert tg.plan_curvature(yaw, yr, t, v, a) == get_curvature_from_plan(yaw, yr, t, v, a)
    assert tg.plan_curvature(yaw, yr, t, v, a, MIN_STABLE_DELAY) == get_curvature_from_plan(yaw, yr, t, v, a)


def test_plan_curvature_direct_sampling_below_03():
  t, yaw, yr = _turn_plan(3.0, 0.1, 1.0 / 8.0)
  for a in (0.2, 0.22, 0.25, 0.28):
    k = tg.plan_curvature(yaw, yr, t, 3.0, a, 0.2)
    assert k == pytest.approx(curv_from_psis(np.interp(a, t, yaw), yr[0], 3.0, a))
    assert k != pytest.approx(get_curvature_from_plan(yaw, yr, t, 3.0, a), rel=1e-3)
  # clamped at the floor, continuous at 0.3 s
  assert tg.plan_curvature(yaw, yr, t, 3.0, 0.1, 0.2) == pytest.approx(tg.plan_curvature(yaw, yr, t, 3.0, 0.2, 0.2))
  assert tg.plan_curvature(yaw, yr, t, 3.0, 0.2999999, 0.2) == pytest.approx(
    get_curvature_from_plan(yaw, yr, t, 3.0, 0.3), rel=1e-4)


def test_rate_limit_freezes_reduction_below_10mph():
  v = 6.0 * MPH
  c = tg.TurnGeometryCorrection(DT_MDL)
  out_angle = 0.0
  full_out = _settle(c, v, lat_active=True, cmd_angle_deg=0.0, out_angle_deg=out_angle)
  full = c.reduction_s
  assert full == pytest.approx(min(0.25, 0.13 + 0.35 / 3.0))
  for _ in range(8):
    out_angle += 12.5
    o = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=v, ref_offset_m=0.35,
                 lat_active=True, cmd_angle_deg=out_angle + 80.0, out_angle_deg=out_angle)
    assert c.rate_limited
    assert c.reduction_s == full           # frozen, not slewed back to stock
    assert o == full_out
  for _ in range(40):
    c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=v, ref_offset_m=0.35,
             lat_active=True, cmd_angle_deg=out_angle, out_angle_deg=out_angle)
  assert c.reduction_s == pytest.approx(full)


# Verbatim 7c49a8e formulas (reference for the >= 10 mph identity pin).
class _Legacy:
  def __init__(self, dt):
    self.dt, self.reduction_s, self._hold_s, self._out_prev = dt, 0.0, 0.0, None

  @staticmethod
  def target(v_ego, ref_offset_m):
    v = max(0.0, float(v_ego))
    d = float(np.interp(v, (5.0, 10.0, 14.0, 19.0, 25.0), (-0.13, -0.10, -0.08, -0.05, 0.0)))
    red = min(0.20, max(0.0, -d + tg.clamp_ref_offset(ref_offset_m) / max(v, 3.0)))
    return red * float(np.clip((v - 3.0) / (4.0 - 3.0), 0.0, 1.0))

  @staticmethod
  def apply(stock, reduction):
    stock = float(stock)
    red = min(0.20, max(0.0, float(reduction)))
    if red <= 0.0:
      return stock
    return max(min(stock, 0.20), stock - red)

  def update(self, *, enabled, stock_lookahead_s, v_ego, ref_offset_m, lat_active=True, cmd_angle_deg=0.0,
             out_angle_deg=None):
    if not enabled:
      self.reduction_s, self._hold_s, self._out_prev = 0.0, 0.0, None
      return stock_lookahead_s
    limited = False
    if out_angle_deg is not None:
      if self._out_prev is not None and lat_active and self.dt > 0.0:
        rate = abs(float(out_angle_deg) - float(self._out_prev)) / self.dt
        limited = rate >= 0.8 * 250.0 and abs(float(cmd_angle_deg) - float(out_angle_deg)) > 2.5
      self._out_prev = float(out_angle_deg)
    self._hold_s = 0.5 if limited else max(0.0, self._hold_s - self.dt)
    target = 0.0 if self._hold_s > 0.0 else self.target(v_ego, ref_offset_m)
    step = 0.5 * self.dt
    self.reduction_s += float(np.clip(target - self.reduction_s, -step, step))
    return self.apply(stock_lookahead_s, self.reduction_s)


def test_at_or_above_10mph_output_identical_to_7c49a8e():
  rng = np.random.default_rng(253)
  t_idx = np.array([10.0 * (i / 32) ** 2 for i in range(33)])
  for trial in range(20):
    new, old = tg.TurnGeometryCorrection(DT_MDL), _Legacy(DT_MDL)
    out_angle, v = 0.0, float(rng.uniform(tg.LOW_SPEED_REACH_END_MS, 12.0))
    offset = float(rng.choice([0.0, 0.35, 0.6, 1.0]))
    stock = float(rng.choice([STOCK, 0.36, 0.52]))
    for step in range(400):
      v = float(np.clip(v + rng.normal(0, 0.3), tg.LOW_SPEED_REACH_END_MS, 40.0))
      burst = (step // 40) % 3 == 1                        # rate-limited bursts
      out_angle += 12.5 if burst else float(rng.normal(0, 1.0))
      cmd = out_angle + (80.0 if burst else float(rng.normal(0, 1.0)))
      enabled = not (trial % 5 == 4 and 150 <= step < 170)
      kw = dict(enabled=enabled, stock_lookahead_s=stock, v_ego=v, ref_offset_m=offset,
                lat_active=True, cmd_angle_deg=cmd, out_angle_deg=out_angle)
      a_new, a_old = new.update(**kw), old.update(**kw)
      assert a_new == a_old, (trial, step, v)
      assert new.sample_floor_s == MIN_STABLE_DELAY
      yr = np.cumsum(rng.normal(0, 0.03, 33))
      yaw = np.concatenate([[0.0], np.cumsum(0.5 * (yr[1:] + yr[:-1]) * np.diff(t_idx))])
      assert tg.plan_curvature(yaw, yr, t_idx, v, a_new, new.sample_floor_s) == \
        get_curvature_from_plan(yaw, yr, t_idx, v, a_old)


@pytest.mark.parametrize("R,v", [(7.0, 6.0 * MPH), (6.0, 7.0 * MPH)])
def test_closed_loop_low_speed_inside_cut_reduced(R, v):
  # effective sample time = what get_curvature_from_plan / plan_curvature actually use
  old = _Legacy(DT_MDL)
  old_t = max(old.apply(STOCK, old.target(v, 0.35)), MIN_STABLE_DELAY)
  c = tg.TurnGeometryCorrection(DT_MDL)
  la = _settle(c, v)
  new_t = max(la, c.sample_floor_s)
  old_in, old_out = _sim_turn(R, v, old_t)
  new_in, new_out = _sim_turn(R, v, new_t)
  assert old_in > 0.35                   # 7c49a8e: still clips the inside at 6-8 mph
  assert new_in < 0.5 * old_in
  assert new_out > -0.2                  # and does not run wide


# ---------- turn-in delay trim (NAPTurnInDelay) ----------

def _la(step, v, stock=STOCK, n=80, enabled=True):
  c = tg.TurnGeometryCorrection(DT_MDL)
  out = None
  for _ in range(n):
    out = c.update(enabled=enabled, stock_lookahead_s=stock, v_ego=v, ref_offset_m=0.35, turn_in_step=step)
  return out, c


def test_turn_in_param_defaults_to_zero_int():
  keys = (ROOT / "common/params_keys.h").read_text()
  assert '{"NAPTurnInDelay", {PERSISTENT, INT, "0"}}' in keys
  assert tg.PARAM_TURN_IN_DELAY == "NAPTurnInDelay"
  assert (tg.TURN_IN_STEP_MIN, tg.TURN_IN_STEP_MAX) == (-2, 3)
  assert tg.TURN_IN_STEP_S == pytest.approx(0.030)
  assert tg.TURN_IN_MIN_LOOKAHEAD_S == pytest.approx(0.10)


def test_turn_in_step_is_clamped_and_junk_is_zero():
  assert [tg.clamp_turn_in_step(x) for x in (-9, -2, -1, 0, 1, 2, 3, 4, 99)] == [-2, -2, -1, 0, 1, 2, 3, 3, 3]
  assert tg.clamp_turn_in_step(2.6) == 3
  assert tg.clamp_turn_in_step(-1.4) == -1
  for junk in (None, "x", float("nan"), float("inf"), [], {}):
    assert tg.clamp_turn_in_step(junk) == 0


def test_turn_in_step_zero_is_bit_identical_to_the_untrimmed_update():
  rng = np.random.default_rng(7)
  t_idx = np.linspace(0.0, 10.0, 33)
  for trial in range(20):
    new, old = tg.TurnGeometryCorrection(DT_MDL), tg.TurnGeometryCorrection(DT_MDL)
    v = float(rng.uniform(0.5, 6.0))
    out_angle = 0.0
    stock = float(rng.uniform(0.25, 0.7))
    for step in range(300):
      v = float(np.clip(v + rng.normal(0, 0.3), 0.0, 40.0))
      burst = (step // 40) % 3 == 1
      out_angle += 12.5 if burst else float(rng.normal(0, 1.0))
      cmd = out_angle + (80.0 if burst else float(rng.normal(0, 1.0)))
      kw = dict(enabled=not (trial % 5 == 4 and 100 <= step < 120), stock_lookahead_s=stock, v_ego=v,
                ref_offset_m=0.35, lat_active=True, cmd_angle_deg=cmd, out_angle_deg=out_angle)
      a = old.update(**kw)
      assert new.update(turn_in_step=0, **kw) == a, (trial, step)
      assert new.sample_floor_s == old.sample_floor_s
      assert new.trim_s == 0.0
      yr = np.cumsum(rng.normal(0, 0.03, 33))
      yaw = np.concatenate([[0.0], np.cumsum(0.5 * (yr[1:] + yr[:-1]) * np.diff(t_idx))])
      assert tg.plan_curvature(yaw, yr, t_idx, v, a, old.sample_floor_s) == \
        tg.plan_curvature(yaw, yr, t_idx, v, a, new.sample_floor_s)


@pytest.mark.parametrize("step", [-2, -1, 1, 2, 3])
def test_turn_in_trim_never_changes_10mph_and_up_or_below_1p5ms(step):
  for v in (10.0 * MPH, 11.0 * MPH, 15.0 * MPH, 25.0, 40.0, 1.5, 1.0, 0.3):
    base, cb = _la(0, v)
    trim, ct = _la(step, v)
    assert trim == base
    assert ct.sample_floor_s == cb.sample_floor_s
    assert ct.trim_s == 0.0


@pytest.mark.parametrize("step", [-2, -1, 1, 2, 3])
@pytest.mark.parametrize("mph", [5.6, 6.0, 7.0, 8.0])
def test_turn_in_each_step_is_30ms_in_the_full_band(step, mph):
  v = max(mph * MPH, 2.5)
  base, _ = _la(0, v)
  trim, c = _la(step, v)
  assert trim - base == pytest.approx(-step * 0.030, abs=1e-9)
  # time shift -> distance shift: this is what the menu description quotes
  assert (base - trim) * v == pytest.approx(step * 0.030 * v, abs=1e-9)
  assert c.trim_s == pytest.approx(step * 0.030)


def test_turn_in_band_fades_out_by_10mph_and_in_between_1p5_and_2p5():
  assert tg.turn_in_weight(2.5) == pytest.approx(1.0)
  assert tg.turn_in_weight(8.0 * MPH) == pytest.approx(1.0)
  assert tg.turn_in_weight(9.0 * MPH) == pytest.approx(0.5)
  assert tg.turn_in_weight(10.0 * MPH) == 0.0
  assert tg.turn_in_weight(1.5) == 0.0
  assert tg.turn_in_weight(2.0) == pytest.approx(0.5)
  vs = np.linspace(0.0, 15.0, 301)
  w = [tg.turn_in_weight(float(x)) for x in vs]
  assert max(abs(a - b) for a, b in zip(w, w[1:], strict=False)) < 0.06      # continuous
  assert tg.turn_in_shift_s(3, 9.0 * MPH) == pytest.approx(0.5 * 3 * 0.030)
  assert tg.turn_in_shift_s(-2, 9.0 * MPH) == pytest.approx(-0.5 * 2 * 0.030)
  assert tg.turn_in_shift_s(99, 6.0 * MPH) == pytest.approx(3 * 0.030)       # clamped


def test_turn_in_lookahead_is_bounded():
  # positive: never below 0.10 s (or below the untrimmed value if that is lower)
  for stock in (0.12, 0.2, 0.3, 0.475):
    base, _ = _la(0, 6.0 * MPH, stock=stock)
    trim, _ = _la(3, 6.0 * MPH, stock=stock)
    assert trim >= min(base, tg.TURN_IN_MIN_LOOKAHEAD_S) - 1e-12
    assert trim <= base + 1e-12
  assert tg.apply_turn_in_trim(0.11, 0.4, 0.09) == pytest.approx(tg.TURN_IN_MIN_LOOKAHEAD_S)
  assert tg.apply_turn_in_trim(0.11, 0.4, 0.09) == pytest.approx(0.10)
  assert tg.apply_turn_in_trim(0.05, 0.05, 0.09) == pytest.approx(0.05)
  # negative: never above stock
  for stock in (0.25, 0.3, 0.475):
    base, _ = _la(0, 6.0 * MPH, stock=stock)
    trim, _ = _la(-2, 6.0 * MPH, stock=stock)
    assert base <= trim <= stock + 1e-12
  assert tg.apply_turn_in_trim(0.30, 0.31, -0.06) == pytest.approx(0.31)
  assert tg.apply_turn_in_trim(0.2, 0.475, 0.0) == 0.2


def test_turn_in_moves_the_sample_floor_with_the_lookahead():
  v = 6.0 * MPH
  base, cb = _la(0, v)
  later, cl = _la(3, v)
  earlier, ce = _la(-2, v)
  assert cb.sample_floor_s == pytest.approx(0.20)
  assert cl.sample_floor_s == pytest.approx(0.20 - 0.09)
  assert ce.sample_floor_s == pytest.approx(0.20 + 0.06)
  assert later < base < earlier
  # the plan really is sampled at the trimmed time (below the old 0.2 s floor)
  t_idx = np.linspace(0.0, 10.0, 2001)
  yr = t_idx.copy()
  yaw = 0.5 * t_idx ** 2           # curvature now depends on the sample time
  k_base = tg.plan_curvature(yaw, yr, t_idx, v, base, cb.sample_floor_s)
  k_later = tg.plan_curvature(yaw, yr, t_idx, v, later, cl.sample_floor_s)
  assert k_base == pytest.approx(curv_from_psis(np.interp(base, t_idx, yaw), 0.0, v, base))
  assert k_later == pytest.approx(curv_from_psis(np.interp(later, t_idx, yaw), 0.0, v, later))
  assert k_later != pytest.approx(curv_from_psis(np.interp(0.2, t_idx, yaw), 0.0, v, 0.2))
  assert later < tg.LOW_SPEED_SAMPLE_FLOOR_S


def test_turn_in_trim_is_slewed_not_stepped():
  c = tg.TurnGeometryCorrection(DT_MDL)
  for _ in range(60):
    c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0 * MPH, ref_offset_m=0.35)
  prev = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0 * MPH, ref_offset_m=0.35)
  worst = 0.0
  for _ in range(20):
    cur = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0 * MPH, ref_offset_m=0.35, turn_in_step=3)
    worst = max(worst, abs(cur - prev))
    prev = cur
  assert worst <= tg.REDUCTION_SLEW_S_PER_S * DT_MDL + 1e-9
  assert c.trim_s == pytest.approx(0.09)
  for _ in range(20):
    cur = c.update(enabled=True, stock_lookahead_s=STOCK, v_ego=6.0 * MPH, ref_offset_m=0.35, turn_in_step=0)
  assert c.trim_s == 0.0


@pytest.mark.parametrize("step", [-2, 3])
def test_turn_in_trim_needs_turn_geometry_on(step):
  out, c = _la(step, 6.0 * MPH, enabled=False)
  assert out == STOCK
  assert c.trim_s == 0.0
  assert c.sample_floor_s == MIN_STABLE_DELAY
  # trimmed, then turned off: the trim is dropped with the rest of the state
  _, c = _la(step, 6.0 * MPH)
  assert c.trim_s != 0.0
  out = c.update(enabled=False, stock_lookahead_s=STOCK, v_ego=6.0 * MPH, ref_offset_m=0.35, turn_in_step=step)
  assert out == STOCK
  assert c.trim_s == 0.0


def test_turn_in_trim_is_wired_into_modeld_and_the_menu():
  modeld = (ROOT / "selfdrive/modeld/modeld.py").read_text()
  assert "turn_in_step = clamp_turn_in_step(params.get(PARAM_TURN_IN_DELAY, return_default=True))" in modeld
  assert "          turn_in_step=turn_in_step)\n" in modeld
  menu = (ROOT / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  assert '"Turn-In Timing (low speed)"' in menu
  assert "self._params.put(NAP_TURN_IN_DELAY, int(TURN_IN_DELAY_STEPS[index]))" in menu
  assert "    self._all_items.append(self._turn_in_buttons)\n" in menu
  mici = (ROOT / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  assert '"turn-in timing"' in mici
  from openpilot.selfdrive.ui.layouts.settings import nap_lateral as nl
  assert nl.TURN_IN_DELAY_LABELS == ["-2", "-1", "0", "+1", "+2", "+3"]
  assert nl.TURN_IN_DELAY_STEPS == [-2, -1, 0, 1, 2, 3]
  assert "30 ms" in nl.TURN_IN_DELAY_DESCRIPTION

  class _P:
    def __init__(self, v):
      self.v = v

    def get(self, key, return_default=False):
      assert key == "NAPTurnInDelay"
      if isinstance(self.v, Exception):
        raise self.v
      return self.v

  assert nl.turn_in_delay_index(_P(0)) == 2
  assert nl.turn_in_delay_index(_P(-2)) == 0
  assert nl.turn_in_delay_index(_P(3)) == 5
  assert nl.turn_in_delay_index(_P(9)) == 5
  assert nl.turn_in_delay_index(_P(KeyError("old params build"))) == 2
