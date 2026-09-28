"""Turn geometry correction (NAPLatTurnGeom / NAPLatRefOffset).

Pins: default ON, OFF == stock lookahead (1119c0f) bit-for-bit, the low-speed
delay table, rear reference offset, speed floor/fade, 0.2 s cap and 0.2 s
floor, 250 deg/s rate-limit hold-off, lateralManeuverPlan untouched, and the
#223 roundabout outer bias stripped while the correction is active.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import lat_turn_geometry as tg
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
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


def test_no_effect_below_3ms_and_continuous_fade():
  for v in (0.0, 1.0, 2.0, 2.99, 3.0):
    assert tg.target_reduction_s(v, 0.35) == 0.0
  assert tg.target_reduction_s(3.5, 0.35) == pytest.approx(0.5 * 0.2)
  vs = np.linspace(0.0, 40.0, 4001)
  r = np.array([tg.target_reduction_s(float(v), 0.35) for v in vs])
  assert np.max(np.abs(np.diff(r))) <= 0.0021         # no jumps (fade slope 0.2 s per m/s)
  assert np.all(r <= tg.MAX_REDUCTION_S + 1e-12)


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
  assert "action = get_action_from_model(model_output, prev_action, lat_action_t, long_delay + frame_delay + action_delay, v_ego)" in src
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
