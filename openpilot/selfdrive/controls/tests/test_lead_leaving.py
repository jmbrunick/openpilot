"""Lead leaving our path: continuous weight on its braking.

Turning lead with clearance at our arrival is released before radard drops
it. A lead that stalls half-out, or drifts inside its own lane, keeps full
weight. The weight is continuous (no jump) and never lifts the command
above the lead-free one (MAX, map, curve).
"""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lead_leaving import (
  EGO_HALF_WIDTH_M,
  LeadLeavingEstimator,
  clearance_needed_m,
  leave_weight,
  time_to_arrive_s,
)
from openpilot.selfdrive.controls.lib.unified_lead import (
  UnifiedLeadController,
  unified_follow_desired,
)
from openpilot.selfdrive.controls.tests.test_unified_lead_planner import (
  _own_lead,
  _planner,
)

DT = 0.05


def _drive(est, ys, *, gap0=30.0, v_ego=16.0, v_lead=11.0, a_lead=-1.5, track=7, probs=None):
  """Feed one lead track; gap closes at v_ego - v_lead. Returns weights."""
  out = []
  gap = gap0
  for i, y in enumerate(ys):
    p = 1.0 if probs is None else probs[i]
    out.append(est.update(dt=DT, present=True, lead_id=track, path_lat=float(y), gap=gap,
                          v_ego=v_ego, v_lead=v_lead, a_lead=a_lead, model_prob=p))
    gap = max(2.0, gap - (v_ego - v_lead) * DT)
  return np.array(out)


def _turn_off_ys(v_out=1.2, y0=0.2, y_end=2.0, pre_s=2.0):
  """On-path, then moving out at v_out until radard's ~2 m gate."""
  ys = [y0] * int(pre_s / DT)
  y = y0
  while y < y_end:
    y += v_out * DT
    ys.append(y)
  return np.array(ys)


def test_turning_lead_released_early_with_clearance():
  est = LeadLeavingEstimator()
  ys = _turn_off_ys()
  w = _drive(est, ys, gap0=40.0, probs=np.linspace(1.0, 0.2, len(ys)))
  # Not before its center is past our half of the lane.
  assert np.all(w[ys < 0.9] == 0.0)
  # Mostly released before radard would drop it at ~2 m (its body still
  # overlapping ours: the projection at our arrival is what clears).
  assert w[ys < 1.9].max() >= 0.7
  first = int(np.argmax(w > 0.5))
  assert ys[first] < 1.8
  # The planner's unified command: the lead brake (-3.4) is released at a bounded jerk.
  planner, inputs, _ = _planner(16.0, accel=-2.5)
  lead = _own_lead(inputs, 40.0, 11.0, a_lead=-1.5, y_rel=0.0, track=7)
  a_out = []
  gap = 40.0
  for y in ys:
    lead.yRel = -float(y)  # no model path -> path offset is -yRel
    lead.dRel = gap
    planner.update(inputs)
    a_out.append(float(planner.output_a_target))
    gap -= 5.0 * DT
  a_out = np.array(a_out)
  pre = int(2.0 / DT)
  assert a_out[pre - 10:pre].max() <= -2.4
  moving = ys > 1.0
  # Before radard's gate the unified controller has begun releasing (jerk-bounded).
  assert a_out[moving & (ys < 1.9)].max() >= -2.7
  # Released >= 0.8 m/s^2 from the on-path brake by the time radard would drop it.
  assert a_out[-1] >= a_out[pre - 1] + 0.8
  assert planner.lead_leave_w > 0.6


def test_stalled_half_out_lead_keeps_braking():
  est = LeadLeavingEstimator()
  ys = list(_turn_off_ys(y_end=1.5))
  ys += [ys[-1]] * int(1.5 / DT)  # stops moving out, half in our lane
  ys = np.array(ys)
  w = _drive(est, ys, probs=np.linspace(1.0, 0.5, len(ys)))
  moving_peak = w[: len(ys) - int(1.5 / DT)].max()
  assert moving_peak > 0.0
  # Within ~0.5 s of the stall the weight is back to 0: full braking.
  assert np.all(w[-int(1.0 / DT):] == 0.0)
  # Static half-out never clears on its own geometry either.
  for y in (1.0, 1.4, 1.8, 2.0):
    assert leave_weight(y, 0.0, 30.0, 16.0, 0.0, 11.0, -1.5) == 0.0

  # Planner: after the stall, the legacy command is the unreleased brake.
  planner, inputs, _ = _planner(16.0, accel=-2.5)
  lead = _own_lead(inputs, 30.0, 11.0, a_lead=-1.5, track=7)
  for y in ys:
    lead.yRel = -float(y)
    planner.update(inputs)
  assert planner.lead_leave_w == 0.0
  assert float(planner.output_a_target) <= -2.4


def test_lead_drifting_within_lane_not_released():
  est = LeadLeavingEstimator()
  # Swerving to its lane line and back at 1.5 m/s, both sides, while the
  # in-path confidence dips: projects well past our width, but the lead's
  # center never leaves its lane, so nothing is released.
  leg = list(np.arange(0.0, 0.85, 1.5 * DT)) + [0.85] * 6
  one = leg + leg[::-1]
  ys = np.array(one * 2 + [-y for y in one] * 2)
  probs = 1.0 - 0.5 * np.abs(ys) / 0.85
  w = _drive(est, ys, gap0=25.0, v_lead=15.0, probs=probs)
  assert np.all(w == 0.0)
  # Even a hard in-lane swerve whose projection clears us is not released
  # while its center is still in our half of the lane.
  for y in (0.0, 0.4, 0.7, 0.9):
    assert leave_weight(y, 2.2, 40.0, 16.0, -0.6, 11.0, -1.0) == 0.0
  assert leave_weight(1.4, 2.2, 40.0, 16.0, -0.6, 11.0, -1.0) > 0.9
  planner, inputs, _ = _planner(16.0, accel=-1.8)
  lead = _own_lead(inputs, 25.0, 15.0, a_lead=-1.0, track=3)
  for y in ys:
    lead.yRel = -float(y)
    planner.update(inputs)
    assert planner.lead_leave_w == 0.0


def test_leave_weight_no_jump_sweep():
  """Continuous in every input: small input steps give small weight steps."""
  ys = np.arange(0.0, 3.0, 0.01)
  for v_ego in (5.0, 12.0, 20.0, 30.0):
    for vy in (0.0, 0.3, 0.8, 1.5, 2.5):
      for gap in (8.0, 20.0, 45.0):
        w = np.array([leave_weight(y, vy, gap, v_ego, -0.3, v_ego - 4.0, -1.0) for y in ys])
        assert np.all((w >= 0.0) & (w <= 1.0))
        assert np.max(np.abs(np.diff(w))) < 0.06
  vys = np.arange(-1.0, 3.0, 0.01)
  for y in (1.0, 1.3, 1.7, 2.0):
    w = np.array([leave_weight(y, vy, 25.0, 16.0, 0.0, 11.0, -1.5) for vy in vys])
    assert np.max(np.abs(np.diff(w))) < 0.06
    assert np.all(np.diff(w) >= -1e-12)  # moving out faster never adds weight back
  gaps = np.arange(2.0, 80.0, 0.1)
  w = np.array([leave_weight(1.5, 1.0, g, 16.0, 0.0, 11.0, -1.5) for g in gaps])
  assert np.max(np.abs(np.diff(w))) < 0.06
  # Arrival time is continuous across a_lead = 0 and the lead-stops switch.
  a_leads = np.arange(-4.0, 1.0, 0.01)
  t = np.array([time_to_arrive_s(30.0, 16.0, 11.0, a) for a in a_leads])
  assert np.max(np.abs(np.diff(t))) < 0.05


def test_clearance_bound_is_ego_half_width_plus_lead_plus_margin():
  need = clearance_needed_m(16.0)
  assert need > EGO_HALF_WIDTH_M + 0.9 + 0.29
  # Projection short of clearance: nothing. Clears: released.
  assert leave_weight(1.6, 0.4, 20.0, 16.0, 0.0, 11.0, -1.5) == 0.0
  assert leave_weight(1.6, 1.5, 30.0, 16.0, -0.6, 11.0, -1.0) > 0.9
  # A lead braking to a stop mid-turn stops moving out when it stops.
  assert leave_weight(1.6, 1.0, 30.0, 16.0, 0.0, 1.0, -3.0) == 0.0


def test_fcw_lead_two_and_new_track_restore_full_weight():
  est = LeadLeavingEstimator()
  ys = _turn_off_ys()
  w = _drive(est, ys)
  assert w[-1] > 0.6
  assert est.update(dt=DT, present=True, lead_id=7, path_lat=2.0, gap=20.0, v_ego=16.0,
                    v_lead=11.0, suppress=True) == 0.0
  est2 = LeadLeavingEstimator()
  _drive(est2, ys)
  # Another car on our path behind the one turning off: keep braking.
  w2 = est2.update(dt=DT, present=True, lead_id=7, path_lat=2.0, gap=20.0, v_ego=16.0,
                   v_lead=11.0, a_lead=-1.5, lead_two_path_lat=0.3)
  assert w2 == pytest.approx(max(0.0, w[-1] - 8.0 * DT), abs=1e-6) or w2 < w[-1]
  for _ in range(5):
    w2 = est2.update(dt=DT, present=True, lead_id=7, path_lat=2.0, gap=20.0, v_ego=16.0,
                     v_lead=11.0, a_lead=-1.5, lead_two_path_lat=0.3)
  assert w2 == 0.0
  # New track id starts from full weight.
  est3 = LeadLeavingEstimator()
  _drive(est3, ys)
  assert est3.update(dt=DT, present=True, lead_id=8, path_lat=1.9, gap=20.0, v_ego=16.0,
                     v_lead=11.0) == 0.0


def test_hold_after_radard_drop_keeps_releasing_only_a_moving_lead():
  est = LeadLeavingEstimator()
  _drive(est, _turn_off_ys())
  w_moving = [est.update(dt=DT, present=True, held=True, v_ego=16.0) for _ in range(10)]
  assert min(w_moving) > 0.6
  est = LeadLeavingEstimator()
  ys = list(_turn_off_ys(y_end=1.5)) + [1.5] * 30
  _drive(est, ys)
  w_stalled = [est.update(dt=DT, present=True, held=True, v_ego=16.0) for _ in range(10)]
  assert max(w_stalled) == 0.0


def test_unified_leave_weight_releases_brake_but_keeps_max_ceiling():
  # Braking lead, on path: firm. Fully leaving: toward 0, never above the
  # MAX speed-error decel (EV mild settle stays -0.22 over MAX).
  base = unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=30.0, path_lat=1.5)
  assert base < -1.0
  assert unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=30.0, path_lat=1.5, leave_w=0.0) == base
  half = unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=30.0, path_lat=1.5, leave_w=0.5)
  full = unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=30.0, path_lat=1.5, leave_w=1.0)
  assert base < half < full
  assert full == pytest.approx(0.0, abs=1e-9)
  over = unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=15.0, path_lat=1.5, leave_w=1.0)
  assert -0.3 < over < 0.0
  # Stalled half-out lead (static 1.5 m) keeps the full command in unified too.
  on_path = unified_follow_desired(22.0, 16.0, 11.0, -1.5, 1.3, v_ceiling=30.0, path_lat=0.0)
  assert base == pytest.approx(on_path, abs=1e-9)

  ctrl = UnifiedLeadController()
  a = 0.0
  for i in range(90):
    w = 0.0 if i < 30 else min(1.0, (i - 30) * 0.075)
    a = ctrl.step(dt=DT, present=True, gap=22.0, v_ego=16.0, v_lead=11.0, a_lead=-1.5, t_follow=1.3,
                  lead_id=1, seed_a=a, v_ceiling=30.0, path_lat=1.5, leave_w=w)
    if i == 29:
      firm = a
  assert firm < -1.0
  # Released toward the lead-free command, not held by a confidence drop.
  assert a > -0.3


def test_planner_leaving_release_respects_max_ceiling():
  # Fully released lead brake: hold speed under MAX, the MAX speed-error
  # decel (EV settle and up) over it, never 0 / +a over MAX.
  outs = {}
  for max_ms in (30.0, 13.0):
    planner, inputs, _ = _planner(16.0, accel=-2.5)
    inputs["carState"].vCruise = max_ms * 3.6
    planner._lead_leave.update = lambda **kw: 1.0
    _own_lead(inputs, 25.0, 11.0, a_lead=-1.5, y_rel=-1.9, track=7)
    for _ in range(40):
      planner.update(inputs)
    assert planner.lead_leave_w == 1.0
    outs[max_ms] = float(planner.output_a_target)
  assert outs[30.0] > -0.1
  assert outs[13.0] < -0.3


@pytest.mark.parametrize("v_ego,d_rel,v_rel,a_lead", [
  (18.5, 25.0, 4.44, -1.04),   # 07:55 class
  (22.0, 30.0, 2.0, -0.65),    # 22:14 class
  (30.0, 35.0, 1.13, -1.18),   # 23:31
])
def test_222_exemplars_unchanged_with_in_lane_wander(v_ego, d_rel, v_rel, a_lead):
  on, inputs, _ = _planner(v_ego, accel=-2.0)
  off, off_inputs, _ = _planner(v_ego, accel=-2.0)
  off._lead_leave.update = lambda **kw: 0.0
  la = _own_lead(inputs, d_rel, v_ego - v_rel, a_lead=a_lead)
  lb = _own_lead(off_inputs, d_rel, v_ego - v_rel, a_lead=a_lead)
  for i in range(60):
    y = 0.6 * math.sin(i * DT * 2.0)
    la.yRel = lb.yRel = y
    on.update(inputs)
    off.update(off_inputs)
    assert on.lead_leave_w == 0.0
    assert float(on.output_a_target) == pytest.approx(float(off.output_a_target), abs=1e-9)
    assert float(on.unified_a_target) == pytest.approx(float(off.unified_a_target), abs=1e-9)
