"""Far-gap soft closing speed inside unified_follow_desired.

At a large gap with nearly matched speed, k_g·slack asks for +1.2…+1.8 m/s²
and the kinematic gate bangs that into catch-up pulses. The classic gap term
fades from 15 → 50 m of slack and the faded portion becomes a desired closing
speed of about 0.10–0.35 m/s. The blend is off for a near gap, a braking PD
term, and a kinematic bound that is already a real brake. The shipped gate
widen is unchanged.
"""
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import unified_lead as ul
from openpilot.selfdrive.controls.lib.unified_lead import (
  GAP_CATCH_A_BOUND_LO,
  GAP_CATCH_A_BOUND_RAMP,
  GAP_CATCH_A_ON,
  GAP_FADE_HI_M,
  GAP_FADE_LO_M,
  K_CATCH,
  V_DES_CLOSE_LO,
  V_DES_CLOSE_RAMP_M,
  V_DES_CLOSE_SPAN,
  UnifiedLeadController,
  _far_gap_catchup_gap,
  gap_set_m,
  unified_follow_desired,
)

DT = 0.05


def _baseline(monkeypatch):
  monkeypatch.setattr(ul, "_far_gap_catchup_gap", lambda a_gap_classic, *a, **k: a_gap_classic)


def _cmd(slack, v_close, vl=28.0, al=0.0, tf=0.7):
  gap = gap_set_m(vl, tf) + slack
  return unified_follow_desired(gap, max(0.0, vl + v_close), vl, al, tf)


def _rising_crossings(cmds, level):
  return sum(1 for prev, cur in zip(cmds, cmds[1:], strict=False) if prev < level <= cur)


def test_constants_match_the_approved_closing_speed():
  assert GAP_FADE_LO_M == 15.0
  assert GAP_FADE_HI_M == 50.0
  assert V_DES_CLOSE_LO == 0.10
  assert V_DES_CLOSE_SPAN == 0.25
  assert V_DES_CLOSE_RAMP_M == 30.0
  assert K_CATCH == 0.20
  assert GAP_CATCH_A_ON == 0.15
  assert GAP_CATCH_A_BOUND_LO == -0.10
  assert GAP_CATCH_A_BOUND_RAMP == 0.10


def test_slack_weight_keeps_the_near_gap_term():
  assert _far_gap_catchup_gap(1.2, 10.0, 0.0, 1.0, 1.0, 0.0) == 1.2


def test_accel_weight_keeps_a_braking_pd_term():
  """A non-positive classic PD is not a catch-up, even at a far slack."""
  assert _far_gap_catchup_gap(1.2, 50.0, 0.0, 1.0, -0.2, 0.0) == 1.2


def test_bound_weight_keeps_the_gap_term_on_a_real_brake():
  assert _far_gap_catchup_gap(1.2, 50.0, 0.0, 1.0, 1.0, -1.0) == 1.2


def test_matched_far_command_is_a_soft_ease():
  """Matched speed past the fade: about +0.07, never a highway pull.

  Slack 40 m is still inside the fade, so a little of the classic gap term
  remains. It stays well under the old slam. At 50 m and beyond the fade is
  done.
  """
  assert 0.0 <= _cmd(40.0, 0.0) < 0.25
  for slack in (50.0, 60.0, 80.0):
    cmd = _cmd(slack, 0.0)
    assert 0.0 <= cmd <= 0.15, (slack, cmd)
  # Trickle has faded out; the command is K_CATCH * 0.35.
  assert _cmd(60.0, 0.0) == pytest.approx(0.07, abs=1e-6)
  assert _cmd(80.0, 0.0) == pytest.approx(0.07, abs=1e-6)


def test_follow7_recovery_comes_back_inside_the_maneuver_ceiling():
  """Close lead, Follow 7, 60 s. The real maneuver must finish ≤ 83.5 m.

  This plant is the unified controller plus the cruise accel cap, and it
  tracks that maneuver about 1 m closer. Ending the fade at 40 m finished
  here at 85 m (86.4 m on the real plant). The fade has to leave enough
  catch-up in the 30 m slack band to come back.
  """
  v_lead = 25.0
  t_follow = 1.9
  ctrl = UnifiedLeadController()
  v_ego = v_lead
  d_ego = 0.0
  d_lead = 20.0
  for _ in range(int(60.0 / DT)):
    gap = max(0.0, d_lead - d_ego)
    a_max = float(np.interp(v_ego, [0.0, 10.0, 25.0, 40.0], [1.6, 1.2, 0.8, 0.6]))
    accel = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
      t_follow=t_follow, lead_id=1, seed_a=0.0, v_ceiling=50.0, a_max=a_max,
    )
    v_ego = max(0.0, v_ego + accel * DT)
    d_lead += v_lead * DT
    d_ego += v_ego * DT
  gap = d_lead - d_ego
  assert gap >= 53.0
  assert gap < 82.0
  assert v_ego == pytest.approx(v_lead, abs=0.5)


def test_e2_shaped_open_loop_does_not_pulse(monkeypatch):
  """Closing speed crossing the old 0.4 m/s gate every ~3 s, slack 50 m.

  Classic gap attraction rises through +0.35 ten times. The soft law does not.
  """
  def trace():
    cmds = []
    for i in range(600):
      t = i * DT
      v_close = 0.25 + 0.30 * math.sin(2.0 * math.pi * t / 3.0)
      cmds.append(_cmd(50.0, v_close))
    return cmds

  soft = trace()
  assert _rising_crossings(soft, 0.35) <= 2
  assert max(soft) <= 0.15
  _baseline(monkeypatch)
  classic = trace()
  assert _rising_crossings(classic, 0.35) >= 8


def test_matched_far_closed_loop_does_not_brake_harder_than_classic(monkeypatch):
  """Constant lead, start matched 55 m behind. Soft ease, no deeper brake."""
  def rollout():
    v = 28.0
    tf = 0.7
    v_ego = v
    gap = gap_set_m(v, tf) + 55.0
    a = 0.0
    cmds, acc, gaps = [], [], []
    for _ in range(int(40.0 / DT)):
      cmd = unified_follow_desired(max(0.0, gap), v_ego, v, 0.0, tf)
      a += (cmd - a) * DT / 0.45
      v_ego = max(0.0, v_ego + a * DT)
      gap += (v - v_ego) * DT
      cmds.append(cmd)
      acc.append(a)
      gaps.append(gap)
    return cmds, acc, gaps

  soft_cmds, soft_a, soft_gaps = rollout()
  _baseline(monkeypatch)
  classic_cmds, classic_a, classic_gaps = rollout()
  assert max(soft_cmds) <= 0.15
  assert max(classic_cmds) > 0.5
  assert min(soft_a) >= min(classic_a) - 1e-9
  assert min(soft_gaps) >= min(classic_gaps) - 1e-6
  assert max(soft_cmds) - min(soft_cmds) < 0.35


def test_closing_faster_than_desired_adds_no_gap_brake():
  """a_soft is negative once we already close faster than v_des; clip it at 0."""
  assert _far_gap_catchup_gap(1.5, 60.0, -1.0, 1.0, 1.0, 0.0) == 0.0
  cmd = _cmd(60.0, 1.0)
  assert -0.03 < cmd < 0.0


def test_braking_lead_at_a_far_gap_keeps_the_classic_command(monkeypatch):
  """Lead brake makes a real bound. The gap rewrite stays off, so no new brake."""
  vl, tf, slack, al = 25.0, 1.3, 60.0, -1.0
  gap = gap_set_m(vl, tf) + slack
  new = unified_follow_desired(gap, vl, vl, al, tf)
  _baseline(monkeypatch)
  old = unified_follow_desired(gap, vl, vl, al, tf)
  assert new == old
  assert new > 0.5


def test_bit_identical_for_slack_at_or_under_15m(monkeypatch):
  rng = np.random.default_rng(15)
  cases = []
  for _ in range(20000):
    vl = float(rng.uniform(0.0, 40.0))
    tf = float(rng.choice([0.7, 0.9, 1.1, 1.3, 1.5, 1.8]))
    slack = float(rng.uniform(-8.0, GAP_FADE_LO_M))
    v_close = float(rng.uniform(-2.0, 6.0))
    al = float(rng.uniform(-2.0, 1.0))
    gap = max(0.0, gap_set_m(vl, tf) + slack)
    cases.append((gap, max(0.0, vl + v_close), vl, al, tf))
  new = [unified_follow_desired(*c) for c in cases]
  _baseline(monkeypatch)
  old = [unified_follow_desired(*c) for c in cases]
  assert new == old


def test_never_firmer_on_catchup_and_never_deeper_when_classic_brakes(monkeypatch):
  rng = np.random.default_rng(4)
  cases = []
  for _ in range(40000):
    vl = float(rng.uniform(0.0, 40.0))
    tf = float(rng.choice([0.7, 0.9, 1.1, 1.3, 1.5, 1.8]))
    slack = float(rng.uniform(-8.0, 90.0))
    v_close = float(rng.uniform(-2.0, 8.0))
    al = float(rng.uniform(-2.5, 1.0))
    gap = max(0.0, gap_set_m(vl, tf) + slack)
    cases.append((gap, max(0.0, vl + v_close), vl, al, tf, slack))
  new = [unified_follow_desired(*c[:5]) for c in cases]
  _baseline(monkeypatch)
  old = [unified_follow_desired(*c[:5]) for c in cases]
  changed = 0
  for n, o, c in zip(new, old, cases, strict=True):
    assert n <= o + 1e-9, c          # never a firmer catch-up
    if o < -0.10:
      assert n >= o - 1e-9, c        # a classic brake is not deepened
    if c[5] <= GAP_FADE_LO_M:
      assert n == o, c
    if n != o:
      changed += 1
  assert changed > 100


def test_catchup_is_continuous_in_slack_and_closing_speed():
  slacks = np.linspace(-5.0, 80.0, 8501)
  ys = np.array([_cmd(float(s), 0.0) for s in slacks])
  assert np.abs(np.diff(ys)).max() < 0.01
  closes = np.linspace(-2.0, 4.0, 6001)
  for slack in (20.0, 50.0):
    ys = np.array([_cmd(slack, float(v)) for v in closes])
    assert np.abs(np.diff(ys)).max() < 0.02
    assert np.all(np.isfinite(ys))
