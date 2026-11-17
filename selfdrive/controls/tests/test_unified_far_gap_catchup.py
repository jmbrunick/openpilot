"""Far-gap soft closing speed inside unified_follow_desired.

At a large gap with nearly matched speed, k_g·slack asks for +1.2…+1.8 m/s²
and the kinematic gate bangs that into catch-up pulses. The classic gap term
fades from 15 → 50 m of slack and the faded portion becomes a desired closing
speed of about 0.10–0.95 m/s. The gain is low enough that a matched gap still
commands under 0.15 m/s². The blend is off for a near gap, a braking PD
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
  assert V_DES_CLOSE_SPAN == 0.85
  assert V_DES_CLOSE_RAMP_M == 30.0
  assert K_CATCH == 0.14
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
  """Matched speed past the fade: about +0.13, never a highway pull.

  Slack 40 m is still inside the fade, so a little of the classic gap term
  remains. It stays well under the old slam. At 50 m and beyond the fade is
  done.
  """
  assert 0.0 <= _cmd(40.0, 0.0) < 0.25
  for slack in (50.0, 60.0, 80.0):
    cmd = _cmd(slack, 0.0)
    assert 0.0 <= cmd <= 0.15, (slack, cmd)
  # Trickle has faded out; the command is K_CATCH * 0.95.
  assert _cmd(60.0, 0.0) == pytest.approx(0.133, abs=1e-6)
  assert _cmd(80.0, 0.0) == pytest.approx(0.133, abs=1e-6)


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


def test_follow_settings_land_inside_the_240s_band():
  """Same-speed close from 100 m, 240 s, every Follow time.

  The Pre-AP maneuver requires the final gap within ±1.5 m of
  t_follow·v + 6. This plant tracks that maneuver closely. A desired
  close of 0.35 m/s left Follow 2 about 1.6 m inside the set gap at
  240 s, on the trough of the settle. The higher desired close has to
  put every setting inside a tighter band here, and the Follow 7
  recovery above still has to finish under 82 m.
  """
  v_lead = 25.0
  follow_times = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)
  gaps = []
  for t_follow in follow_times:
    ctrl = UnifiedLeadController()
    v_ego = v_lead
    d_ego = 0.0
    d_lead = 100.0
    for _ in range(int(240.0 / DT)):
      gap = max(0.0, d_lead - d_ego)
      a_max = float(np.interp(v_ego, [0.0, 10.0, 25.0, 40.0], [1.6, 1.2, 0.8, 0.6]))
      accel = ctrl.step(
        dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
        t_follow=t_follow, lead_id=1, seed_a=0.0, v_ceiling=50.0, a_max=a_max,
      )
      v_ego = max(0.0, v_ego + accel * DT)
      d_lead += v_lead * DT
      d_ego += v_ego * DT
    gaps.append(d_lead - d_ego)
  expected = [t_follow * v_lead + 6.0 for t_follow in follow_times]
  assert gaps == pytest.approx(expected, abs=1.40)
  assert np.all(np.diff(gaps) > 0.0)


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


def test_braking_lead_at_a_far_gap_gets_no_catchup_push(monkeypatch):
  """Nov 17: a lead braking 60 m past the gap is not chased.

  The bound weight used to hand the command back to k_g·slack whenever the
  lead's accel read negative, so a matched car 60 m back was told +1.8 m/s²
  toward a lead braking at -1.0. The planned-approach ceiling holds it to the
  soft closing law. It removes accel only: no brake is added.
  """
  vl, tf, slack, al = 25.0, 1.3, 60.0, -1.0
  gap = gap_set_m(vl, tf) + slack
  new = unified_follow_desired(gap, vl, vl, al, tf)
  _baseline(monkeypatch)
  old = unified_follow_desired(gap, vl, vl, al, tf)
  assert old > 0.5  # the shipped push
  assert 0.0 <= new <= K_CATCH * (V_DES_CLOSE_LO + V_DES_CLOSE_SPAN) + 1e-6


def test_slowing_lead_never_gets_a_harder_catchup_than_a_steady_one():
  """A far lead easing off (aLeadK -0.05…-0.5) is chased no harder than the same lead holding speed."""
  rng = np.random.default_rng(1117)
  for _ in range(4000):
    vl = float(rng.uniform(8.0, 35.0))
    tf = float(rng.choice([0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]))
    slack = float(rng.uniform(GAP_FADE_HI_M, 120.0))
    v_open = float(rng.uniform(-0.4, 2.0))  # matched to opening
    al = float(rng.uniform(-0.5, -0.05))
    gap = gap_set_m(vl, tf) + slack
    v_e = max(0.0, vl - v_open)
    steady = unified_follow_desired(gap, v_e, vl, 0.0, tf)
    slowing = unified_follow_desired(gap, v_e, vl, al, tf)
    assert slowing <= steady + 1e-9, (gap, v_e, vl, al, tf)


def test_planned_approach_ceiling_only_removes_accel(monkeypatch):
  """The rework can only lower a positive command, never below 0; a braking command is bit-identical."""
  rng = np.random.default_rng(17)
  cases = []
  for _ in range(40000):
    vl = float(rng.uniform(0.0, 40.0))
    tf = float(rng.choice([0.7, 0.9, 1.1, 1.3, 1.5, 1.8]))
    slack = float(rng.uniform(-8.0, 120.0))
    v_close = float(rng.uniform(-3.0, 8.0))
    al = float(rng.uniform(-3.0, 1.0))
    gap = max(0.0, gap_set_m(vl, tf) + slack)
    cases.append((gap, max(0.0, vl + v_close), vl, al, tf))
  new = [unified_follow_desired(*c) for c in cases]
  monkeypatch.setattr(ul, "CATCHUP_PLAN_A_BOUND", -1.0e9)  # plan law = shipped law: ceiling is a no-op
  old = [unified_follow_desired(*c) for c in cases]
  lowered = 0
  for n, o, c in zip(new, old, cases, strict=True):
    if o <= 0.0:
      assert n == o, c
    else:
      assert 0.0 <= n <= o + 1e-12, c
      lowered += n < o - 0.05
  assert lowered > 100


# Nov 17 2026 13:14:11-17 CST, route 00000008 seg 5, nap-alpha ef0c30ca, Follow
# Hwy 1 (0.70 s): lead ~75 m ahead at ~25.3 m/s with aLeadK drifting from 0
# to -0.2. The far catch-up pushed +0.30 → +0.80 and dropped to ~0 in 0.3 s
# (2.5 m/s³) as ego passed the lead's speed.
NOV17_1314 = (  # (dRel, vEgo, vLead, aLeadK, logged aTarget) 13:14:11.00-17.00 CST, 20 Hz
  (73.62, 24.555, 25.491, 0.003, 0.34),
  (73.69, 24.549, 25.43, 0.003, 0.342),
  (73.75, 24.557, 25.424, 0.007, 0.338),
  (73.75, 24.568, 25.434, 0.012, 0.338),
  (73.81, 24.564, 25.443, 0.019, 0.34),
  (73.88, 24.566, 25.501, 0.041, 0.348),
  (73.88, 24.543, 25.501, 0.041, 0.358),
  (73.94, 24.55, 25.418, 0.034, 0.357),
  (74.0, 24.547, 25.488, 0.048, 0.363),
  (74.0, 24.561, 25.485, 0.058, 0.364),
  (74.06, 24.562, 25.561, 0.087, 0.375),
  (74.06, 24.564, 25.561, 0.087, 0.38),
  (74.12, 24.57, 25.564, 0.11, 0.39),
  (74.12, 24.571, 25.564, 0.11, 0.394),
  (74.19, 24.597, 25.508, 0.107, 0.393),
  (74.19, 24.614, 25.534, 0.111, 0.39),
  (74.19, 24.635, 25.534, 0.111, 0.387),
  (74.31, 24.658, 25.51, 0.105, 0.388),
  (74.31, 24.669, 25.51, 0.105, 0.38),
  (74.44, 24.677, 25.544, 0.106, 0.383),
  (74.5, 24.683, 25.552, 0.106, 0.381),
  (74.5, 24.716, 25.552, 0.106, 0.375),
  (74.62, 24.719, 25.529, 0.106, 0.374),
  (74.62, 24.753, 25.529, 0.106, 0.368),
  (74.62, 24.739, 25.566, 0.111, 0.371),
  (74.69, 24.773, 25.551, 0.109, 0.364),
  (74.75, 24.768, 25.586, 0.115, 0.371),
  (74.81, 24.764, 25.578, 0.115, 0.372),
  (74.81, 24.773, 25.578, 0.115, 0.374),
  (74.81, 24.775, 25.523, 0.098, 0.369),
  (74.88, 24.803, 25.462, 0.066, 0.355),
  (74.88, 24.803, 25.462, 0.066, 0.349),
  (74.94, 24.826, 25.425, 0.029, 0.331),
  (74.94, 24.833, 25.425, 0.029, 0.322),
  (74.94, 24.829, 25.458, 0.009, 0.313),
  (75.0, 24.844, 25.391, 0.009, 0.304),
  (75.0, 24.844, 25.407, -0.021, 0.292),
  (75.0, 24.869, 25.406, -0.045, 0.279),
  (75.0, 24.853, 25.406, -0.045, 0.28),
  (75.06, 24.875, 25.415, -0.061, 0.301),
  (75.06, 24.915, 25.438, -0.061, 0.33),
  (75.06, 24.894, 25.415, -0.072, 0.377),
  (75.06, 24.927, 25.457, -0.068, 0.411),
  (75.06, 24.918, 25.457, -0.068, 0.441),
  (75.06, 24.915, 25.48, -0.057, 0.452),
  (75.12, 24.895, 25.477, -0.048, 0.448),
  (75.19, 24.895, 25.456, -0.063, 0.466),
  (75.19, 24.895, 25.456, -0.063, 0.48),
  (75.19, 24.918, 25.456, -0.063, 0.483),
  (75.25, 24.939, 25.48, -0.049, 0.466),
  (75.25, 24.931, 25.439, -0.049, 0.454),
  (75.31, 24.948, 25.431, -0.051, 0.446),
  (75.38, 24.942, 25.385, -0.065, 0.463),
  (75.38, 24.985, 25.385, -0.065, 0.465),
  (75.38, 25.008, 25.36, -0.081, 0.496),
  (75.38, 25.039, 25.383, -0.086, 0.525),
  (75.5, 25.054, 25.298, -0.12, 0.602),
  (75.5, 25.058, 25.298, -0.12, 0.678),
  (75.5, 25.092, 25.298, -0.12, 0.74),
  (75.5, 25.114, 25.342, -0.125, 0.768),
  (75.5, 25.164, 25.301, -0.137, 0.793),
  (75.5, 25.182, 25.301, -0.137, 0.798),
  (75.44, 25.213, 25.307, -0.142, 0.797),
  (75.5, 25.235, 25.269, -0.142, 0.797),
  (75.56, 25.256, 25.297, -0.146, 0.797),
  (75.56, 25.308, 25.256, -0.146, 0.796),
  (75.56, 25.349, 25.256, -0.146, 0.796),
  (75.56, 25.391, 25.271, -0.152, 0.795),
  (75.56, 25.439, 25.266, -0.156, 0.794),
  (75.44, 25.501, 25.189, -0.178, 0.669),
  (75.44, 25.565, 25.189, -0.178, 0.544),
  (75.44, 25.573, 25.189, -0.178, 0.419),
  (75.38, 25.62, 25.198, -0.189, 0.294),
  (75.38, 25.646, 25.183, -0.199, 0.169),
  (75.31, 25.717, 25.208, -0.196, 0.044),
  (75.25, 25.787, 25.217, -0.196, -0.006),
  (75.25, 25.845, 25.217, -0.196, -0.006),
  (75.25, 25.87, 25.269, -0.173, -0.007),
  (75.19, 25.877, 25.308, -0.173, -0.007),
  (75.12, 25.888, 25.189, -0.174, -0.007),
  (75.06, 25.894, 25.138, -0.185, -0.008),
  (75.06, 25.867, 25.138, -0.185, -0.008),
  (75.0, 25.866, 25.182, -0.179, -0.008),
  (74.94, 25.835, 25.116, -0.189, -0.007),
  (74.88, 25.825, 25.085, -0.203, -0.008),
  (74.88, 25.809, 25.013, -0.23, -0.008),
  (74.88, 25.8, 25.013, -0.23, -0.008),
  (74.75, 25.813, 24.987, -0.257, -0.009),
  (74.69, 25.795, 24.938, -0.286, -0.01),
  (74.62, 25.775, 24.983, -0.295, -0.01),
  (74.62, 25.737, 24.983, -0.295, -0.01),
  (74.62, 25.719, 24.737, -0.363, -0.011),
  (74.56, 25.717, 24.594, -0.452, -0.013),
  (74.5, 25.712, 24.592, -0.515, -0.015),
  (74.44, 25.728, 24.65, -0.544, -0.017),
  (74.38, 25.719, 24.79, -0.514, -0.018),
  (74.38, 25.693, 24.79, -0.514, -0.018),
  (74.31, 25.683, 24.764, -0.491, -0.018),
  (74.25, 25.659, 24.746, -0.467, -0.018),
  (74.19, 25.658, 24.721, -0.45, -0.019),
  (74.12, 25.648, 24.72, -0.425, -0.018),
  (74.12, 25.642, 24.72, -0.425, -0.019),
  (74.12, 25.626, 24.767, -0.386, -0.017),
  (74.06, 25.601, 24.75, -0.352, -0.017),
  (74.06, 25.605, 24.73, -0.326, -0.016),
  (74.0, 25.598, 24.855, -0.262, -0.015),
  (74.0, 25.608, 24.855, -0.262, -0.014),
  (74.0, 25.601, 24.796, -0.223, -0.014),
  (73.94, 25.583, 24.789, -0.19, -0.013),
  (73.88, 25.578, 24.77, -0.166, -0.012),
  (73.88, 25.565, 24.701, -0.163, -0.012),
  (73.88, 25.574, 24.701, -0.163, -0.012),
  (73.81, 25.573, 24.636, -0.176, -0.012),
  (73.75, 25.574, 24.698, -0.165, -0.012),
  (73.69, 25.568, 24.637, -0.172, -0.013),
  (73.56, 25.555, 24.63, -0.175, -0.013),
  (73.56, 25.57, 24.63, -0.175, -0.013),
  (73.44, 25.576, 24.569, -0.193, -0.014),
  (73.44, 25.595, 24.569, -0.193, -0.015),
  (73.38, 25.597, 24.595, -0.192, -0.015),
)


def _replay_nov17():
  ctrl = UnifiedLeadController()
  out = []
  for d, v, vl, al, _log in NOV17_1314:
    a_max = float(np.interp(v, [0.0, 10.0, 25.0, 40.0], [1.6, 1.2, 0.8, 0.6]))
    out.append(ctrl.step(dt=DT, present=True, gap=d, v_ego=v, v_lead=vl, a_lead=al, t_follow=0.7,
                         lead_id=1, seed_a=0.34, v_ceiling=104.6 / 3.6, a_max=a_max))
  return np.array(out)


def test_nov17_far_catchup_is_a_taper_not_a_push_and_drop():
  cmds = _replay_nov17()
  logged = np.array([r[4] for r in NOV17_1314])
  assert logged.max() > 0.75  # the event is in the fixture
  assert cmds.max() <= 0.40
  assert np.abs(np.diff(cmds)).max() / DT <= 0.75  # comfort-scale release, was 2.5 m/s³
  # Once the lead reads as slowing, the command does not climb again.
  k = next(i for i, r in enumerate(NOV17_1314) if r[3] <= -0.05)
  assert np.all(np.diff(cmds[k:]) <= 0.02)


def test_nov17_braking_after_the_event_is_unchanged(monkeypatch):
  """Where the shipped command already braked, the rework is bit-identical."""
  new = _replay_nov17()
  monkeypatch.setattr(ul, "CATCHUP_PLAN_A_BOUND", -1.0e9)
  old = _replay_nov17()
  assert old.max() > 0.75
  first_brake = int(np.argmax(old < 0.0))
  assert first_brake > 0
  tail = slice(first_brake + 40, None)  # slew state converged
  assert np.allclose(new[tail], old[tail], atol=0.02)


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
