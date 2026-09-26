"""Continuous lead-follow controller.

Exemplar geometries are the Sep 23 #222 samples, the #241 inside-gap /
over-MAX shapes, the #240 early ease, cut-in, opening/depart, and the MAX
ceiling. The open-loop #240 comfort cap (never below −0.45 before 3 m of
slack) is not reproduced: that cap is what deferred braking into a late bite.
"""
import pathlib

import pytest

from openpilot.selfdrive.controls.lib.unified_lead import (
  A_MIN_MS2,
  UNIFIED_JERK_LIMIT_MS3,
  UnifiedLeadController,
  gap_set_m,
  unified_follow_desired,
)

DT = 0.05
MPH = 0.44704
ROOT = pathlib.Path(__file__).resolve().parents[3]


def _gap(slack, v_lead, t_follow):
  return gap_set_m(v_lead, t_follow) + slack


def _settle(n, **kw):
  ctrl = UnifiedLeadController()
  accel = 0.0
  for _ in range(n):
    accel = ctrl.step(dt=DT, present=True, seed_a=0.0, lead_id=1, **kw)
  return accel, ctrl


def test_glide_is_quiet_when_matched_at_the_setpoint():
  v = 25.0
  tf = 1.3
  gap = gap_set_m(v, tf)
  accel = unified_follow_desired(gap, v, v, 0.0, tf, v_ceiling=40.0)
  assert abs(accel) < 0.05
  settled, _ = _settle(40, gap=gap, v_ego=v, v_lead=v, a_lead=0.0, t_follow=tf, v_ceiling=40.0)
  assert abs(settled) < 0.05


def test_small_error_stays_near_mild_regen():
  """3 m inside, closing 1 m/s, lead not braking: near the EV's −0.22 settle."""
  v_lead = 25.0
  accel = unified_follow_desired(
    _gap(-3.0, v_lead, 1.3), 26.0, v_lead, 0.0, 1.3, v_ceiling=40.0,
  )
  assert -0.55 <= accel <= -0.15


def test_sep23_222_exemplars_match_a_braking_lead():
  """07:55, 22:14-class, and 23:31 stay firm. They do not fall to −0.22."""
  # 07:55:09 — closing ~1.8 m/s, aLead ~−0.8, a few meters outside the gap.
  v_ego = 66.3 * MPH
  v_lead = 62.3 * MPH
  early, _ = _settle(
    80, gap=_gap(7.0, v_lead, 1.3), v_ego=v_ego, v_lead=v_lead,
    a_lead=-0.79, t_follow=1.3, v_ceiling=40.0,
  )
  assert early <= -0.60
  assert early > -2.0

  # 07:55:13 — closing 4.44, aLead −1.04, ~2 m inside. Firm match, not full regen.
  v_ego = 62.5 * MPH
  v_lead = v_ego - 4.44
  late, _ = _settle(
    80, gap=25.2, v_ego=v_ego, v_lead=v_lead, a_lead=-1.04,
    t_follow=0.9, v_ceiling=40.0,
  )
  assert late <= -0.90
  assert late > -2.2

  # 23:31 — outside the gap, aLead ~−1.2, closing ~1.1. This is the
  # mutation target for dropping the a_lead term.
  v_ego = 30.0
  v_lead = v_ego - 1.13
  keep, _ = _settle(
    80, gap=35.0, v_ego=v_ego, v_lead=v_lead, a_lead=-1.18,
    t_follow=0.9, v_ceiling=40.0,
  )
  assert keep <= -0.95
  assert keep > -2.2

  # 22:14-class — aLead −0.65 is firm, −0.35 is not a step to full regen.
  mild_line, _ = _settle(
    80, gap=_gap(10.0, v_ego - 1.6, 0.9), v_ego=v_ego, v_lead=v_ego - 1.6,
    a_lead=-0.34, t_follow=0.9, v_ceiling=40.0,
  )
  firm, _ = _settle(
    80, gap=_gap(10.0, v_ego - 1.6, 0.9), v_ego=v_ego, v_lead=v_ego - 1.6,
    a_lead=-0.65, t_follow=0.9, v_ceiling=40.0,
  )
  assert firm <= -0.55
  assert firm < mild_line - 0.15
  assert abs(firm - mild_line) < 1.2


def test_inside_fd_0803_and_short_headway():
  """08:03 is about −0.40. Headway under 0.5 s reaches about −1."""
  accel, _ = _settle(
    60, gap=19.7, v_ego=33.5, v_lead=32.5, a_lead=0.0, t_follow=0.9, v_ceiling=40.0,
  )
  assert accel == pytest.approx(-0.40, abs=0.10)
  assert accel <= -0.35
  for closing in (1.1, 0.8, 0.5, 0.35):
    sample = unified_follow_desired(
      19.7, 33.5, 33.5 - closing, 0.0, 0.9, v_ceiling=40.0,
    )
    assert sample <= -0.35
  short = unified_follow_desired(8.0, 25.0, 23.0, 0.0, 0.9, v_ceiling=30.0)
  assert short <= -0.90
  assert short >= A_MIN_MS2


def test_closing_rate_around_1_5_does_not_cliff():
  published = {}
  for closing in (1.49, 1.50):
    published[closing] = unified_follow_desired(
      20.0, 33.0, 33.0 - closing, 0.0, 0.9, v_ceiling=33.0,
    )
    assert published[closing] <= -0.35
  assert abs(published[1.50] - published[1.49]) < 0.02


def test_firm_lead_handoff_does_not_step_to_mild():
  """08:07: aLead rising through −0.23 must not drop the command to −0.22."""
  v_ego = 30.0
  v_lead = 28.4
  ctrl = UnifiedLeadController()
  prev = 0.0
  for i in range(70):
    a_lead = -1.0 if i < 50 else -0.23
    accel = ctrl.step(
      dt=DT, present=True, gap=_gap(6.0, v_lead, 0.9), v_ego=v_ego, v_lead=v_lead,
      a_lead=a_lead, t_follow=0.9, lead_id=3, seed_a=-0.2, v_ceiling=40.0,
    )
    if i == 49:
      prev = accel
    if i == 50:
      assert accel < -0.80
      assert abs(accel - prev) <= UNIFIED_JERK_LIMIT_MS3 * DT + 1e-6
      assert accel < -0.50


def test_flat_approach_eases_early_and_physics_deepens_late():
  """#240: 4.1 m/s from 65 m reaches ≤ −0.26 before 30 m of slack.

  The old profile also refused to go below −0.45 before 3 m. This
  controller follows the stopping bound instead, so the open-loop trace
  (closing speed held constant) is deeper than −0.45 near the gap. It
  stays above full regen until the remaining gap is actually short.
  """
  v_rel = 4.1
  v_ego = 31.1
  v_lead = v_ego - v_rel
  slack = 65.0
  ctrl = UnifiedLeadController()
  saw = False
  deepest = 0.0
  while slack > 3.0:
    accel = ctrl.step(
      dt=DT, present=True, gap=_gap(slack, v_lead, 1.3), v_ego=v_ego,
      v_lead=v_lead, a_lead=0.0, t_follow=1.3, lead_id=7, seed_a=0.0, v_ceiling=40.0,
    )
    if slack > 30.0 and accel <= -0.26:
      saw = True
    deepest = min(deepest, accel)
    assert accel > A_MIN_MS2 + 0.05 or slack < 8.0
    slack -= v_rel * DT
  assert saw
  assert deepest < -0.45


def test_far_firm_lead_is_capped_near_one_and_near_gap_is_not():
  v_ego = 30.0
  v_lead = 26.0
  far, _ = _settle(
    80, gap=_gap(40.0, v_lead, 1.3), v_ego=v_ego, v_lead=v_lead,
    a_lead=-2.0, t_follow=1.3, v_ceiling=40.0,
  )
  assert -1.15 <= far <= -0.60
  v_ego = 36.0 * MPH
  v_lead = v_ego - 4.2
  near, _ = _settle(
    80, gap=_gap(12.0, v_lead, 0.7), v_ego=v_ego, v_lead=v_lead,
    a_lead=-2.06, t_follow=0.7, v_ceiling=40.0,
  )
  assert near < -1.5


def test_max_ceiling_blocks_a_faster_lead_and_over_max_brakes():
  """MAX is a hard ceiling. 08:10 is ego over a 70 mph MAX: the speed term brakes."""
  v_max = 60.0 * MPH
  faster = v_max + 3.0 * MPH
  held, _ = _settle(
    40, gap=_gap(40.0, faster, 1.3), v_ego=v_max, v_lead=faster,
    a_lead=0.0, t_follow=1.3, v_ceiling=v_max,
  )
  assert held <= 0.05
  # 08:10: 75 mph ego, 70 mph MAX, lead 2 m/s slower. Road-relative; grade is the plant.
  over = unified_follow_desired(
    _gap(40.0, 75.0 * MPH - 2.0, 0.9), 75.0 * MPH, 75.0 * MPH - 2.0, 0.0, 0.9,
    v_ceiling=70.0 * MPH,
  )
  assert over <= -0.35
  mapped = unified_follow_desired(
    _gap(40.0, 75.0 * MPH - 2.0, 0.9), 75.0 * MPH, 75.0 * MPH - 2.0, 0.0, 0.9,
    v_ceiling=70.0 * MPH, a_map=-0.80,
  )
  assert mapped <= -0.35
  assert mapped <= over + 1e-9


def test_curve_speed_ceiling_is_a_min():
  """A bend's comfort speed caps the command even when the lead is faster."""
  v_ego = 25.0
  v_lead = 28.0
  gap = _gap(15.0, v_lead, 1.3)
  open_road = unified_follow_desired(gap, v_ego, v_lead, 0.0, 1.3, v_ceiling=40.0)
  assert open_road > 0.0
  v_curve = 22.0
  capped = unified_follow_desired(gap, v_ego, v_lead, 0.0, 1.3, v_ceiling=v_curve)
  assert capped <= (v_curve - v_ego) / 2.4 + 1e-9
  assert capped < open_road


def test_opening_and_depart_release_without_a_one_frame_dump():
  v_ego = 20.0
  v_lead = 16.0
  ctrl = UnifiedLeadController()
  accel = 0.0
  for _ in range(30):
    accel = ctrl.step(
      dt=DT, present=True, gap=_gap(8.0, v_lead, 0.9), v_ego=v_ego, v_lead=v_lead,
      a_lead=-1.5, t_follow=0.9, lead_id=1, seed_a=0.0, v_ceiling=30.0,
    )
  assert accel < -0.8
  prev = accel
  for i in range(50):
    accel = ctrl.step(
      dt=DT, present=True, gap=_gap(9.0 + i * 0.15, v_ego + 0.8, 0.9),
      v_ego=v_ego, v_lead=v_ego + 0.8, a_lead=-0.30, t_follow=0.9, lead_id=1,
      seed_a=prev, v_ceiling=30.0,
    )
    assert abs(accel - prev) <= UNIFIED_JERK_LIMIT_MS3 * DT + 1e-6
    prev = accel
  assert accel > -0.45

  # On-path close stays firm. A straight depart fades. The same |y| in a
  # bend is the lane, so the command stays firm.
  v_ego = 16.0
  v_lead = 12.0
  on_path, _ = _settle(
    40, gap=_gap(12.0, v_lead, 0.7), v_ego=v_ego, v_lead=v_lead, a_lead=-2.06,
    t_follow=0.7, v_ceiling=30.0, y_rel=0.5,
  )
  assert on_path < -1.5
  d_rel = 45.0
  kappa = (2.0 * 1.6) / (d_rel * d_rel)
  in_bend = unified_follow_desired(
    d_rel, 20.0, 16.0, -1.2, 1.3, v_ceiling=30.0, y_rel=-2.6, curvature=kappa,
  )
  straight = unified_follow_desired(
    d_rel, 20.0, 16.0, -1.2, 1.3, v_ceiling=30.0, y_rel=-2.6, curvature=0.0,
  )
  assert in_bend < -0.6
  assert straight > in_bend + 0.4


def test_cut_in_and_radar_blip_stay_inside_the_jerk_limit():
  ctrl = UnifiedLeadController()
  prev = 0.4
  for _ in range(5):
    prev = ctrl.step(
      dt=DT, present=False, gap=0.0, v_ego=20.0, v_lead=20.0, a_lead=0.0,
      t_follow=1.3, lead_id=None, seed_a=prev,
    )
  accel = ctrl.step(
    dt=DT, present=True, gap=12.0, v_ego=22.0, v_lead=18.0, a_lead=-2.5,
    t_follow=1.3, lead_id=9, seed_a=0.4, v_ceiling=30.0,
  )
  assert abs(accel - 0.4) <= UNIFIED_JERK_LIMIT_MS3 * DT + 1e-6

  ctrl = UnifiedLeadController()
  for _ in range(30):
    prev = ctrl.step(
      dt=DT, present=True, gap=gap_set_m(20.0, 1.3), v_ego=20.0, v_lead=20.0,
      a_lead=0.0, t_follow=1.3, lead_id=1, seed_a=0.0, v_ceiling=30.0,
    )
  blip = ctrl.step(
    dt=DT, present=True, gap=gap_set_m(20.0, 1.3), v_ego=20.0, v_lead=20.0,
    a_lead=-3.5, t_follow=1.3, lead_id=1, seed_a=prev, v_ceiling=30.0,
  )
  assert abs(blip - prev) <= UNIFIED_JERK_LIMIT_MS3 * DT + 1e-6
  assert blip > -1.0


def test_stateful_sweep_has_no_jerk_cliff():
  """Closing 0–4 m/s and gap setpoint ±30 m, one sample per frame."""
  limit = UNIFIED_JERK_LIMIT_MS3 * DT + 1e-9
  v_lead = 25.0
  tf = 1.3
  gap_set = gap_set_m(v_lead, tf)
  ctrl = UnifiedLeadController()
  prev = 0.0
  for closing in (i * 0.1 for i in range(41)):
    for slack in (i - 30.0 for i in range(61)):
      accel = ctrl.step(
        dt=DT, present=True, gap=gap_set + slack, v_ego=v_lead + closing,
        v_lead=v_lead, a_lead=0.0, t_follow=tf, lead_id=1, seed_a=prev, v_ceiling=40.0,
      )
      assert abs(accel - prev) <= limit
      prev = accel


def test_desired_accel_is_continuous_for_small_input_steps():
  v_lead = 25.0
  tf = 1.3
  gap_set = gap_set_m(v_lead, tf)
  worst = 0.0
  for closing in (i * 0.05 for i in range(0, 81)):
    for slack in (i * 0.25 - 30.0 for i in range(241)):
      accel = unified_follow_desired(
        gap_set + slack, v_lead + closing, v_lead, 0.0, tf, v_ceiling=40.0,
      )
      nxt = unified_follow_desired(
        gap_set + slack + 0.25, v_lead + closing, v_lead, 0.0, tf, v_ceiling=40.0,
      )
      worst = max(worst, abs(nxt - accel))
  assert worst < 0.85


def test_arrives_at_the_setpoint_already_speed_matched():
  v_lead = 20.0
  tf = 1.3
  gap_set = gap_set_m(v_lead, tf)
  v_ego = 22.5
  gap = gap_set + 28.0
  ctrl = UnifiedLeadController()
  for _ in range(int(55.0 / DT)):
    accel = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
      t_follow=tf, lead_id=1, seed_a=0.0, v_ceiling=35.0,
    )
    v_ego = max(0.0, v_ego + accel * DT)
    gap += (v_lead - v_ego) * DT
  assert abs(gap - gap_set) < 3.0
  assert abs(v_ego - v_lead) < 0.35

  v_ego = v_lead + 1.2
  gap = gap_set - 10.0
  ctrl = UnifiedLeadController()
  for _ in range(int(50.0 / DT)):
    accel = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=v_ego, v_lead=v_lead, a_lead=0.0,
      t_follow=tf, lead_id=2, seed_a=0.0, v_ceiling=35.0,
    )
    v_ego = max(0.0, v_ego + accel * DT)
    gap += (v_lead - v_ego) * DT
    assert gap > 4.0
  assert abs(gap - gap_set) < 4.0
  assert abs(v_ego - v_lead) < 0.45


def test_ui_and_param_default_off():
  keys = (ROOT / "common" / "params_keys.h").read_text()
  assert '{"NAPLongUnified", {PERSISTENT, BOOL, "0"}}' in keys
  nap = (ROOT / "selfdrive/ui/layouts/settings/nap.py").read_text()
  mici = (ROOT / "selfdrive/ui/mici/layouts/settings/nap.py").read_text()
  content = (ROOT / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  assert "NAP_LONG_UNIFIED" in nap
  assert "Unified Lead Follow" in nap
  assert 'section_header_item("Longitudinal Control")' in nap
  assert nap.index("Unified Lead Follow") > nap.index("Longitudinal Control")
  assert nap.index("Unified Lead Follow") < nap.index("Driving Mannerisms")
  assert "unified lead follow" in mici
  assert "NAP_LONG_UNIFIED" in mici
  assert "UNIFIED_LEAD_DESCRIPTION" in content
  planner = (ROOT / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  assert "unifiedATarget" in planner
  assert "NAP_LONG_UNIFIED_GATE" in planner
  assert planner.index("self.output_a_target = np.clip") < planner.index("self._apply_unified_lead")
