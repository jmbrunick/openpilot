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
  kinematic_required_accel,
  speed_ceiling_accel,
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


def test_far_firm_lead_scales_with_physics_not_a_far_cap():
  """No far comfort cap: depth comes from the kinematic and braking-lead terms.

  The old −1.0 far cap (and the −0.55 soft cap until time-to-gap < 3.2 s)
  deferred braking on rapid closes (Sep 25 10:07:59). A hard-braking lead
  40 m out is now firm, and a gentle close at the same distance stays mild.
  """
  v_ego = 30.0
  v_lead = 26.0
  far, _ = _settle(
    80, gap=_gap(40.0, v_lead, 1.3), v_ego=v_ego, v_lead=v_lead,
    a_lead=-2.0, t_follow=1.3, v_ceiling=40.0,
  )
  assert -2.5 <= far <= -0.60
  gentle, _ = _settle(
    80, gap=_gap(40.0, v_ego - 1.0, 1.3), v_ego=v_ego, v_lead=v_ego - 1.0,
    a_lead=0.0, t_follow=1.3, v_ceiling=40.0,
  )
  assert gentle > -0.35
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
  """A bend's comfort speed caps the command even when the lead is faster.

  Above the cap the speed-error term brakes in proportion to overspeed
  (starting at the EV mild settle), not a clamp toward full regen.
  """
  v_ego = 25.0
  v_lead = 28.0
  gap = _gap(15.0, v_lead, 1.3)
  open_road = unified_follow_desired(gap, v_ego, v_lead, 0.0, 1.3, v_ceiling=40.0)
  assert open_road > 0.0
  v_curve = 22.0
  capped = unified_follow_desired(gap, v_ego, v_lead, 0.0, 1.3, v_ceiling=v_curve)
  assert capped <= -0.22
  assert capped >= -1.2
  assert capped < open_road
  assert capped == pytest.approx(speed_ceiling_accel(v_ego, v_curve), abs=1e-9)


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
  main = nap.split("def _build_items", 1)[1].split("def _add_toggle", 1)[0]
  assert main.index("Unified Lead Follow") < main.index('section_header_item("Longitudinal Control")')
  assert main.index("Unified Lead Follow") < main.index("Pedal Interceptor")
  assert main.index("Unified Lead Follow") < main.index("Driving Mannerisms")
  nap_mici = mici.split("class NAPLayoutMici", 1)[1]
  widgets = nap_mici.split("self._scroller.add_widgets", 1)[1]
  assert widgets.index("unified_lead") < widgets.index("pedal_enabled")
  assert "NAP_LONG_UNIFIED" in mici
  assert "UNIFIED_LEAD_DESCRIPTION" in content
  planner = (ROOT / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  assert "unifiedATarget" in planner
  assert "NAP_LONG_UNIFIED_GATE" in planner
  assert planner.index("self.output_a_target = np.clip") < planner.index("self._apply_unified_lead")


# ---- Sep 25 2026 drives (Scallywag qlogs, routes 11a/11b/11c) ----------------

# 09:21:42.00–09:21:47.75 CT at 0.25 s: dRel, vEgo, vRel, aLeadK (t_follow 0.7).
SEP25_0921 = (
  (39.2, 29.94, -4.81, -0.26), (37.9, 29.87, -4.81, -0.24), (36.8, 29.81, -4.81, -0.26),
  (35.4, 29.76, -4.81, -0.25), (34.3, 29.68, -4.83, -0.25), (33.1, 29.61, -4.88, -0.28),
  (32.0, 29.53, -4.88, -0.31), (30.7, 29.45, -4.88, -0.33), (29.3, 29.39, -4.85, -0.33),
  (28.2, 29.32, -4.79, -0.30), (27.2, 29.23, -4.71, -0.23), (26.0, 29.14, -4.60, -0.15),
  (24.8, 29.06, -4.52, -0.09), (23.6, 28.98, -4.42, 0.00), (22.7, 28.87, -4.31, 0.01),
  (21.5, 28.78, -4.14, 0.07), (20.3, 28.65, -3.98, 0.06), (19.6, 28.43, -3.73, 0.11),
  (18.7, 28.14, -3.41, 0.12), (18.0, 27.81, -3.01, 0.16), (17.3, 27.45, -2.68, 0.18),
  (16.6, 27.10, -2.29, 0.13), (16.0, 26.75, -1.91, 0.12), (15.6, 26.42, -1.62, 0.11),
)


def _interp_rows(rows, step_s=0.25):
  n = int(round(step_s / DT))
  out = []
  for a, b in zip(rows, rows[1:], strict=False):
    for k in range(n):
      f = k / n
      out.append(tuple(x + (y - x) * f for x, y in zip(a, b, strict=True)))
  out.append(rows[-1])
  return out


def _closed_loop(ctrl, *, d, v_ego, lead_v, lead_a, t_follow, seconds, clamp=-1.5,
                 lag_s=0.3, seed=0.0, v_ceiling=30.0, a0=0.0):
  """Road plant with a first-order actuator lag and the Pre-AP decel limit."""
  a_act = a0
  min_gap = d
  min_ttc = 99.0
  cmds = []
  for k in range(int(round(seconds / DT))):
    t = k * DT
    v_l = lead_v(t)
    cmd = ctrl.step(
      dt=DT, present=True, gap=d, v_ego=v_ego, v_lead=v_l, a_lead=lead_a(t),
      t_follow=t_follow, lead_id=1, seed_a=seed, v_ceiling=v_ceiling,
      radar=True, model_prob=0.9,
    )
    cmds.append(cmd)
    a_act += (max(cmd, clamp) - a_act) * DT / lag_s
    v_ego = max(0.0, v_ego + a_act * DT)
    d += (v_l - v_ego) * DT
    min_gap = min(min_gap, d)
    if v_ego > v_l + 1e-3:
      min_ttc = min(min_ttc, d / (v_ego - v_l))
  return min_gap, min_ttc, cmds


def test_sep25_1007_rapid_close_is_at_least_legacy_firm():
  """10:07:59: lead at 61 m, 60 mph, closing 9.5 m/s. Legacy asked about −2.4.

  The old far soft/comfort caps held the unified target at −0.6..−0.8. The
  kinematic term −v²/(2·room) now sets the depth from the first frame.
  """
  desired = unified_follow_desired(61.2, 26.68, 17.19, -0.27, 0.7, v_ceiling=29.2)
  assert desired <= -2.4
  assert desired >= A_MIN_MS2 + 0.3
  # From zero (no seed from legacy): full depth within about a second.
  ctrl = UnifiedLeadController()
  reached = None
  for k in range(40):
    gap = 61.2 - 9.5 * k * DT
    accel = ctrl.step(
      dt=DT, present=True, gap=gap, v_ego=26.68, v_lead=17.19, a_lead=-0.27,
      t_follow=0.7, lead_id=1027, seed_a=0.0, v_ceiling=29.2, radar=True, model_prob=0.87,
    )
    if reached is None and accel <= -2.4:
      reached = k * DT
  assert reached is not None and reached <= 1.2
  # A slow close at the same distance stays small (continuous, not a rule).
  slow = unified_follow_desired(61.2, 26.68, 26.68 - 1.0, 0.0, 0.7, v_ceiling=29.2)
  assert -0.45 <= slow <= 0.0


# 10:10:13.0 CT onward at 0.25 s: logged vLead, aLeadK of the lead first seen at 51.8 m.
SEP25_1010_LEAD = (
  (-0.23, -1.3), (-0.2, -1.08), (-0.12, -0.83), (-0.11, -0.46), (-0.11, -0.24), (-0.22, -0.13),
  (-0.06, -0.14), (-0.03, -0.09), (0.05, 0.0), (0.28, -0.08), (0.32, -0.06), (0.0, 0.0),
  (0.14, 0.15), (0.31, 0.22), (0.48, 0.3), (0.54, 0.41), (0.69, 0.47), (0.95, 0.47),
  (0.92, 0.56), (1.41, 0.59), (1.99, 0.74), (2.41, 1.14), (2.58, 1.41), (2.85, 1.42),
  (3.0, 1.42), (3.18, 1.47), (3.31, 1.51), (3.23, 1.51), (3.33, 1.38), (3.8, 1.22),
  (4.11, 1.28), (4.56, 1.33),
)


def _series(rows, col):
  def f(t):
    i = min(len(rows) - 1, int(t / 0.25))
    j = min(len(rows) - 1, i + 1)
    x = min(1.0, max(0.0, t / 0.25 - i))
    return rows[i][col] + (rows[j][col] - rows[i][col]) * x
  return f


def test_sep25_1010_own_target_keeps_a_safe_gap_from_zero():
  """10:10:13: 52 m to a stopped lead at 25 mph (closing 11.4 m/s).

  The unified target starts from 0 (not seeded from legacy). The plant
  starts where the car was (aEgo −1.2) and is limited to the Pre-AP −1.5.
  Legacy's real outcome was about 13 m / TTC 3.4 s; the old unified
  target reached 7.8 m / 2.2 s in the same loop.
  """
  min_gap, min_ttc, cmds = _closed_loop(
    UnifiedLeadController(), d=51.8, v_ego=11.18,
    lead_v=lambda t: max(0.0, _series(SEP25_1010_LEAD, 0)(t)),
    lead_a=_series(SEP25_1010_LEAD, 1),
    t_follow=1.7, seconds=8.0, v_ceiling=21.0, a0=-1.22,
  )
  assert min_gap >= 12.0
  assert min_ttc >= 3.2
  assert min(cmds[:20]) <= -1.5


def test_sep25_0921_close_has_no_spike_or_long_hold():
  """09:21:44: 4.9 m/s close from 29 m at 70 mph, t_follow 0.7.

  The old controller spiked to −3.5 and held −1.7..−3.2 for ~5 s while
  legacy asked −1.1. The target is now firm but proportionate.
  """
  rows = _interp_rows(SEP25_0921)
  ctrl = UnifiedLeadController()
  d0, v0, vr0, al0 = rows[0]
  for _ in range(40):
    ctrl.step(
      dt=DT, present=True, gap=d0, v_ego=v0, v_lead=v0 + vr0, a_lead=al0,
      t_follow=0.7, lead_id=920, seed_a=-0.45, v_ceiling=31.0, radar=True, model_prob=1.0,
    )
  out = []
  for d, v, vr, al in rows:
    out.append(ctrl.step(
      dt=DT, present=True, gap=d, v_ego=v, v_lead=v + vr, a_lead=al,
      t_follow=0.7, lead_id=920, seed_a=-0.45, v_ceiling=31.0, radar=True, model_prob=1.0,
    ))
  assert min(out) > -2.0
  assert sum(1 for a in out if a < -1.7) * DT <= 0.25
  assert min(out) <= -0.8


def test_sep25_1043_overspeed_is_proportional_not_a_clamp():
  """10:43:56: MAX/curve cap 6.4 m/s at 13.7 m/s. Old: −3.5, then −2.7 held.

  Above the cap the speed-error term starts at the mild settle and only
  gets firmer when well over. It never accelerates above the cap.
  """
  over = speed_ceiling_accel(13.7, 6.4)
  assert -1.2 <= over <= -0.5
  assert -0.22 <= speed_ceiling_accel(6.6, 6.4) <= 0.0
  assert abs(speed_ceiling_accel(6.4 + 1e-3, 6.4) - speed_ceiling_accel(6.4 - 1e-3, 6.4)) < 0.01
  prev = 0.0
  for i in range(1, 200):
    a = speed_ceiling_accel(6.4 + i * 0.05, 6.4)
    assert a <= prev + 1e-12
    assert a <= 0.0
    prev = a
  assert speed_ceiling_accel(6.4 + 0.45, 6.4) == pytest.approx(-0.22, abs=0.03)
  ctrl = UnifiedLeadController()
  worst = 0.0
  for _ in range(100):
    accel = ctrl.step(
      dt=DT, present=True, gap=35.8, v_ego=13.7, v_lead=12.4, a_lead=0.0,
      t_follow=1.3, lead_id=4, seed_a=0.0, v_ceiling=6.4, radar=True, model_prob=0.9,
    )
    worst = min(worst, accel)
  assert worst > -1.3
  assert worst < -0.3


@pytest.mark.parametrize("gap,v_ego,v_rel", [
  (47.8, 28.57, 0.30), (27.3, 25.67, 0.98), (36.7, 26.15, 2.02), (46.6, 27.55, 1.52),
])
def test_sep25_pulling_away_lead_near_gap_is_a_trickle(gap, v_ego, v_rel):
  """09:47:41–58 and 10:54:08–19: legacy about +0.08; old unified +0.5..+0.8."""
  accel, _ = _settle(
    60, gap=gap, v_ego=v_ego, v_lead=v_ego + v_rel, a_lead=0.0, t_follow=0.7,
    v_ceiling=31.3,
  )
  assert 0.0 <= accel <= 0.15


def test_no_jump_across_the_follow_distance():
  """One smooth gap curve: no step at the setpoint, bounded slope around it."""
  for v_lead in (8.0, 20.0, 30.0):
    for tf in (0.7, 1.3, 1.7):
      gap_set = gap_set_m(v_lead, tf)
      for closing in (0.0, 0.5, 1.5, 3.0, 6.0, 9.0):
        for a_lead in (0.0, -1.0):
          def f(slack, closing=closing, a_lead=a_lead, gap_set=gap_set, v_lead=v_lead, tf=tf):
            return unified_follow_desired(
              gap_set + slack, v_lead + closing, v_lead, a_lead, tf, v_ceiling=40.0,
            )
          assert abs(f(0.01) - f(-0.01)) < 0.02
          prev = f(-8.0)
          for i in range(1, 321):
            cur = f(-8.0 + i * 0.05)
            assert abs(cur - prev) < 0.08
            prev = cur
      # The kinematic term itself is continuous in the gap error.
      for closing in (1.0, 4.0, 9.0):
        lo = kinematic_required_accel(-1e-3, closing, tf, v_lead)
        hi = kinematic_required_accel(1e-3, closing, tf, v_lead)
        assert abs(hi - lo) < 0.01


def test_lead_confidence_weights_weak_readings_and_ramps_strong_ones():
  """Blips have little effect; a consistent fast close owns the command fast."""
  def run(n, **quality):
    ctrl = UnifiedLeadController()
    out = []
    for _ in range(n):
      out.append(ctrl.step(
        dt=DT, present=True, gap=30.0, v_ego=25.0, v_lead=21.0, a_lead=0.0,
        t_follow=1.3, lead_id=5, seed_a=0.0, v_ceiling=35.0, **quality,
      ))
    return out, ctrl

  strong, ctrl_s = run(20, radar=True, model_prob=0.95)
  weak, _ = run(20, radar=False, model_prob=0.15)
  assert strong[-1] <= -0.8
  assert abs(weak[-1]) < 0.5 * abs(strong[-1])

  # Rapid close: confidence reaches ~full within a fraction of a second.
  ctrl = UnifiedLeadController()
  t_full = None
  for k in range(20):
    ctrl.step(
      dt=DT, present=True, gap=61.0 - 9.5 * k * DT, v_ego=26.7, v_lead=17.2, a_lead=0.0,
      t_follow=0.7, lead_id=6, seed_a=0.0, v_ceiling=30.0, radar=True, model_prob=0.9,
    )
    if t_full is None and ctrl.last_confidence >= 0.9:
      t_full = k * DT
  assert t_full is not None and t_full <= 0.4

  # One-frame overhead/ghost reading while following at the setpoint.
  ctrl = UnifiedLeadController()
  gs = gap_set_m(25.0, 1.3)
  prev = 0.0
  for _ in range(60):
    prev = ctrl.step(
      dt=DT, present=True, gap=gs, v_ego=25.0, v_lead=25.0, a_lead=0.0,
      t_follow=1.3, lead_id=1, seed_a=0.0, v_ceiling=35.0, radar=True, model_prob=0.95,
    )
  base = prev
  worst = 0.0
  ctrl.step(
    dt=DT, present=True, gap=20.0, v_ego=25.0, v_lead=0.0, a_lead=0.0,
    t_follow=1.3, lead_id=99, seed_a=prev, v_ceiling=35.0, radar=True, model_prob=0.3,
  )
  for _ in range(40):
    a = ctrl.step(
      dt=DT, present=True, gap=gs, v_ego=25.0, v_lead=25.0, a_lead=0.0,
      t_follow=1.3, lead_id=1, seed_a=prev, v_ceiling=35.0, radar=True, model_prob=0.95,
    )
    worst = max(worst, abs(a - base))
  assert worst < 0.15


def test_release_rate_is_proportional_to_how_far_below_target():
  """Far too firm releases fast; close to the target releases gently."""
  ctrl = UnifiedLeadController()
  v_ego = 20.0
  prev = 0.0
  for _ in range(60):
    prev = ctrl.step(
      dt=DT, present=True, gap=12.0, v_ego=v_ego, v_lead=14.0, a_lead=-2.5,
      t_follow=1.3, lead_id=2, seed_a=0.0, v_ceiling=30.0, radar=True, model_prob=0.95,
    )
  assert prev <= -2.0
  rates = []
  for _ in range(60):
    a = ctrl.step(
      dt=DT, present=True, gap=_gap(0.0, v_ego, 1.3), v_ego=v_ego, v_lead=v_ego, a_lead=0.0,
      t_follow=1.3, lead_id=2, seed_a=prev, v_ceiling=30.0, radar=True, model_prob=0.95,
    )
    rates.append(((a - prev) / DT, ctrl.last_desired - a))
    prev = a
  far = [r for r, gap_to_target in rates if gap_to_target > 1.0]
  assert far and min(far) >= 1.8
  near = [r for r, gap_to_target in rates if 0.02 < gap_to_target < 0.15]
  assert near and max(near) <= 0.8
