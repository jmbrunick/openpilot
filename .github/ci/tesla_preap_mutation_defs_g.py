"""Firm-brake latch mutation definitions (see tesla_preap_longitudinal_mutations.py).

The latch keeps the Pre-AP follow guard from re-clipping a firm mid-gap
brake to MILD while the close eases under 1.5 m/s (15a 11:26:49). It sits
outside the guard, may only make the command firmer while the plan is
still firm, never overrides the early kinematic cap, and must expire on a soft plan, a collapsed close, a lost lead, a reset, or
4 s. Each entry breaks one rule and must fail the pinned tests with an
AssertionError.
"""
from tesla_preap_mutation_common import HistoricalMutation

T = "selfdrive/controls/tests/test_firm_brake_latch.py"
LA = "selfdrive/controls/lib/firm_brake_latch.py"
LC = "selfdrive/controls/lib/longcontrol.py"


def _m(name, path, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=path, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


MUTATIONS_G = (
  # --- constants -----------------------------------------------------------------
  _m("latch-enter-plan-softer", LA,
     b"LEAD_FIRM_LATCH_ENTER_PLAN_MS2 = -0.80\n", b"LEAD_FIRM_LATCH_ENTER_PLAN_MS2 = -0.40\n",
     "test_constants_are_pinned", "test_latch_enters_on_a_firm_plan_and_a_firm_close"),
  _m("latch-hold-plan-softer", LA,
     b"LEAD_FIRM_LATCH_HOLD_PLAN_MS2 = -0.50\n", b"LEAD_FIRM_LATCH_HOLD_PLAN_MS2 = -0.10\n",
     "test_constants_are_pinned", "test_latch_releases_when_the_plan_goes_soft",
     "test_hold_pass_changes_nothing_when_the_plan_is_not_firm"),
  _m("latch-hold-close-zero", LA,
     b"LEAD_FIRM_LATCH_HOLD_CLOSE_MS = 0.80\n", b"LEAD_FIRM_LATCH_HOLD_CLOSE_MS = 0.0\n",
     "test_constants_are_pinned", "test_latch_releases_when_the_close_collapses"),
  _m("latch-max-ten-times-longer", LA,
     b"LEAD_FIRM_LATCH_MAX_S = 4.0\n", b"LEAD_FIRM_LATCH_MAX_S = 40.0\n",
     "test_constants_are_pinned", "test_latch_expires_after_four_seconds"),
  _m("latch-enter-close-lowered", LA,
     b"LEAD_FIRM_LATCH_ENTER_CLOSE_MS = LEAD_APPROACH_SOFT_LIMIT_CLOSE_MS\n",
     b"LEAD_FIRM_LATCH_ENTER_CLOSE_MS = 0.5\n",
     "test_constants_are_pinned", "test_hit_needs_a_firm_close_in_the_mid_gap_band"),
  # --- latch state machine -------------------------------------------------------
  _m("latch-never-expires-by-age", LA,
     b"      if (self.age > LEAD_FIRM_LATCH_MAX_S\n          or p >",
     b"      if (False\n          or p >",
     "test_latch_expires_after_four_seconds", "test_longcontrol_latch_is_never_softer_than_legacy_and_expires"),
  _m("latch-age-never-advances", LA,
     b"      self.age += float(dt)\n", b"      pass\n",
     "test_latch_expires_after_four_seconds"),
  _m("latch-ignores-soft-plan", LA,
     b"          or p > LEAD_FIRM_LATCH_HOLD_PLAN_MS2\n", b"          or False\n",
     "test_latch_releases_when_the_plan_goes_soft", "test_longcontrol_mild_and_settle_plans_are_bit_identical_to_legacy"),
  _m("latch-ignores-collapsed-close", LA,
     b"          or v < LEAD_FIRM_LATCH_HOLD_CLOSE_MS):\n", b"          or False):\n",
     "test_latch_releases_when_the_close_collapses", "test_longcontrol_latch_is_never_softer_than_legacy_and_expires"),
  _m("latch-enters-without-a-hit", LA,
     b"    if not self.on and hit and p <= LEAD_FIRM_LATCH_ENTER_PLAN_MS2:\n",
     b"    if not self.on and p <= LEAD_FIRM_LATCH_ENTER_PLAN_MS2:\n",
     "test_latch_enters_on_a_firm_plan_and_a_firm_close", "test_latch_releases_when_the_plan_goes_soft"),
  _m("latch-enters-on-soft-plan", LA,
     b"    if not self.on and hit and p <= LEAD_FIRM_LATCH_ENTER_PLAN_MS2:\n",
     b"    if not self.on and hit:\n",
     "test_latch_enters_on_a_firm_plan_and_a_firm_close",
     "test_longcontrol_mild_and_settle_plans_are_bit_identical_to_legacy"),
  _m("latch-survives-lost-lead", LA,
     b"      self.reset()\n      return False\n    p = float(planner_a)\n",
     b"      return False\n    p = float(planner_a)\n",
     "test_latch_resets_when_the_lead_or_plan_is_missing", "test_longcontrol_drops_the_latch_with_the_lead_and_on_reset"),
  # --- latch seam ------------------------------------------------------------------
  _m("apply-ignores-hold", LA,
     b"  if not hold or guarded_a is None or raw_a is None or planner_a is None:\n",
     b"  if guarded_a is None or raw_a is None or planner_a is None:\n",
     "test_hold_pass_is_never_softer_than_the_unlatched_guard", "test_latch_seam_ignores_missing_inputs_and_soft_plans"),
  _m("apply-has-no-effect", LA,
     b"  return min(float(guarded_a), float(raw_a))\n", b"  return guarded_a\n",
     "test_hold_pass_lets_the_firm_plan_reach_the_plant_below_the_close_gate",
     "test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff"),
  _m("apply-takes-the-softer-command", LA,
     b"  return min(float(guarded_a), float(raw_a))\n", b"  return max(float(guarded_a), float(raw_a))\n",
     "test_hold_pass_lets_the_firm_plan_reach_the_plant_below_the_close_gate",
     "test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff"),
  _m("apply-ungated-by-plan", LA,
     b"  if float(planner_a) > LEAD_FIRM_LATCH_HOLD_PLAN_MS2:\n    return guarded_a\n", b"",
     "test_hold_pass_changes_nothing_when_the_plan_is_not_firm", "test_latch_seam_ignores_missing_inputs_and_soft_plans"),
  _m("apply-overrides-early-kinematic-cap", LA,
     (b"  if (v_rel is not None and not lead_approach_is_rapid(float(v_rel))\n"
      + b"      and lead_firm_large_slack_a(v_rel, slack, a_lead) is not None):\n    return guarded_a\n"), b"",
     "test_early_kinematic_cap_keeps_ownership_of_its_frame"),
  # --- hit helper --------------------------------------------------------------------
  _m("hit-ignores-mid-gap-band", LA,
     b"  if not lead_mid_gap_map_band(slack, d_rel):\n    return False\n  return lead_firm_alead(",
     b"  return lead_firm_alead(",
     "test_hit_needs_a_firm_close_in_the_mid_gap_band"),
  _m("hit-ignores-near-bumper", LA,
     (b"  if d_rel is not None and float(d_rel) <= LEAD_MPC_SOFT_NEAR_M:\n    return False\n"
      + b"  if not lead_mid_gap_map_band(slack, d_rel):\n    return False\n  return lead_firm_alead("),
     b"  if not lead_mid_gap_map_band(slack, d_rel):\n    return False\n  return lead_firm_alead(",
     "test_hit_needs_a_firm_close_in_the_mid_gap_band"),
  _m("hit-accepts-any-close", LA,
     b"  return lead_firm_alead(a_lead) or lead_rising_or_residual_brake(\n    v, prev_v_rel, a_ego, dt,\n  )\n\n\nclass FirmBrakeLatch",
     b"  return True\n\n\nclass FirmBrakeLatch",
     "test_hit_needs_a_firm_close_in_the_mid_gap_band", "test_hit_accepts_the_222_residual_brake_without_a_firm_alead"),
  _m("hit-drops-222-residual", LA,
     b"  return lead_firm_alead(a_lead) or lead_rising_or_residual_brake(\n    v, prev_v_rel, a_ego, dt,\n  )\n\n\nclass FirmBrakeLatch",
     b"  return lead_firm_alead(a_lead)\n\n\nclass FirmBrakeLatch",
     "test_hit_accepts_the_222_residual_brake_without_a_firm_alead"),
  # --- LongControl wiring --------------------------------------------------------------
  _m("longcontrol-never-applies-latch", LC,
     b"        guarded_accel, raw_accel, a_target, hold_pass,\n", b"        guarded_accel, raw_accel, a_target, False,\n",
     "test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff", "test_longcontrol_wires_the_latch_into_the_guard"),
  _m("longcontrol-latch-restores-the-guarded-value", LC,
     b"        guarded_accel, raw_accel, a_target, hold_pass,\n", b"        guarded_accel, guarded_accel, a_target, hold_pass,\n",
     "test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff"),
  _m("longcontrol-latch-clock-wrong", LC,
     b"        DT_CTRL,\n      )\n      raw_accel = self.last_output_accel\n",
     b"        0.05,\n      )\n      raw_accel = self.last_output_accel\n",
     "test_longcontrol_removes_the_1p5_to_mild_to_1p5_cliff", "test_longcontrol_wires_the_latch_into_the_guard"),
  _m("longcontrol-reset-keeps-latch", LC,
     b"    self._lead_residual.reset()\n    self._firm_latch.reset()\n", b"    self._lead_residual.reset()\n",
     "test_longcontrol_drops_the_latch_with_the_lead_and_on_reset"),
)
