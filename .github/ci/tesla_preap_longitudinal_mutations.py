import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LONGCONTROL_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_longcontrol.py"
FOLLOWING_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_following.py"
GAS_LIFT_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_gas_lift_handoff.py"
BRAKE_CANCEL_TEST_PATH = (
  "selfdrive/controls/tests/test_tesla_preap_brake_cancel_regen.py"
)
UNIFIED_TEST_PATH = "selfdrive/controls/tests/test_unified_lead.py"
UNIFIED_PLANNER_TEST_PATH = "selfdrive/controls/tests/test_unified_lead_planner.py"
CURVE_PREVIEW_TEST_PATH = "selfdrive/controls/tests/test_curve_preview.py"
CURVE_MAX_HOLD_TEST_PATH = "selfdrive/controls/lib/tests/test_curve_max_hold.py"
LEAD_LEAVING_TEST_PATH = "selfdrive/controls/tests/test_lead_leaving.py"
TURN_GEOM_TEST_PATH = "selfdrive/controls/lib/tests/test_lat_turn_geometry.py"
LC_REPLAY_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_confirm_replay.py"
LC_NUDGE_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_nudge.py"
OFFSET_TEST_PATH = "selfdrive/mapd/tests/test_map_speed_offset_slew.py"
ENGAGE_MAX_TEST_PATH = "selfdrive/car/tests/test_preap_engage_max_after_pause.py"
FRONTAGE_TEST_PATH = "selfdrive/mapd/tests/test_map_match_frontage.py"
LANE_CHANGE_TARGET_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_target.py"
LC_TURN_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_turn.py"
NOISE_GATE_TEST_NODE = (
  "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py::TestInnerPID::" +
  "test_sub_deadband_sign_changing_noise_does_not_accumulate_residual_authority"
)
INNER_DEADBAND_TEST_NODE = (
  "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py::TestInnerPID::" +
  "test_persistent_sub_deadband_error_earns_residual_authority"
)


@dataclass(frozen=True)
class HistoricalMutation:
  name: str
  source_path: str
  original: bytes
  replacement: bytes
  test_nodes: tuple[str, ...]


MUTATIONS = (
  HistoricalMutation(
    name="gas-lift-keeps-full-engage-grace",
    source_path="opendbc_repo/opendbc/car/tesla/preap/carcontroller.py",
    original=b"        if gas_handoff:\n",
    replacement=b"        if False:\n",
    test_nodes=(
      f"{GAS_LIFT_TEST_PATH}::test_gas_lift_after_long_engage_does_not_floor_a_for_half_second",
    ),
  ),
  HistoricalMutation(
    name="brake-cancel-keeps-interceptor-after-long-drop",
    source_path="opendbc_repo/opendbc/car/tesla/preap/carcontroller.py",
    original=b"      authority_requested = pedal_long_allowed and long_active and not brake_pressed and not gas_pressed\n",
    replacement=b"      authority_requested = pedal_long_allowed and not gas_pressed\n",
    test_nodes=(
      f"{BRAKE_CANCEL_TEST_PATH}::test_short_tip_releases_interceptor_immediately",
    ),
  ),
  HistoricalMutation(
    name="gas-lift-seed-ignores-planner-climb",
    source_path="opendbc_repo/opendbc/car/tesla/preap/carcontroller.py",
    original=b"  for candidate in (last_nonneg_a_ego, measured_accel, planner_accel):\n",
    replacement=b"  for candidate in (last_nonneg_a_ego, measured_accel):\n",
    test_nodes=(
      f"{GAS_LIFT_TEST_PATH}::test_gas_lift_open_road_seed_uses_mannerisms_accel_not_only_aego",
    ),
  ),
  HistoricalMutation(
    name="gas-lift-zero-command-seed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/carcontroller.py",
    original=(
      b"          commanded_accel = gas_lift_handoff_seed_accel(\n" +
      b"            self.last_nonneg_a_ego,\n" +
      b"            CS.out.aEgo,\n" +
      b"            self.engage_a_max,\n" +
      b"            float(actuators.accel),\n" +
      b"          )\n"
    ),
    replacement=b"          commanded_accel = 0.0\n",
    test_nodes=(
      f"{GAS_LIFT_TEST_PATH}::test_gas_lift_after_long_engage_does_not_floor_a_for_half_second",
    ),
  ),
  HistoricalMutation(
    name="historical-outer-ki",
    source_path="opendbc_repo/opendbc/car/tesla/preap/constants.py",
    original=b"PEDAL_LONG_KI_V = [0.0, 0.0, 0.0, 0.0]\n",
    replacement=b"PEDAL_LONG_KI_V = [0.05, 0.08, 0.10, 0.15]\n",
    test_nodes=(
      f"{LONGCONTROL_TEST_PATH}::test_vdas_receives_route_shaped_planner_target_trace_unchanged",
      f"{LONGCONTROL_TEST_PATH}::test_road_load_history_cannot_reverse_finite_jerk_negative_planner_target",
      f"{LONGCONTROL_TEST_PATH}::test_negative_planner_target_reaches_regen_side_of_coast_anchor",
    ),
  ),
  HistoricalMutation(
    name="adaptive-follow-cap-bypassed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b"        cap_strength = get_preap_follow_cap_strength(" +
      b"v_ego, lead.dRel, lead.vLead, self.t_follow)\n"
    ),
    replacement=b"        cap_strength = 0.0\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::" +
      "test_planner_adaptive_cap_changes_the_delivered_acceleration_for_unequal_speed_lead",
    ),
  ),
  HistoricalMutation(
    name="longcontrol-feedforward-coupling-bypassed",
    source_path="selfdrive/controls/lib/longcontrol.py",
    original=b"                                     feedforward=a_target)\n",
    replacement=b"                                     feedforward=0.0)\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_max_follow_full_closed_loop_recovers_gap_with_production_fallback",
    ),
  ),
  HistoricalMutation(
    name="hard-inner-error-deadband-restored",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"    error = self._gate_pid_error_noise(error, freeze_integrator)\n",
    replacement=(
      b"    if abs(error) < PID_ERROR_DEADBAND:\n" +
      b"      error = 0.0\n"
    ),
    test_nodes=(INNER_DEADBAND_TEST_NODE,),
  ),
  HistoricalMutation(
    name="inner-error-noise-gate-call-bypassed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"    error = self._gate_pid_error_noise(error, freeze_integrator)\n",
    replacement=b"    error = error\n",
    test_nodes=(NOISE_GATE_TEST_NODE,),
  ),
  HistoricalMutation(
    name="negative-handoff-integral-slew-regressed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"NEGATIVE_HANDOFF_INTEGRAL_SLEW = 0.25  # m/s\xc2\xb3\n",
    replacement=b"NEGATIVE_HANDOFF_INTEGRAL_SLEW = 0.20  # m/s\xc2\xb3\n",
    test_nodes=(
      f"{LONGCONTROL_TEST_PATH}::test_negative_planner_target_reaches_regen_side_of_coast_anchor",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-removed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=b"      a_limited,\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-sign-flipped",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=b"      a_limited - steady_grade_compensation - transient_pitch_compensation,\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-doubled",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=(
      b"      a_limited + 2.0 * steady_grade_compensation " +
      b"+ 2.0 * transient_pitch_compensation,\n"
    ),
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-drops-braking-lead-term",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=(
      b"  a_pd = a_gv + k_a * a_l\n" +
      b"\n" +
      b"  a_kin = _a_kin(slack, v_close, gap_f, v_e, t_follow, v_l)\n" +
      b"  # Braking-lead contribution on the bound. Positive a_lead stays in a_pd only.\n" +
      b"  a_bound = a_kin + min(a_l, 0.0) * k_a\n"
    ),
    replacement=(
      b"  a_pd = a_gv\n" +
      b"\n" +
      b"  a_kin = _a_kin(slack, v_close, gap_f, v_e, t_follow, v_l)\n" +
      b"  # Braking-lead contribution on the bound. Positive a_lead stays in a_pd only.\n" +
      b"  a_bound = a_kin\n"
    ),
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_sep23_222_exemplars_match_a_braking_lead",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-jerk-limit-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    nxt = _limit_jerk(prev, target, frame_dt, down=down, up=up)\n",
    replacement=b"    nxt = target\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_stateful_sweep_has_no_jerk_cliff",
    ),
  ),
  HistoricalMutation(
    name="unified-lead kinematic close term removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  return max(A_MIN_MS2, -(v * v) / (2.0 * room))\n",
    replacement=b"  return 0.0\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_sep25_1007_rapid_close_is_at_least_legacy_firm",
    ),
  ),
  HistoricalMutation(
    name="unified-lead gap curve discontinuity restored",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  room = outside + inside - REACTION_S * v\n",
    replacement=b"  room = (outside if e > 0.0 else inside) - REACTION_S * v\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_no_jump_across_the_follow_distance",
    ),
  ),
  HistoricalMutation(
    name="unified-lead MAX hard clamp restored",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  return -OVERSPEED_MAX_MS2 * math.tanh(raw / OVERSPEED_MAX_MS2)\n",
    replacement=b"  return A_MIN_MS2\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_sep25_1043_overspeed_is_proportional_not_a_clamp",
    ),
  ),
  HistoricalMutation(
    name="unified-lead confidence weighting removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    target_conf = on_path * lead_quality(model_prob, radar)\n",
    replacement=b"    target_conf = on_path\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_lead_confidence_weights_weak_readings_and_ramps_strong_ones",
    ),
  ),
  HistoricalMutation(
    name="unified-lead release rate not proportional",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    up = min(JERK_PHYS_MS3, release_base + JERK_PROP_1_S * max(0.0, err)) * frame_dt\n",
    replacement=b"    up = release_base * frame_dt\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_release_rate_is_proportional_to_how_far_below_target",
    ),
  ),
  HistoricalMutation(
    name="unified-lead opening trickle removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    a_cmd = w * min(a_cmd, trickle) + (1.0 - w) * a_cmd\n",
    replacement=b"    a_cmd = a_cmd\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_sep25_pulling_away_lead_near_gap_is_a_trickle",
    ),
  ),
  HistoricalMutation(
    name="engage MAX stale seed restored",
    source_path="selfdrive/car/card.py",
    original=(
      b"      restore_a_ms2=map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel),\n" +
      b"      long_active=soft_long,\n"
    ),
    replacement=(
      b"      restore_a_ms2=map_accel_a_ms2(self._map_speed_lookahead, self._map_speed_accel),\n" +
      b"      long_active=True,\n"
    ),
    test_nodes=(
      f"{ENGAGE_MAX_TEST_PATH}::test_resume_with_gas_after_pause_turn_publishes_held_max_first_frame",
      f"{ENGAGE_MAX_TEST_PATH}::test_paused_long_curve_does_not_rewrite_hud_max",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-toggle-forced-on",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"    enabled = self._unified_enabled\n",
    replacement=b"    enabled = True\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_unified_off_keeps_mode_weight_at_zero",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-shadow-guard-reraises",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b"      except Exception:\n" +
      b"        self._note_unified_fault()\n"
    ),
    replacement=(
      b"      except Exception:\n" +
      b"        raise\n"
    ),
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_unified_shadow_fault_with_toggle_off_matches_legacy",
      f"{UNIFIED_PLANNER_TEST_PATH}::test_unified_shadow_fault_with_toggle_on_falls_back_to_legacy",
    ),
  ),
  HistoricalMutation(
    name="same-direction nudge during ALC treated as takeover",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=(
      b"  # Same-direction nudge during tipped ALC is the lane-change confirm, not a takeover.\n"
      b"  if same_direction:\n"
      b"    return False\n"
    ),
    replacement=(
      b"  # Same-direction nudge during tipped ALC is the lane-change confirm, not a takeover.\n"
      b"  if same_direction:\n"
      b"    return True\n"
    ),
    test_nodes=(
      "selfdrive/controls/lib/tests/test_lane_change_nudge.py::"
      "test_tip_at_45_same_direction_nudge_confirms_through_flash_gap",
    ),
  ),
  HistoricalMutation(
    name="emergency yank suppressed during ALC",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=(
      b"  # Emergency yank always releases, including a same-direction yank during ALC.\n"
      b"  if emergency:\n"
      b"    return True\n"
    ),
    replacement=(
      b"  # Emergency yank always releases, including a same-direction yank during ALC.\n"
      b"  if emergency:\n"
      b"    return False\n"
    ),
    test_nodes=(
      "selfdrive/controls/lib/tests/test_lane_change_nudge.py::"
      "test_emergency_yank_same_direction_releases_and_cancels",
    ),
  ),
  HistoricalMutation(
    name="same-direction bump pauses target-locked lane change",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"    suspended = locked and not lateral_active\n",
    replacement=b"    suspended = locked and (not lateral_active or bool(carstate.steeringPressed))\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_same_direction_bump_mid_change_continues_with_no_pause",
    ),
  ),
  HistoricalMutation(
    name="resume after release disabled",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"    if not engaged or (not lateral_active and not locked) or \\\n",
    replacement=b"    if not engaged or not lateral_active or \\\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_ordinary_release_pauses_then_resumes_to_target",
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_resume_from_partially_over_finishes_centered_in_target_lane",
    ),
  ),
  HistoricalMutation(
    name="desire held through release so resume is not a fresh pulse",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"    if self.target is not None and (self.target_suspended or pulse_gap):\n",
    replacement=b"    if self.target is not None and pulse_gap:\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_ordinary_release_pauses_then_resumes_to_target",
    ),
  ),
  HistoricalMutation(
    name="opposite pull no longer cancels",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"        elif locked and opposite_pull:\n",
    replacement=b"        elif False:\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_opposite_pull_cancels",
    ),
  ),
  HistoricalMutation(
    name="opposite tip no longer cancels target-locked lane change",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"        elif opposite_press:\n",
    replacement=b"        elif False:\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_opposite_tip_cancels",
    ),
  ),
  HistoricalMutation(
    name="target-locked lane change completes on crossing instead of centered",
    source_path="selfdrive/controls/lib/lane_change_target.py",
    original=b"    return self.crossed and self.centered_s + 1e-9 >= CENTERED_HOLD_S\n",
    replacement=b"    return self.crossed\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_completes_when_centered_in_target_lane_not_on_a_timer",
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_resume_from_partially_over_finishes_centered_in_target_lane",
    ),
  ),
  HistoricalMutation(
    name="target lock timeout removed",
    source_path="selfdrive/controls/lib/lane_change_target.py",
    original=b"    return self.age_s > TARGET_LOCK_TIMEOUT_S\n",
    replacement=b"    return False\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_timeout_cancels_including_released_time",
    ),
  ),
  HistoricalMutation(
    name="low-confidence lane lines keep guessing",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"    elif locked and (self.target.timed_out or self.target.low_confidence):\n",
    replacement=b"    elif locked and self.target.timed_out:\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_low_lane_line_confidence_cancels",
    ),
  ),
  HistoricalMutation(
    name="target lock accepts low-confidence ego lines",
    source_path="selfdrive/controls/lib/lane_change_target.py",
    original=b"    if probs[1] < LANE_LINE_MIN_PROB or probs[2] < LANE_LINE_MIN_PROB:\n",
    replacement=b"    if False:\n",
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_low_confidence_at_start_does_not_guess",
    ),
  ),
  HistoricalMutation(
    name="map-offset-double-add-restored",
    source_path="selfdrive/car/card.py",
    original=b"  return (float(displayed_kph) - float(offset_kph)) * CV.KPH_TO_MS\n",
    replacement=b"  return float(displayed_kph) * CV.KPH_TO_MS\n",
    test_nodes=(
      f"{OFFSET_TEST_PATH}::test_engage_mph_offset_is_limit_plus_offset_after_1s",
      f"{OFFSET_TEST_PATH}::test_engage_kph_offset_is_limit_plus_offset_after_1s",
      f"{OFFSET_TEST_PATH}::test_posted_raise_keeps_single_offset",
    ),
  ),
  HistoricalMutation(
    name="current-way-stickiness-removed",
    source_path="selfdrive/mapd/osm_db.py",
    original=(
      b"    \"\"\"Keep the previous way unless a candidate is clearly closer for several lookups.\"\"\"\n"
      b"    if prev_dist_m > STICK_KEEP_M or not heading_aligned:\n"
      b"      return False\n"
      b"    if closer_m >= STICK_SWITCH_CLOSER_M and pending_lookups >= STICK_SWITCH_LOOKUPS:\n"
      b"      return False\n"
      b"    return True\n"
    ),
    replacement=(
      b"    \"\"\"Keep the previous way unless a candidate is clearly closer for several lookups.\"\"\"\n"
      b"    _ = prev_dist_m, heading_aligned, closer_m, pending_lookups\n"
      b"    return False\n"
    ),
    test_nodes=(
      f"{FRONTAGE_TEST_PATH}::test_nearer_parallel_road_does_not_steal_without_clear_advantage",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-decel-anticipation-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b'    a_eff = self._a_f - lead_decel_anticipation(self._a_f, self._lead_jerk, gap, v_ego)\n',
    replacement=b'    a_eff = self._a_f\n',
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_real_lead_brake_onset_is_anticipated_but_gradual_slowdown_is_not",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-brake-scale-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b'  k_a = _k_a(slack) * _k_a_brake_scale(a_l)\n',
    replacement=b'  k_a = _k_a(slack)\n',
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_mild_lead_slowing_is_under_matched_and_braking_over_matched",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-glide-inside-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b'  glide_keep = GLIDE_KEEP + (GLIDE_KEEP_INSIDE - GLIDE_KEEP) * _smooth01(-slack / GLIDE_GAP_M)\n',
    replacement=b'  glide_keep = GLIDE_KEEP\n',
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_small_inside_gap_keeps_easing_until_the_gap_recovers",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-far-close-fade-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b'  k_v_close *= 1.0 - (1.0 - K_V_FAR_MIN) * _smooth01(\n',
    replacement=b'  k_v_close *= 1.0 - 0.0 * _smooth01(\n',
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_far_slow_close_coasts_near_close_does_not",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-far-trust-bound-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  a_cmd = trust_bound(a_cmd, hard_decel_floor(gap_f, v_e, v_l, a_l, t_follow), trust)\n",
    replacement=b"  a_cmd = a_cmd\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_far_offset_or_weak_lead_cannot_reach_full_regen",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-far-trust-ignores-lateral-offset",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  lat_w = 1.0 - (1.0 - TRUST_LAT_MIN) * _smooth01((y - TRUST_LAT_Y0_M) / (DEPART_Y0_M - TRUST_LAT_Y0_M))\n",
    replacement=b"  lat_w = 1.0\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_far_offset_or_weak_lead_cannot_reach_full_regen",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-far-trust-ignores-range",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  remote = far * slow\n",
    replacement=b"  remote = slow\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_222_exemplars_are_fully_trusted_so_unchanged",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-kinematic-need-zeroed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  return max(A_MIN_MS2, min(0.0, a_match))\n",
    replacement=b"  return 0.0\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_far_lead_that_kinematically_needs_full_regen_still_gets_it",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-inherited-seed-carried-into-remote-lead",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    self._cutin_from = trust_bound(self._cutin_from, carry, carry_trust)\n",
    replacement=b"    self._cutin_from = self._cutin_from\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_inherited_firm_seed_is_not_carried_into_a_remote_lead",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-fresh-seed-unbounded",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"      prev = trust_bound(prev, carry, carry_trust)\n",
    replacement=b"      prev = prev\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_inherited_firm_seed_is_not_carried_into_a_remote_lead",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-seed-carry-ignores-confidence",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    carry_trust = trust * self._conf\n",
    replacement=b"    carry_trust = trust\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_sep28_1107_far_ambiguous_lead_is_not_a_full_regen_flash",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-close-closer-trust-reduced",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  if remote <= 0.0:\n    return 1.0\n",
    replacement=b"  if remote <= 0.0:\n    return TRUST_REMOTE\n",
    test_nodes=(
      f"{UNIFIED_TEST_PATH}::test_close_rapid_closer_still_gets_full_fast_brake",
      f"{UNIFIED_TEST_PATH}::test_moderate_range_cut_in_still_responds",
    ),
  ),
  HistoricalMutation(
    name="curve-preview-min-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b'      output_a_target = min(float(output_a_target), self.curve_preview_a)\n',
    replacement=b'      output_a_target = float(output_a_target)\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_planner_preview_only_lowers_the_command",
    ),
  ),
  HistoricalMutation(
    name="curve-preview-may-raise-command",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b'      output_a_target = min(float(output_a_target), self.curve_preview_a)\n',
    replacement=(
      b'      output_a_target = self.curve_preview_a if self.curve_preview_a < PREVIEW_FREE_A_MS2 else float(output_a_target)\n'
    ),
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_planner_preview_only_lowers_the_command",
    ),
  ),
  HistoricalMutation(
    name="curve-preview-decel-limit-removed",
    source_path="selfdrive/controls/lib/curve_preview.py",
    original=b'  return min(PREVIEW_FREE_A_MS2, max(a, -preview_decel_limit_ms2(v)))\n',
    replacement=b'  return min(PREVIEW_FREE_A_MS2, a)\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_highway_bend_preview_is_a_lift",
      f"{CURVE_PREVIEW_TEST_PATH}::test_town_bend_preview_slows_early_and_gently",
    ),
  ),
  HistoricalMutation(
    name="curve-preview-release-rate-removed",
    source_path="selfdrive/controls/lib/curve_preview.py",
    original=b'    if self.a < 0.0:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_preview_sweep_has_no_jump",
    ),
  ),
  HistoricalMutation(
    name="turn-clip-ignores-path-ahead",
    source_path="selfdrive/controls/lib/curve_preview.py",
    original=b'    ay = min(ay, abs(float(a_y_ahead)))\n',
    replacement=b'    ay = max(ay, 0.0)\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_turn_clip_returns_accel_near_the_apex",
    ),
  ),
  HistoricalMutation(
    name="curve-cap-unwind-disabled",
    source_path="selfdrive/controls/lib/curve_max_hold.py",
    original=b'      elif proposed > self._cap_kph and unwind > 0.0:\n',
    replacement=b'      elif proposed > self._cap_kph and False:\n',
    test_nodes=(
      f"{CURVE_MAX_HOLD_TEST_PATH}::test_lat_accel_smooths_and_cap_holds_then_ramps",
    ),
  ),
  HistoricalMutation(
    name="curve-lat-target-constant",
    source_path="selfdrive/controls/lib/curve_max_hold.py",
    original=b'  return _interp(float(v_ms), CURVE_LAT_TARGET_BP_MS, CURVE_LAT_TARGET_MS2)\n',
    replacement=b'  return 2.0\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_lateral_target_vs_speed",
    ),
  ),
  HistoricalMutation(
    name="curve-restore-profile-constant",
    source_path="selfdrive/controls/lib/curve_max_hold.py",
    original=b'  base = _interp(float(v_ms), CURVE_RESTORE_BP_MS, CURVE_RESTORE_A_MS2)\n',
    replacement=b'  base = 0.40\n',
    test_nodes=(
      f"{CURVE_PREVIEW_TEST_PATH}::test_restore_rate_is_speed_dependent",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-clearance-bound-removed",
    source_path="selfdrive/controls/lib/lead_leaving.py",
    original=b"  clear_w = _smooth01((y_proj - clearance_needed_m(v_ego)) / CLEAR_BAND_M)\n",
    replacement=b"  clear_w = 1.0\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_stalled_half_out_lead_keeps_braking",
      f"{LEAD_LEAVING_TEST_PATH}::test_clearance_bound_is_ego_half_width_plus_lead_plus_margin",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-stall-gate-removed",
    source_path="selfdrive/controls/lib/lead_leaving.py",
    original=b"  vy_eff = max(0.0, float(vy_away) - VY_DEADBAND_MS)\n",
    replacement=b"  vy_eff = 1.5\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_stalled_half_out_lead_keeps_braking",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-overlap-gate-removed",
    source_path="selfdrive/controls/lib/lead_leaving.py",
    original=b"  overlap_w = _smooth01((y - OVERLAP_Y0_M) / OVERLAP_BAND_M)\n",
    replacement=b"  overlap_w = 1.0\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_lead_drifting_within_lane_not_released",
      f"{LEAD_LEAVING_TEST_PATH}::test_turning_lead_released_early_with_clearance",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-fcw-suppress-removed",
    source_path="selfdrive/controls/lib/lead_leaving.py",
    original=b"    if suppress:\n",
    replacement=b"    if False:\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_fcw_lead_two_and_new_track_restore_full_weight",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-legacy-application-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"      output_a_target = release_lead_brake(output_a_target, self.lead_leave_w, free_a)\n",
    replacement=b"      output_a_target = output_a_target\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_turning_lead_released_early_with_clearance",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-release-ignores-max-ceiling",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"      free_a = min(float(lead_free_a), self._lead_free_ceiling_a(float(v_ego)))\n",
    replacement=b"      free_a = 0.0\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_planner_leaving_release_respects_max_ceiling",
    ),
  ),
  HistoricalMutation(
    name="lead-leaving-unified-application-removed",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"  fade = _leave_fade(y_rel, curvature, gap_f, path_lat, leave_w)\n",
    replacement=b"  fade = _depart_fade(y_rel, curvature, gap_f, path_lat)\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_unified_leave_weight_releases_brake_but_keeps_max_ceiling",
    ),
  ),
  HistoricalMutation(
    name="unified-static-depart-fades-half-out-lead",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"DEPART_Y0_M = EGO_HALF_WIDTH_M + LEAD_HALF_WIDTH_M + CLEAR_MARGIN_M\n",
    replacement=b"DEPART_Y0_M = 1.2\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_unified_leave_weight_releases_brake_but_keeps_max_ceiling",
    ),
  ),
  HistoricalMutation(
    name="unified-leaving-drops-confidence-instead",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    on_path = 1.0 - _depart_fade(y_rel, curvature, gap, path_lat)\n",
    replacement=b"    on_path = 1.0 - fade\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_unified_leave_weight_releases_brake_but_keeps_max_ceiling",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-ref-offset-ignored",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'-delay_offset_s(v) + ref_offset_s(v, ref_offset_m)))',
    replacement=b'-delay_offset_s(v)))',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_ref_offset_is_used",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-delay-table-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'max(0.0, -delay_offset_s(v) + ref_offset_s',
    replacement=b'max(0.0, ref_offset_s',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_delay_table_values",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-speed-floor-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'/ max(float(v_ego), REF_SPEED_FLOOR_MS)',
    replacement=b'/ max(float(v_ego), 1e-3)',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_ref_offset_speed_floor",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-highway-fade-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'DELAY_OFFSET_S = (-0.13, -0.10, -0.08, -0.05, 0.0)',
    replacement=b'DELAY_OFFSET_S = (-0.13, -0.10, -0.08, -0.05, -0.05)',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_highway_only_reference_offset",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-fade-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return red * speed_fade(v)\n',
    replacement=b'  return red\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_no_effect_below_3ms_and_continuous_fade",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-toggle-forced-on",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'    if not enabled:\n      self.reset()\n',
    replacement=b'    if False:\n      self.reset()\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_off_is_stock_bit_for_bit",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-active-ignores-toggle",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return bool(is_preap) and bool(param_on)\n',
    replacement=b'  return bool(is_preap)\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_active_only_preap_and_toggle_on",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-toggle-forced-off",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return bool(is_preap) and bool(param_on)\n',
    replacement=b'  return False\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_active_only_preap_and_toggle_on",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-default-off",
    source_path="common/params_keys.h",
    original=b'{"NAPLatTurnGeom", {PERSISTENT, BOOL, "1"}}',
    replacement=b'{"NAPLatTurnGeom", {PERSISTENT, BOOL, "0"}}',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_params_default_on_and_offset_035",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-reduction-cap-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  red = min(MAX_REDUCTION_S, max(0.0, -delay_offset_s(v)',
    replacement=b'  red = min(9.9, max(0.0, -delay_offset_s(v)',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_reduction_capped_at_02",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-min-lookahead-clamp-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return max(min(stock, MIN_LOOKAHEAD_S), stock - red)\n',
    replacement=b'  return stock - red\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_min_lookahead_floor_never_raises",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-rate-limit-hold-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'target = 0.0 if self._hold_s > 0.0 else target_reduction_s(v_ego, ref_offset_m)',
    replacement=b'target = target_reduction_s(v_ego, ref_offset_m)',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_rate_limit_holds_stock_then_slews_back",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-applied-to-maneuver-plan",
    source_path="selfdrive/controls/controlsd.py",
    original=b"      model_or_plan_curvature = self.sm['lateralManeuverPlan'].desiredCurvature\n",
    replacement=b'      model_or_plan_curvature = model_v2.action.desiredCurvature\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_lateral_maneuver_plan_bypasses_correction",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-modeld-off-path-not-stock",
    source_path="selfdrive/modeld/modeld.py",
    original=b'      lat_action_t = lat_delay + frame_delay + action_delay\n',
    replacement=b'      lat_action_t = lat_delay + frame_delay\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_modeld_applies_only_to_model_action_lookahead",
    ),
  ),
  HistoricalMutation(
    name="roundabout-outer-bias-kept-with-turn-geom",
    source_path="selfdrive/mapd/roundabout.py",
    original=b'  if turn_geometry_active:\n    return 0.0\n  return roundabout_outer_curvature_bias(',
    replacement=b'  return roundabout_outer_curvature_bias(',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_roundabout_outer_bias_stripped_when_correction_active",
    ),
  ),
  HistoricalMutation(
    name="roundabout-controlsd-ignores-turn-geom",
    source_path="selfdrive/controls/controlsd.py",
    original=b'turn_geometry_active=self._turn_geom_active,',
    replacement=b'turn_geometry_active=False,',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_controlsd_gates_roundabout_bias_on_turn_geometry",
    ),
  ),
  HistoricalMutation(
    name="roundabout-off-path-bias-dropped",
    source_path="selfdrive/mapd/roundabout.py",
    original=b'  if turn_geometry_active:\n    return 0.0\n',
    replacement=b'  if True:\n    return 0.0\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_roundabout_outer_bias_off_path_is_legacy",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-rise-line-restored",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"EMERGENCY_RISE_NM = 4.0 * SOFT_YIELD_TRIGGER_NM\n",
    replacement=b"EMERGENCY_RISE_NM = 3.0 * SOFT_YIELD_TRIGGER_NM\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_new_thresholds",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-rise-window-restored",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"EMERGENCY_RISE_WINDOW_S = 0.250\n",
    replacement=b"EMERGENCY_RISE_WINDOW_S = 0.150\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_new_thresholds",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-torque-line-restored",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"EMERGENCY_TORQUE_NM = 2.5 * float(STEER_THRESHOLD)\n",
    replacement=b"EMERGENCY_TORQUE_NM = 2.0 * float(STEER_THRESHOLD)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_new_thresholds",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-rise-sustain-removed",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"                      (t - self._rise_cross_t) >= EMERGENCY_SUSTAIN_S - 1e-9)\n",
    replacement=b"                      (t - self._rise_cross_t) >= -1.0)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_one_frame_spike_over_the_lines_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-torque-sustain-removed",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"                        self.over_torque_s >= EMERGENCY_SUSTAIN_S - 1e-9)\n",
    replacement=b"                        self.over_torque_s >= -1.0)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_one_frame_spike_over_the_lines_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-emergency-rise-window-check-removed",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"        self._rise_ok = (self._last_low_t is not None and\n                         (t - self._last_low_t) <= EMERGENCY_RISE_WINDOW_S + 1e-9)\n",
    replacement=b"        self._rise_ok = True\n",
    test_nodes=(
      f"{LC_NUDGE_TEST_PATH}::test_fast_torque_rise_is_an_emergency_yank",
    ),
  ),
  HistoricalMutation(
    name="lc-hands3-same-direction-still-emergency",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"    if not same_direction:\n      return True\n",
    replacement=b"    if True:\n      return True\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_hands_on_level_3_same_direction_confirm_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-hands3-never-emergency",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"  if int(hands_on_level or 0) >= EMERGENCY_HANDS_ON_LEVEL:\n",
    replacement=b"  if False:\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_hands_on_level_3_same_direction_confirm_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-card-ignores-sustained-over-torque",
    source_path="selfdrive/car/car_specific.py",
    original=b"      over_torque=self._yank.over_torque, alc_direction=direction if alc_confirm else 0)\n",
    replacement=b"      over_torque=False, alc_direction=direction if alc_confirm else 0)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_slow_hard_pull_against_the_change_releases",
    ),
  ),
  HistoricalMutation(
    name="lc-card-hands3-ignores-alc-direction",
    source_path="selfdrive/car/car_specific.py",
    original=b"      over_torque=self._yank.over_torque, alc_direction=direction if alc_confirm else 0)\n",
    replacement=b"      over_torque=self._yank.over_torque, alc_direction=0)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_hands_on_level_3_same_direction_confirm_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-desire-ignores-sustained-over-torque",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"      over_torque=self._yank.over_torque,\n",
    replacement=b"      over_torque=False,\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_slow_hard_pull_against_the_change_releases",
    ),
  ),
  HistoricalMutation(
    name="lc-desire-hands3-ignores-alc-direction",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"      alc_direction=lc_dir,\n",
    replacement=b"      alc_direction=0,\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_hands_on_level_3_same_direction_confirm_is_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-not-applied",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b"        torque_applied = torque_applied or soft_confirm\n",
    replacement=b"        torque_applied = torque_applied\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_first_nudge_below_steering_pressed_now_confirms",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-line-too-low",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"SOFT_CONFIRM_NM = 0.65 * float(STEER_THRESHOLD)\n",
    replacement=b"SOFT_CONFIRM_NM = 0.45 * float(STEER_THRESHOLD)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_road_bumps_and_crown_do_not_confirm",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-sustain-removed",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"    return self._held_s >= SOFT_CONFIRM_SUSTAIN_S - 1e-9\n",
    replacement=b"    return True\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_road_bumps_and_crown_do_not_confirm",
      f"{LC_REPLAY_TEST_PATH}::test_soft_confirm_needs_the_sustain_time",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-any-direction",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"    signed = torque if direction == 1 else (-torque if direction == 2 else 0.0)\n",
    replacement=b"    signed = abs(torque)\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_soft_confirm_only_same_direction_and_at_speed",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-not-armed-only",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"    if not armed or signed < SOFT_CONFIRM_NM:\n",
    replacement=b"    if signed < SOFT_CONFIRM_NM:\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_soft_confirm_only_same_direction_and_at_speed",
    ),
  ),
  HistoricalMutation(
    name="lc-soft-confirm-no-reset-on-dip",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b"      self._held_s = 0.0\n      return False\n",
    replacement=b"      return False\n",
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_road_bumps_and_crown_do_not_confirm",
    ),
  ),
  HistoricalMutation(
    name="lc-takeover-is-an-emergency",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b'  if over_torque and not same_direction:\n',
    replacement=b'  if over_torque:\n',
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_sustained_same_direction_pull_is_a_takeover_not_an_emergency",
      f"{LC_TURN_TEST_PATH}::test_sustained_hard_turn_is_never_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-takeover-sustain-too-short",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b'TAKEOVER_SUSTAIN_S = 0.30\n',
    replacement=b'TAKEOVER_SUSTAIN_S = 0.10\n',
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_new_thresholds",
      f"{LC_REPLAY_TEST_PATH}::test_hard_confirm_peak_is_not_a_takeover",
      f"{LC_REPLAY_TEST_PATH}::test_sustained_same_direction_pull_is_a_takeover_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-takeover-never-clears",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b'    if self.takeover and self._hands_off_s >= TAKEOVER_HANDS_OFF_S - 1e-9:\n      self.takeover = False\n',
    replacement=b'',
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_sustained_same_direction_pull_is_a_takeover_not_an_emergency",
    ),
  ),
  HistoricalMutation(
    name="lc-opposite-pull-sustain-removed",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b'    return self._opposite_s >= OPPOSITE_PULL_SUSTAIN_S - 1e-9\n',
    replacement=b'    return self._opposite_s > 0.0\n',
    test_nodes=(
      f"{LC_REPLAY_TEST_PATH}::test_sep28_1037_right_confirm_still_confirms",
      f"{LC_REPLAY_TEST_PATH}::test_sep28_1101_left_confirm_still_confirms",
      f"{LC_REPLAY_TEST_PATH}::test_sep28_1111_right_confirm_still_confirms",
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_confirm_spring_back_does_not_cancel",
    ),
  ),
  HistoricalMutation(
    name="lc-opposite-pull-any-torque",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b'      return torque <= -OPPOSITE_PULL_NM\n',
    replacement=b'      return torque < 0.0\n',
    test_nodes=(
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_confirm_spring_back_does_not_cancel",
    ),
  ),
  HistoricalMutation(
    name="lc-turn-angle-too-low",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'TURN_ENTER_ANGLE_DEG = 45.0\n',
    replacement=b'TURN_ENTER_ANGLE_DEG = 20.0\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_thresholds_from_logs",
    ),
  ),
  HistoricalMutation(
    name="lc-turn-without-driver-steering",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'  return (not lat_active) or _signed(torque_nm, direction) >= TURN_DRIVER_TORQUE_NM\n',
    replacement=b'  return True\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_curve_steered_by_openpilot_is_not_a_turn",
    ),
  ),
  HistoricalMutation(
    name="lc-turn-exit-hold-removed",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'        if self._exit_s >= TURN_EXIT_HOLD_S - 1e-9:\n',
    replacement=b'        if True:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_turn_hold_unit_exit_needs_the_hold_time",
    ),
  ),
  HistoricalMutation(
    name="lc-turn-hold-timeout-removed",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'    if self._t >= TURN_HOLD_TIMEOUT_S - 1e-9:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_blinker_hold_never_flashes_forever",
    ),
  ),
  HistoricalMutation(
    name="lc-turn-hold-opposite-stalk-ignored",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'    if stalk == opposite and prev_stalk != opposite:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_blinker_hold_never_flashes_forever",
    ),
  ),
  HistoricalMutation(
    name="lc-speed-hold-never-resumes",
    source_path="selfdrive/controls/lib/lane_change_turn.py",
    original=b'      elif float(v_ego) >= HOLD_RESUME_SPEED:\n',
    replacement=b'      elif False:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_speed_hold_ends_when_back_up_to_speed_without_a_turn",
    ),
  ),
  HistoricalMutation(
    name="lc-desire-ignores-driver-turn",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b'        elif driver_turn:\n',
    replacement=b'        elif False:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_turn_mid_change_pauses_lat_keeps_engaged_and_blinker",
      f"{LC_TURN_TEST_PATH}::test_turn_then_straight_road_resumes_on_lane_center_not_old_target",
    ),
  ),
  HistoricalMutation(
    name="lc-desire-keeps-change-under-20mph",
    source_path="selfdrive/controls/lib/desire_helper.py",
    original=b'        elif below_lane_change_speed and self.lane_change_state in (\n',
    replacement=b'        elif False and self.lane_change_state in (\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_speed_under_20_mid_change_ends_it_and_holds_the_blinker_for_the_turn",
    ),
  ),
  HistoricalMutation(
    name="lc-hold-ignores-driver-turn",
    source_path="selfdrive/controls/lib/blinker_lateral_pause.py",
    original=b'    if stalk_is_turn or driver_turn:\n',
    replacement=b'    if stalk_is_turn:\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_turn_mid_change_pauses_lat_keeps_engaged_and_blinker",
    ),
  ),
  HistoricalMutation(
    name="lc-post-change-hand-on-hold-removed",
    source_path="selfdrive/controls/lib/blinker_lateral_pause.py",
    original=b'          if steering_pressed:\n            self.holding = True\n',
    replacement=b'          if False:\n            self.holding = True\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_lane_change_ending_with_hands_on_keeps_the_disengage_block",
    ),
  ),
  HistoricalMutation(
    name="lc-controlsd-yields-on-emergency-only",
    source_path="selfdrive/controls/controlsd.py",
    original=b'emergency_yank=bool(self._lane_change_torque.release),',
    replacement=b'emergency_yank=bool(self._lane_change_torque.emergency),',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_controlsd_wires_turn_hold_and_tipped_torque",
    ),
  ),
  HistoricalMutation(
    name="lc-controlsd-drops-turn-blinker",
    source_path="selfdrive/controls/controlsd.py",
    original=b'      turn_hold_dir)\n',
    replacement=b'      0)\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_controlsd_wires_turn_hold_and_tipped_torque",
    ),
  ),
  HistoricalMutation(
    name="lc-controlsd-no-driver-turn-pause",
    source_path="selfdrive/controls/controlsd.py",
    original=b'driver_turn=self._lane_change_turn.turning,',
    replacement=b'driver_turn=False,',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_controlsd_wires_turn_hold_and_tipped_torque",
    ),
  ),
  HistoricalMutation(
    name="lc-slow-turn-ramp-counted-as-yank",
    source_path="selfdrive/controls/lib/lane_change_nudge.py",
    original=b'EMERGENCY_RISE_WINDOW_S = 0.250\n',
    replacement=b'EMERGENCY_RISE_WINDOW_S = 0.600\n',
    test_nodes=(
      f"{LC_TURN_TEST_PATH}::test_sep28_1113_sustained_turn_with_latched_stalk_is_not_an_emergency",
    ),
  ),
)


class JUnitReportError(RuntimeError):
  pass


def run_pytest(test_nodes: tuple[str, ...], junit_path: Path) -> subprocess.CompletedProcess[str]:
  environment = os.environ.copy()
  environment["PYTHONDONTWRITEBYTECODE"] = "1"
  environment["PYTHONPATH"] = str(REPO_ROOT)
  return subprocess.run(
    [
      sys.executable, "-m", "pytest", "-q", "-n", "0", "-p", "no:cacheprovider",
      f"--junitxml={junit_path}", *test_nodes,
    ],
    cwd=REPO_ROOT,
    env=environment,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    check=False,
  )


def junit_testcases(junit_path: Path) -> list[ET.Element]:
  try:
    return list(ET.parse(junit_path).iter("testcase"))
  except (OSError, ET.ParseError) as exc:
    raise JUnitReportError(f"cannot read {junit_path.name}: {exc}") from exc


def has_only_assertion_failures(testcases: list[ET.Element]) -> bool:
  failures = [failure for testcase in testcases for failure in testcase.findall("failure")]
  errors = [error for testcase in testcases for error in testcase.findall("error")]
  return (
    bool(failures)
    and not errors
    and all(
      (failure.get("type") or "").endswith("AssertionError")
      or (failure.get("message") or "").startswith("AssertionError:")
      or "AssertionError" in (failure.text or "")
      for failure in failures
    )
  )


def apply_mutation(mutation: HistoricalMutation) -> tuple[Path, bytes]:
  source_path = REPO_ROOT / mutation.source_path
  original_source = source_path.read_bytes()
  match_count = original_source.count(mutation.original)
  if match_count != 1:
    raise RuntimeError(
      f"{mutation.name}: expected one source match in {mutation.source_path}, found {match_count}"
    )
  source_path.write_bytes(original_source.replace(mutation.original, mutation.replacement, 1))
  return source_path, original_source


def main() -> int:
  with tempfile.TemporaryDirectory(prefix="tesla-preap-parent-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = run_pytest(
      (LONGCONTROL_TEST_PATH, FOLLOWING_TEST_PATH, NOISE_GATE_TEST_NODE),
      baseline_xml,
    )
    if baseline.returncode != 0:
      print("BASELINE FAILED: parent longitudinal regression tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = junit_testcases(baseline_xml)
    except JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} parent tests")

    survivors = []
    for mutation in MUTATIONS:
      source_path = None
      original_source = None
      mutation_result = None
      mutation_error = None
      restored = False
      try:
        source_path, original_source = apply_mutation(mutation)
        mutation_result = run_pytest(
          mutation.test_nodes,
          temp_root / f"{mutation.name}.xml",
        )
      except Exception as exc:  # pragma: no cover - failure reporting path
        mutation_error = exc
      finally:
        if source_path is not None and original_source is not None:
          source_path.write_bytes(original_source)
          restored = source_path.read_bytes() == original_source

      if not restored:
        print(f"INVALID: {mutation.name} source restoration was not byte-identical")
        return 1
      if mutation_error is not None:
        print(f"INVALID: {mutation.name} could not run: {mutation_error}")
        return 1
      if mutation_result is None:
        print(f"INVALID: {mutation.name} produced no pytest result")
        return 1

      mutation_xml = temp_root / f"{mutation.name}.xml"
      try:
        mutation_testcases = junit_testcases(mutation_xml)
      except JUnitReportError as exc:
        print(f"INVALID: {mutation.name} {exc}")
        return 1
      if mutation_result.returncode == 1 and has_only_assertion_failures(mutation_testcases):
        print(f"KILLED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      elif mutation_result.returncode == 0:
        survivors.append(mutation.name)
        print(f"SURVIVED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      else:
        print(f"INVALID: {mutation.name} exited without assertion-only test failures "
              + f"(pytest status {mutation_result.returncode})")
        print(mutation_result.stdout)
        return 1

    if survivors:
      print(f"Historical mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} parent longitudinal mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
