"""Parent longitudinal mutation definitions, part 1 (see tesla_preap_longitudinal_mutations.py).
"""
from tesla_preap_mutation_common import (
  BRAKE_CANCEL_TEST_PATH,
  CURVE_MAX_HOLD_TEST_PATH,
  CURVE_PREVIEW_TEST_PATH,
  ENGAGE_MAX_TEST_PATH,
  FOLLOWING_TEST_PATH,
  FRONTAGE_TEST_PATH,
  GAS_LIFT_TEST_PATH,
  HistoricalMutation,
  INNER_DEADBAND_TEST_NODE,
  LANE_CHANGE_TARGET_TEST_PATH,
  LEAD_LEAVING_TEST_PATH,
  LONGCONTROL_TEST_PATH,
  NOISE_GATE_TEST_NODE,
  OFFSET_TEST_PATH,
  UNIFIED_PLANNER_TEST_PATH,
  UNIFIED_TEST_PATH,
)


MUTATIONS_A = (
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
      f"{UNIFIED_PLANNER_TEST_PATH}::test_unified_fault_latches_and_publishes_the_lead_free_command",
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
)
