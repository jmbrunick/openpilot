"""Parent longitudinal mutation definitions, part 2 (see tesla_preap_longitudinal_mutations.py).
"""
# ruff: noqa: F403, F405
from tesla_preap_mutation_common import *


MUTATIONS_B = (
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
      f"{TURN_GEOM_TEST_PATH}::test_no_effect_below_1p5ms_and_continuous_fade",
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
    original=b'  red = min(reduction_cap_s(v), max(0.0, -delay_offset_s(v)',
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
    name="turn-geom-low-speed-fade-start-restored",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'MIN_SPEED_MS = 1.5 ',
    replacement=b'MIN_SPEED_MS = 3.0 ',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_no_effect_below_1p5ms_and_continuous_fade",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-reach-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return float(np.clip(w, 0.0, 1.0))\n',
    replacement=b'  return 0.0\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_low_speed_extra_reach_at_5_to_8_mph",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-reach-not-faded-by-10mph",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  return float(np.clip(w, 0.0, 1.0))\n',
    replacement=b'  return 1.0\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_at_or_above_10mph_output_identical_to_7c49a8e",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-cap-not-raised",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'LOW_SPEED_MAX_REDUCTION_S = 0.25\n',
    replacement=b'LOW_SPEED_MAX_REDUCTION_S = 0.20\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_low_speed_extra_reach_at_5_to_8_mph",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-direct-sampling-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  if floor >= MIN_STABLE_DELAY or float(action_t) >= MIN_STABLE_DELAY:\n',
    replacement=b'  if True:\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_plan_curvature_direct_sampling_below_03",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-direct-sampling-ignores-floor",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'  t = max(float(action_t), floor, LOW_SPEED_SAMPLE_FLOOR_S)\n',
    replacement=b'  t = max(float(action_t), 0.05)\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_plan_curvature_direct_sampling_below_03",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-freeze-removed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'    if self._hold_s > 0.0 and low_speed_reach_weight(v_ego) > 0.0:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_rate_limit_freezes_reduction_below_10mph",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-freeze-applied-above-10mph",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'    if self._hold_s > 0.0 and low_speed_reach_weight(v_ego) > 0.0:\n',
    replacement=b'    if self._hold_s > 0.0:\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_at_or_above_10mph_output_identical_to_7c49a8e",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-low-speed-cap-not-passed",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'    return apply_reduction(stock_lookahead_s, self.reduction_s, reduction_cap_s(v_ego))\n',
    replacement=b'    return apply_reduction(stock_lookahead_s, self.reduction_s)\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_low_speed_extra_reach_at_5_to_8_mph",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-sample-floor-not-updated",
    source_path="selfdrive/controls/lib/lat_turn_geometry.py",
    original=b'    self.sample_floor_s = sample_floor_s(v_ego)\n',
    replacement=b'    pass\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_low_speed_extra_reach_at_5_to_8_mph",
    ),
  ),
  HistoricalMutation(
    name="turn-geom-modeld-sample-floor-not-wired",
    source_path="selfdrive/modeld/modeld.py",
    original=b'        lat_sample_floor_s = turn_geom.sample_floor_s\n',
    replacement=b'        pass\n',
    test_nodes=(
      f"{TURN_GEOM_TEST_PATH}::test_modeld_applies_only_to_model_action_lookahead",
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
  HistoricalMutation(
    name="unified-output-not-published",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"    self.output_a_target = float(command)\n",
    replacement=b"    self.output_a_target = self.output_a_target\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_sole_path_follows_the_continuous_command_for_sep23",
    ),
  ),
  HistoricalMutation(
    name="unified-seed-bound-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"            seed = min(existing, max(wanted, 0.0))\n",
    replacement=b"            seed = existing\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_first_far_closing_lead_is_never_rematched_with_plus_a",
    ),
  ),
  HistoricalMutation(
    name="unified-seed-brake-head-start-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"            seed = min(existing, max(wanted, min(existing, 0.0) - SEED_BRAKE_STEP_MS2))\n",
    replacement=b"            seed = existing\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_first_hard_close_frame_starts_firmer_than_the_lead_free_command",
    ),
  ),
  HistoricalMutation(
    name="unified-fcw-clamp-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"      command = min(command, existing)\n",
    replacement=b"      command = command\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_fcw_is_not_weaker_than_the_lead_free_command",
    ),
  ),
  HistoricalMutation(
    name="unified-curve-ceiling-removed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"      v_cap = min(v_cap, float(v_curve))\n",
    replacement=b"      v_cap = v_cap\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_curve_ceiling_brakes_with_a_lead_present",
    ),
  ),
  HistoricalMutation(
    name="unified-gate-ignores-fingerprint",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"CP.brand == \"tesla\" and CP.carFingerprint == \"TESLA_MODEL_S_PREAP\"",
    replacement=b"CP.brand == \"tesla\"",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_other_cars_never_run_the_unified_controller",
    ),
  ),
  HistoricalMutation(
    name="unified-gate-ignores-pcm-cruise",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"                       and CP.openpilotLongitudinalControl and not CP.pcmCruise)\n",
    replacement=b"                       and CP.openpilotLongitudinalControl)\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_other_cars_never_run_the_unified_controller",
    ),
  ),
  HistoricalMutation(
    name="unified-lead-release-ignores-max-ceiling",
    source_path="selfdrive/controls/lib/unified_lead.py",
    original=b"    a_cmd = min(a_cmd, speed_ceiling_accel(v_e, float(v_ceiling)))\n",
    replacement=b"    a_cmd = a_cmd\n",
    test_nodes=(
      f"{LEAD_LEAVING_TEST_PATH}::test_unified_leave_weight_releases_brake_but_keeps_max_ceiling",
      f"{LEAD_LEAVING_TEST_PATH}::test_planner_leaving_release_respects_max_ceiling",
    ),
  ),
  HistoricalMutation(
    name="planner-cycle-log-never-closes-window",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=b"    if self.n < self.every_n:\n",
    replacement=b"    if True:\n",
    test_nodes=(
      f"{UNIFIED_PLANNER_TEST_PATH}::test_cycle_time_log_reports_mean_and_max_per_window",
    ),
  ),
)
