"""Curve-follow mutation definitions (see tesla_preap_longitudinal_mutations.py).

The continuous curve term (NAPCurveFollow) ships in shadow first. Each entry
breaks one rule of the term, its wiring, or the shadow guarantee (commands
and MAX untouched) and must fail the pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import (
  CURVE_FOLLOW_CARD_TEST_PATH,
  CURVE_FOLLOW_PLANNER_TEST_PATH,
  CURVE_FOLLOW_TEST_PATH,
  HistoricalMutation,
)


MUTATIONS_D = (
  HistoricalMutation(
    name="release-buffer-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'float(d) - RELEASE_BEFORE_TIGHTEST_S * v - min_room'
    ),
    replacement=(
      b'float(d) - 0.0 * v - min_room'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_release_buffer_is_one_named_constant_and_is_wired",
    ),
  ),
  HistoricalMutation(
    name="spike-clip-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'if nb and kc[i] > SPIKE_MIN_K and kc[i] > SPIKE_RATIO * max(nb):'
    ),
    replacement=(
      b'if False and nb and kc[i] > SPIKE_MIN_K and kc[i] > SPIKE_RATIO * max(nb):'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_curvature_filter_clips_a_spike_and_opens_slowly",
    ),
  ),
  HistoricalMutation(
    name="k-filter-opens-as-fast-as-it-tightens",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'if x > cur else K_DOWN_TAU_S'
    ),
    replacement=(
      b'if x > cur else K_UP_TAU_S'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_curvature_filter_clips_a_spike_and_opens_slowly",
    ),
  ),
  HistoricalMutation(
    name="k-filter-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'self._kf[i] = cur + dt / (tau + dt) * (x - cur)'
    ),
    replacement=(
      b'self._kf[i] = x'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_curvature_filter_clips_a_spike_and_opens_slowly",
    ),
  ),
  HistoricalMutation(
    name="snr-accumulates-unsigned-curvature",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'k_mid = 0.5 * (float(ks[i]) + float(ks[i - 1]))'
    ),
    replacement=(
      b'k_mid = 0.5 * (abs(float(ks[i])) + abs(float(ks[i - 1])))'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_zero_mean_curvature_noise_does_not_accumulate_into_a_bend",
    ),
  ),
  HistoricalMutation(
    name="snr-gate-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'w[i] = _smooth01((abs(y) / std - SNR_S0) / (SNR_S1 - SNR_S0))'
    ),
    replacement=(
      b'w[i] = 1.0'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_high_position_std_silences_far_points_low_std_trusts_them",
    ),
  ),
  HistoricalMutation(
    name="position-std-ignored",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'std = max(STD_FLOOR_M, float(y_std[i]))'
    ),
    replacement=(
      b'std = STD_FLOOR_M'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_high_position_std_silences_far_points_low_std_trusts_them",
    ),
  ),
  HistoricalMutation(
    name="low-speed-gate-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'gate = 1.0 - (slow if not blinker else 0.0)'
    ),
    replacement=(
      b'gate = 1.0'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_low_speed_rules",
    ),
  ),
  HistoricalMutation(
    name="blinker-does-not-open-the-low-speed-gate",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'gate = 1.0 - (slow if not blinker else 0.0)'
    ),
    replacement=(
      b'gate = 1.0 - slow'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_low_speed_rules",
    ),
  ),
  HistoricalMutation(
    name="low-speed-horizon-cut-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'far_cut = slow * _smooth01((float(t) - LOW_HORIZON_S) / LOW_HORIZON_BAND_S)'
    ),
    replacement=(
      b'far_cut = 0.0'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_low_speed_rules",
    ),
  ),
  HistoricalMutation(
    name="lane-change-does-not-fade-the-term",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'ctx_ok = not (lane_change or rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)'
    ),
    replacement=(
      b'ctx_ok = not (rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_lane_change_fades_the_term_out_without_a_step",
    ),
  ),
  HistoricalMutation(
    name="roundabout-funnel-does-not-fade-the-term",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'ctx_ok = not (lane_change or rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)'
    ),
    replacement=(
      b'ctx_ok = not (lane_change or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_active_roundabout_funnel_fades_the_term_out",
    ),
  ),
  HistoricalMutation(
    name="frame-drops-do-not-fade-the-term",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'ctx_ok = not (lane_change or rb_active or float(frame_drop_pct) > FRAME_DROP_MAX_PCT)'
    ),
    replacement=(
      b'ctx_ok = not (lane_change or rb_active)'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_dropped_frames_fade_the_term_out_and_light_drops_do_not",
    ),
  ),
  HistoricalMutation(
    name="confidence-steps-instead-of-ramping",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'self._conf = min(1.0, max(0.0, self._conf + min(1.0, rate * dt) * (target - self._conf)))'
    ),
    replacement=(
      b'self._conf = target'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_lane_change_fades_the_term_out_without_a_step",
    ),
  ),
  HistoricalMutation(
    name="brake-build-jerk-limit-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'build = min(BUILD_MAX_MS3, base + BUILD_PROP_1_S * (-err)) * dt'
    ),
    replacement=(
      b'build = 1e9 * dt'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_constant_radius_bend_slows_early_gently_and_to_the_table",
    ),
  ),
  HistoricalMutation(
    name="brake-release-rate-unbounded",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'release = min(RELEASE_MAX_MS3, rel_base + RELEASE_PROP_1_S * max(0.0, err)) * dt'
    ),
    replacement=(
      b'release = 1e9 * dt'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_proportional_release_rate_is_bounded",
    ),
  ),
  HistoricalMutation(
    name="near-field-term-allowed-to-brake-hard",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'max(over, -NEAR_BRAKE_MAX_MS2)'
    ),
    replacement=(
      b'over'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_near_field_term_is_a_lift_not_a_brake",
    ),
  ),
  HistoricalMutation(
    name="apex-detection-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'self.passed = _smooth01((PEAK_RATIO0 - k_ahead / k_now) / PEAK_BAND)'
    ),
    replacement=(
      b'self.passed = 0.0'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_apex_detection_reads_the_path_peaking",
    ),
  ),
  HistoricalMutation(
    name="relief-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'relief = r0 * (1.0 - _smooth01((v - v_env) / max(v_prof - v_env, 0.5)))'
    ),
    replacement=(
      b'relief = 0.0'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_s_curve_family_regens_back_down_without_chatter_or_latch",
    ),
  ),
  HistoricalMutation(
    name="decel-limit-removed",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'a_c = min(FREE_A_MS2, max(a_c, -preview_decel_limit_ms2(v)))'
    ),
    replacement=(
      b'a_c = min(FREE_A_MS2, a_c)'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_braking_never_exceeds_the_speed_dependent_decel_limit",
    ),
  ),
  HistoricalMutation(
    name="mode-param-unknown-value-not-shadow",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'return mode if mode in (MODE_OFF, MODE_SHADOW, MODE_ACTIVE) else DEFAULT_MODE'
    ),
    replacement=(
      b'return mode'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_mode_param_reader_defaults_to_shadow_and_clamps",
    ),
  ),
  HistoricalMutation(
    name="mode-param-default-is-active",
    source_path="selfdrive/controls/lib/curve_follow.py",
    original=(
      b'DEFAULT_MODE = MODE_SHADOW'
    ),
    replacement=(
      b'DEFAULT_MODE = MODE_ACTIVE'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_TEST_PATH}::test_mode_param_reader_defaults_to_shadow_and_clamps",
    ),
  ),
  HistoricalMutation(
    name="shadow-overwrites-the-preview",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b'if self._cf_mode == MODE_ACTIVE:\n' +
      b'        self.curve_preview_a = a_cf'
    ),
    replacement=(
      b'if self._cf_mode != MODE_OFF:\n' +
      b'        self.curve_preview_a = a_cf'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_PLANNER_TEST_PATH}::test_shadow_commands_are_bit_identical_to_off",
    ),
  ),
  HistoricalMutation(
    name="reactive-cap-skipped-when-untrusted",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b'cf_owns_curve = self._cf_mode == MODE_ACTIVE and self._cf_trusted and not self._cf_faulted'
    ),
    replacement=(
      b'cf_owns_curve = self._cf_mode == MODE_ACTIVE and not self._cf_faulted'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_PLANNER_TEST_PATH}::test_dead_camera_keeps_the_reactive_cap_in_active",
    ),
  ),
  HistoricalMutation(
    name="reactive-cap-skipped-in-shadow",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b'cf_owns_curve = self._cf_mode == MODE_ACTIVE and self._cf_trusted and not self._cf_faulted'
    ),
    replacement=(
      b'cf_owns_curve = self._cf_mode != MODE_OFF and self._cf_trusted and not self._cf_faulted'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_PLANNER_TEST_PATH}::test_reactive_cap_stays_unless_active_and_trusted",
    ),
  ),
  HistoricalMutation(
    name="fault-does-not-latch",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b'      self._cf_faulted = True\n' +
      b'      self._cf_trusted = False'
    ),
    replacement=(
      b'      self._cf_trusted = False'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_PLANNER_TEST_PATH}::test_a_fault_latches_and_the_old_preview_stays",
    ),
  ),
  HistoricalMutation(
    name="log-failure-reaches-the-command",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b'    except Exception:\n' +
      b'      pass  # logging must never touch the command'
    ),
    replacement=(
      b'    except ZeroDivisionError:\n' +
      b'      pass  # logging must never touch the command'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_PLANNER_TEST_PATH}::test_a_logging_exception_cannot_touch_the_command",
    ),
  ),
  HistoricalMutation(
    name="shadow-retires-the-max-cap",
    source_path="selfdrive/car/card.py",
    original=(
      b'curve_follow_on = getattr(self, "_curve_follow_mode", MODE_SHADOW) == MODE_ACTIVE'
    ),
    replacement=(
      b'curve_follow_on = getattr(self, "_curve_follow_mode", MODE_SHADOW) != MODE_SHADOW'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_CARD_TEST_PATH}::test_off_and_shadow_still_cap_max_in_a_bend",
    ),
  ),
  HistoricalMutation(
    name="active-keeps-the-max-cap",
    source_path="selfdrive/car/card.py",
    original=(
      b'curve_follow_on = getattr(self, "_curve_follow_mode", MODE_SHADOW) == MODE_ACTIVE'
    ),
    replacement=(
      b'curve_follow_on = False'
    ),
    test_nodes=(
      f"{CURVE_FOLLOW_CARD_TEST_PATH}::test_active_never_touches_max",
    ),
  ),
)
