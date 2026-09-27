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
OFFSET_TEST_PATH = "selfdrive/mapd/tests/test_map_speed_offset_slew.py"
ENGAGE_MAX_TEST_PATH = "selfdrive/car/tests/test_preap_engage_max_after_pause.py"
FRONTAGE_TEST_PATH = "selfdrive/mapd/tests/test_map_match_frontage.py"
LANE_CHANGE_TARGET_TEST_PATH = "selfdrive/controls/lib/tests/test_lane_change_target.py"
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
    original=b"        elif locked and self._opposite_pull(carstate, nudge_dir):\n",
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
      f"{LANE_CHANGE_TARGET_TEST_PATH}::test_low_confidence_lane_lines_cancel" if False else
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
