"""Roundabout map + Roundabout Steering Assist mutations for the Tesla Pre-AP gate.

Kept separate from tesla_preap_longitudinal_mutations.py (which stays the parent list).
Reuses that file's HistoricalMutation / apply / pytest / JUnit helpers so a mutation is
KILLED only by assertion failures, sources are restored byte-identically, and any survivor
or invalid run fails the job.
"""
import importlib.util
import sys
import tempfile
from pathlib import Path


_PARENT_PATH = Path(__file__).resolve().parent / "tesla_preap_longitudinal_mutations.py"
_spec = importlib.util.spec_from_file_location("tesla_preap_longitudinal_mutations", _PARENT_PATH)
assert _spec is not None and _spec.loader is not None
parent = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = parent
_spec.loader.exec_module(parent)

HistoricalMutation = parent.HistoricalMutation
RB_GUIDE_TEST_PATH = "selfdrive/controls/lib/tests/test_roundabout_guide.py"
RB_MAP_TEST_PATH = "selfdrive/mapd/tests/test_roundabout_map.py"
EXPECTED_COUNT = 23


MUTATIONS = (
  HistoricalMutation(
    name="rb-guide-early-left-before-turn-distance",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      k_raw = max(model_k, k_pp)   # hold / add right only; never an early left\n',
    replacement=b'      k_raw = k_pp\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_right_entry_held_until_turn_distance",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-correction-not-slewed",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    self.dk_out += min(max(dk - self.dk_out, -OUT_SLEW * dt), OUT_SLEW * dt)\n',
    replacement=b'    self.dk_out = dk\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_within_jerk_limit",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-output-exceeds-jerk-limit",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      allowed = max(OUT_SLEW * dt, abs(model_k - self.prev_model))\n',
    replacement=b'      allowed = 1.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_within_jerk_limit",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-no-preview",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    px, py, ppsi = _arc_ahead(x, y, psi, v, yaw_rate, PREVIEW_S)\n',
    replacement=b'    px, py, ppsi = x, y, psi\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_assist_keeps_both_passes_in_the_ring_band",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-circle-follow-wrong-sign",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'  return -sense * k_center\n',
    replacement=b'  return sense * k_center\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_circle_follow_sign_convention",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-no-map-bias",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    bn += BIAS_GAIN * (ex - bn)\n',
    replacement=b'    bn += 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_map_match_bias_removes_gnss_offset_beyond_a_lane",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ignores-fix-age",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    c_fix = _interp(t - self.last_fix_t, (FIX_FRESH_S, FIX_STALE_S), (1.0, 0.0))\n',
    replacement=b'    c_fix = 1.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_pose_error_lowers_confidence_and_fades",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ignores-camera-edges",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    dy = max(dy, min(0.0, allowed))\n',
    replacement=b'    dy = dy\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_edge_limit_keeps_camera_authority",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-never-hands-back",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if self.phase == EXITING and self.phase_w <= 0.0 and abs(self.dk_out) < HANDOFF_DK:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_hands_back_to_model_after_exit",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-ignores-toggle",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'          on = bool(self.params.get_bool(PARAM_ROUNDABOUT_ASSIST))\n',
    replacement=b'          on = True\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_off_or_not_preap",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-not-preap-only",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if not self.preap:\n      return float(model_k)\n',
    replacement=b'    if False:\n      return float(model_k)\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_off_or_not_preap",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-runs-during-maneuver",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if maneuver_active or lane_change_active:\n      on = False\n',
    replacement=b'    if False:\n      on = False\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_driver_has_wheel_or_maneuver",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-without-hint",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    on = (t - self._last_hint_t) <= HINT_HOLD_S or busy\n',
    replacement=b'    on = True\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_stale_ring_does_not_engage",
    ),
  ),
  HistoricalMutation(
    name="rb-controlsd-legacy-bias-stacks-on-assist",
    source_path="selfdrive/controls/controlsd.py",
    original=b'(0.0 if self.rb_assist.active else rb_bias)',
    replacement=b'rb_bias',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_controlsd_applies_assist_before_legacy_bias_and_pins_nothing_else",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-default-on",
    source_path="common/params_keys.h",
    original=b'{"NAPRoundaboutAssist", {PERSISTENT, BOOL, "0"}}',
    replacement=b'{"NAPRoundaboutAssist", {PERSISTENT, BOOL, "1"}}',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_params_and_menu_wiring",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-nan-odometry-poisons-pose",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if not (math.isfinite(t) and math.isfinite(v) and math.isfinite(yaw_rate)):\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_nan_odometry_sample_does_not_poison_the_pose",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-geometry-searches-around-car",
    source_path="selfdrive/mapd/osm_db.py",
    original=b'      lat, lon = 0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3])\n',
    replacement=b'      lat, lon = float(lat), float(lon)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_v4_pack_ring_geometry_from_first_hint_distance",
    ),
  ),
  HistoricalMutation(
    name="rb-map-find-roundabout-skips-rb-ways",
    source_path="selfdrive/mapd/osm_db.py",
    original=b'    rows += [r for r in self._rb_candidates(float(lat), float(lon), RB_SEARCH_PAD_DEG, role=ROLE_RING)\n',
    replacement=b'    rows += [r for r in []\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_v4_pack_find_roundabout_sees_untagged_speed_ring",
    ),
  ),
  HistoricalMutation(
    name="rb-map-overpass-drops-rings",
    source_path="selfdrive/mapd/overpass.py",
    original=b'  rb_union = "" if not include_roundabouts else "\\n  .rb;\\n  .ap;"\n',
    replacement=b'  rb_union = ""\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_overpass_queries_include_rings_and_approaches",
    ),
  ),
  HistoricalMutation(
    name="rb-map-refresh-drops-rb-rows",
    source_path="selfdrive/mapd/local_refresh.py",
    original=b'      for row in rb_rows:\n        OsmSpeedLimitDB.insert_rb_way(con, row)\n',
    replacement=b'      for row in []:\n        OsmSpeedLimitDB.insert_rb_way(con, row)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_refresh_merge_replaces_rb_rows_in_bbox",
    ),
  ),
  HistoricalMutation(
    name="rb-map-approach-direction-lost",
    source_path="selfdrive/mapd/roundabout_map.py",
    original=b'    fwd = list(reversed(trimmed)) if inbound else trimmed\n',
    replacement=b'    fwd = trimmed\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_approaches_are_stored_in_travel_direction",
    ),
  ),
  HistoricalMutation(
    name="rb-map-comfort-speed-outer-lane",
    source_path="selfdrive/mapd/roundabout_map.py",
    original=b'  r_inner = max(4.0, float(radius_m) - 0.5 * (lanes_n - 1) * LANE_WIDTH_M)\n',
    replacement=b'  r_inner = max(4.0, float(radius_m) + 0.5 * (lanes_n - 1) * LANE_WIDTH_M)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_comfort_speed_soco_about_15_mph",
    ),
  ),
  HistoricalMutation(
    name="rb-map-builder-skips-roundabouts",
    source_path="scripts/nap/build_osm_speed_limits.py",
    original=b'      rb_rows = rb_rows_from_overpass(payload)\n',
    replacement=b'      rb_rows = []\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_builder_from_json_writes_pack_v4",
    ),
  ),
)


def main() -> int:
  if len(MUTATIONS) != EXPECTED_COUNT:
    print(f"INVALID: expected {EXPECTED_COUNT} roundabout mutations, found {len(MUTATIONS)}")
    return 1
  with tempfile.TemporaryDirectory(prefix="tesla-preap-roundabout-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = parent.run_pytest((RB_GUIDE_TEST_PATH, RB_MAP_TEST_PATH), baseline_xml)
    if baseline.returncode != 0:
      print("BASELINE FAILED: roundabout tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = parent.junit_testcases(baseline_xml)
    except parent.JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} roundabout tests")

    survivors = []
    for mutation in MUTATIONS:
      source_path = None
      original_source = None
      mutation_result = None
      mutation_error = None
      restored = False
      mutation_xml = temp_root / f"{mutation.name}.xml"
      try:
        source_path, original_source = parent.apply_mutation(mutation)
        mutation_result = parent.run_pytest(mutation.test_nodes, mutation_xml)
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
      try:
        mutation_testcases = parent.junit_testcases(mutation_xml)
      except parent.JUnitReportError as exc:
        print(f"INVALID: {mutation.name} {exc}")
        return 1
      if mutation_result.returncode == 1 and parent.has_only_assertion_failures(mutation_testcases):
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
      print(f"Roundabout mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} roundabout mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
