"""Lane-change lock robustness mutations for the Tesla Pre-AP gate.

Kept separate from tesla_preap_longitudinal_mutations.py (the parent list) and reuses its
HistoricalMutation / apply / pytest / JUnit helpers: a mutation is KILLED only by assertion
failures, sources are restored byte-identically, and any survivor or invalid run fails the job.
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
T = "selfdrive/controls/lib/tests/test_lane_change_lock.py"
TARGET = "selfdrive/controls/lib/lane_change_target.py"
DH = "selfdrive/controls/lib/desire_helper.py"
EVENTS = "selfdrive/selfdrived/lane_change_alerts.py"
EXPECTED_COUNT = 11

MUTATIONS = (
  HistoricalMutation(
    name="lc-lock-needs-crossing-line",
    source_path=TARGET,
    original=b"      if probs[1 if direction == LEFT else 2] < LANE_LINE_MIN_PROB:\n",
    replacement=b"      if False:\n",
    test_nodes=(
      "selfdrive/controls/lib/tests/test_lane_change_target.py::test_low_confidence_at_start_does_not_guess",
      f"{T}::test_weak_crossing_line_waits_shows_unclear_and_never_silently_cancels",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-far-line-weak-refuses-again",
    source_path=TARGET,
    original=b"      width = last_width if last_width is not None and _sane_width(last_width) else DEFAULT_LANE_WIDTH_M\n",
    replacement=b"      return None\n",
    test_nodes=(
      f"{T}::test_sep29_1200_48_left_right_line_020_locks_on_left_line",
      f"{T}::test_sep29_1200_51_left_right_line_013_soft_confirm_locks",
      f"{T}::test_right_change_needs_only_the_right_line",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-ignores-last-good-width",
    source_path=TARGET,
    original=b"      width = last_width if last_width is not None and _sane_width(last_width) else DEFAULT_LANE_WIDTH_M\n",
    replacement=b"      width = DEFAULT_LANE_WIDTH_M\n",
    test_nodes=(
      f"{T}::test_far_line_weak_uses_last_good_lane_width",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-crossing-line-wrong-side-accepted",
    source_path=TARGET,
    original=b"      if direction == LEFT and not left_y < 0.0:\n",
    replacement=b"      if False:\n",
    test_nodes=(
      f"{T}::test_lock_refuses_when_the_crossing_line_is_on_the_wrong_side",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-retry-gives-up-at-once",
    source_path=DH,
    original=b"              if self._lock_wait_s >= LOCK_RETRY_S - 1e-9:\n",
    replacement=b"              if True:\n",
    test_nodes=(
      f"{T}::test_brief_crossing_line_dip_retries_and_locks",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-refusal-cancels-silently",
    source_path=DH,
    original=b"                self._lock_gave_up = True\n",
    replacement=b"                self._reset()\n                self._suppress_next_tip = True\n",
    test_nodes=(
      f"{T}::test_weak_crossing_line_waits_shows_unclear_and_never_silently_cancels",
      f"{T}::test_line_returns_after_retry_ran_out_locks_while_nudge_held",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-pending-survives-slow-speed-and-blindspot",
    source_path=DH,
    original=b"        if blindspot_detected or below_lane_change_speed:\n          self._lock_pending = False\n",
    replacement=b"        if False:\n          self._lock_pending = False\n",
    test_nodes=(
      f"{T}::test_pending_lock_is_dropped_if_speed_falls_under_20_mph_before_the_line_returns",
      f"{T}::test_pending_lock_is_dropped_on_blindspot_before_the_line_returns",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-unclear-not-flagged-to-selfdrived",
    source_path=DH,
    original=b"        self.signals_remaining = LANE_LINES_UNCLEAR_SIGNAL\n",
    replacement=b"        pass\n",
    test_nodes=(
      f"{T}::test_weak_crossing_line_waits_shows_unclear_and_never_silently_cancels",
    ),
  ),
  HistoricalMutation(
    name="lc-lock-released-nudge-locks-later",
    source_path=DH,
    original=b"        elif model is not None and (self._lock_pending or (self._lock_gave_up and eligible)):\n",
    replacement=b"        elif model is not None and (self._lock_pending or self._lock_gave_up):\n",
    test_nodes=(
      f"{T}::test_released_nudge_after_retry_ran_out_does_not_lock_later",
    ),
  ),
  HistoricalMutation(
    name="lc-unclear-alert-text-dropped",
    source_path=EVENTS,
    original=b"      unclear = sm['modelV2'].meta.laneChangeSignalsRemaining == LANE_LINES_UNCLEAR_SIGNAL\n",
    replacement=b"      unclear = False\n",
    test_nodes=(
      f"{T}::test_unclear_alert_text_only_when_flagged",
    ),
  ),
  HistoricalMutation(
    name="lc-unclear-alert-not-installed",
    source_path="selfdrive/selfdrived/helpers.py",
    original=b"\ninstall_lane_change_alerts()\n",
    replacement=b"\n",
    test_nodes=(
      f"{T}::test_selfdrived_installs_the_unclear_alert",
    ),
  ),
)


def main() -> int:
  if len(MUTATIONS) != EXPECTED_COUNT:
    print(f"INVALID: expected {EXPECTED_COUNT} lane-change mutations, found {len(MUTATIONS)}")
    return 1
  with tempfile.TemporaryDirectory(prefix="tesla-preap-lane-change-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = parent.run_pytest((T,), baseline_xml)
    if baseline.returncode != 0:
      print("BASELINE FAILED: lane-change target tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = parent.junit_testcases(baseline_xml)
    except parent.JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} lane-change target tests")

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
      print(f"Lane-change mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} lane-change mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
