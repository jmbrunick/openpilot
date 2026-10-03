"""Lateral-yield (A cancels / B never cancels) mutations for the Tesla Pre-AP gate.

Panda and the card infer "lateral yielded" from the 0x488 type-1 send history
(opendbc preap/lat_yield.py, safety/modes/tesla_preap_latyield.h). The opendbc
repo's own gate kills the panda / tracker mutations; this one covers what only
runs with cereal: the CarController stamp and block, the card <-> car_specific
plumbing, the early yield, and the guard that ties the handoff timing to the
inference windows.

Kept separate from tesla_preap_longitudinal_mutations.py (the parent list) and
reuses its HistoricalMutation / apply / pytest / JUnit helpers: a mutation is
KILLED only by assertion failures, sources are restored byte-identically, and
any survivor or invalid run fails the job.
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
TL = "selfdrive/car/tesla/tests/test_preap_lat_yield.py"
TE = "selfdrive/controls/lib/tests/test_driver_lateral_handoff_early_yield.py"
TG = "selfdrive/controls/lib/tests/test_lat_yield_inference_guard.py"
CARD = "selfdrive/car/tesla/preap_blinker_lat_pause.py"
CAR_SPECIFIC = "selfdrive/car/car_specific.py"
NUDGE = "selfdrive/controls/lib/lane_change_nudge.py"
HANDOFF = "selfdrive/controls/lib/driver_lateral_handoff.py"
CONTROLSD = "selfdrive/controls/controlsd.py"
CARCONTROLLER = "opendbc_repo/opendbc/car/tesla/carcontroller.py"
LAT_YIELD = "opendbc_repo/opendbc/car/tesla/preap/lat_yield.py"
HEADER = "opendbc_repo/opendbc/safety/modes/tesla_preap_latyield.h"
EXPECTED_COUNT = 22


def _m(name, source_path, original, replacement, *test_nodes):
  return HistoricalMutation(name, source_path, original, replacement, tuple(test_nodes))


MUTATIONS = (
  # -- card <-> car_specific plumbing (B must not cancel, A must) ----------
  _m("card-handoff-ignores-lat-full-control", CARD,
     b"  _ORIG_HANDLE(self, steering_disengage, lat_full_control)\n",
     b"  _ORIG_HANDLE(self, steering_disengage)\n",
     f"{TL}::test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long"),
  _m("card-stash-always-says-full-control", CARD,
     b"encode_hands_stash(hands, bool(full_control))", b"encode_hands_stash(hands, True)",
     f"{TL}::test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long"),
  _m("car-specific-ignores-the-stash", CAR_SPECIFIC,
     b"        lat_full_control=lat_full_control):", b"        lat_full_control=True):",
     f"{TL}::test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long"),
  _m("steer-disengage-event-never-fires", NUDGE,
     b"  if confirm or not lat_full_control:\n", b"  if True:\n",
     f"{TL}::test_A_yank_on_full_lateral_cancels_everything"),
  # -- CarController: the 0x488 frames ARE the yield signal --------------
  _m("carcontroller-never-stamps-type1", CARCONTROLLER,
     b"      lat_yield.note_steer_tx(bool(lat_active), engaged=bool(CS.engagement.cruiseEnabled))\n",
     b"      pass\n",
     f"{TL}::test_carcontroller_stamps_the_type1_frames_it_sends",
     f"{TL}::test_A_yank_on_full_lateral_cancels_everything"),
  _m("carcontroller-stamps-type0-frames", CARCONTROLLER,
     b"lat_yield.note_steer_tx(bool(lat_active),", b"lat_yield.note_steer_tx(True,",
     f"{TL}::test_carcontroller_type0_frames_do_not_stamp",
     f"{TL}::test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long"),
  _m("carcontroller-ignores-the-b-edge-block", CARCONTROLLER,
     b" and CS.hands_on_level < 3 and not lat_yield.blocked\n", b" and CS.hands_on_level < 3\n",
     f"{TL}::test_carcontroller_refuses_type1_while_blocked_and_resumes_after_release",
     f"{TL}::test_B_yank_on_yielded_lateral_cancels_nothing_and_keeps_long"),
  # -- early yield on steeringPressed ------------------------------------
  _m("early-yield-disabled", HANDOFF,
     b"    return self._early_cnt >= EARLY_YIELD_FRAMES\n", b"    return False\n",
     f"{TE}::test_sustained_same_sign_push_yields_without_hands"),
  _m("early-yield-ignores-sign-flips", HANDOFF,
     b"        self._early_cnt == 0 or sign == self._early_sign):", b"        True):",
     f"{TE}::test_alternating_sign_chatter_does_not_yield"),
  _m("early-yield-ignores-steering-pressed", HANDOFF,
     b"    if steering_pressed and mag >= EARLY_YIELD_NM and (",
     b"    if mag >= EARLY_YIELD_NM and (",
     f"{TE}::test_requires_steering_pressed"),
  _m("early-yield-threshold-1nm", HANDOFF,
     b"EARLY_YIELD_NM = 2.0\n", b"EARLY_YIELD_NM = 1.0\n",
     f"{TE}::test_below_threshold_does_not_yield", f"{TG}::test_handoff_constants_are_pinned"),
  _m("controlsd-drops-steering-pressed", CONTROLSD,
     b"      steering_pressed=bool(CS.steeringPressed),\n    )\n    CC.latActive = lat_active_after_handoff(",
     b"    )\n    CC.latActive = lat_active_after_handoff(",
     f"{TE}::test_controlsd_and_card_pass_steering_pressed_to_the_handoff"),
  _m("card-handoff-drops-steering-pressed", CARD,
     b"    steering_pressed=steering_pressed,\n    dt=dt,\n", b"    dt=dt,\n",
     f"{TL}::test_card_handoff_forwards_steering_pressed_to_the_early_yield"),
  # -- guard: handoff timing vs inference windows -----------------------
  _m("handoff-blend-diverges-from-inference", HANDOFF,
     b"BLEND_TIME_S = 1.0\n", b"BLEND_TIME_S = 1.6\n",
     f"{TG}::test_handoff_timing_matches_what_the_inference_assumes",
     f"{TG}::test_handoff_constants_are_pinned"),
  _m("handoff-confirm-diverges-from-inference", HANDOFF,
     b"HANDS_OFF_CONFIRM_S = 0.15\n", b"HANDS_OFF_CONFIRM_S = 0.05\n",
     f"{TG}::test_handoff_timing_matches_what_the_inference_assumes",
     f"{TG}::test_handoff_constants_are_pinned"),
  _m("inference-grace-shorter-than-blend", LAT_YIELD,
     b"GRACE_S = 1.0\n", b"GRACE_S = 0.5\n",
     f"{TG}::test_grace_covers_the_blend_and_gap_sits_below_the_shortest_yield",
     f"{TG}::test_blend_and_yield_read_correctly_through_the_real_handoff"),
  _m("inference-gap-above-shortest-yield", LAT_YIELD,
     b"GAP_S = 0.14\n", b"GAP_S = 0.30\n",
     f"{TG}::test_grace_covers_the_blend_and_gap_sits_below_the_shortest_yield",
     f"{TG}::test_blend_and_yield_read_correctly_through_the_real_handoff"),
  # -- guard: the contract notes must stay where an editor will see them -
  _m("contract-note-removed-from-handoff-module", HANDOFF,
     b"INFERENCE CONTRACT (lateral yield is inferred, never announced):",
     b"NOTE (lateral yield is inferred, never announced):",
     f"{TG}::test_inference_contract_comment_present"),
  _m("contract-note-removed-from-lat-active-doc", HANDOFF,
     b"  INFERENCE CONTRACT: this bit becomes", b"  NOTE: this bit becomes",
     f"{TG}::test_lat_active_after_handoff_documents_the_contract"),
  _m("contract-note-removed-from-carcontroller", CARCONTROLLER,
     b"# INFERENCE CONTRACT: type 1 must mean", b"# NOTE: type 1 must mean",
     f"{TG}::test_inference_contract_comment_present"),
  _m("contract-note-removed-from-lat-yield-module", LAT_YIELD,
     b"INFERENCE CONTRACT (read before touching", b"NOTE (read before touching",
     f"{TG}::test_inference_contract_comment_present"),
  _m("contract-note-removed-from-safety-header", HEADER,
     b"// INFERENCE CONTRACT: openpilot does not send panda", b"// NOTE: openpilot does not send panda",
     f"{TG}::test_inference_contract_comment_present"),
)

BASELINE_NODES = (TL, TE, TG)


def main() -> int:
  if len(MUTATIONS) != EXPECTED_COUNT:
    print(f"INVALID: expected {EXPECTED_COUNT} lateral-yield mutations, found {len(MUTATIONS)}")
    return 1
  with tempfile.TemporaryDirectory(prefix="tesla-preap-lat-yield-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = parent.run_pytest(BASELINE_NODES, baseline_xml)
    if baseline.returncode != 0:
      print("BASELINE FAILED: lateral-yield tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = parent.junit_testcases(baseline_xml)
    except parent.JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} lateral-yield tests")

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

      if source_path is not None and not restored:
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
      print(f"Lateral-yield mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} lateral-yield mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
