"""Crossing stop-guard mutation definitions (Nov 17 stopped car at a light).

A stopped car at 130 m was classified as a crosser from range-grown radar
lateral noise, the brake eased, and the exit handover commanded +1.08 m/s²
toward it. Each entry reverts one of the three guards and must fail the
pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import HistoricalMutation

T = "selfdrive/controls/tests/test_crossing_stopped_guard.py"
CV = "selfdrive/controls/lib/crossing_vehicle.py"
UL = "selfdrive/controls/lib/unified_lead.py"

_REPLAY = "test_nov17_stopped_car_at_130m_is_not_a_crosser_and_never_accelerates"
_VY = "test_crossing_vy_required_grows_past_the_reference_range"
_GUARD_FN = "test_stop_guard_needs_a_real_stop_and_an_unverified_crosser"
_UNVERIFIED = "test_unverified_exit_handover_does_not_accelerate_toward_a_stop"
_MODEL = "test_model_lead_during_handover_returns_to_braking_at_once"
_FAR_MODEL = "test_model_lead_past_55m_clears_the_crossing_class"


def _m(name, path, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=path, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


MUTATIONS_K = (
  _m("crossing-vy-not-range-scaled", CV,
     b"  return CROSS_VY_MS * max(1.0, float(gap) / CROSS_VY_RANGE_REF_M)\n",
     b"  return CROSS_VY_MS\n",
     _REPLAY, _VY),
  _m("crossing-vision-veto-back-to-55m", CV,
     b"CROSS_VISION_RANGE_M = 150.0", b"CROSS_VISION_RANGE_M = 55.0",
     _FAR_MODEL),
  _m("crossing-stop-guard-removed", UL,
     b"      if crossing_stop_guard_active(need, model_prob, resume_w > 0.0, v_lat, self._cross.direction):\n",
     b"      if False:\n",
     _UNVERIFIED, _MODEL),
  _m("crossing-stop-guard-allows-positive-accel", UL,
     b"        desired = min(float(desired), float(need), 0.0)\n",
     b"        desired = min(float(desired), max(float(need), 0.3))\n",
     _UNVERIFIED),
  _m("crossing-stop-guard-ignores-unverified-exit", CV,
     b"    return not leaving\n",
     b"    return False\n",
     _UNVERIFIED, _GUARD_FN),
  _m("crossing-stop-guard-ignores-model-lead", CV,
     b"  if prob >= CROSS_VISION_PROB:\n    return True\n",
     b"  if False:\n    return True\n",
     _GUARD_FN),
)
