"""Far-gap soft closing-speed mutation definitions.

The catch-up blend fades k_g·slack from 15 → 50 m and replaces it with a soft
desired closing speed, and only while the classic PD term is a catch-up and
the kinematic bound is not a real brake. Each entry breaks one of those guards
and must fail the pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import HistoricalMutation

T = "selfdrive/controls/tests/test_unified_far_gap_catchup.py"
UL = "selfdrive/controls/lib/unified_lead.py"

_BIT = "test_bit_identical_for_slack_at_or_under_15m"
_SLACK = "test_slack_weight_keeps_the_near_gap_term"
_ACCEL = "test_accel_weight_keeps_a_braking_pd_term"
_BOUND = "test_bound_weight_keeps_the_gap_term_on_a_real_brake"
_BRAKE = "test_braking_lead_at_a_far_gap_gets_no_catchup_push"
_SOFT = "test_matched_far_command_is_a_soft_ease"
_CLIP = "test_closing_faster_than_desired_adds_no_gap_brake"
_PROP = "test_never_firmer_on_catchup_and_never_deeper_when_classic_brakes"
_NOV17 = "test_nov17_far_catchup_is_a_taper_not_a_push_and_drop"
_SLOW = "test_slowing_lead_never_gets_a_harder_catchup_than_a_steady_one"
_ONLY = "test_planned_approach_ceiling_only_removes_accel"


def _m(name, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=UL, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


_W = b"  w = w_slack * w_accel * w_bound\n"


MUTATIONS_J = (
  _m("catchup-drops-slack-weight",
     _W, b"  w = w_accel * w_bound\n",
     _SLACK, _BIT, _PROP),
  _m("catchup-drops-accel-weight",
     _W, b"  w = w_slack * w_bound\n",
     _ACCEL, _PROP),
  _m("catchup-drops-bound-weight",
     _W, b"  w = w_slack * w_accel\n",
     _BOUND, _BRAKE, _PROP),
  _m("catchup-gain-raised-fivefold",
     b"K_CATCH = 0.14", b"K_CATCH = 1.00",
     _SOFT),
  _m("catchup-negative-soft-term-not-clipped",
     b"  a_gap = (1.0 - w) * a_gap_classic + w * max(0.0, a_soft)\n",
     b"  a_gap = (1.0 - w) * a_gap_classic + w * a_soft\n",
     _CLIP),
  # Nov 17: planned-approach ceiling (the soft law without the bound weight).
  _m("catchup-plan-ceiling-removed",
     b"    a_cmd = min(a_cmd, max(a_far, 0.0))\n",
     b"    a_cmd = a_cmd\n",
     _NOV17, _SLOW, _BRAKE, _ONLY),
  _m("catchup-plan-ceiling-may-brake",
     b"    a_cmd = min(a_cmd, max(a_far, 0.0))\n",
     b"    a_cmd = min(a_cmd, a_far)\n",
     _ONLY),
  _m("catchup-plan-keeps-the-bound-weight",
     b"CATCHUP_PLAN_A_BOUND = 0.0\n",
     b"CATCHUP_PLAN_A_BOUND = -1.0\n",
     _NOV17, _SLOW, _BRAKE),
)
