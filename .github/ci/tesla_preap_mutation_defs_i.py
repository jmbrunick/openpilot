"""Far-gap kinematic gate widening mutation definitions (see tesla_preap_longitudinal_mutations.py).

The unified lead controller widens its kinematic gate at a far gap while the PD
term accelerates and the kinematic bound is not a real brake. It must stay
inert for slack under 15 m, a real bound, or a non-accelerating PD, never be
firmer than the old gate, be capped, and be continuous. Each entry breaks one
rule and must fail the pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import HistoricalMutation

T = "selfdrive/controls/tests/test_unified_far_gap_gate.py"
UL = "selfdrive/controls/lib/unified_lead.py"

_BIT = "test_bit_identical_to_the_old_gate_inside_the_far_gap_line"
_FIRM = "test_never_firmer_and_unchanged_where_the_old_command_braked"
_ZERO = "test_widen_is_zero_unless_all_three_conditions_hold"
_CONST = "test_constants_are_pinned"
_E2 = "test_far_gap_slow_close_keeps_the_pd_accel_through_the_old_gate_edge"
_CONT = "test_widen_is_continuous_in_slack_bound_and_pd"


def _m(name, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=UL, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


MUTATIONS_I = (
  _m("gate-widen-slack-line-lowered",
     b"KIN_GATE_WIDEN_SLACK_LO_M = 15.0\n", b"KIN_GATE_WIDEN_SLACK_LO_M = 5.0\n",
     _CONST, _BIT, _ZERO),
  _m("gate-widen-slack-ramp-hardened",
     b"KIN_GATE_WIDEN_SLACK_RAMP_M = 5.0\n", b"KIN_GATE_WIDEN_SLACK_RAMP_M = 0.01\n",
     _CONST, _CONT),
  _m("gate-widen-real-bound-threshold-lowered",
     b"KIN_GATE_WIDEN_A_BOUND_LO = -0.10\n", b"KIN_GATE_WIDEN_A_BOUND_LO = -0.60\n",
     _CONST, _FIRM, _ZERO),
  _m("gate-widen-bound-ramp-hardened",
     b"KIN_GATE_WIDEN_A_BOUND_RAMP = 0.10\n", b"KIN_GATE_WIDEN_A_BOUND_RAMP = 0.001\n",
     _CONST, _CONT),
  _m("gate-widen-pd-cap-raised",
     b"KIN_GATE_WIDEN_A_CAP = 0.80\n", b"KIN_GATE_WIDEN_A_CAP = 2.0\n",
     _CONST, "test_widen_full_weight_and_cap", _FIRM),
  _m("gate-widen-gain-halved",
     b"KIN_GATE_WIDEN_K = 1.0  #", b"KIN_GATE_WIDEN_K = 0.5  #",
     _CONST, "test_widen_full_weight_and_cap"),
  _m("gate-widen-ignores-slack",
     b"  return wide * w_slack * w_bound\n", b"  return wide * w_bound\n",
     _BIT, _ZERO),
  _m("gate-widen-ignores-real-bound",
     b"  return wide * w_slack * w_bound\n", b"  return wide * w_slack\n",
     _FIRM, _ZERO),
  _m("gate-widen-without-pd-headroom",
     b"  wide = min(a_pd, KIN_GATE_WIDEN_A_CAP) / KIN_GATE_WIDEN_K - KIN_GATE_MS\n",
     b"  wide = min(a_pd, KIN_GATE_WIDEN_A_CAP) / KIN_GATE_WIDEN_K\n",
     _ZERO, _BIT, _FIRM),
  _m("gate-widen-pd-uncapped",
     b"  wide = min(a_pd, KIN_GATE_WIDEN_A_CAP) / KIN_GATE_WIDEN_K - KIN_GATE_MS\n",
     b"  wide = a_pd / KIN_GATE_WIDEN_K - KIN_GATE_MS\n",
     "test_widen_full_weight_and_cap"),
  _m("gate-widen-bound-step-not-smooth",
     b"  w_bound = _smooth01((a_bound - KIN_GATE_WIDEN_A_BOUND_LO) / KIN_GATE_WIDEN_A_BOUND_RAMP)\n",
     b"  w_bound = float(a_bound > KIN_GATE_WIDEN_A_BOUND_LO)\n",
     _CONT),
  _m("gate-widen-slack-step-not-smooth",
     b"  w_slack = _smooth01((slack - KIN_GATE_WIDEN_SLACK_LO_M) / KIN_GATE_WIDEN_SLACK_RAMP_M)\n",
     b"  w_slack = float(slack > KIN_GATE_WIDEN_SLACK_LO_M)\n",
     _CONT),
  _m("gate-widen-narrows-instead",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_pd, a_bound)\n",
     b"  gate_w = KIN_GATE_MS - _kin_gate_widen(slack, a_pd, a_bound)\n",
     _FIRM, _E2, "test_close_sweep_has_a_smaller_step_than_the_old_gate"),
  _m("gate-widen-not-wired",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_pd, a_bound)\n",
     b"  gate_w = KIN_GATE_MS\n",
     _E2, "test_close_sweep_has_a_smaller_step_than_the_old_gate", _FIRM),
  _m("gate-widen-wired-with-swapped-args",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_pd, a_bound)\n",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_bound, a_pd)\n",
     _E2, _FIRM),
  _m("gate-widen-wired-with-gap-not-slack",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(slack, a_pd, a_bound)\n",
     b"  gate_w = KIN_GATE_MS + _kin_gate_widen(gap_f, a_pd, a_bound)\n",
     _BIT, "test_inside_the_follow_gap_and_real_closes_match_the_old_command"),
)
