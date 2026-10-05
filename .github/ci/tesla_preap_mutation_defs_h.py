"""Regen-limit prompt command-pin mutation definitions (see tesla_preap_longitudinal_mutations.py).

The Pre-AP "regen limit, add brake" prompt triggers on the unclamped plan. With
the post-guard actuator command available it must also be pinned at the
envelope floor, an unknown command keeps the plan-only rule, and an active
prompt is kept by the plan alone. Each entry breaks one rule and must fail the
pinned tests with an AssertionError.
"""
from tesla_preap_mutation_common import HistoricalMutation

T = "selfdrive/selfdrived/tests/test_preap_regen_cmd_pin.py"
PR = "selfdrive/selfdrived/preap_regen.py"
SD = "selfdrive/selfdrived/selfdrived.py"


def _m(name, path, original, replacement, *tests):
  return HistoricalMutation(
    name=name, source_path=path, original=original, replacement=replacement,
    test_nodes=tuple(f"{T}::{t}" for t in tests),
  )


MUTATIONS_H = (
  _m("regen-pin-margin-widened", PR,
     b"REGEN_DEMAND_CMD_PIN_MARGIN = 0.05  #", b"REGEN_DEMAND_CMD_PIN_MARGIN = 1.5  #",
     "test_constants_are_pinned", "test_phantom_prompt_is_gone_when_the_command_is_not_at_the_rail",
     "test_command_just_inside_the_pin_margin_prompts_just_outside_does_not"),
  _m("regen-pin-margin-zero", PR,
     b"REGEN_DEMAND_CMD_PIN_MARGIN = 0.05  #", b"REGEN_DEMAND_CMD_PIN_MARGIN = 0.0  #",
     "test_constants_are_pinned", "test_command_just_inside_the_pin_margin_prompts_just_outside_does_not"),
  _m("regen-pin-ignored", PR,
     b"      and cmd_pinned\n", b"",
     "test_phantom_prompt_is_gone_when_the_command_is_not_at_the_rail",
     "test_a_command_off_the_rail_cannot_build_evidence_between_pinned_frames",
     "test_never_more_prompts_than_the_plan_only_rule"),
  _m("regen-pin-inverted", PR,
     b"      or a_cmd <= accel_floor + REGEN_DEMAND_CMD_PIN_MARGIN\n",
     b"      or a_cmd >= accel_floor + REGEN_DEMAND_CMD_PIN_MARGIN\n",
     "test_phantom_prompt_is_gone_when_the_command_is_not_at_the_rail",
     "test_real_overflow_still_prompts_when_the_command_is_pinned"),
  _m("regen-unknown-command-suppresses", PR,
     b"      a_cmd is None or not math.isfinite(a_cmd)\n      or a_cmd <=",
     b"      a_cmd is not None and math.isfinite(a_cmd)\n      and a_cmd <=",
     "test_unknown_command_is_the_plan_only_rule"),
  _m("regen-nonfinite-command-suppresses", PR,
     b"      a_cmd is None or not math.isfinite(a_cmd)\n      or a_cmd <=",
     b"      a_cmd is None\n      or a_cmd <=",
     "test_unknown_command_is_the_plan_only_rule"),
  _m("regen-keep-prompting-needs-the-command", PR,
     b"        and a_target <= accel_floor - REGEN_DEMAND_CLEAR_MARGIN\n",
     (b"        and a_target <= accel_floor - REGEN_DEMAND_CLEAR_MARGIN\n"
      + b"        and (a_cmd is None or a_cmd <= accel_floor + REGEN_DEMAND_CMD_PIN_MARGIN)\n"),
     "test_active_prompt_keeps_going_on_the_plan_alone"),
  _m("regen-selfdrived-drops-the-command", SD,
     b"          a_cmd=float(self.sm['carControl'].actuators.accel),\n", b"",
     "test_selfdrived_passes_the_guarded_command"),
  _m("regen-selfdrived-passes-the-plan-as-command", SD,
     b"          a_cmd=float(self.sm['carControl'].actuators.accel),\n",
     b"          a_cmd=float(self.sm['longitudinalPlan'].aTarget),\n",
     "test_selfdrived_passes_the_guarded_command"),
)
