"""F2: the regen-limit prompt needs the post-guard command pinned at the rail.

15d 15:27:31: plan -1.94 while the guarded actuator sat at -0.22, so the car
was not asking for more than regen could give and the prompt was phantom.
With the command unknown the plan-only rule is unchanged (when in doubt, prompt).
"""
import inspect
import math
from pathlib import Path

from openpilot.selfdrive.selfdrived import preap_regen
from openpilot.selfdrive.selfdrived.preap_regen import (
  REGEN_DEMAND_CMD_PIN_MARGIN,
  REGEN_DEMAND_EVIDENCE_COUNT,
  RegenDemandCheck,
)

FLOOR = -1.5  # get_preap_accel_limits floor


def _run(frames, *, a_target, a_cmd="unset", v_ego=15.0, check=None):
  check = check or RegenDemandCheck()
  fired = False
  for _ in range(frames):
    kw = {} if a_cmd == "unset" else {"a_cmd": a_cmd}
    fired = check.update(pedal_long_active=True, brake_pressed=False,
                         a_target=a_target, v_ego=v_ego, **kw) or fired
  return fired, check


def test_constants_are_pinned():
  assert REGEN_DEMAND_CMD_PIN_MARGIN == 0.05
  assert REGEN_DEMAND_EVIDENCE_COUNT == 30


def test_phantom_prompt_is_gone_when_the_command_is_not_at_the_rail():
  """15d 15:27:31: plan -1.94, command -0.22."""
  fired, check = _run(10 * REGEN_DEMAND_EVIDENCE_COUNT, a_target=-1.94, a_cmd=-0.22)
  assert not fired and not check.active
  assert check.evidence_updates == 0


def test_real_overflow_still_prompts_when_the_command_is_pinned():
  """11:26:49 / 05:55:36: command at the -1.5 rail."""
  for a_target in (-1.94, -3.0):
    fired, check = _run(REGEN_DEMAND_EVIDENCE_COUNT, a_target=a_target, a_cmd=-1.5)
    assert fired and check.active


def test_command_just_inside_the_pin_margin_prompts_just_outside_does_not():
  fired, _ = _run(REGEN_DEMAND_EVIDENCE_COUNT, a_target=-2.0, a_cmd=FLOOR + REGEN_DEMAND_CMD_PIN_MARGIN)
  assert fired
  fired, _ = _run(10 * REGEN_DEMAND_EVIDENCE_COUNT, a_target=-2.0, a_cmd=FLOOR + REGEN_DEMAND_CMD_PIN_MARGIN + 0.01)
  assert not fired


def test_unknown_command_is_the_plan_only_rule():
  for a_cmd in ("unset", None, math.nan, math.inf):
    fired, _ = _run(REGEN_DEMAND_EVIDENCE_COUNT, a_target=-2.0, a_cmd=a_cmd)
    assert fired, a_cmd


def test_never_more_prompts_than_the_plan_only_rule():
  for a_target in (-1.4, -1.6, -1.69, -1.71, -2.5):
    for v_ego in (1.0, 2.0, 15.0):
      for a_cmd in (-1.5, -1.45, -1.0, -0.22, 0.0, 0.5):
        with_cmd, _ = _run(3 * REGEN_DEMAND_EVIDENCE_COUNT, a_target=a_target, a_cmd=a_cmd, v_ego=v_ego)
        plan_only, _ = _run(3 * REGEN_DEMAND_EVIDENCE_COUNT, a_target=a_target, v_ego=v_ego)
        assert plan_only or not with_cmd, (a_target, v_ego, a_cmd)


def test_a_command_off_the_rail_cannot_build_evidence_between_pinned_frames():
  """Evidence is a saturating up/down counter: unpinned frames drain it."""
  check = RegenDemandCheck()
  fired = False
  for _ in range(5 * REGEN_DEMAND_EVIDENCE_COUNT):
    fired = _run(1, a_target=-2.0, a_cmd=-1.5, check=check)[0] or fired
    fired = _run(2, a_target=-2.0, a_cmd=-0.22, check=check)[0] or fired
  assert not fired


def test_active_prompt_keeps_going_on_the_plan_alone():
  """Keep-prompting is unchanged: the plan decides, even if the command eases."""
  fired, check = _run(REGEN_DEMAND_EVIDENCE_COUNT, a_target=-2.0, a_cmd=-1.5)
  assert fired and check.active
  assert check.update(pedal_long_active=True, brake_pressed=False, a_target=-2.0,
                      v_ego=15.0, a_cmd=-0.22)
  assert not check.update(pedal_long_active=True, brake_pressed=False, a_target=-1.5,
                          v_ego=15.0, a_cmd=-1.5)


def test_selfdrived_passes_the_guarded_command():
  src = Path(preap_regen.__file__).with_name("selfdrived.py").read_text()
  assert "a_cmd=" in src
  assert "carControl'].actuators.accel" in src
  assert "a_cmd" in inspect.signature(preap_regen.RegenDemandCheck.update).parameters
