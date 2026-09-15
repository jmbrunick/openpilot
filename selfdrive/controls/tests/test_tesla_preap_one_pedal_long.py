"""One-Pedal Long: gas-from-rest silent long pause + RELEASE regen.

Toggle On + OP long already on + accelerator rising from rest pauses
software long (same function as brake today — lat stays, not
USER_DISABLE / full session cancel). Engage-while-gas-pressed (one or
two SET pulls) then lift still arms / starts long via A+B + A3.
Interceptor RELEASEs on a from-rest pause and stays RELEASED on lift.
Resume-long after pause is still one SET (sticky MAX). Toggle Off is
stock gas override + A+B/A3 resume.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.carcontroller import (
  PedalAuthorityState,
  PedalCommandAction,
  PreAPLongController,
)
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.preap.tests.test_pedal_authority import (
  _activate_longitudinal,
  _decode_pedal_command,
)


def _pedal_conf():
  return SimpleNamespace(
    use_pedal=True,
    pedal_factor=1.0,
    di_to_pedal=lambda pedal_di: pedal_di,
    get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
    one_pedal_long=False,
  )


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: 0.0,
    update=lambda *_args, **_kwargs: None,
  )


def controller_env(monkeypatch):
  zero_torque = _zero_torque()
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', _pedal_conf())
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  engagement = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    enableJustCC=False,
    engagement=engagement,
    real_brake_pressed=False,
    out=SimpleNamespace(vEgo=20.0, aEgo=0.0, gasPressed=False),
    pedal_interceptor_value=0.0,
    cruise_buttons=0,
    prev_cruise_buttons=0,
    pedal=feedback,
    pedal_timeout=feedback.timeout,
    pccEvent=None,
    preap_cc_cancel_needed=False,
  )
  cc = SimpleNamespace(
    actuators=SimpleNamespace(accel=0.4),
    longActive=False,
    orientationNED=[],
  )
  return PreAPLongController(), cc, cs, TeslaCANPreAP({})


def _kick_long_on_gas(cc, cs):
  cs.engagement.maybe_one_pedal_gas_kick(False, True)
  assert cs.engagement.maybe_one_pedal_gas_kick(True, True)
  cs.enableLongControl = cs.engagement.enableLongControl
  cs.enableJustCC = cs.engagement.enableJustCC
  cc.longActive = False
  cs.out.gasPressed = True


def test_toggle_off_gas_override_releases_then_reacquires(monkeypatch):
  """Stock: gas does not drop enableLongControl; lift ACQUIREs (A+B)."""
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert cs.engagement.enableLongControl

  cs.out.gasPressed = True
  cc.longActive = False
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.engagement.enableLongControl
  assert not cs.engagement.maybe_one_pedal_gas_kick(True, False)

  cs.out.gasPressed = False
  cc.longActive = True
  acquire = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_one_pedal_gas_kick_releases_and_stays_released_on_lift(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  _kick_long_on_gas(cc, cs)
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert _decode_pedal_command(release[0]).raw_command == 0
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
  assert not cs.enableLongControl
  assert cs.engagement.cruiseEnabled

  cs.out.gasPressed = False
  cs.engagement.maybe_one_pedal_gas_kick(False, True)
  silent = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert silent == []
  assert not cs.enableLongControl
  # No re-ACQUIRE / ENABLE chatter: interceptor stays out so Tesla
  # physical pedal at zero is stock lift-regen.
  assert controller.pedal_authority.state == PedalAuthorityState.INACTIVE


def test_one_pedal_press_after_kick_stays_pass_through(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _kick_long_on_gas(cc, cs)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = False
  cs.engagement.maybe_one_pedal_gas_kick(False, True)
  controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = True
  cs.engagement.maybe_one_pedal_gas_kick(True, True)
  assert controller.update(cc, cs, frame=6, tesla_can=tesla_can, can_bus_party=0) == []
  assert not cs.enableLongControl


def test_brake_cancel_still_works_with_one_pedal_on(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  cs.enableLongControl = False
  cs.engagement.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  cs.out.aEgo = -2.0
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
  assert not getattr(cs, "pedal_brake_tip_glide", False)


def test_one_pedal_wires_carstate_after_interceptor_gas():
  """Pause must use interceptor gasPressed, published after GAS_SENSOR parse."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  cs = (root / "opendbc_repo/opendbc/car/tesla/preap/carstate.py").read_text()
  gas_at = cs.find("ret.gasPressed = cs.pedal.gas_pressed")
  kick_at = cs.find("maybe_one_pedal_gas_kick")
  assert 0 <= gas_at < kick_at
  assert "nap_conf.one_pedal_long" in cs
  eng = (root / "opendbc_repo/opendbc/car/tesla/preap/engagement.py").read_text()
  fn = eng.split("def maybe_one_pedal_gas_kick", 1)[1]
  body = fn.split('"""', 2)[2].split("\n  def ", 1)[0]
  assert "_drop_longitudinal_keep_lateral" in body
  assert "_preap_one_pedal_long_was_on" in body
  assert "hard_cancel_session(" not in body
  assert "cruiseEnabled = False" not in body
  keys = (root / "common/params_keys.h").read_text()
  assert "NAPOnePedalLong" in keys
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPOnePedalLong"' in ln)


def test_one_pedal_gas_pause_never_user_disables():
  """DisengageOnAccelerator must not full-cancel when One-Pedal Long is On."""
  from openpilot.selfdrive.selfdrived.preap_regen import gas_should_user_disable

  assert gas_should_user_disable(disengage_on_accelerator=True, one_pedal_long=False)
  assert not gas_should_user_disable(disengage_on_accelerator=True, one_pedal_long=True)
  assert not gas_should_user_disable(disengage_on_accelerator=False, one_pedal_long=True)
  assert not gas_should_user_disable(disengage_on_accelerator=False, one_pedal_long=False)

  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  sd = (root / "selfdrive/selfdrived/selfdrived.py").read_text()
  assert "gas_should_user_disable" in sd
  gas_block = sd.split("gas_disable = (", 1)[1].split("if gas_disable:", 1)[0]
  assert "one_pedal_long" in gas_block
  assert "disengage_on_accelerator" in gas_block
  events = (root / "selfdrive/selfdrived/events.py").read_text()
  pedal = events.split("EventName.pedalPressed:", 1)[1].split("EventName.", 1)[0]
  assert "USER_DISABLE" in pedal
  override = events.split("EventName.gasPressedOverride:", 1)[1].split("EventName.", 1)[0]
  assert "OVERRIDE_LONGITUDINAL" in override
  assert "USER_DISABLE" not in override


def test_one_pedal_gas_pause_keeps_session_for_resume(monkeypatch):
  """After gas pause, cruiseEnabled stays so one SET can resume like brake."""
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _kick_long_on_gas(cc, cs)
  assert cs.engagement.cruiseEnabled
  assert not cs.engagement.enableLongControl
  assert not getattr(cs.engagement, "preap_cc_cancel_needed", False)


def test_engage_while_gas_held_lift_acquires_with_one_pedal_on(monkeypatch):
  """SET with foot already on gas, then lift: A+B ACQUIRE, not a one-pedal pause."""
  from opendbc.car.tesla.preap.carcontroller import ENGAGE_GRACE_FRAMES

  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  monkeypatch.setattr(
    'opendbc.car.tesla.preap.carcontroller.get_preap_accel_limits',
    lambda _v_ego: (-1.5, 0.8),
  )
  assert not cs.engagement.maybe_one_pedal_gas_kick(True, True)
  cs.engagement.process_buttons(
    cruise_buttons=2, prev_cruise_buttons=0,
    curr_time_ms=1000, v_ego=15.0, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  assert cs.engagement.cruiseEnabled
  assert cs.engagement.enableLongControl
  assert not cs.engagement.maybe_one_pedal_gas_kick(True, True)
  assert cs.engagement.enableLongControl
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cs.out.gasPressed = True
  cs.out.aEgo = 0.72
  cc.longActive = False
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  assert cs.engagement.enableLongControl

  cs.out.gasPressed = False
  cs.out.aEgo = 0.08
  assert not cs.engagement.maybe_one_pedal_gas_kick(False, True)
  assert cs.engagement.enableLongControl
  cs.enableLongControl = True
  cc.longActive = True
  cc.actuators.accel = 0.55
  acquire = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)
  assert (2 - controller.preap_long_engage_frame) >= ENGAGE_GRACE_FRAMES
  assert controller.vdas.jerk_limiter.a_limited == pytest.approx(0.72, abs=0.08)
