"""One-Pedal Long: gas-from-rest silent long pause + RELEASE regen.

Toggle On + OP long already on + accelerator rising from rest pauses
software long (same function as brake today — lat stays, not
USER_DISABLE / full session cancel) and **latches** that pause until
a stalk SET. Lift must not restore long / re-ACQUIRE. Engage-while-
gas-pressed (one or two SET pulls) then lift still arms / starts long
via A+B + A3. Brake pause does not use this latch.
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


def _pedal_conf(one_pedal_long=False):
  return SimpleNamespace(
    use_pedal=True,
    pedal_factor=1.0,
    di_to_pedal=lambda pedal_di: pedal_di,
    get_pedal_profile_values=lambda: PEDAL_MAX_VALUES,
    one_pedal_long=one_pedal_long,
  )


def _zero_torque():
  return SimpleNamespace(
    get=lambda _v_ego: 0.0,
    update=lambda *_args, **_kwargs: None,
  )


def controller_env(monkeypatch, one_pedal_long=False, double_pull=False):
  zero_torque = _zero_torque()
  conf = _pedal_conf(one_pedal_long=one_pedal_long)
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.nap_conf', conf)
  monkeypatch.setattr('opendbc.car.tesla.preap.carcontroller.get_zero_torque', lambda: zero_torque)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.nap_conf', conf)
  monkeypatch.setattr('opendbc.car.tesla.preap.virtual_das.get_zero_torque', lambda: zero_torque)

  feedback = PedalFeedback()
  feedback.update({"INTERCEPTOR_GAS": 0.0, "INTERCEPTOR_GAS2": 0.0, "STATE": 0, "IDX": 1}, 0)
  engagement = PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)
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
  assert cs.engagement._one_pedal_pause_latched

  cs.out.gasPressed = False
  cs.engagement.maybe_one_pedal_gas_kick(False, True)
  # On-car, gasPressedOverride ends on lift so CC.longActive goes True
  # while the session stays up. Latch must still block ACQUIRE.
  cc.longActive = True
  silent = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert silent == []
  assert not cs.enableLongControl
  assert cs.engagement._one_pedal_pause_latched
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
  assert "_one_pedal_pause_latched" in body
  assert "_one_pedal_had_long_at_rest" in body
  assert "latch_one_pedal_gas_takeover" in body
  assert "hard_cancel_session(" not in body
  assert "cruiseEnabled = False" not in body
  keys = (root / "common/params_keys.h").read_text()
  assert "NAPOnePedalLong" in keys
  assert 'BOOL, "0"' in next(ln for ln in keys.splitlines() if '"NAPOnePedalLong"' in ln)
  overlay = (root / "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "one_pedal_gas_for_pause(" in overlay
  assert "ONE_PEDAL_GAS_DI_PRESSED = 1.0" in overlay
  assert "PEDAL_DI_PRESSED_STOCK = 2.0" in overlay
  assert "interceptor_value" in overlay
  assert "maybe_one_pedal_gas_kick(True, True)" in overlay


def test_one_pedal_light_tip_in_latches_pause():
  """One-Pedal pause is slightly more sensitive than stock interceptor gasPressed.

  Stock gasPressed is DI > 2. Pause latches at DI > 1 so a light tip-in
  takes long. Coast / tiny noise at 0–1 stays off. SET-while-gas and lift
  still do not resume (existing tests).
  """
  from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import (
    ONE_PEDAL_GAS_DI_PRESSED,
    PEDAL_DI_PRESSED_STOCK,
    one_pedal_gas_for_pause,
  )

  assert abs(PEDAL_DI_PRESSED_STOCK - 2.0) < 1e-9
  assert abs(ONE_PEDAL_GAS_DI_PRESSED - 1.0) < 1e-9
  assert 0.0 < ONE_PEDAL_GAS_DI_PRESSED < PEDAL_DI_PRESSED_STOCK

  assert not one_pedal_gas_for_pause(None)
  assert not one_pedal_gas_for_pause(0.0)
  assert not one_pedal_gas_for_pause(0.5)
  assert not one_pedal_gas_for_pause(ONE_PEDAL_GAS_DI_PRESSED)
  assert one_pedal_gas_for_pause(1.1)
  assert one_pedal_gas_for_pause(PEDAL_DI_PRESSED_STOCK)
  assert one_pedal_gas_for_pause(2.1)

  from opendbc.car.tesla.preap.engagement import PreAPEngagement

  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)
  eng.cruiseEnabled = True
  eng.enableLongControl = True
  assert not eng.maybe_one_pedal_gas_kick(False, True)
  assert not eng._one_pedal_pause_latched
  # Light tip-in below stock gasPressed still pauses.
  assert one_pedal_gas_for_pause(1.2)
  assert 1.2 < PEDAL_DI_PRESSED_STOCK
  assert eng.maybe_one_pedal_gas_kick(True, True)
  assert eng._one_pedal_pause_latched
  assert not eng.enableLongControl
  # Lift stays paused.
  assert not eng.maybe_one_pedal_gas_kick(False, True)
  assert eng._one_pedal_pause_latched

  # Overlay extra kick still honors SET-while-gas skip_resume.
  eng2 = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=400)
  eng2.cruiseEnabled = True
  eng2.enableLongControl = True
  assert not eng2.maybe_one_pedal_gas_kick(False, True)
  eng2._nap_set_resume_long = True
  assert not eng2.maybe_one_pedal_gas_kick(True, True)
  assert not eng2._one_pedal_pause_latched
  assert eng2.enableLongControl


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


def test_one_pedal_set_after_gas_pause_acquires(monkeypatch):
  """After a from-rest pause, lift stays paused; one SET re-ACQUIREs."""
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _kick_long_on_gas(cc, cs)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = False
  cs.engagement.maybe_one_pedal_gas_kick(False, True)
  cc.longActive = True
  silent = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert silent == []
  assert cs.engagement._one_pedal_pause_latched
  assert not cs.enableLongControl

  cs.engagement.process_buttons(
    cruise_buttons=2, prev_cruise_buttons=0,
    curr_time_ms=4000, v_ego=20.0, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  assert not cs.engagement._one_pedal_pause_latched
  assert cs.engagement.enableLongControl
  cs.enableLongControl = True
  cs.cruiseEnabled = True
  cc.longActive = True
  acquire = controller.update(cc, cs, frame=6, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_one_pedal_gas_pause_keeps_session_for_resume(monkeypatch):
  """After gas pause, cruiseEnabled stays so one SET can resume like brake."""
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _kick_long_on_gas(cc, cs)
  assert cs.engagement.cruiseEnabled
  assert not cs.engagement.enableLongControl
  assert not getattr(cs.engagement, "preap_cc_cancel_needed", False)
  assert cs.engagement._one_pedal_pause_latched


def test_engage_while_gas_held_lift_acquires_with_one_pedal_on(monkeypatch):
  """SET with foot already on gas, then lift: A+B ACQUIRE, not a one-pedal pause."""
  from opendbc.car.tesla.preap.carcontroller import ENGAGE_GRACE_FRAMES

  controller, cc, cs, tesla_can = controller_env(monkeypatch, one_pedal_long=True)
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
  assert not getattr(cs.engagement, "_one_pedal_pause_latched", False)
  cs.enableLongControl = True
  cc.longActive = True
  cc.actuators.accel = 0.55
  acquire = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)
  assert (2 - controller.preap_long_engage_frame) >= ENGAGE_GRACE_FRAMES
  assert controller.vdas.jerk_limiter.a_limited == pytest.approx(0.72, abs=0.08)


def _set_resume(cs, t_ms=4000, v_ego=22.0):
  cs.engagement.process_buttons(
    cruise_buttons=2, prev_cruise_buttons=0,
    curr_time_ms=t_ms, v_ego=v_ego, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  cs.enableLongControl = cs.engagement.enableLongControl
  cs.cruiseEnabled = cs.engagement.cruiseEnabled
  cs.enableJustCC = cs.engagement.enableJustCC


def test_gas_then_lift_stays_paused_until_set(monkeypatch):
  """Road test: long on → gas → lift. Override end must not resume long."""
  controller, cc, cs, tesla_can = controller_env(monkeypatch, one_pedal_long=True)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  _kick_long_on_gas(cc, cs)
  release = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert not cs.enableLongControl
  assert cs.engagement._one_pedal_pause_latched

  cs.out.gasPressed = False
  for i, frame in enumerate(range(6, 20, 2)):
    cs.engagement.maybe_one_pedal_gas_kick(False, True)
    # Stock gasPressedOverride ends on lift: CC.longActive goes True
    # while the session stays up. Must not ACQUIRE or flip long back.
    cc.longActive = True
    cs.enableLongControl = True
    out = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert out == []
    assert not cs.enableLongControl
    assert not cs.engagement.enableLongControl
    assert cs.engagement._one_pedal_pause_latched
    assert controller.pedal_authority.state == PedalAuthorityState.INACTIVE

  _set_resume(cs)
  assert not cs.engagement._one_pedal_pause_latched
  assert cs.engagement.enableLongControl
  cs.enableLongControl = True
  cc.longActive = True
  acquire = controller.update(cc, cs, frame=20, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_controller_latches_when_engagement_kick_misses(monkeypatch):
  """Kick-miss hole: gas override + lift with enableLong still true.

  Interceptor RELEASEs on gas; lift must not A+B ACQUIRE until SET.
  """
  controller, cc, cs, tesla_can = controller_env(monkeypatch, one_pedal_long=True)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert controller._saw_long_without_gas
  assert cs.engagement.enableLongControl
  assert not cs.engagement._one_pedal_pause_latched

  # Soft / late gas: do not call maybe_one_pedal_gas_kick.
  cs.out.gasPressed = True
  cc.longActive = False
  release = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.engagement._one_pedal_pause_latched
  assert not cs.enableLongControl
  assert not cs.engagement.enableLongControl

  cs.out.gasPressed = False
  for frame in range(6, 16, 2):
    cc.longActive = True
    cs.enableLongControl = True
    silent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert silent == []
    assert not cs.enableLongControl
    assert not cs.engagement.enableLongControl
    assert cs.engagement._one_pedal_pause_latched
    assert controller.pedal_authority.state == PedalAuthorityState.INACTIVE

  _set_resume(cs)
  assert not cs.engagement._one_pedal_pause_latched
  cs.enableLongControl = True
  cc.longActive = True
  acquire = controller.update(cc, cs, frame=16, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def _on_car_cycle(controller, cc, cs, tesla_can, frame, *, gas, set_edge=False, t_ms=4000, v_ego=22.0):
  """Carstate then carcontroller: process_buttons, kick, then long update."""
  from opendbc.car.tesla.values import CruiseButtons

  cs.out.gasPressed = gas
  cs.engagement._nap_gas_pressed = gas
  buttons = CruiseButtons.MAIN if set_edge else 0
  cs.engagement.process_buttons(
    cruise_buttons=buttons, prev_cruise_buttons=0,
    curr_time_ms=t_ms, v_ego=v_ego, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  cs.engagement.maybe_one_pedal_gas_kick(gas, True)
  cs.enableLongControl = cs.engagement.enableLongControl
  cs.cruiseEnabled = cs.engagement.cruiseEnabled
  cs.enableJustCC = cs.engagement.enableJustCC
  cc.longActive = (not gas) and bool(cs.enableLongControl)
  return controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)


def test_one_pedal_set_while_gas_held_controller_does_not_relatch(monkeypatch):
  """On-car order: long holding → gas pause → SET with foot still down.

  Overlay one-SET resume must stick; controller `_saw_long_without_gas`
  must not re-latch. Lift then ACQUIREs (A+B). Lift alone stays paused.
  """
  from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause

  install_blinker_lat_pause()
  controller, cc, cs, tesla_can = controller_env(
    monkeypatch, one_pedal_long=True, double_pull=True)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert controller._saw_long_without_gas

  _kick_long_on_gas(cc, cs)
  release = controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.engagement._one_pedal_pause_latched
  assert not cs.enableLongControl

  out = _on_car_cycle(controller, cc, cs, tesla_can, 6, gas=True, set_edge=True, t_ms=4000)
  assert cs.engagement.enableLongControl
  assert not cs.engagement._one_pedal_pause_latched
  assert getattr(cs.engagement, "_nap_set_resume_long", False)
  assert not controller._saw_long_without_gas
  # Gas still down: interceptor stays RELEASED (pass-through).
  if out:
    assert not _decode_pedal_command(out[0]).enabled

  # Later frames with foot still down must not re-latch.
  later = _on_car_cycle(controller, cc, cs, tesla_can, 12, gas=True, t_ms=4100)
  assert cs.engagement.enableLongControl
  assert not cs.engagement._one_pedal_pause_latched
  if later:
    assert not _decode_pedal_command(later[0]).enabled

  cs.out.gasPressed = False
  cs.engagement._nap_gas_pressed = False
  assert not cs.engagement.maybe_one_pedal_gas_kick(False, True)
  assert cs.engagement.enableLongControl
  cs.enableLongControl = True
  cc.longActive = True
  acquire = controller.update(cc, cs, frame=14, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_one_pedal_set_after_lift_with_overlay_double_pull_acquires(monkeypatch):
  """Rolling one SET after lift resumes at held MAX with double-pull On."""
  from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause

  install_blinker_lat_pause()
  controller, cc, cs, tesla_can = controller_env(
    monkeypatch, one_pedal_long=True, double_pull=True)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _kick_long_on_gas(cc, cs)
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  silent = _on_car_cycle(controller, cc, cs, tesla_can, 4, gas=False, t_ms=3000)
  assert silent == []
  assert cs.engagement._one_pedal_pause_latched
  assert not cs.enableLongControl

  acquire = _on_car_cycle(controller, cc, cs, tesla_can, 6, gas=False, set_edge=True, t_ms=4000)
  assert not cs.engagement._one_pedal_pause_latched
  assert cs.engagement.enableLongControl
  assert _decode_pedal_command(acquire[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)
