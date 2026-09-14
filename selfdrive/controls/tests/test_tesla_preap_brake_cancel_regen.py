"""Brake cancel is firm/stock: interceptor RELEASEs as soon as long drops.

No tip/hold regen ramp, no coast-then-bite. A light tap that knocks
software long off must RELEASE immediately to Tesla regen. FCW / hard
lead / full cancel unchanged. Gas-lift A+B / A3 is untouched.
"""
from types import SimpleNamespace

from opendbc.car.tesla.preap.carcontroller import (
  ENGAGE_GRACE_FRAMES,
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
    out=SimpleNamespace(vEgo=15.0, aEgo=0.0, gasPressed=False),
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


def _drop_long_on_brake(cc, cs, *, a_ego=0.0):
  cs.enableLongControl = False
  cs.engagement.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  cs.out.aEgo = a_ego


def test_short_tip_releases_interceptor_immediately(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  _drop_long_on_brake(cc, cs, a_ego=-0.25)
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert _decode_pedal_command(release[0]).raw_command == 0
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
  assert not getattr(cs, "pedal_brake_cancel_ramp", False)
  assert not hasattr(controller, "brake_cancel")

  cs.real_brake_pressed = False
  cs.out.aEgo = 0.05
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0) == []


def test_held_brake_also_releases_immediately(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=0.0)

  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
  assert controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0) == []


def test_hard_lead_fcw_path_unchanged(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  cc.actuators.accel = 0.0
  for frame in range(0, ENGAGE_GRACE_FRAMES + 2, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
  cc.actuators.accel = -1.3
  cs.out.aEgo = -0.5
  limited = []
  for frame in range(ENGAGE_GRACE_FRAMES + 2, ENGAGE_GRACE_FRAMES + 24, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert _decode_pedal_command(sent[0]).enabled
    assert not getattr(cs, "pedal_brake_cancel_ramp", False)
    limited.append(controller.vdas.jerk_limiter.a_limited)
  assert min(limited) < 0.0


def test_full_cancel_releases(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  cs.cruiseEnabled = False
  cs.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
