"""Tip-brake soft-glide: speed target after a short cancel, stock on hold.

A light tap that knocks software long off keeps interceptor ENABLE and
commands a decelerating profile toward ~12 mph over ~2.5 s — not stock
bite on frame 1, and not the reverted 0.75 s 0→REGEN_MAX fade. Held /
firm aEgo RELEASEs immediately. FCW / hard lead / full cancel unchanged.
Gas-lift A+B / A3 is untouched.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.preap.brake_tip_glide import (
  BRAKE_GLIDE_A_MIN,
  BRAKE_GLIDE_DURATION_S,
  BRAKE_TIP_HOLD_S,
  glide_target_accel,
)
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


V_GLIDE = 20.0 * CV.MPH_TO_MS


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
    out=SimpleNamespace(vEgo=V_GLIDE, aEgo=0.0, gasPressed=False),
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


def test_short_tip_keeps_interceptor_and_glides(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  _drop_long_on_brake(cc, cs, a_ego=-0.25)
  pending = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(pending[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.ENABLE)
  assert cs.pedal_brake_tip_glide
  assert hasattr(controller, "brake_cancel")
  assert controller.brake_cancel.commanded_accel() == pytest.approx(0.0)

  cs.real_brake_pressed = False
  cs.out.aEgo = 0.05
  cc.actuators.accel = -1.2
  accels = []
  for frame in range(4, 40, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    assert _decode_pedal_command(sent[0]).enabled
    assert cs.pedal_brake_tip_glide
    accels.append(controller.brake_cancel.commanded_accel())

  expected_a = glide_target_accel(V_GLIDE)
  assert accels[0] == pytest.approx(expected_a)
  assert accels[-1] == pytest.approx(expected_a)
  assert accels[-1] < -0.5
  assert accels[-1] > BRAKE_GLIDE_A_MIN + 0.02
  assert BRAKE_GLIDE_DURATION_S == pytest.approx(2.5)
  assert controller.brake_cancel.glide_s > 0.75


def test_held_brake_releases_after_tip_window(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=0.0)

  released = False
  for frame in range(2, 80, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if sent and not _decode_pedal_command(sent[0]).enabled:
      released = True
      assert _decode_pedal_command(sent[0]).raw_command == 0
      assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
      assert frame * 0.01 >= BRAKE_TIP_HOLD_S - 0.03
      break
    assert sent and _decode_pedal_command(sent[0]).enabled
  assert released
  assert controller.update(cc, cs, frame=frame + 2, tesla_can=tesla_can, can_bus_party=0) == []


def test_firm_brake_releases_immediately(monkeypatch):
  controller, cc, cs, tesla_can = controller_env(monkeypatch)
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=-2.0)
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert _decode_pedal_command(release[0]).raw_command == 0
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)
  assert not getattr(cs, "pedal_brake_tip_glide", False)


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
    assert not getattr(cs, "pedal_brake_tip_glide", False)
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
