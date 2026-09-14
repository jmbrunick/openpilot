"""R1: ramp interceptor regen on a short/light Pre-AP brake cancel.

Short cancel keeps interceptor ENABLE and interpolates commanded
regen 0 → stock. Held / deeper brake RELEASEs immediately. FCW /
hard lead / full cancel unchanged. No GAS_COMMAND DI rewrite /
PostEngageCoast. Gas-lift A3 is untouched.
"""
from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.brake_cancel_regen import (
  BRAKE_CANCEL_RAMP_S,
  BRAKE_CANCEL_STOCK_REGEN_A,
  BRAKE_TIP_HOLD_S,
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


@pytest.fixture
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


def test_short_cancel_ramps_regen_before_stock(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  _drop_long_on_brake(cc, cs, a_ego=-0.25)
  tip = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert _decode_pedal_command(tip[0]).enabled
  assert cs.pedal_brake_cancel_ramp

  cs.real_brake_pressed = False
  cs.out.aEgo = 0.05
  cc.actuators.accel = -1.4
  enabled_frames = 0
  released = False
  commanded = []
  pedal_di = []
  for frame in range(4, 200, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if not sent:
      released = True
      break
    decoded = _decode_pedal_command(sent[0])
    if not decoded.enabled:
      released = True
      assert decoded.raw_command == 0
      break
    enabled_frames += 1
    commanded.append(controller.brake_cancel.commanded_accel())
    pedal_di.append(controller.prev_pedal_di)

  assert released
  assert enabled_frames * 0.01 >= BRAKE_CANCEL_RAMP_S - 0.08
  assert commanded[0] == pytest.approx(0.0, abs=0.05)
  assert commanded[-1] < commanded[len(commanded) // 2] < commanded[0]
  assert commanded[-1] <= BRAKE_CANCEL_STOCK_REGEN_A * 0.80
  # Progressive regen, not a coast plateau then a step.
  assert sum(1 for a in commanded if a < -0.05) >= 3 * len(commanded) // 4
  assert pedal_di[-1] < pedal_di[0]


def test_held_brake_is_firm_stock_regen(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=0.0)

  released_at = None
  for frame in range(2, 100, 2):
    sent = controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    if sent and not _decode_pedal_command(sent[0]).enabled:
      released_at = frame
      break
    assert sent and _decode_pedal_command(sent[0]).enabled
  assert released_at is not None
  # Classifier dt=0.01 per update(); tests skip odd frames.
  assert released_at >= int(BRAKE_TIP_HOLD_S / 0.01) - 2


def test_firm_brake_releases_immediately(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  _drop_long_on_brake(cc, cs, a_ego=-2.0)
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
  assert cs.pedal_authority_action == int(PedalCommandAction.RELEASE)


def test_hard_lead_fcw_path_unchanged(controller_env):
  controller, cc, cs, tesla_can = controller_env
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


def test_full_cancel_does_not_soft_regen(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  cs.cruiseEnabled = False
  cs.enableLongControl = False
  cc.longActive = False
  cs.real_brake_pressed = True
  release = controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)
  assert not _decode_pedal_command(release[0]).enabled
