"""Gas-lift long handoff: expire engage grace and seed last non-negative aEgo.

A+B only. No PostEngageCoast / pedal-hold / GAS_COMMAND rewrite.
"""

from types import SimpleNamespace

import pytest

from opendbc.car.tesla.preap.carcontroller import (
  ENGAGE_GRACE_FRAMES,
  PedalCommandAction,
  PreAPLongController,
  gas_lift_handoff_seed_accel,
)
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.nap_conf import PEDAL_MAX_VALUES
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP


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
    actuators=SimpleNamespace(accel=0.0),
    longActive=False,
    orientationNED=[],
  )
  return PreAPLongController(), cc, cs, TeslaCANPreAP({})


def _activate_longitudinal(cc, cs):
  cs.engagement.cruiseEnabled = True
  cs.engagement.enableLongControl = True
  cs.cruiseEnabled = True
  cs.enableLongControl = True
  cc.longActive = True


def test_gas_lift_handoff_seed_uses_non_negative_aego():
  assert gas_lift_handoff_seed_accel(0.62, -0.4, 0.8, 0.5) == pytest.approx(0.62)
  assert gas_lift_handoff_seed_accel(-0.3, 0.41, 0.8, 0.5) == pytest.approx(0.41)
  assert gas_lift_handoff_seed_accel(-0.3, -0.2, 0.8, 0.5) == pytest.approx(0.0)


def test_gas_lift_handoff_seed_max_brake_and_lead_win():
  assert gas_lift_handoff_seed_accel(1.618, 0.1, 0.8, 0.67) == pytest.approx(0.8)
  assert gas_lift_handoff_seed_accel(0.62, 0.1, 0.8, -1.2) == pytest.approx(0.0)
  assert gas_lift_handoff_seed_accel(0.62, 0.1, 0.8, float('nan')) == pytest.approx(0.0)


def test_engage_without_prior_gas_keeps_grace_floor(controller_env):
  controller, cc, cs, tesla_can = controller_env
  _activate_longitudinal(cc, cs)
  cs.out.aEgo = 0.55
  cc.actuators.accel = 0.67

  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  assert controller.preap_long_engage_frame == 0
  assert controller.vdas.jerk_limiter.a_limited == pytest.approx(0.0)
  assert cs.pedal_authority_action == int(PedalCommandAction.ACQUIRE)


def test_gas_lift_after_long_engage_does_not_floor_a_for_half_second(controller_env, monkeypatch):
  controller, cc, cs, tesla_can = controller_env
  monkeypatch.setattr(
    'opendbc.car.tesla.preap.carcontroller.get_preap_accel_limits',
    lambda _v_ego: (-1.5, 0.8),
  )
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = True
  cs.out.aEgo = 0.72
  cc.longActive = False
  for frame in range(2, 20, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = False
  cs.out.aEgo = 0.08
  cc.longActive = True
  cc.actuators.accel = 0.55
  limited = {}
  for frame in range(20, 72, 2):
    controller.update(cc, cs, frame=frame, tesla_can=tesla_can, can_bus_party=0)
    limited[frame] = controller.vdas.jerk_limiter.a_limited

  assert (20 - controller.preap_long_engage_frame) >= ENGAGE_GRACE_FRAMES
  assert limited[20] == pytest.approx(0.72, abs=0.08)
  assert min(limited.values()) > 0.35


def test_gas_lift_lead_hard_decel_still_wins(controller_env, monkeypatch):
  controller, cc, cs, tesla_can = controller_env
  monkeypatch.setattr(
    'opendbc.car.tesla.preap.carcontroller.get_preap_accel_limits',
    lambda _v_ego: (-1.5, 0.8),
  )
  _activate_longitudinal(cc, cs)
  controller.update(cc, cs, frame=0, tesla_can=tesla_can, can_bus_party=0)
  cs.out.gasPressed = True
  cs.out.aEgo = 0.72
  cc.longActive = False
  controller.update(cc, cs, frame=2, tesla_can=tesla_can, can_bus_party=0)

  cs.out.gasPressed = False
  cc.longActive = True
  cc.actuators.accel = -1.2
  controller.update(cc, cs, frame=4, tesla_can=tesla_can, can_bus_party=0)

  assert (4 - controller.preap_long_engage_frame) >= ENGAGE_GRACE_FRAMES
  assert controller.vdas.jerk_limiter.a_limited < 0.0
