"""Pre-AP 1 s post-engage coast: no regen dip on throttle lift."""
from pathlib import Path

import pytest

from openpilot.selfdrive.controls.lib.post_engage_coast import (
  HARD_DECEL_MS2,
  LEAD_KEEP_DECEL_MS2,
  POST_ENGAGE_COAST_S,
  PostEngageCoast,
)

# Mirror production steps so helper tests do not import setproctitle / hardware.
DT_CTRL = 0.01
DT_MDL = 0.05


def _coast(dt=DT_MDL):
  return PostEngageCoast(dt=dt)


def _engage_with_gas(coast, dt=DT_MDL, frames=1):
  for _ in range(frames):
    coast.update(long_engaged=True, gas_pressed=True, dt=dt)
  return coast


def test_window_is_one_full_second():
  assert POST_ENGAGE_COAST_S == pytest.approx(1.0)
  assert LEAD_KEEP_DECEL_MS2 == pytest.approx(0.55)
  assert HARD_DECEL_MS2 > LEAD_KEEP_DECEL_MS2


def test_engage_then_gas_release_within_1s_zeros_soft_negative():
  coast = _coast()
  _engage_with_gas(coast)
  coast.update(long_engaged=True, gas_pressed=False)

  assert coast.active
  assert coast.apply(-0.35) == 0.0
  assert coast.apply(-0.80) == 0.0
  assert coast.apply(0.40) == pytest.approx(0.40)


def test_brake_still_decelerates_inside_window():
  coast = _coast()
  _engage_with_gas(coast)
  coast.update(long_engaged=True, gas_pressed=False)

  assert coast.apply(-0.55, brake_pressed=True) == pytest.approx(-0.55)
  assert coast.apply(-1.20, brake_pressed=True) == pytest.approx(-1.20)


def test_after_1s_normal_regen_allowed():
  coast = _coast()
  _engage_with_gas(coast)
  steps = int(POST_ENGAGE_COAST_S / DT_MDL) + 2
  for _ in range(steps):
    coast.update(long_engaged=True, gas_pressed=False)

  assert not coast.active
  assert coast.apply(-0.40) == pytest.approx(-0.40)


def test_engage_without_prior_gas_does_not_clamp():
  coast = _coast()
  coast.update(long_engaged=True, gas_pressed=False)

  assert not coast.active
  assert coast.apply(-0.40) == pytest.approx(-0.40)
  assert coast.apply(0.30) == pytest.approx(0.30)


def test_gas_tap_during_window_then_lift_clamps():
  """Engage with no gas, tap throttle inside 1 s, lift → same dip path."""
  coast = _coast()
  coast.update(long_engaged=True, gas_pressed=False)
  assert not coast.active

  coast.update(long_engaged=True, gas_pressed=True)
  coast.update(long_engaged=True, gas_pressed=False)
  assert coast.active
  assert coast.apply(-0.25) == 0.0


def test_safety_paths_pass_through():
  coast = _coast()
  _engage_with_gas(coast)
  coast.update(long_engaged=True, gas_pressed=False)

  assert coast.apply(-0.55, fcw=True) == pytest.approx(-0.55)
  assert coast.apply(-0.40, should_stop=True) == pytest.approx(-0.40)
  # Far-lead nibble / inversion must still be zeroed.
  assert coast.apply(-0.30, has_lead=True) == 0.0
  assert coast.apply(-LEAD_KEEP_DECEL_MS2, has_lead=True) == pytest.approx(-LEAD_KEEP_DECEL_MS2)
  assert coast.apply(-HARD_DECEL_MS2, has_lead=True) == pytest.approx(-HARD_DECEL_MS2)
  # Lone hard-looking inversion (no lead / FCW / brake) is the dip — hold.
  assert coast.apply(-1.50) == 0.0


def test_long_drop_clears_window():
  coast = _coast()
  _engage_with_gas(coast)
  coast.update(long_engaged=False, gas_pressed=False)
  assert not coast.active
  assert coast.apply(-0.50) == pytest.approx(-0.50)

  coast.update(long_engaged=True, gas_pressed=False)
  assert not coast.active


def test_controlsd_dt_covers_full_second():
  coast = PostEngageCoast(dt=DT_CTRL)
  coast.update(long_engaged=True, gas_pressed=True)
  # 0.99 s still inside; 1.00 s + one step expires.
  for _ in range(int(0.99 / DT_CTRL)):
    coast.update(long_engaged=True, gas_pressed=False)
  assert coast.active
  assert coast.apply(-0.20) == 0.0

  coast.update(long_engaged=True, gas_pressed=False)
  coast.update(long_engaged=True, gas_pressed=False)
  assert not coast.active


def _planner_imports():
  pytest.importorskip("cereal")
  pytest.importorskip("numpy")
  from cereal import car, log, messaging
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  from openpilot.selfdrive.modeld.constants import ModelConstants
  return car, log, messaging, LongCtrlState, LongitudinalPlanner, ModelConstants


def make_preap_params():
  car, _log, _messaging, _L, _P, _M = _planner_imports()
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  return params


def make_planner_inputs(*, v_ego, v_cruise, a_ego=0.0, gas=False, long_on=False,
                        brake=0.0, lead=None, pitch=0.0):
  import numpy as np
  car, log, messaging, LongCtrlState, _P, ModelConstants = _planner_imports()
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.off if gas else LongCtrlState.pid
  car_state.vEgo = v_ego
  car_state.aEgo = a_ego
  car_state.vCruise = v_cruise * 3.6
  car_state.gasPressed = gas
  car_state.brake = brake
  if hasattr(car_state, "enableLongControl"):
    car_state.enableLongControl = long_on
  car_control.orientationNED = [0.0, pitch, 0.0]

  if lead is not None:
    radar.leadOne.status = True
    radar.leadOne.dRel = lead["dRel"]
    radar.leadOne.vLead = lead["vLead"]
    radar.leadOne.modelProb = lead.get("modelProb", 0.9)
    radar.leadOne.radar = True

  position = log.XYZTData.new_message()
  position.x = ((v_ego + 0.5) * np.array(ModelConstants.T_IDXS)).tolist()
  model.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = ((v_ego + 0.5) * np.ones_like(ModelConstants.T_IDXS)).tolist()
  velocity.x[0] = v_ego
  model.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.acceleration = acceleration
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6

  return {
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
  }


def _new_planner(init_v, init_a):
  _car, _log, _messaging, _L, LongitudinalPlanner, _M = _planner_imports()
  return LongitudinalPlanner(make_preap_params(), init_v=init_v, init_a=init_a)


def _run(planner, **kwargs):
  planner.update(make_planner_inputs(**kwargs))
  return float(planner.output_a_target)


def test_planner_gas_release_after_engage_does_not_command_regen():
  """Engage while accelerating, lift, MAX is higher → no negative aTarget."""
  v_ego = 20.0
  v_cruise = 25.0
  planner = _new_planner(v_ego, 1.2)

  for _ in range(8):
    _run(planner, v_ego=v_ego, v_cruise=v_cruise, a_ego=1.4, gas=True, long_on=True)
    v_ego += 1.4 * DT_MDL

  # Lift: Tesla regen already pulling aEgo negative. MAX still higher.
  for _ in range(6):
    a = _run(planner, v_ego=v_ego, v_cruise=v_cruise, a_ego=-1.1, gas=False, long_on=True)
    assert a >= -1e-6, a
    v_ego += max(a, -1.1) * DT_MDL


def test_planner_brake_inside_window_does_not_clamp():
  """Applied brake is overlay pass-through, not a planner decel source.

  A one-frame MAX drop does not make MPC command −a (CI saw +0.40). The
  contract is: CS.brake >= 0.5 reaches apply(brake_pressed=True) and that
  call must not zero the raw command.
  """
  planner = _new_planner(22.0, 0.8)
  for _ in range(4):
    _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=0.8, gas=True, long_on=True)

  assert planner._post_engage_coast is not None
  assert planner._post_engage_coast.active
  assert planner._post_engage_coast.apply(-0.35) == 0.0
  assert planner._post_engage_coast.apply(-0.35, brake_pressed=True) == pytest.approx(-0.35)

  seen = []
  orig = planner._post_engage_coast.apply

  def wrapped(a_cmd, **kwargs):
    out = orig(a_cmd, **kwargs)
    seen.append((float(a_cmd), dict(kwargs), float(out)))
    return out

  planner._post_engage_coast.apply = wrapped
  _run(planner, v_ego=22.0, v_cruise=18.0, a_ego=-0.6, gas=False, long_on=True, brake=1.0)

  assert seen, "planner must call PostEngageCoast.apply"
  raw, kwargs, out = seen[-1]
  assert kwargs.get("brake_pressed") is True
  assert out == pytest.approx(raw)


def test_planner_after_window_allows_regen_to_lower_max():
  planner = _new_planner(25.0, 0.5)
  _run(planner, v_ego=25.0, v_cruise=20.0, a_ego=0.5, gas=True, long_on=True)
  frames = int(POST_ENGAGE_COAST_S / DT_MDL) + 4
  a = 0.0
  for _ in range(frames):
    a = _run(planner, v_ego=25.0, v_cruise=20.0, a_ego=-0.2, gas=False, long_on=True)
  assert a < -0.05, a


def test_planner_engage_without_gas_still_regens_to_lower_max():
  planner = _new_planner(25.0, 0.0)
  a = 0.0
  for _ in range(12):
    a = _run(planner, v_ego=25.0, v_cruise=20.0, a_ego=0.0, gas=False, long_on=True)
  assert a < -0.05, a


def test_planner_lead_hard_brake_not_clamped():
  planner = _new_planner(22.0, 0.8)
  for _ in range(4):
    _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=0.8, gas=True, long_on=True)

  lead = {"dRel": 12.0, "vLead": 8.0, "modelProb": 0.9}
  a = _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=-0.4, gas=False, long_on=True, lead=lead)
  assert a < -0.05, a


def test_planner_and_controlsd_wire_the_overlay():
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  controlsd = (root / "selfdrive/controls/controlsd.py").read_text()
  docs = (root / "docs-nap/engagement.md").read_text()

  lead_at = planner.find("apply_lead_approach_overlay(output_a_target, a_lead)")
  coast_at = planner.find("self._post_engage_coast.apply(")
  clip_at = planner.find("self.output_a_target = np.clip(output_a_target")
  assert 0 <= lead_at < coast_at < clip_at
  assert "self.a_desired = 0.0" in planner
  assert "enableLongControl" in planner
  assert "PostEngageCoast" in controlsd
  assert "a_target = self._post_engage_coast.apply(" in controlsd
  assert "1.0 s" in docs or "1 s" in docs
  assert "post-engage" in docs.lower() or "pedal lift" in docs.lower()
