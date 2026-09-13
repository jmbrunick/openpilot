"""Pre-AP post-engage: first pedal drop climbs toward MAX, no a=0 coast."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.post_engage_coast import (
  BINARY_PRESSED_DI,
  CLIMB_FLOOR_MS2,
  HARD_DECEL_MS2,
  LEAD_KEEP_DECEL_MS2,
  MAX_HOLD_SLACK_MS,
  PEDAL_DROP_DI,
  PEDAL_PRESSED_DI,
  POST_ENGAGE_COAST_S,
  PostEngageCoast,
  cs_pedal_di,
)

# Mirror production steps so helper tests do not import setproctitle / hardware.
DT_CTRL = 0.01
DT_MDL = 0.05


def _coast(dt=DT_MDL):
  return PostEngageCoast(dt=dt)


def _engage_with_gas(coast, dt=DT_MDL, frames=1, pedal=12.0, a_ego=1.20):
  for _ in range(frames):
    coast.update(long_engaged=True, gas_pressed=True, pedal_pos=pedal, a_ego=a_ego, dt=dt)
  return coast


def _lift(coast, pedal=10.5, a_ego=0.40, gas_pressed=True, dt=DT_MDL):
  coast.update(long_engaged=True, gas_pressed=gas_pressed, pedal_pos=pedal, a_ego=a_ego, dt=dt)
  return coast


def test_window_and_deadband_constants():
  assert POST_ENGAGE_COAST_S == pytest.approx(1.0)
  assert LEAD_KEEP_DECEL_MS2 == pytest.approx(0.55)
  assert HARD_DECEL_MS2 > LEAD_KEEP_DECEL_MS2
  assert PEDAL_PRESSED_DI == pytest.approx(2.0)
  # First detectable drop — not the 0.40 deadband that left early lift unprotected.
  assert 0.0 < PEDAL_DROP_DI <= 0.10
  assert CLIMB_FLOOR_MS2 > 0.0


def test_first_pedal_decrease_climbs_no_zero_coast():
  """Engage + first drop → positive climb. No a=0 sit, no soft regen."""
  coast = _coast()
  _engage_with_gas(coast, pedal=12.0, a_ego=1.20)
  _lift(coast, pedal=12.0 - PEDAL_DROP_DI, a_ego=-0.80)

  assert coast.active
  assert coast.hold_pedal == pytest.approx(12.0)
  assert coast.hold_accel == pytest.approx(1.20)
  assert coast.apply(-0.35) == pytest.approx(1.20)
  assert coast.apply(-0.80) == pytest.approx(1.20)
  assert coast.apply(0.0) == pytest.approx(1.20)
  assert coast.apply(0.40) == pytest.approx(1.20)
  # Planner already climbing harder toward MAX — do not freeze at last pedal.
  assert coast.apply(1.60) == pytest.approx(1.60)


def test_tiny_drop_latches_without_waiting_for_old_deadband():
  coast = _coast()
  _engage_with_gas(coast, pedal=16.0, a_ego=1.40)
  # Old PEDAL_DROP_DI=0.40 would still be unprotected here.
  _lift(coast, pedal=16.0 - 0.05, a_ego=-0.60)

  assert coast.active
  assert coast.hold_pedal == pytest.approx(16.0)
  assert coast.apply(0.0) == pytest.approx(1.40)
  assert coast.apply(-0.25) > 0.0


def test_noise_smaller_than_drop_does_not_latch():
  coast = _coast()
  _engage_with_gas(coast, pedal=12.0, a_ego=0.90)
  coast.update(long_engaged=True, gas_pressed=True, pedal_pos=12.0 - 0.02, a_ego=0.90)

  assert not coast.active
  assert coast.apply(-0.20) == pytest.approx(-0.20)


def test_hold_gas_past_one_second_then_lift_still_climbs():
  """Timer starts on the lift. Engage+gas for >1 s must not miss the handoff."""
  coast = _coast()
  steps = int(1.2 / DT_MDL) + 1
  _engage_with_gas(coast, frames=steps, pedal=13.0, a_ego=0.80)
  assert not coast.active

  _lift(coast, pedal=12.0, a_ego=-0.50)
  assert coast.active
  assert coast.apply(0.0) >= CLIMB_FLOOR_MS2
  assert coast.apply(-0.40) > 0.0


def test_brake_still_decelerates_inside_window():
  coast = _coast()
  _engage_with_gas(coast)
  _lift(coast)

  assert coast.apply(-0.55, brake_pressed=True) == pytest.approx(-0.55)
  assert coast.apply(-1.20, brake_pressed=True) == pytest.approx(-1.20)
  assert not coast.should_hold_pedal(-0.55, brake_pressed=True)


def test_after_1s_from_lift_normal_regen_allowed():
  coast = _coast()
  _engage_with_gas(coast)
  _lift(coast)
  steps = int(POST_ENGAGE_COAST_S / DT_MDL) + 2
  for _ in range(steps):
    coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0, a_ego=-0.20)

  assert not coast.active
  assert coast.apply(-0.40) == pytest.approx(-0.40)


def test_engage_without_prior_gas_does_not_hold():
  coast = _coast()
  coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0, a_ego=0.0)

  assert not coast.active
  assert coast.apply(-0.40) == pytest.approx(-0.40)
  assert coast.apply(0.30) == pytest.approx(0.30)


def test_gas_tap_during_window_then_lift_climbs():
  """Engage with no gas, tap throttle, lift → same climb path."""
  coast = _coast()
  coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0, a_ego=0.0)
  assert not coast.active

  coast.update(long_engaged=True, gas_pressed=True, pedal_pos=9.0, a_ego=0.70)
  coast.update(long_engaged=True, gas_pressed=True, pedal_pos=8.0, a_ego=0.20)
  assert coast.active
  assert coast.hold_pedal == pytest.approx(9.0)
  assert coast.apply(-0.25) == pytest.approx(0.70)
  assert coast.apply(1.10) == pytest.approx(1.10)


def test_no_decrease_does_not_hold():
  coast = _coast()
  for _ in range(6):
    coast.update(long_engaged=True, gas_pressed=True, pedal_pos=11.0, a_ego=0.90)

  assert not coast.active
  assert coast.apply(-0.30) == pytest.approx(-0.30)


def test_binary_gas_pressed_fallback_climbs_on_lift():
  coast = _coast()
  coast.update(long_engaged=True, gas_pressed=True, a_ego=1.10)
  coast.update(long_engaged=True, gas_pressed=False, a_ego=-0.90)

  assert coast.active
  assert coast.hold_pedal == pytest.approx(BINARY_PRESSED_DI)
  assert coast.apply(-0.50) == pytest.approx(1.10)
  assert coast.apply(0.0) == pytest.approx(1.10)


def test_climb_does_not_push_past_max():
  """Sticky/cruise MAX overrides a climb that would keep accelerating."""
  coast = _coast()
  _engage_with_gas(coast, pedal=14.0, a_ego=1.30)
  _lift(coast, pedal=12.0, a_ego=1.00)
  assert coast.active
  assert coast.apply(-0.20, v_ego=20.0, v_cruise=25.0) == pytest.approx(1.30)
  # At/above MAX: drop the climb and let normal long command through.
  assert coast.apply(-0.40, v_ego=25.0, v_cruise=25.0) == pytest.approx(-0.40)
  assert coast.apply(0.80, v_ego=25.0 + MAX_HOLD_SLACK_MS, v_cruise=25.0) == pytest.approx(0.80)
  assert not coast.should_hold_pedal(0.80, v_ego=25.0, v_cruise=25.0)
  assert coast.should_hold_pedal(0.80, v_ego=20.0, v_cruise=25.0)


def test_safety_paths_pass_through():
  coast = _coast()
  _engage_with_gas(coast)
  _lift(coast)

  assert coast.apply(-0.55, fcw=True) == pytest.approx(-0.55)
  assert coast.apply(-0.40, should_stop=True) == pytest.approx(-0.40)
  # Far-lead nibble / inversion must still climb.
  assert coast.apply(-0.30, has_lead=True) == pytest.approx(1.20)
  assert coast.apply(-LEAD_KEEP_DECEL_MS2, has_lead=True) == pytest.approx(-LEAD_KEEP_DECEL_MS2)
  assert coast.apply(-HARD_DECEL_MS2, has_lead=True) == pytest.approx(-HARD_DECEL_MS2)
  # Lone hard-looking inversion (no lead / FCW / brake) is the dip — climb.
  assert coast.apply(-1.50) == pytest.approx(1.20)


def test_long_drop_clears_window():
  coast = _coast()
  _engage_with_gas(coast)
  _lift(coast)
  coast.update(long_engaged=False, gas_pressed=False, pedal_pos=0.0)
  assert not coast.active
  assert coast.apply(-0.50) == pytest.approx(-0.50)

  coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0)
  assert not coast.active


def test_controlsd_dt_covers_full_second_from_lift():
  coast = PostEngageCoast(dt=DT_CTRL)
  coast.update(long_engaged=True, gas_pressed=True, pedal_pos=12.0, a_ego=0.80)
  for _ in range(int(0.99 / DT_CTRL)):
    coast.update(long_engaged=True, gas_pressed=True, pedal_pos=10.0, a_ego=0.20)
  assert coast.active
  assert coast.apply(-0.20) == pytest.approx(0.80)

  coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0)
  coast.update(long_engaged=True, gas_pressed=False, pedal_pos=0.0)
  assert not coast.active


def test_zero_aego_while_pressed_still_climbs_on_lift():
  """Pressed but not accelerating — lift must still produce +a toward MAX."""
  coast = _coast()
  _engage_with_gas(coast, pedal=10.0, a_ego=0.0)
  _lift(coast, pedal=8.0, a_ego=-0.40)
  assert coast.active
  assert coast.apply(0.0) == pytest.approx(CLIMB_FLOOR_MS2)
  assert coast.apply(-0.15) == pytest.approx(CLIMB_FLOOR_MS2)


def test_cs_pedal_di_prefers_interceptor_then_command():
  assert cs_pedal_di(SimpleNamespace(pedal_interceptor_value=14.5, gasPressed=True)) == pytest.approx(14.5)
  assert cs_pedal_di(SimpleNamespace(pedalCommandDi=7.0, gasPressed=False)) == pytest.approx(7.0)
  assert cs_pedal_di(SimpleNamespace(gasPressed=True)) == pytest.approx(BINARY_PRESSED_DI)
  assert cs_pedal_di(SimpleNamespace(gasPressed=False)) == pytest.approx(0.0)


class _FakeTeslaCan:
  def __init__(self, replacement=None):
    self.calls = []
    self.replacement = replacement if replacement is not None else (
      0x551, bytes([0, 0, 0, 0, 0x81, 0]), 2)

  def create_pedal_command(self, pedal, enable=0):
    self.calls.append((pedal, enable))
    return self.replacement


class _FakeVdas:
  def __init__(self):
    self.resets = []
    self.prev_pedal_di = 0.0

  def reset(self, measured_accel=0.0, commanded_accel=0.0, pedal_di_init=0.0,
            preserve_grade=False):
    self.resets.append({
      "measured_accel": measured_accel,
      "commanded_accel": commanded_accel,
      "pedal_di_init": pedal_di_init,
      "preserve_grade": preserve_grade,
    })
    self.prev_pedal_di = pedal_di_init


def test_apply_held_pedal_rewrites_enabled_gas_command():
  from openpilot.selfdrive.car.tesla import preap_post_engage_hold as hold_mod

  tesla_can = _FakeTeslaCan()
  controller = SimpleNamespace(
    prev_pedal_di=3.0,
    preap_long_engage_frame=100,
    preap_long_handoff_slew_active=True,
    vdas=SimpleNamespace(prev_pedal_di=3.0),
  )
  cs = SimpleNamespace(pedal_command_di=0.0, pedal_command_counter=0)
  sends = [
    (0x123, b"\x00", 0),
    (0x551, bytes([1, 2, 3, 4, 0x80, 5]), 2),
  ]

  assert hold_mod.apply_held_pedal_command(
    controller, cs, tesla_can, sends, 15.0, di_to_pedal=lambda di: di * 2.0)
  assert sends[-1] == tesla_can.replacement
  assert tesla_can.calls == [(30.0, 1)]
  assert controller.prev_pedal_di == pytest.approx(15.0)
  assert controller.vdas.prev_pedal_di == pytest.approx(15.0)
  assert controller.preap_long_handoff_slew_active is False
  assert cs.pedal_command_di == pytest.approx(15.0)
  # Grace must be expired so the next VDAS update cannot floor at a=0.
  assert controller.preap_long_engage_frame <= 100 - hold_mod.ENGAGE_GRACE_FRAMES


def test_apply_held_pedal_does_not_rewrite_enable_zero():
  from openpilot.selfdrive.car.tesla import preap_post_engage_hold as hold_mod

  tesla_can = _FakeTeslaCan()
  controller = SimpleNamespace(prev_pedal_di=0.0, preap_long_engage_frame=0,
                               preap_long_handoff_slew_active=False)
  cs = SimpleNamespace()
  sends = [(0x551, bytes([0, 0, 0, 0, 0x00, 0]), 2)]
  assert not hold_mod.apply_held_pedal_command(controller, cs, tesla_can, sends, 12.0)
  assert tesla_can.calls == []


def test_climb_handoff_expires_grace_and_seeds_vdas_without_enable():
  """gasPressed→ENABLE gap: still expire grace / seed +a so ACQUIRE cannot coast."""
  from openpilot.selfdrive.car.tesla import preap_post_engage_hold as hold_mod

  coast = _coast()
  _engage_with_gas(coast, pedal=14.0, a_ego=1.10)
  _lift(coast, pedal=13.0, a_ego=-0.40)
  tesla_can = _FakeTeslaCan()
  vdas = _FakeVdas()
  controller = SimpleNamespace(
    prev_pedal_di=0.0,
    preap_long_engage_frame=200,
    preap_long_handoff_slew_active=True,
    vdas=vdas,
    _nap_climb_seeded=False,
  )
  cs = SimpleNamespace(aEgo=-0.40, pedal_command_di=0.0)
  # ENABLE=0 passthrough — panda still blocks ENABLE=1 while gas_pressed.
  sends = [(0x551, bytes([0, 0, 0, 0, 0x00, 0]), 2)]

  assert hold_mod.apply_climb_handoff(
    controller, cs, tesla_can, sends, coast, frame=200)
  assert tesla_can.calls == []
  assert sends[0][1][4] == 0x00
  assert controller.preap_long_engage_frame <= 200 - hold_mod.ENGAGE_GRACE_FRAMES
  assert controller.preap_long_handoff_slew_active is False
  assert vdas.resets
  assert vdas.resets[-1]["commanded_accel"] == pytest.approx(1.10)
  assert vdas.resets[-1]["pedal_di_init"] == pytest.approx(14.0)


def test_climb_handoff_rewrites_grace_enable_then_lets_op_long_climb():
  from openpilot.selfdrive.car.tesla import preap_post_engage_hold as hold_mod

  coast = _coast()
  _engage_with_gas(coast, pedal=14.0, a_ego=1.10)
  _lift(coast, pedal=13.0, a_ego=-0.40)
  tesla_can = _FakeTeslaCan()
  vdas = _FakeVdas()
  controller = SimpleNamespace(
    prev_pedal_di=0.0,
    preap_long_engage_frame=50,
    preap_long_handoff_slew_active=True,
    vdas=vdas,
    _nap_climb_seeded=False,
  )
  cs = SimpleNamespace(aEgo=-0.40, pedal_command_di=0.0, pedal_command_counter=0)
  sends = [(0x551, bytes([1, 2, 3, 4, 0x80, 5]), 2)]

  # ACQUIRE frame (elapsed=0) is still in grace — rewrite the a=0 command.
  assert hold_mod.apply_climb_handoff(
    controller, cs, tesla_can, sends, coast, frame=50,
    di_to_pedal=lambda di: di)
  assert tesla_can.calls == [(14.0, 1)]
  assert vdas.resets[-1]["commanded_accel"] == pytest.approx(1.10)

  tesla_can.calls.clear()
  vdas.resets.clear()
  # Grace already expired — later ENABLE=1 frames are normal OP long.
  sends = [(0x551, bytes([9, 9, 9, 9, 0x80, 5]), 2)]
  assert hold_mod.apply_climb_handoff(
    controller, cs, tesla_can, sends, coast, frame=52,
    di_to_pedal=lambda di: di)
  assert tesla_can.calls == []
  assert sends[0][1][0] == 9


def test_expire_engage_grace_clears_half_second_floor():
  from openpilot.selfdrive.car.tesla import preap_post_engage_hold as hold_mod

  controller = SimpleNamespace(preap_long_engage_frame=80,
                               preap_long_handoff_slew_active=True)
  hold_mod.expire_engage_grace(controller, frame=80)
  assert (80 - controller.preap_long_engage_frame) >= hold_mod.ENGAGE_GRACE_FRAMES
  assert controller.preap_long_handoff_slew_active is False


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
                        brake=0.0, lead=None, pitch=0.0, pedal_di=0.0):
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
  if hasattr(car_state, "pedalCommandDi") and pedal_di:
    car_state.pedalCommandDi = float(pedal_di)
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


def test_planner_gas_release_after_engage_climbs_no_coast():
  """Engage while accelerating, lift, MAX is higher → +a climb, not coast/regen."""
  v_ego = 20.0
  v_cruise = 25.0
  planner = _new_planner(v_ego, 1.2)

  for _ in range(8):
    _run(planner, v_ego=v_ego, v_cruise=v_cruise, a_ego=1.4, gas=True, long_on=True, pedal_di=14.0)
    v_ego += 1.4 * DT_MDL

  for _ in range(6):
    a = _run(planner, v_ego=v_ego, v_cruise=v_cruise, a_ego=-1.1, gas=False, long_on=True, pedal_di=0.0)
    assert planner._post_engage_coast.active
    assert a > 0.3, a
    v_ego += max(a, -1.1) * DT_MDL


def test_planner_brake_inside_window_may_decelerate():
  planner = _new_planner(22.0, 0.8)
  for _ in range(4):
    _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=0.8, gas=True, long_on=True, pedal_di=12.0)

  # Digital Applied on CS.brake (brakePressed is forced false on Pre-AP).
  # Above MAX so map/MPC already want −a; brake must not be held.
  a = _run(planner, v_ego=22.0, v_cruise=18.0, a_ego=-0.6, gas=False, long_on=True, brake=1.0)
  assert a < -0.05, a


def test_planner_after_window_allows_regen_to_lower_max():
  planner = _new_planner(25.0, 0.5)
  _run(planner, v_ego=25.0, v_cruise=20.0, a_ego=0.5, gas=True, long_on=True, pedal_di=10.0)
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
    _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=0.8, gas=True, long_on=True, pedal_di=12.0)

  lead = {"dRel": 12.0, "vLead": 8.0, "modelProb": 0.9}
  a = _run(planner, v_ego=22.0, v_cruise=28.0, a_ego=-0.4, gas=False, long_on=True, lead=lead)
  assert a < -0.05, a


def test_planner_and_controlsd_wire_the_overlay():
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  controlsd = (root / "selfdrive/controls/controlsd.py").read_text()
  docs = (root / "docs-nap/engagement.md").read_text()
  card = (root / "selfdrive/car/card.py").read_text()
  hold = (root / "selfdrive/car/tesla/preap_post_engage_hold.py").read_text()

  lead_at = planner.find("apply_lead_approach_overlay(output_a_target, a_lead)")
  coast_at = planner.find("self._post_engage_coast.apply(")
  clip_at = planner.find("self.output_a_target = np.clip(output_a_target")
  assert 0 <= lead_at < coast_at < clip_at
  assert "hold_accel" in planner
  assert "cs_pedal_di" in planner
  assert "enableLongControl" in planner
  assert "PostEngageCoast" in controlsd
  assert "a_target = self._post_engage_coast.apply(" in controlsd
  assert "cs_pedal_di" in controlsd
  assert "1.0 s" in docs or "1 s" in docs
  assert "pedal" in docs.lower()
  assert "install_post_engage_hold" in card
  assert card.index("install_force_offroad_handoff()") < card.index("install_post_engage_hold()")
  assert "expire_engage_grace" in hold
  assert "seed_vdas_climb" in hold
  assert "apply_climb_handoff" in hold
  assert "commanded_accel=0" in hold
