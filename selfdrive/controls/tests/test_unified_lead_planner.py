"""Planner integration for NAPLongUnified.

Default off leaves the layered command alone and still publishes the shadow.
On, with a lead, the published accel is the continuous controller. The mode
weight snaps while disengaged and slews for about a second while following.
"""
import numpy as np
import pytest

from cereal import car, log, messaging
from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource, T_IDXS
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.unified_lead import MODE_BLEND_S
from openpilot.selfdrive.modeld.constants import ModelConstants


class _PlannerInputs(dict):
  logMonoTime = {"modelV2": 0}

  @staticmethod
  def all_checks(service_list):
    return set(service_list) == {"carState", "controlsState", "selfdriveState", "radarState"}


class _CapturingPubMaster:
  def send(self, service, message):
    assert service == "longitudinalPlan"
    self.message = message


class _MutablePlannerParams:
  """Same unknown-key behavior as the following-suite double.

  NAPLongUnified is not special-cased, so a missing key raises and the
  planner must keep the controller off.
  """

  def __init__(self, nap_follow_dist, adaptive_accel=False, map_speed_accel=5):
    self.nap_follow_dist = nap_follow_dist
    self.adaptive_accel = adaptive_accel
    self.map_speed_accel = map_speed_accel
    self.city = nap_follow_dist
    self.hwy = nap_follow_dist
    self.migrated = True

  def __bool__(self):
    return False

  def get(self, key, return_default=False):
    assert return_default
    if key == "NAPFollowDistance":
      return self.nap_follow_dist
    if key == "NAPFollowDistanceCity":
      return self.city
    if key == "NAPFollowDistanceHwy":
      return self.hwy
    if key == "NAPMapSpeedAccel":
      return self.map_speed_accel
    if key == "NAPMapSpeedMode":
      return 0
    if key == "NAPMapSpeedOffsetMph":
      return 0
    if key == "NAPMapSpeedLookahead":
      return 2
    raise AssertionError(key)

  def get_bool(self, key):
    if key == "NAPAdaptiveAccel":
      return self.adaptive_accel
    if key == "NAPHypermile":
      return False
    if key == "NAPHypermileHillClimb":
      return True
    if key == "NAPFollowDistanceSplitMigrated":
      return self.migrated
    raise AssertionError(key)

  def put_bool(self, key, value):
    if key == "NAPFollowDistanceSplitMigrated":
      self.migrated = bool(value)
    elif key == "NAPAdaptiveAccel":
      self.adaptive_accel = bool(value)


class _UnifiedParams(_MutablePlannerParams):
  def __init__(self, *args, unified=False, **kwargs):
    super().__init__(*args, **kwargs)
    self.unified = unified
    self.unified_reads = 0

  def get_bool(self, key):
    if key == "NAPLongUnified":
      self.unified_reads += 1
      return bool(self.unified)
    return super().get_bool(key)


class _ConstantAccelerationMpc:
  def __init__(self, speed_mps, acceleration_mps2):
    self.v_solution = speed_mps + acceleration_mps2 * T_IDXS
    self.a_solution = np.full(len(T_IDXS), acceleration_mps2)
    self.j_solution = np.zeros(len(T_IDXS) - 1)
    self.params = np.zeros((len(T_IDXS), 6))
    self.source = LongitudinalPlanSource.cruise
    self.crash_cnt = 0
    self.solve_time = 0.0

  @staticmethod
  def set_weights(prev_accel_constraint, personality):
    pass

  @staticmethod
  def set_cur_state(speed_mps, acceleration_mps2):
    pass

  def update(self, radar_state, cruise_speed_mps, t_follow):
    self.params[:, 4] = t_follow


def _make_preap_params():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  params.longitudinalTuning.kpBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kpV = PEDAL_LONG_KP_V
  params.longitudinalTuning.kiBP = PEDAL_LONG_K_BP
  params.longitudinalTuning.kiV = PEDAL_LONG_KI_V
  params.longitudinalTuning.kf = 1.0
  params.vEgoStarting = 0.1
  return params


def _set_v_cruise_ms(inputs, v_cruise_ms):
  inputs["carState"].vCruise = float(v_cruise_ms) * CV.MS_TO_KPH


def _make_planner_inputs(speed_mps):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.pid
  selfdrive.personality = log.LongitudinalPersonality.standard
  car_state.vEgo = speed_mps
  car_state.vCruise = speed_mps * 3.6
  car_control.orientationNED = [0.0, 0.0, 0.0]

  model.position.x = (speed_mps * np.array(ModelConstants.T_IDXS)).tolist()
  model.velocity.x = (speed_mps * np.ones_like(ModelConstants.T_IDXS)).tolist()
  model.acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6

  return _PlannerInputs({
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
  })


def _planner(v_ego, *, unified=False, nap_follow_dist=2, accel=-3.5):
  params = _UnifiedParams(nap_follow_dist=nap_follow_dist, unified=unified)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=accel)
  planner.prev_accel_clip = [-3.5, 2.0]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, max(v_ego + 8.0, 40.0))
  return planner, inputs, params


def _own_lead(inputs, d_rel, v_lead, a_lead=0.0, y_rel=0.0, track=1):
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = float(d_rel)
  lead.vLead = float(v_lead)
  lead.aLeadK = float(a_lead)
  lead.yRel = float(y_rel)
  lead.modelProb = 1.0
  lead.radar = True
  lead.radarTrackId = int(track)
  return lead


def _run(planner, inputs, n, mpc_accel=None):
  if mpc_accel is not None:
    crash = float(getattr(planner.mpc, "crash_cnt", 0) or 0)
    planner.mpc = _ConstantAccelerationMpc(inputs["carState"].vEgo, acceleration_mps2=mpc_accel)
    planner.mpc.crash_cnt = crash
  for _ in range(n):
    planner.update(inputs)
  return float(planner.output_a_target)


def test_unified_off_keeps_mode_weight_at_zero():
  """Absent / false NAPLongUnified must not move the mode weight while engaged.

  Forcing `enabled = True` slews the weight by dt/1s on this frame.
  """
  v_ego = 30.0
  params = _MutablePlannerParams(nap_follow_dist=2)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-3.5)
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, 40.0)
  _own_lead(inputs, 35.0, v_ego - 1.13, a_lead=-1.18)
  assert planner._unified_enabled is False
  planner.update(inputs)
  assert planner._plan_engaged is True
  # Layered command stays in force. A forced-on toggle would have slewed this off zero.
  assert planner._mode_w == 0.0


def test_toggle_off_matches_legacy_and_still_logs_shadow():
  v_ego = 30.0
  v_lead = v_ego - 1.13
  legacy_params = _MutablePlannerParams(nap_follow_dist=2)
  legacy = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=legacy_params)
  legacy.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-3.5)
  explicit, inputs, _ = _planner(v_ego, unified=False, accel=-3.5)
  _own_lead(inputs, 35.0, v_lead, a_lead=-1.18)
  legacy_inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(legacy_inputs, 40.0)
  _own_lead(legacy_inputs, 35.0, v_lead, a_lead=-1.18)
  for _ in range(40):
    legacy.update(legacy_inputs)
    explicit.update(inputs)
  assert explicit._mode_w == 0.0
  assert float(explicit.output_a_target) == pytest.approx(float(legacy.output_a_target), abs=1e-9)
  assert float(explicit.unified_a_target) <= -0.5
  publisher = _CapturingPubMaster()
  explicit.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.unifiedATarget == pytest.approx(explicit.unified_a_target)


def test_toggle_on_follows_the_continuous_command_for_sep23():
  """23:31 geometry: firm match to a braking lead, not the raw MPC −3.5."""
  v_ego = 30.0
  v_lead = v_ego - 1.13
  on, inputs, _ = _planner(v_ego, unified=True, accel=-3.5)
  off, off_inputs, _ = _planner(v_ego, unified=False, accel=-3.5)
  _own_lead(inputs, 35.0, v_lead, a_lead=-1.18)
  _own_lead(off_inputs, 35.0, v_lead, a_lead=-1.18)
  _run(on, inputs, 80)
  _run(off, off_inputs, 80)
  assert on._mode_w == 1.0
  assert float(on.output_a_target) == pytest.approx(float(on.unified_a_target), abs=1e-6)
  assert -2.2 < float(on.output_a_target) <= -0.90
  assert abs(float(off.output_a_target) - float(off.unified_a_target)) > 0.15 or abs(
    float(on.output_a_target) - float(off.output_a_target)
  ) > 0.05


def test_mode_weight_slews_while_following_and_snaps_while_disengaged():
  v_ego = 22.0
  planner, inputs, params = _planner(v_ego, unified=False, accel=0.0)
  _own_lead(inputs, 40.0, v_ego, a_lead=0.0)
  planner.update(inputs)
  assert planner._mode_w == 0.0

  params.unified = True
  planner._unified_enabled = True
  planner.update(inputs)
  step = float(planner.dt) / MODE_BLEND_S
  assert planner._mode_w == pytest.approx(step, abs=1e-9)
  assert planner._mode_w < 0.5

  for _ in range(25):
    planner.update(inputs)
  assert planner._mode_w == 1.0

  params.unified = False
  planner._unified_enabled = False
  planner.update(inputs)
  assert planner._mode_w == pytest.approx(1.0 - step, abs=1e-6)
  assert planner._mode_w > 0.5

  inputs["controlsState"].longControlState = LongCtrlState.off
  planner.update(inputs)
  assert planner._plan_engaged is False
  assert planner._mode_w == 0.0

  params.unified = True
  planner._unified_enabled = True
  planner.update(inputs)
  assert planner._mode_w == 1.0


def test_no_lead_keeps_the_cruise_command_when_unified_is_on():
  v_ego = 20.0
  on, inputs, _ = _planner(v_ego, unified=True, accel=0.40)
  off, off_inputs, _ = _planner(v_ego, unified=False, accel=0.40)
  inputs["controlsState"].longControlState = LongCtrlState.off
  off_inputs["controlsState"].longControlState = LongCtrlState.off
  on.update(inputs)
  off.update(off_inputs)
  assert on._mode_w == 1.0
  inputs["controlsState"].longControlState = LongCtrlState.pid
  off_inputs["controlsState"].longControlState = LongCtrlState.pid
  for _ in range(10):
    on.update(inputs)
    off.update(off_inputs)
  assert inputs["radarState"].leadOne.status is False
  assert float(on.output_a_target) == pytest.approx(float(off.output_a_target), abs=1e-9)
  assert float(on.unified_a_target) == pytest.approx(0.0, abs=1e-9)


def test_faster_lead_at_max_does_not_overrun_when_unified_is_on():
  v_max = 60.0 * CV.MPH_TO_MS
  v_lead = v_max + 3.0 * CV.MPH_TO_MS
  planner, inputs, _ = _planner(v_max, unified=True, nap_follow_dist=4, accel=1.2)
  _set_v_cruise_ms(inputs, v_max)
  _own_lead(inputs, 80.0, v_lead, a_lead=0.2)
  inputs["controlsState"].longControlState = LongCtrlState.off
  planner.update(inputs)
  inputs["controlsState"].longControlState = LongCtrlState.pid
  _run(planner, inputs, 30, mpc_accel=1.2)
  assert planner._mode_w == 1.0
  assert float(planner.output_a_target) <= 0.05


def test_fcw_is_not_weaker_than_the_layered_command():
  v_ego = 30.0
  v_lead = v_ego - 1.13
  on, inputs, _ = _planner(v_ego, unified=True, accel=-3.5)
  off, off_inputs, _ = _planner(v_ego, unified=False, accel=-3.5)
  _own_lead(inputs, 35.0, v_lead, a_lead=-1.18)
  _own_lead(off_inputs, 35.0, v_lead, a_lead=-1.18)
  on.mpc.crash_cnt = 3
  off.mpc.crash_cnt = 3
  inputs["controlsState"].longControlState = LongCtrlState.off
  on.update(inputs)
  inputs["controlsState"].longControlState = LongCtrlState.pid
  _run(on, inputs, 8, mpc_accel=-3.5)
  _run(off, off_inputs, 8, mpc_accel=-3.5)
  assert on.fcw is True
  assert off.fcw is True
  assert float(on.output_a_target) <= float(off.output_a_target) + 1e-6


def test_curve_ceiling_brakes_only_on_the_unified_path():
  v_ego = 25.0
  on, inputs, _ = _planner(v_ego, unified=True, accel=0.0)
  off, off_inputs, _ = _planner(v_ego, unified=False, accel=0.0)
  _own_lead(inputs, 50.0, v_ego + 1.0, a_lead=0.0)
  _own_lead(off_inputs, 50.0, v_ego + 1.0, a_lead=0.0)
  inputs["carState"].steeringAngleDeg = 15.0
  off_inputs["carState"].steeringAngleDeg = 15.0
  inputs["controlsState"].longControlState = LongCtrlState.off
  on.update(inputs)
  inputs["controlsState"].longControlState = LongCtrlState.pid
  _run(on, inputs, 40, mpc_accel=0.0)
  _run(off, off_inputs, 40, mpc_accel=0.0)
  assert float(on.output_a_target) <= -0.50
  assert float(off.output_a_target) > float(on.output_a_target)


def test_unified_param_is_read_about_once_per_second():
  v_ego = 15.0
  planner, inputs, params = _planner(v_ego, unified=False, accel=0.0)
  assert params.unified_reads == 1
  for _ in range(10):
    planner.update(inputs)
  assert params.unified_reads == 1
  for _ in range(15):
    planner.update(inputs)
  assert params.unified_reads == 2
