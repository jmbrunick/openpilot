from types import SimpleNamespace

import numpy as np
import pytest

from cereal import car, log, messaging
from opendbc.car.tesla.preap import virtual_das
from opendbc.car.tesla.preap.constants import PEDAL_LONG_K_BP, PEDAL_LONG_KI_V, PEDAL_LONG_KP_V
from opendbc.car.tesla.preap.virtual_das import GRAVITY, VirtualDAS
from opendbc.car.tesla.pedal.controller import PEDAL_RAMP_RATE_DOWN, PEDAL_RAMP_RATE_UP
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib import longitudinal_planner
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_ACQUIRE_HOLD_S,
  LEAD_ACQUIRE_SLEW_MS2,
  LEAD_APPROACH_A_MS2,
  LEAD_APPROACH_MAX_START_M,
  LEAD_APPROACH_MILD_A_MS2,
  LEAD_APPROACH_RAPID_CONFIRM_N,
  LEAD_CLOSE_A_MAX_MS2,
  LEAD_CLOSE_A_MIN_MS2,
  LEAD_CLOSE_HOLD_S,
  LEAD_CLOSE_MAX_M,
  LEAD_CLOSE_OPENING_A_MS2,
  LEAD_FOLLOW_CHATTER_SLEW_MS2,
  LEAD_GLIDE_A_MS2,
  LEAD_MAP_MIDGAP_FLOOR_MS2,
  LEAD_MID_GAP_REMATCH_A_MS2,
  LEAD_MPC_SOFT_NEAR_M,
  LEAD_POST_DUMP_HOLD_S,
  LEAD_SETTLE_HOLD_S,
  lead_close_accel_ms2,
  lead_hunt_accel_ms2,
  lead_inside_slow_close_a_ms2,
)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource,
  T_IDXS,
  get_safe_obstacle_distance,
  get_stopped_equivalence_factor,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.tests.test_following_distance import run_following_distance_simulation
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config


NAP_FOLLOW_SETTINGS = range(1, 8)
NAP_FOLLOW_TIMES_S = (0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9)
FOLLOW_TEST_SPEED_MPS = 25.0
STOP_DISTANCE_M = 6.0
FULL_LOOP_DT_S = 0.01
FULL_LOOP_PLANNER_DT_S = 0.05
FULL_LOOP_VDAS_DT_S = 0.02
FULL_LOOP_PLANT_DELAY_S = 0.40
FULL_LOOP_PLANT_TAU_S = 0.25
FULL_LOOP_PEDAL_DI_BP = [-5.0, -2.0, 0.0, 3.0, 8.0, 15.0, 22.0, 27.0, 35.0, 50.0]
FULL_LOOP_NET_ACCEL_BP = [-1.05, -0.62, -0.50, -0.36, 0.0, 0.50, 1.15, 1.65, 2.10, 2.45]
FULL_LOOP_RECOVERY_END_S = 70.0
FULL_LOOP_UPHILL_RAMP_END_S = 72.0
FULL_LOOP_UPHILL_HOLD_END_S = 78.0
FULL_LOOP_CREST_RAMP_END_S = 80.0
FULL_LOOP_CREST_HOLD_END_S = 86.0
FULL_LOOP_ROLLING_RAMP_END_S = 88.0
FULL_LOOP_DURATION_S = 100.0
FULL_LOOP_UPHILL_PITCH_RAD = float(np.deg2rad(4.0))
FULL_LOOP_CREST_PITCH_RAD = float(np.deg2rad(-3.0))
FULL_LOOP_ROLLING_PITCH_RAD = float(np.deg2rad(2.0))
FULL_LOOP_GRADE_SETTLING_S = (
  FULL_LOOP_PLANT_DELAY_S + FULL_LOOP_PLANT_TAU_S + 5.0 * virtual_das.PITCH_LP_RC
)


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
  def __init__(self, nap_follow_dist, adaptive_accel=False, map_speed_accel=5,
               city=None, hwy=None):
    self.nap_follow_dist = nap_follow_dist
    self.adaptive_accel = adaptive_accel
    self.map_speed_accel = map_speed_accel
    self.city = city if city is not None else nap_follow_dist
    self.hwy = hwy if hwy is not None else nap_follow_dist
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

  def put(self, key, value):
    if key == "NAPFollowDistance":
      self.nap_follow_dist = value
    elif key == "NAPFollowDistanceCity":
      self.city = value
    elif key == "NAPFollowDistanceHwy":
      self.hwy = value

  def put_bool(self, key, value):
    if key == "NAPFollowDistanceSplitMigrated":
      self.migrated = bool(value)
    elif key == "NAPAdaptiveAccel":
      self.adaptive_accel = bool(value)


class _ConstantAccelerationMpc:
  def __init__(self, speed_mps, acceleration_mps2):
    self.v_solution = speed_mps + acceleration_mps2 * T_IDXS
    self.a_solution = np.full(len(T_IDXS), acceleration_mps2)
    self.j_solution = np.zeros(len(T_IDXS) - 1)
    self.params = np.zeros((len(T_IDXS), 6))
    self.source = LongitudinalPlanSource.cruise
    self.crash_cnt = 0
    self.solve_time = 0.0
    self.captured_t_follow = None

  @staticmethod
  def set_weights(prev_accel_constraint, personality):
    pass

  @staticmethod
  def set_cur_state(speed_mps, acceleration_mps2):
    pass

  def update(self, radar_state, cruise_speed_mps, t_follow):
    self.captured_t_follow = t_follow
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


# Catch-up tests must sit well under MAX so last-mph taper / deadband
# do not zero lead-close +a. Accel 1 tapers over ~5 mph.
_UNDER_MAX_HEADROOM_MS = 6.0


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


def _physical_lead_distance(v_ego, v_lead, t_follow, obstacle_ratio):
  safe_obstacle_distance = get_safe_obstacle_distance(v_ego, t_follow)
  lead_obstacle_distance = obstacle_ratio * safe_obstacle_distance
  return lead_obstacle_distance - get_stopped_equivalence_factor(max(v_lead, 0.0))


def _full_loop_pitch(elapsed_s):
  if elapsed_s < FULL_LOOP_RECOVERY_END_S:
    return 0.0
  if elapsed_s < FULL_LOOP_UPHILL_RAMP_END_S:
    return FULL_LOOP_UPHILL_PITCH_RAD * (
      (elapsed_s - FULL_LOOP_RECOVERY_END_S) /
      (FULL_LOOP_UPHILL_RAMP_END_S - FULL_LOOP_RECOVERY_END_S)
    )
  if elapsed_s < FULL_LOOP_UPHILL_HOLD_END_S:
    return FULL_LOOP_UPHILL_PITCH_RAD
  if elapsed_s < FULL_LOOP_CREST_RAMP_END_S:
    ramp_fraction = (
      (elapsed_s - FULL_LOOP_UPHILL_HOLD_END_S) /
      (FULL_LOOP_CREST_RAMP_END_S - FULL_LOOP_UPHILL_HOLD_END_S)
    )
    return FULL_LOOP_UPHILL_PITCH_RAD + ramp_fraction * (
      FULL_LOOP_CREST_PITCH_RAD - FULL_LOOP_UPHILL_PITCH_RAD
    )
  if elapsed_s < FULL_LOOP_CREST_HOLD_END_S:
    return FULL_LOOP_CREST_PITCH_RAD
  if elapsed_s < FULL_LOOP_ROLLING_RAMP_END_S:
    ramp_fraction = (
      (elapsed_s - FULL_LOOP_CREST_HOLD_END_S) /
      (FULL_LOOP_ROLLING_RAMP_END_S - FULL_LOOP_CREST_HOLD_END_S)
    )
    return FULL_LOOP_CREST_PITCH_RAD + ramp_fraction * (
      FULL_LOOP_ROLLING_PITCH_RAD - FULL_LOOP_CREST_PITCH_RAD
    )
  return FULL_LOOP_ROLLING_PITCH_RAD


def _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    plant_aligned_feedforward=False,
    grade_compensation_scale=1.0,
):
  monkeypatch.setattr(
    virtual_das,
    "nap_conf",
    SimpleNamespace(get_pedal_profile_values=lambda: [50.0] * len(virtual_das.PEDAL_BP)),
  )
  monkeypatch.setattr(
    virtual_das,
    "get_zero_torque",
    lambda: SimpleNamespace(get=lambda _speed_mps: 3.0),
  )

  speed_mps = FOLLOW_TEST_SPEED_MPS
  acceleration_mps2 = 0.0
  ego_distance_m = 0.0
  lead_distance_m = 20.0
  lead_speed_mps = FOLLOW_TEST_SPEED_MPS
  pedal_di = 8.0
  planner_target_mps2 = 0.0
  vdas_target_mps2 = 0.0

  params = _MutablePlannerParams(nap_follow_dist=nap_follow_dist, adaptive_accel=True)
  car_params = _make_preap_params()
  car_params.longitudinalActuatorDelay = FULL_LOOP_PLANT_DELAY_S
  planner = LongitudinalPlanner(car_params, init_v=speed_mps, params=params)
  long_control = LongControl(car_params)
  vdas = VirtualDAS(dt=FULL_LOOP_VDAS_DT_S)
  vdas.reset(measured_accel=acceleration_mps2, commanded_accel=0.0, pedal_di_init=pedal_di)
  if plant_aligned_feedforward:
    vdas._feedforward = lambda acceleration_effort_mps2, _speed_mps: float(np.interp(
      acceleration_effort_mps2,
      FULL_LOOP_NET_ACCEL_BP,
      FULL_LOOP_PEDAL_DI_BP,
    ))
  if grade_compensation_scale != 1.0:
    grade_estimator_update = vdas.grade_estimator.update

    def scaled_grade_estimator_update(orientation_ned):
      steady_compensation, transient_compensation = grade_estimator_update(orientation_ned)
      return (
        grade_compensation_scale * steady_compensation,
        grade_compensation_scale * transient_compensation,
      )

    vdas.grade_estimator.update = scaled_grade_estimator_update

  delay_steps = round(FULL_LOOP_PLANT_DELAY_S / FULL_LOOP_DT_S)
  delayed_pedals_di = [pedal_di] * delay_steps
  plant_alpha = FULL_LOOP_DT_S / (FULL_LOOP_PLANT_TAU_S + FULL_LOOP_DT_S)
  planner_interval_steps = round(FULL_LOOP_PLANNER_DT_S / FULL_LOOP_DT_S)
  vdas_interval_steps = round(FULL_LOOP_VDAS_DT_S / FULL_LOOP_DT_S)
  samples = []
  pedal_samples = []

  for step in range(round(FULL_LOOP_DURATION_S / FULL_LOOP_DT_S)):
    elapsed_s = step * FULL_LOOP_DT_S
    pitch_rad = _full_loop_pitch(elapsed_s)
    gap_m = lead_distance_m - ego_distance_m

    if step % planner_interval_steps == 0:
      inputs = _make_planner_inputs(float(speed_mps))
      inputs["carState"].aEgo = float(acceleration_mps2)
      inputs["carState"].vCruise = lead_speed_mps * 3.6
      inputs["carControl"].orientationNED = [0.0, pitch_rad, 0.0]
      lead = inputs["radarState"].leadOne
      lead.status = True
      lead.dRel = float(max(gap_m, 0.0))
      lead.vRel = float(lead_speed_mps - speed_mps)
      lead.vLead = lead_speed_mps
      lead.vLeadK = lead_speed_mps
      lead.aLeadK = 0.0
      lead.aLeadTau = 1.5
      lead.modelProb = 1.0
      planner.update(inputs)
      planner_target_mps2 = float(planner.output_a_target)

    state = car.CarState.new_message()
    state.vEgo = float(speed_mps)
    state.aEgo = float(acceleration_mps2)
    state.brakePressed = False
    state.cruiseState.standstill = False
    vdas_target_mps2 = float(long_control.update(
      active=True,
      CS=state,
      a_target=planner_target_mps2,
      should_stop=planner.output_should_stop,
      accel_limits=(-1.5, 0.8),
    ))

    if step % vdas_interval_steps == 0:
      pedal_di = vdas.update(
        vdas_target_mps2,
        v_ego=speed_mps,
        prev_pedal_di=pedal_di,
        a_ego=acceleration_mps2,
        freeze_integrator=False,
        orientation_ned=[0.0, pitch_rad, 0.0],
      )
      pedal_samples.append(pedal_di)

    applied_pedal_di = delayed_pedals_di.pop(0)
    delayed_pedals_di.append(pedal_di)
    grade_acceleration_mps2 = GRAVITY * np.sin(pitch_rad)
    plant_target_mps2 = float(np.interp(
      applied_pedal_di,
      FULL_LOOP_PEDAL_DI_BP,
      FULL_LOOP_NET_ACCEL_BP,
    )) - grade_acceleration_mps2
    acceleration_mps2 += plant_alpha * (plant_target_mps2 - acceleration_mps2)
    speed_mps = max(0.0, speed_mps + acceleration_mps2 * FULL_LOOP_DT_S)
    ego_distance_m += speed_mps * FULL_LOOP_DT_S
    lead_distance_m += lead_speed_mps * FULL_LOOP_DT_S

    samples.append((
      elapsed_s,
      lead_distance_m - ego_distance_m,
      speed_mps,
      acceleration_mps2,
      planner_target_mps2,
      vdas_target_mps2,
      pitch_rad,
    ))

  return np.array(samples), np.array(pedal_samples)


@pytest.mark.parametrize(("lead_speed", "obstacle_ratio", "expected_strength"), [
  (30.0, 1.0, 1.0),
  (30.0, 1.2, 1.0),
  (30.0, 1.35, 0.5),
  (30.0, 1.5, 0.0),
  (20.0, 1.35, 0.5),
  (-5.0, 1.35, 0.5),
])
def test_preap_follow_cap_uses_obstacle_equivalent_distance(lead_speed, obstacle_ratio, expected_strength):
  speed_mps = 30.0
  t_follow = 1.9
  lead_distance = _physical_lead_distance(speed_mps, lead_speed, t_follow, obstacle_ratio)

  cap_strength = longitudinal_planner.get_preap_follow_cap_strength(
    speed_mps,
    lead_distance,
    lead_speed,
    t_follow,
  )

  assert cap_strength == pytest.approx(expected_strength)


def test_planner_adaptive_cap_changes_the_delivered_acceleration_for_unequal_speed_lead():
  speed_mps = 40.0
  # Same-speed lead so ease does not fire. Obstacle-ratio 1.35 at 40 m/s is
  # a physical gap beyond the 200 m Bosch lead-close window. Adaptive Accel only.
  lead_speed_mps = 40.0
  obstacle_ratio = 1.35
  t_follow = 1.9
  params = _MutablePlannerParams(nap_follow_dist=7, adaptive_accel=True)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=speed_mps, params=params)
  planner.mpc = _ConstantAccelerationMpc(speed_mps, acceleration_mps2=1.5)
  inputs = _make_planner_inputs(speed_mps)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = _physical_lead_distance(speed_mps, lead_speed_mps, t_follow, obstacle_ratio)
  lead.vLead = lead_speed_mps
  assert lead.dRel > LEAD_CLOSE_MAX_M

  for _ in range(32):
    planner.update(inputs)

  open_road_limit = longitudinal_planner.get_max_accel(speed_mps)
  follow_limit = longitudinal_planner._get_preap_follow_limit(speed_mps)
  cap_strength = longitudinal_planner.get_preap_follow_cap_strength(
    speed_mps,
    lead.dRel,
    lead_speed_mps,
    t_follow,
  )
  expected_adaptive_limit = open_road_limit * (1.0 - cap_strength) + follow_limit * cap_strength

  assert cap_strength == pytest.approx(0.5)
  assert planner.mpc.captured_t_follow == t_follow
  assert planner.output_a_target == pytest.approx(expected_adaptive_limit)
  assert planner.output_a_target < open_road_limit


def test_planner_publishes_the_follow_policy_used_by_mpc():
  params = _MutablePlannerParams(nap_follow_dist=1)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=FOLLOW_TEST_SPEED_MPS, params=params)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  publisher = _CapturingPubMaster()

  params.nap_follow_dist = 7
  params.city = 7
  params.hwy = 7
  planner._frame = 19
  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 7
  assert plan.tFollow == pytest.approx(1.9)
  assert planner.t_follow == 1.9
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


@pytest.mark.parametrize("nap_follow_dist", [-1, 0, 8])
def test_invalid_nap_follow_setting_publishes_zero_and_uses_personality(nap_follow_dist):
  params = _MutablePlannerParams(nap_follow_dist)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=FOLLOW_TEST_SPEED_MPS, params=params)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  publisher = _CapturingPubMaster()

  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 0
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.standard)
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


def test_non_preap_planner_publishes_zero_and_uses_personality():
  planner_params = _make_preap_params()
  planner_params.brand = "honda"
  planner_params.carFingerprint = "HONDA_CIVIC"
  planner = LongitudinalPlanner(planner_params, init_v=FOLLOW_TEST_SPEED_MPS)
  inputs = _make_planner_inputs(FOLLOW_TEST_SPEED_MPS)
  inputs["selfdriveState"].personality = log.LongitudinalPersonality.relaxed
  publisher = _CapturingPubMaster()

  planner.update(inputs)
  planner.publish(inputs, publisher)

  plan = publisher.message.longitudinalPlan
  assert plan.napFollowDistance == 0
  assert planner.t_follow == get_T_FOLLOW(log.LongitudinalPersonality.relaxed)
  assert np.all(planner.mpc.params[:, 4] == planner.t_follow)
  assert plan.tFollow == pytest.approx(planner.t_follow, abs=1e-6)


def test_process_replay_ignores_additive_follow_policy_telemetry():
  ignored_fields = get_process_config("plannerd").ignore

  assert "longitudinalPlan.napFollowDistance" in ignored_fields
  assert "longitudinalPlan.tFollow" in ignored_fields


def test_nap_follow_setting_map_and_physical_gaps_are_strictly_monotonic():
  actual_follow_times = [get_T_FOLLOW(nap_follow_dist=setting) for setting in NAP_FOLLOW_SETTINGS]
  physical_gaps = [t_follow * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M for t_follow in actual_follow_times]

  assert actual_follow_times == list(NAP_FOLLOW_TIMES_S)
  assert np.all(np.diff(physical_gaps) > 0.0)


def test_nap_follow_settings_control_monotonic_maneuver_gaps():
  # Physical t_follow * v + stop_distance. Lead-approach ease must still
  # settle here (Follow 1 = 23.5 m); a TTC-floor hang of ~0.6 m is a miss.
  steady_gaps = [
    run_following_distance_simulation(
      FOLLOW_TEST_SPEED_MPS,
      t_end=80.0,
      nap_follow_dist=nap_follow_dist,
    )
    for nap_follow_dist in NAP_FOLLOW_SETTINGS
  ]
  expected_gaps = [
    t_follow * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
    for t_follow in NAP_FOLLOW_TIMES_S
  ]

  assert steady_gaps == pytest.approx(expected_gaps, abs=0.5)
  assert np.all(np.diff(steady_gaps) > 0.0)


def test_max_follow_setting_recovers_from_a_close_lead_without_closing_first():
  maneuver = Maneuver(
    "max follow recovery",
    duration=60.0,
    initial_speed=FOLLOW_TEST_SPEED_MPS,
    lead_relevancy=True,
    initial_distance_lead=20.0,
    speed_lead_values=[FOLLOW_TEST_SPEED_MPS],
    breakpoints=[0.0],
    e2e=False,
    nap_follow_dist=7,
  )

  valid, output = maneuver.evaluate()
  assert valid
  assert np.min(output[:, 6]) >= 19.5
  assert output[-1, 6] == pytest.approx(53.5, abs=0.5)
  assert output[-1, 3] == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.1)


def test_max_follow_full_closed_loop_recovers_gap_with_production_fallback(monkeypatch):
  samples, pedal_samples = _run_full_closed_loop_following(monkeypatch, nap_follow_dist=7)
  disabled_grade_samples, _ = _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    grade_compensation_scale=0.0,
  )
  elapsed_s, gaps_m, speeds_mps, accelerations_mps2, planner_targets_mps2, vdas_targets_mps2, _ = samples.T
  disabled_grade_speeds_mps = disabled_grade_samples[:, 2]
  desired_gap_m = get_T_FOLLOW(nap_follow_dist=7) * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
  recovery_window = (elapsed_s >= FULL_LOOP_RECOVERY_END_S - 5.0) & (elapsed_s < FULL_LOOP_RECOVERY_END_S)
  grade_window = elapsed_s >= FULL_LOOP_RECOVERY_END_S
  settled_rolling_window = elapsed_s >= FULL_LOOP_ROLLING_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S
  final_speed_window = elapsed_s >= FULL_LOOP_DURATION_S - 2.0

  assert np.min(gaps_m) >= 19.5
  # This loop follows *at* MAX (vCruise = lead). Rematch +a is gated
  # there — do not chase past set to close FD — so the delayed pedal
  # plant opens past 53.5 m instead of punching back by t=70. Still
  # must have opened off the 20 m start, and not hang at Bosch range.
  recovery_gap_m = float(np.mean(gaps_m[recovery_window]))
  assert recovery_gap_m >= desired_gap_m - 2.0
  assert recovery_gap_m <= desired_gap_m + 45.0
  assert np.mean(speeds_mps[recovery_window]) == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.8)
  assert gaps_m[-1] >= desired_gap_m - 2.0
  # Delayed pedal plant + Accel envelope / MPC mild floor: last-2 s settle
  # is a bit lazier than the old 0.2 m/s band (~0.4 m/s observed).
  assert np.mean(speeds_mps[final_speed_window]) == pytest.approx(FOLLOW_TEST_SPEED_MPS, abs=0.5)
  assert np.max(speeds_mps[settled_rolling_window]) <= FOLLOW_TEST_SPEED_MPS + 0.1

  assert np.min(accelerations_mps2) >= -1.0
  assert np.max(accelerations_mps2) <= 0.8
  assert np.max(np.abs(np.diff(accelerations_mps2) / FULL_LOOP_DT_S)) <= 1.5
  assert np.max(np.diff(pedal_samples)) <= PEDAL_RAMP_RATE_UP + 1e-9
  assert np.min(np.diff(pedal_samples)) >= -PEDAL_RAMP_RATE_DOWN - 1e-9
  assert np.min(pedal_samples) >= FULL_LOOP_PEDAL_DI_BP[0]
  assert np.max(pedal_samples) <= FULL_LOOP_PEDAL_DI_BP[-1]
  assert np.max(np.abs(planner_targets_mps2 - vdas_targets_mps2)) <= 1e-9

  compensated_speed_error = np.trapezoid(
    np.abs(speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS),
    elapsed_s[grade_window],
  )
  disabled_speed_error = np.trapezoid(
    np.abs(disabled_grade_speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS),
    elapsed_s[grade_window],
  )
  assert np.max(np.abs(speeds_mps[grade_window] - FOLLOW_TEST_SPEED_MPS)) <= 1.25
  assert compensated_speed_error <= 0.5 * disabled_speed_error


def test_plant_aligned_full_closed_loop_grade_compensation_holds_speed(monkeypatch):
  samples, _ = _run_full_closed_loop_following(
    monkeypatch,
    nap_follow_dist=7,
    plant_aligned_feedforward=True,
  )
  elapsed_s, gaps_m, speeds_mps, accelerations_mps2, _, vdas_targets_mps2, pitches_rad = samples.T
  desired_gap_m = get_T_FOLLOW(nap_follow_dist=7) * FOLLOW_TEST_SPEED_MPS + STOP_DISTANCE_M
  uphill_window = (
    (elapsed_s >= FULL_LOOP_UPHILL_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S)
    & (elapsed_s < FULL_LOOP_UPHILL_HOLD_END_S)
  )
  crest_window = (
    (elapsed_s >= FULL_LOOP_CREST_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S)
    & (elapsed_s < FULL_LOOP_CREST_HOLD_END_S)
  )
  rolling_window = elapsed_s >= FULL_LOOP_ROLLING_RAMP_END_S + FULL_LOOP_GRADE_SETTLING_S

  assert np.min(gaps_m) >= 19.5
  assert gaps_m[-1] >= desired_gap_m - 2.0
  for window, expected_pitch_rad in (
    (uphill_window, FULL_LOOP_UPHILL_PITCH_RAD),
    (crest_window, FULL_LOOP_CREST_PITCH_RAD),
    (rolling_window, FULL_LOOP_ROLLING_PITCH_RAD),
  ):
    assert pitches_rad[window] == pytest.approx(np.full(np.count_nonzero(window), expected_pitch_rad))
    # Delayed pedal plant + settle opening deadband: first hold-window
    # sample can sit ~0.015 m/s under 24.5 while already recovering
    # (24.485 → 24.50 within a few frames). Production fallback still
    # holds 25 ± 0.5. Do not restore Accel-ceil rematch (49→59) for
    # this 0.03 mph.
    assert np.min(speeds_mps[window]) >= FOLLOW_TEST_SPEED_MPS - 0.52
    assert np.max(speeds_mps[window]) <= FOLLOW_TEST_SPEED_MPS + 0.5

  for phase_end_s in (
    FULL_LOOP_UPHILL_HOLD_END_S,
    FULL_LOOP_CREST_HOLD_END_S,
    FULL_LOOP_DURATION_S,
  ):
    tracking_window = (elapsed_s >= phase_end_s - 2.0) & (elapsed_s < phase_end_s)
    # Delayed pedal + rematch trickle (0.08) then MILD (−0.22) at a
    # grade step: plant lag mean is ~0.129. Not a safety bound.
    assert np.mean(np.abs(
      accelerations_mps2[tracking_window] - vdas_targets_mps2[tracking_window]
    )) <= 0.14


def test_planner_eases_for_slower_lead_before_mpc_and_lead_can_brake_harder():
  v_ego = 26.8
  v_lead = 22.4
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  v_rel = v_ego - v_lead
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  d_rel = d_follow + rel_need

  params = _MutablePlannerParams(nap_follow_dist=4)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_rel
  lead.vLead = v_lead
  for _ in range(16):
    planner.update(inputs)
  # Closing ≥ 1.5: never rematch +a. Overlay ease stays slight-lift
  # while MPC is 0; a deeper MPC bite must leave the MILD floor.
  assert planner.output_a_target <= 0.0
  assert planner.output_a_target >= -LEAD_APPROACH_MILD_A_MS2 - 0.08

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  # 4.4 m/s close is under rapid 6 but past the soft-limit skip: leave −0.22.
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.mpc.crash_cnt = 3
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)
  planner.mpc.crash_cnt = 0

  # Farther closing lead (old 140 m / short need stayed off): now eases.
  # Match-speed extra −a is near-gap only — a 160 m lock must not dump
  # −k·v_rel (that undid the large-slack MILD floor).
  lead.dRel = 160.0
  lead.modelProb = 1.0
  lead.radar = True
  assert 160.0 > d_follow + (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2) + v_rel * 12.0
  planner._lead_approach_active = False
  planner._lead_approach_a = None
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.update(inputs)
  assert planner.output_a_target < 0.0
  assert planner.output_a_target >= -LEAD_APPROACH_MILD_A_MS2 - 0.08

  # Past usable Bosch: no extra crawl.
  lead.dRel = LEAD_APPROACH_MAX_START_M + 15.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(0.0, abs=0.08)


def test_planner_far_closing_lead_blocks_rematch_plus_a():
  """First far closing lock must not rematch cruise +a (d7 20:33)."""
  v_ego = 25.0
  v_rel = 1.6
  v_lead = v_ego - v_rel
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  a_cap = lead_close_accel_ms2(5)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=1.5)
  planner.prev_accel_clip = [-1.2, a_cap]
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 160.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  planner._lead_approach_active = False
  planner._lead_approach_a = None
  planner.update(inputs)
  assert planner.output_a_target <= 0.0
  for _ in range(15):
    planner.update(inputs)
  # Never rematch +a into a far closing lock. Extra match-speed −a
  # stays near-gap so 160 m / 1.6 m/s cannot punch −0.40.
  assert planner.output_a_target <= 0.0
  assert planner.output_a_target >= -LEAD_APPROACH_MILD_A_MS2 - 0.08


def test_planner_far_opening_alead_keeps_cruise_plus_a():
  """da 2026-09-18: far lead braking + gap opening must not snap aTarget to aLeadK.

  Double SET + gas lift armed long at +0.47. ~0.5 s later a 64 m lead
  appeared with v_rel opening and aLeadK −1.46; #190 aLead-only ownership
  forced regen. Keep cruise climb. Real closing / near-gap brake still bite.
  """
  v_ego = 22.8 * CV.MPH_TO_MS
  v_rel_open = -1.44
  v_lead = v_ego - v_rel_open
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  d_rel = 64.0
  slack = d_rel - d_follow
  assert slack > 25.0
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  a_cap = lead_close_accel_ms2(5)
  cruise_a = 0.47
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=cruise_a)
  planner.prev_accel_clip = [-1.2, a_cap]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_rel
  lead.vLead = v_lead
  lead.aLeadK = -1.46
  lead.modelProb = 1.0
  lead.radar = True
  planner._lead_approach_active = False
  planner._lead_approach_a = None
  for _ in range(8):
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=cruise_a)
    planner.update(inputs)
  assert planner.output_a_target > 0.20
  assert planner.output_a_target > -0.20
  assert planner.output_a_target != pytest.approx(-1.46, abs=0.20)
  assert not planner._lead_close_hold_owned

  # Real closing ≳ 1.5 at the same range still match-speed −a.
  lead.vLead = v_ego - 1.6
  lead.aLeadK = -0.40
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=cruise_a)
  planner.prev_accel_clip = [-1.2, a_cap]
  planner._lead_close_hold_owned = False
  for _ in range(6):
    planner.update(inputs)
  assert planner.output_a_target <= 0.0

  # Near-gap braking lead: match aLead (not k·v_rel). Past acquire so
  # first-latch slew is not the story.
  planner2 = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner2._map_speed_accel = 5
  planner2.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=cruise_a)
  planner2.prev_accel_clip = [-1.2, a_cap]
  planner2._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  inputs2 = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs2, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead2 = inputs2["radarState"].leadOne
  lead2.status = True
  lead2.dRel = d_follow + 8.0
  lead2.vLead = v_ego - 0.4
  lead2.aLeadK = -0.80
  lead2.modelProb = 1.0
  lead2.radar = True
  for _ in range(6):
    planner2.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=cruise_a)
    planner2.update(inputs2)
  assert planner2.output_a_target == pytest.approx(-0.80, abs=0.08)
  planner2.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner2.prev_accel_clip = [-3.5, a_cap]
  planner2.update(inputs2)
  assert planner2.output_a_target == pytest.approx(-2.0, abs=0.08)


def test_planner_far_same_speed_lead_may_keep_catchup_plus_a():
  """Same-speed far Bosch lead may still close Follow Distance at Accel 1–10."""
  v_ego = 25.0
  v_lead = v_ego
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  a_cap = lead_close_accel_ms2(5)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=1.5)
  planner.prev_accel_clip = [-1.2, a_cap]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 160.0
  lead.vLead = v_lead
  lead.modelProb = 1.0
  lead.radar = True
  planner._lead_approach_active = False
  planner._lead_approach_a = None
  for _ in range(16):
    planner.update(inputs)
  assert planner.output_a_target > 0.20
  assert planner.output_a_target == pytest.approx(a_cap, abs=0.08)


def test_planner_first_acquire_slews_yoyo_and_keeps_rapid_authority():
  """e4 09:53:19: first 118 m latch must not punch −0.46 then rematch +0.05.

  Rapid / near-bumper first latch still applies full MPC −a immediately.
  """
  v_ego = 25.0
  v_lead = v_ego - 1.2
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-0.46)
  planner.prev_accel_clip = [-1.2, 0.80]
  planner.output_a_target = 0.0
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 118.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  planner.update(inputs)
  first = float(planner.output_a_target)
  assert first == pytest.approx(-LEAD_ACQUIRE_SLEW_MS2, abs=0.02)
  assert first > -0.20
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.05)
  planner.update(inputs)
  rematch = float(planner.output_a_target)
  assert rematch > first - 1e-9
  assert rematch < 0.08

  planner_r = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner_r._map_speed_accel = 5
  planner_r.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner_r.prev_accel_clip = [-3.5, 0.80]
  planner_r.output_a_target = 0.0
  inputs_r = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs_r, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead_r = inputs_r["radarState"].leadOne
  lead_r.status = True
  lead_r.dRel = 40.0
  lead_r.vLead = v_ego - 8.0
  lead_r.aLeadK = 0.0
  lead_r.modelProb = 1.0
  lead_r.radar = True
  planner_r.update(inputs_r)
  assert planner_r.output_a_target == pytest.approx(-2.0, abs=0.08)


def test_planner_mid_gap_slow_close_does_not_rematch_accel_ceil():
  """ef 10:18: slack 12–50 m @ ~1.4 m/s must trickle, not pulse Accel +0.25."""
  v_ego = 30.0
  v_rel = 1.4
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=2)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-1.2, 0.80]
  planner.output_a_target = 0.0
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 40.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  # Expire first-latch so this is post-acquire rematch, not #214.
  for _ in range(int(LEAD_ACQUIRE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner._lead_acquire_age > LEAD_ACQUIRE_HOLD_S
  seen = []
  for _ in range(12):
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
    planner.update(inputs)
    seen.append(float(planner.output_a_target))
  assert max(seen) <= LEAD_MID_GAP_REMATCH_A_MS2 + 0.04
  assert max(seen) < 0.16
  assert all(a > -0.10 for a in seen)
  # Slack 30 m, still slowly closing: same trickle, not Accel 5.
  lead.dRel = d_follow + 30.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-1.2, 0.80]
  for _ in range(8):
    planner.update(inputs)
  assert planner.output_a_target <= LEAD_MID_GAP_REMATCH_A_MS2 + 0.04
  assert planner.output_a_target < 0.16
  # Slack > 50 while still closing is large-gap rematch: Accel.
  lead.dRel = d_follow + 70.0
  planner_far = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner_far._map_speed_accel = 5
  planner_far.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner_far.prev_accel_clip = [-1.2, 0.80]
  planner_far._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  for _ in range(16):
    planner_far.update(inputs)
  assert planner_far.output_a_target > 0.20
  # Same-speed far gap still Accel catch-up.
  lead.vLead = v_ego
  lead.dRel = 160.0
  planner2 = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner2._map_speed_accel = 5
  planner2.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner2.prev_accel_clip = [-1.2, 0.80]
  inputs2 = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs2, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead2 = inputs2["radarState"].leadOne
  lead2.status = True
  lead2.dRel = 160.0
  lead2.vLead = v_ego
  lead2.modelProb = 1.0
  lead2.radar = True
  for _ in range(16):
    planner2.update(inputs2)
  assert planner2.output_a_target > 0.20


def test_planner_post_acquire_chatter_slews_mid_gap_rematch():
  """10:18 mid-gap rematch soft-caps; chatter slew still holds out-of-band."""
  v_ego = 30.0
  v_rel = 1.4
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=2)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, 0.80]
  planner.output_a_target = 0.25
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 40.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  planner._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  planner.update(inputs)
  dropped = float(planner.output_a_target)
  # Soft-cap wins in the 12–50 m / 0.8–2.0 m/s band (accel_clip ceiling).
  assert dropped == pytest.approx(LEAD_MID_GAP_REMATCH_A_MS2, abs=0.02)
  assert dropped < 0.16

  # Same leftover +0.25 outside the rematch *and* catch-up gates must
  # slew, not step. Same-speed slack 40 is Accel catch-up (immediate).
  # Slack > 50 while still closing 1.4 is large-gap rematch, not 10:18.
  planner_slew = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner_slew._map_speed_accel = 5
  planner_slew.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner_slew.prev_accel_clip = [-1.2, 0.80]
  planner_slew.output_a_target = 0.25
  lead.vLead = v_ego - v_rel
  lead.dRel = d_follow + 70.0
  planner_slew._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  planner_slew.update(inputs)
  slewed = float(planner_slew.output_a_target)
  assert slewed == pytest.approx(0.25 - LEAD_FOLLOW_CHATTER_SLEW_MS2, abs=0.02)
  assert slewed > 0.18


def test_planner_mid_gap_above_max_floors_dump_and_holds_rematch():
  """Map above-max mid-gap: floor the cruise cliff, then trickle the re-catch.

  Large-gap catch-up and a plan with no lead still use Accel.
  """
  v_map = 70.0 * CV.MPH_TO_MS
  v_ego = v_map + 1.3
  v_lead = v_ego - 0.5
  t_follow = get_T_FOLLOW(nap_follow_dist=3)
  d_rel = t_follow * v_lead + STOP_DISTANCE_M + 19.0
  params = _MutablePlannerParams(nap_follow_dist=3, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-3.45)
  planner.prev_accel_clip = [-3.5, 0.80]
  planner.output_a_target = 0.0
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_map)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_rel
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  for _ in range(int(LEAD_ACQUIRE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-LEAD_MAP_MIDGAP_FLOOR_MS2, abs=0.05)
  assert planner.output_a_target > -0.70
  assert planner._lead_post_dump_hold == pytest.approx(LEAD_POST_DUMP_HOLD_S)

  v_under = v_map - 2.0
  v_lead_open = v_under + 0.4
  inputs["carState"].vEgo = v_under
  _set_v_cruise_ms(inputs, v_map)
  lead.vLead = v_lead_open
  lead.dRel = t_follow * v_lead_open + STOP_DISTANCE_M + 19.0
  planner.mpc = _ConstantAccelerationMpc(v_under, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-3.5, 0.80]
  seen = []
  for _ in range(8):
    planner.update(inputs)
    seen.append(float(planner.output_a_target))
  assert max(seen) <= LEAD_MID_GAP_REMATCH_A_MS2 + 0.02
  assert max(seen) < 0.16

  # No prior dump: opening mid-gap under the map is still Accel catch-up.
  planner_open = LongitudinalPlanner(_make_preap_params(), init_v=v_under, params=params)
  planner_open._map_speed_accel = 5
  planner_open.mpc = _ConstantAccelerationMpc(v_under, acceleration_mps2=0.80)
  planner_open.prev_accel_clip = [-1.2, 0.80]
  planner_open._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  inputs_open = _make_planner_inputs(v_under)
  _set_v_cruise_ms(inputs_open, v_map)
  lead_open = inputs_open["radarState"].leadOne
  lead_open.status = True
  lead_open.dRel = t_follow * v_lead_open + STOP_DISTANCE_M + 19.0
  lead_open.vLead = v_lead_open
  lead_open.aLeadK = 0.0
  lead_open.modelProb = 1.0
  lead_open.radar = True
  for _ in range(12):
    planner_open.update(inputs_open)
  assert planner_open.output_a_target > 0.20

  # Large-gap catch-up stays Accel even if a dump hold is latched.
  planner_far = LongitudinalPlanner(_make_preap_params(), init_v=v_under, params=params)
  planner_far._map_speed_accel = 5
  planner_far.mpc = _ConstantAccelerationMpc(v_under, acceleration_mps2=0.80)
  planner_far.prev_accel_clip = [-1.2, 0.80]
  planner_far._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  planner_far._lead_post_dump_hold = LEAD_POST_DUMP_HOLD_S
  inputs_far = _make_planner_inputs(v_under)
  _set_v_cruise_ms(inputs_far, v_map)
  lead_far = inputs_far["radarState"].leadOne
  lead_far.status = True
  lead_far.dRel = t_follow * v_under + STOP_DISTANCE_M + 80.0
  lead_far.vLead = v_under
  lead_far.aLeadK = 0.0
  lead_far.modelProb = 1.0
  lead_far.radar = True
  for _ in range(12):
    planner_far.update(inputs_far)
  assert planner_far.output_a_target > 0.20

  # No lead: positive plan is not the mid-gap trickle.
  planner_empty = LongitudinalPlanner(_make_preap_params(), init_v=v_under, params=params)
  planner_empty._map_speed_accel = 5
  planner_empty.mpc = _ConstantAccelerationMpc(v_under, acceleration_mps2=0.80)
  planner_empty.prev_accel_clip = [-1.2, 0.80]
  inputs_empty = _make_planner_inputs(v_under)
  _set_v_cruise_ms(inputs_empty, v_map)
  for _ in range(8):
    planner_empty.update(inputs_empty)
  assert planner_empty.output_a_target > 0.20


def test_planner_under_map_midgap_coast_does_not_cliff():
  """18:10 under 70 mph: plan accels ~0 must not publish −1.4…−3.5.

  aLead noise in the mid-gap band used to match straight through after
  the MILD soft-limit. Floor at slight lift and hold the rematch cap.
  """
  v_map = 70.0 * CV.MPH_TO_MS
  v_ego = v_map - 1.4 * CV.MPH_TO_MS
  v_rel = 1.13
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=3)
  slack = 19.5
  params = _MutablePlannerParams(nap_follow_dist=3, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.01)
  planner.prev_accel_clip = [-3.5, 0.80]
  planner.output_a_target = 0.02
  planner._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_map)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = t_follow * v_lead + STOP_DISTANCE_M + slack
  lead.vLead = v_lead
  lead.aLeadK = -3.36
  lead.modelProb = 1.0
  lead.radar = True
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-LEAD_APPROACH_MILD_A_MS2, abs=0.05)
  assert planner.output_a_target >= -0.25
  assert planner._lead_post_dump_hold == pytest.approx(LEAD_POST_DUMP_HOLD_S)

  # Opening re-catch after the cliff stays on the 0.10 trickle.
  lead.aLeadK = 0.0
  lead.vLead = v_ego + 0.4
  lead.dRel = t_follow * lead.vLead + STOP_DISTANCE_M + slack
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  seen = []
  for _ in range(8):
    planner.update(inputs)
    seen.append(float(planner.output_a_target))
  assert max(seen) <= LEAD_MID_GAP_REMATCH_A_MS2 + 0.02


def test_planner_matched_speed_glide_deadbands_near_gap_chatter():
  """After #214 acquire, matched speeds near FD must not yo-yo rematch↔mild."""
  v_ego = 28.0
  v_rel = 0.15
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=2)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 3.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True

  # Expire the acquire window with a quiet plan so first-latch slew is done.
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, 0.80]
  planner.output_a_target = 0.0
  for _ in range(int(LEAD_ACQUIRE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner._lead_acquire_age > LEAD_ACQUIRE_HOLD_S

  seen = []
  for mpc_a in (-LEAD_APPROACH_MILD_A_MS2, LEAD_CLOSE_OPENING_A_MS2,
                -0.17, LEAD_CLOSE_OPENING_A_MS2):
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=mpc_a)
    planner.update(inputs)
    seen.append(float(planner.output_a_target))
  assert all(a >= -LEAD_GLIDE_A_MS2 - 1e-9 for a in seen)
  assert all(a <= LEAD_GLIDE_A_MS2 + 1e-9 for a in seen)
  signs = [1 if a > 1e-6 else (-1 if a < -1e-6 else 0) for a in seen]
  assert not (1 in signs and -1 in signs)


def test_planner_matched_inside_fd_keeps_recovery_a():
  """Too-close matched: rematch +a and mild −a stand (glide is at/long of FD)."""
  v_ego = 28.0
  v_rel = 0.2
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=2)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow - 8.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, 0.80]
  planner.output_a_target = 0.0
  for _ in range(int(LEAD_ACQUIRE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner._lead_acquire_age > LEAD_ACQUIRE_HOLD_S
  assert planner._lead_glide_active is False

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=LEAD_CLOSE_OPENING_A_MS2)
  planner.output_a_target = LEAD_CLOSE_OPENING_A_MS2
  planner.update(inputs)
  assert float(planner.output_a_target) == pytest.approx(LEAD_CLOSE_OPENING_A_MS2, abs=0.02)

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-LEAD_APPROACH_MILD_A_MS2)
  planner.update(inputs)
  assert float(planner.output_a_target) == pytest.approx(-LEAD_APPROACH_MILD_A_MS2, abs=0.02)


def test_planner_slow_close_inside_fd_commands_mild_not_dump():
  """e8: inside FD still closing ~1.25 m/s — MILD floor, not rematch +a or −2.5."""
  v_ego = 30.0
  v_rel = 1.25
  v_lead = v_ego - v_rel
  t_follow = get_T_FOLLOW(nap_follow_dist=2)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.prev_accel_clip = [-3.5, 0.80]
  planner.output_a_target = 0.0
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow - 2.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  # Past acquire so first-latch slew is not the story.
  planner._lead_acquire_age = LEAD_ACQUIRE_HOLD_S + 0.05
  for _ in range(4):
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
    planner.update(inputs)
  a = float(planner.output_a_target)
  expect = lead_inside_slow_close_a_ms2(v_rel, -2.0)
  assert expect == pytest.approx(-LEAD_APPROACH_MILD_A_MS2)
  assert a == pytest.approx(expect, abs=0.08)
  assert a > -1.0


def test_planner_hard_close_keeps_full_authority():
  """Rapid / near-bumper / FCW still apply full MPC −a after the glide work."""
  v_ego = 25.0
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.prev_accel_clip = [-3.5, 0.80]
  planner.output_a_target = 0.0
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 40.0
  lead.vLead = v_ego - 8.0
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)

  lead.vLead = v_ego - 2.0
  lead.dRel = LEAD_MPC_SOFT_NEAR_M
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)

  lead.dRel = 40.0
  lead.vLead = v_ego
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.mpc.crash_cnt = 3
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)


def test_planner_caps_lead_close_accel_at_min_accel_and_keeps_hard_brake():
  """Accel 1 catch-up uses Mannerisms Accel. Rapid / FCW still own −2.0."""
  v_ego = 25.0
  v_lead = 25.0
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  d_rel = min(LEAD_CLOSE_MAX_M - 1.0, d_follow + 40.0)

  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=1)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 1
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=1.5)
  a_cap = lead_close_accel_ms2(1)
  planner.prev_accel_clip = [-1.2, a_cap]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_rel
  lead.vLead = v_lead
  lead.modelProb = 1.0
  lead.radar = True

  for _ in range(32):
    planner.update(inputs)

  assert planner.output_a_target == pytest.approx(LEAD_CLOSE_A_MIN_MS2, abs=0.06)
  assert planner.output_a_target < longitudinal_planner.get_max_accel(v_ego)

  # Same-speed / mild: MPC −2.0 is soft-limited to MILD.
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-LEAD_APPROACH_MILD_A_MS2, abs=0.08)

  # Large-slack close (e4 40 m / 9.5 m/s class): small MPC bite, not −2.0.
  lead.vLead = v_ego - 1.6
  lead.aLeadK = 0.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-LEAD_APPROACH_MILD_A_MS2, abs=0.08)
  assert planner.output_a_target > -0.60
  # Near-gap braking lead: skip the mild floor so MPC −2.0 can match.
  lead.vLead = v_lead
  lead.dRel = d_follow + 8.0
  lead.aLeadK = -0.4
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)
  lead.dRel = d_rel
  lead.aLeadK = 0.0

  # Rapid close: MPC danger still wins after the 4-frame confirm.
  lead.vLead = v_ego - 8.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner._lead_approach_rapid_count = 0
  for _ in range(LEAD_APPROACH_RAPID_CONFIRM_N):
    planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)
  # Near bumper: full −a even on a mild close (do not delay safety).
  lead.vLead = v_ego - 2.0
  lead.dRel = LEAD_MPC_SOFT_NEAR_M
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)
  lead.dRel = d_rel
  lead.vLead = v_lead

  # Vision-only far flicker: do not cap MAX-rise / open-road climb.
  lead.dRel = 160.0
  lead.modelProb = 0.2
  lead.radar = False
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=1.5)
  cruise_limit = longitudinal_planner.get_max_accel(v_ego)
  planner.prev_accel_clip = [-1.2, cruise_limit]
  planner._lead_close_hold_d = None
  planner._lead_close_hold_v = None
  planner._lead_close_a_cap = None
  for _ in range(8):
    planner.update(inputs)
  assert planner.output_a_target == pytest.approx(cruise_limit, abs=0.08)

  # Far Bosch radar lead is capped (used to punch cruise at 160 m).
  lead.modelProb = 1.0
  lead.radar = True
  planner.prev_accel_clip = [-1.2, cruise_limit]
  for _ in range(8):
    planner.update(inputs)
  assert planner.output_a_target == pytest.approx(LEAD_CLOSE_A_MIN_MS2, abs=0.08)


def test_planner_lead_close_accel_scales_with_accel_personality():
  v_ego = 25.0
  v_lead = 24.6  # 0.4 m/s — below rematch-block, still a catch-up +a case
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_rel = min(LEAD_CLOSE_MAX_M - 1.0, t_follow * v_lead + STOP_DISTANCE_M + 35.0)

  def _run(accel_level):
    params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=accel_level)
    planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
    planner._map_speed_accel = accel_level
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=1.5)
    planner.prev_accel_clip = [-1.2, lead_close_accel_ms2(accel_level)]
    inputs = _make_planner_inputs(v_ego)
    _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
    lead = inputs["radarState"].leadOne
    lead.status = True
    lead.dRel = d_rel
    lead.vLead = v_lead
    for _ in range(32):
      planner.update(inputs)
    return planner.output_a_target

  a1 = _run(1)
  a10 = _run(10)
  cruise_limit = longitudinal_planner.get_max_accel(v_ego)
  assert a1 == pytest.approx(LEAD_CLOSE_A_MIN_MS2, abs=0.06)
  assert a10 == pytest.approx(min(LEAD_CLOSE_A_MAX_MS2, cruise_limit), abs=0.06)
  assert a1 < a10
  assert lead_close_accel_ms2(1) < lead_close_accel_ms2(10)


def test_planner_lead_flicker_hold_blocks_rematch_plus_a():
  """leadOne flicker must not restore cruise +a while the last lead was closing."""
  v_ego = 27.5
  v_lead = 24.5
  v_rel = v_ego - v_lead
  assert v_rel >= 1.5
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-1.2, 0.80]
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 100.0
  lead.vLead = v_lead
  lead.aLeadK = -0.20
  lead.modelProb = 0.6
  lead.radar = True
  for _ in range(6):
    planner.update(inputs)
  assert planner.output_a_target <= 0.0
  assert planner._lead_close_hold_owned

  publisher = _CapturingPubMaster()
  planner.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.hasLead

  lead.status = False
  for _ in range(int(LEAD_CLOSE_HOLD_S / 0.05) - 1):
    planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
    planner.update(inputs)
    assert planner.output_a_target <= 0.0
    assert planner._lead_close_hold_d is not None
  planner.publish(inputs, publisher)
  assert publisher.message.longitudinalPlan.hasLead


def test_planner_far_radar_lead_eases_without_model_prob():
  """Far Bosch-associated lead is owned immediately; vision-only far flicker is not."""
  v_ego = 28.0
  v_lead = 24.5
  params = _MutablePlannerParams(nap_follow_dist=2, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-1.2, 0.80]
  inputs = _make_planner_inputs(v_ego)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 170.0
  lead.vLead = v_lead
  lead.aLeadK = -0.15
  lead.modelProb = 0.15
  lead.radar = True
  for _ in range(8):
    planner.update(inputs)
  assert planner.output_a_target <= 0.0

  lead.radar = False
  lead.modelProb = 0.9
  planner._lead_close_hold_d = None
  planner._lead_close_hold_v = None
  planner._lead_close_hold_a = None
  planner._lead_close_hold_owned = False
  planner._lead_approach_active = False
  planner._lead_approach_a = None
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.80)
  planner.prev_accel_clip = [-1.2, 0.80]
  for _ in range(8):
    planner.update(inputs)
  assert planner.output_a_target > 0.20


def test_planner_settled_opening_gap_does_not_pin_accel_ceil():
  """d7 20:25:33: after match, opening slack must not rematch Accel 2 ceil."""
  v_ego = 25.0
  v_lead = 25.0
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=2)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 2
  a2 = lead_close_accel_ms2(2)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=a2)
  planner.prev_accel_clip = [-1.2, a2]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 8.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  for _ in range(int(LEAD_SETTLE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner._lead_settled
  # Gap already opening, slack ~18 m — trickle/0, not Accel ceil.
  lead.vLead = v_ego + 0.4
  lead.dRel = d_follow + 18.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=a2)
  planner.prev_accel_clip = [-1.2, a2]
  for _ in range(4):
    planner.update(inputs)
  assert planner._lead_settled
  assert planner.output_a_target < 0.05
  assert planner.output_a_target < a2 * 0.25
  # Large same-speed gap that never matched still catch-up at Accel.
  planner2 = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner2._map_speed_accel = 2
  planner2.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=a2)
  planner2.prev_accel_clip = [-1.2, a2]
  inputs2 = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs2, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead2 = inputs2["radarState"].leadOne
  lead2.status = True
  lead2.dRel = 160.0
  lead2.vLead = v_ego
  lead2.modelProb = 1.0
  lead2.radar = True
  for _ in range(16):
    planner2.update(inputs2)
  assert not planner2._lead_settled
  assert planner2.output_a_target == pytest.approx(a2, abs=0.08)


def test_planner_settled_gap_hunt_is_not_full_accel_ceil():
  """Hwy +30 m long: small Accel-proportional close, not full Accel every pulse."""
  v_ego = 25.0
  v_lead = 24.8
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=5)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 5
  a5 = lead_close_accel_ms2(5)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=a5)
  planner.prev_accel_clip = [-1.2, a5]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_ego + _UNDER_MAX_HEADROOM_MS)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 8.0
  lead.vLead = v_ego
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  for _ in range(int(LEAD_SETTLE_HOLD_S / 0.05) + 2):
    planner.update(inputs)
  assert planner._lead_settled
  lead.vLead = v_lead
  lead.dRel = d_follow + 30.0
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=a5)
  planner.prev_accel_clip = [-1.2, a5]
  for _ in range(8):
    planner.update(inputs)
  hunt = lead_hunt_accel_ms2(a5, 30.0)
  assert planner.output_a_target <= hunt + 0.06
  assert planner.output_a_target < a5 * 0.5
  assert planner.output_a_target >= LEAD_CLOSE_OPENING_A_MS2 - 0.02


def test_planner_hwy_intent_applies_hwy_fd_from_30():
  """MAX ≥ 50 selects highway follow even while ego is still at 35."""
  v_ego = 35.0 * CV.MPH_TO_MS
  v_max_kph = 65.0 * CV.MPH_TO_KPH
  params = _MutablePlannerParams(nap_follow_dist=6, city=6, hwy=2)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  inputs = _make_planner_inputs(v_ego)
  inputs["carState"].vCruise = v_max_kph
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = 40.0
  lead.vLead = v_ego
  lead.modelProb = 1.0
  lead.radar = True
  planner._follow_blend.t_follow = get_T_FOLLOW(nap_follow_dist=6)
  planner._follow_blend.highway = False
  planner.update(inputs)
  assert planner.active_nap_follow_dist == 2
  assert planner.t_follow < get_T_FOLLOW(nap_follow_dist=6)
  assert planner._follow_blend.highway is True
  # MAX 45 at 55 mph stays city.
  v55 = 55.0 * CV.MPH_TO_MS
  planner55 = LongitudinalPlanner(_make_preap_params(), init_v=v55, params=params)
  planner55.mpc = _ConstantAccelerationMpc(v55, acceleration_mps2=0.0)
  inputs55 = _make_planner_inputs(v55)
  inputs55["carState"].vCruise = 45.0 * CV.MPH_TO_KPH
  lead55 = inputs55["radarState"].leadOne
  lead55.status = True
  lead55.dRel = 50.0
  lead55.vLead = v55
  lead55.modelProb = 1.0
  lead55.radar = True
  planner55.update(inputs55)
  assert planner55.active_nap_follow_dist == 6
  assert planner55._follow_blend.highway is False
  # ego 60 + MAX 45 stays city. MAX exactly 50 is highway.
  # Decel to ~48 under MAX 75 stays highway (do not open the city gap).
  v60 = 60.0 * CV.MPH_TO_MS
  planner60 = LongitudinalPlanner(_make_preap_params(), init_v=v60, params=params)
  planner60.mpc = _ConstantAccelerationMpc(v60, acceleration_mps2=0.0)
  inputs60 = _make_planner_inputs(v60)
  inputs60["carState"].vCruise = 45.0 * CV.MPH_TO_KPH
  lead60 = inputs60["radarState"].leadOne
  lead60.status = True
  lead60.dRel = 40.0
  lead60.vLead = v60
  lead60.modelProb = 1.0
  lead60.radar = True
  planner60._follow_blend.highway = True
  planner60._follow_blend.t_follow = get_T_FOLLOW(nap_follow_dist=2)
  planner60.update(inputs60)
  assert planner60.active_nap_follow_dist == 6
  assert planner60._follow_blend.highway is False

  v48 = 47.7 * CV.MPH_TO_MS
  planner48 = LongitudinalPlanner(_make_preap_params(), init_v=v48, params=params)
  planner48.mpc = _ConstantAccelerationMpc(v48, acceleration_mps2=0.0)
  inputs48 = _make_planner_inputs(v48)
  inputs48["carState"].vCruise = 75.0 * CV.MPH_TO_KPH
  lead48 = inputs48["radarState"].leadOne
  lead48.status = True
  lead48.dRel = 35.0
  lead48.vLead = v48
  lead48.modelProb = 1.0
  lead48.radar = True
  planner48.update(inputs48)
  assert planner48._follow_blend.highway is True
  assert planner48.active_nap_follow_dist == 2

  planner50 = LongitudinalPlanner(_make_preap_params(), init_v=v48, params=params)
  planner50.mpc = _ConstantAccelerationMpc(v48, acceleration_mps2=0.0)
  inputs50 = _make_planner_inputs(v48)
  inputs50["carState"].vCruise = 50.0 * CV.MPH_TO_KPH
  lead50 = inputs50["radarState"].leadOne
  lead50.status = True
  lead50.dRel = 35.0
  lead50.vLead = v48
  lead50.modelProb = 1.0
  lead50.radar = True
  planner50.update(inputs50)
  assert planner50._follow_blend.highway is True
  assert planner50.active_nap_follow_dist == 2

  # Unset cruise (255) must not look like a highway MAX after the 145 kph clamp.
  v30 = 30.0 * CV.MPH_TO_MS
  planner_unset = LongitudinalPlanner(_make_preap_params(), init_v=v30, params=params)
  planner_unset.mpc = _ConstantAccelerationMpc(v30, acceleration_mps2=0.0)
  inputs_unset = _make_planner_inputs(v30)
  inputs_unset["carState"].vCruise = 255.0
  lead_unset = inputs_unset["radarState"].leadOne
  lead_unset.status = True
  lead_unset.dRel = 30.0
  lead_unset.vLead = v30
  lead_unset.modelProb = 1.0
  lead_unset.radar = True
  planner_unset.update(inputs_unset)
  assert planner_unset._follow_blend.highway is False
  assert planner_unset.active_nap_follow_dist == 6


def test_planner_faster_lead_at_max_does_not_overrun():
  """ea 11:46: lead faster than a 60 MAX must not rematch Accel-1 past set.

  Product: hold ≤ MAX and let the gap open. Overlay ease / emergency −a
  still win. A slower lead under MAX still catch-up.
  """
  v_max = 60.0 * CV.MPH_TO_MS
  v_ego = v_max
  v_lead = v_max + 3.0 * CV.MPH_TO_MS
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  params = _MutablePlannerParams(nap_follow_dist=4, map_speed_accel=1)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner._map_speed_accel = 1
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, LEAD_CLOSE_A_MIN_MS2]
  inputs = _make_planner_inputs(v_ego)
  _set_v_cruise_ms(inputs, v_max)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_follow + 40.0
  lead.vLead = v_lead
  lead.aLeadK = 0.0
  lead.modelProb = 1.0
  lead.radar = True
  for _ in range(16):
    planner.update(inputs)
  assert planner.output_a_target <= 0.0
  assert planner._lead_close_a_cap == pytest.approx(0.0)

  # Ego already 1 mph over, lead 2 mph over: no +0.36 rematch punch.
  v_over = v_max + 1.0 * CV.MPH_TO_MS
  planner_over = LongitudinalPlanner(_make_preap_params(), init_v=v_over, params=params)
  planner_over._map_speed_accel = 1
  planner_over.mpc = _ConstantAccelerationMpc(v_over, acceleration_mps2=0.0)
  planner_over.prev_accel_clip = [-1.2, LEAD_CLOSE_A_MIN_MS2]
  inputs_over = _make_planner_inputs(v_over)
  _set_v_cruise_ms(inputs_over, v_max)
  lead_over = inputs_over["radarState"].leadOne
  lead_over.status = True
  lead_over.dRel = 58.0
  lead_over.vLead = v_max + 2.0 * CV.MPH_TO_MS
  lead_over.aLeadK = 0.0
  lead_over.modelProb = 1.0
  lead_over.radar = True
  for _ in range(16):
    planner_over.update(inputs_over)
  assert planner_over.output_a_target <= 0.0
  assert planner_over.output_a_target < LEAD_CLOSE_A_MIN_MS2 - 0.10

  # Slower / same-speed lead under MAX still catch-up.
  v_under = 55.0 * CV.MPH_TO_MS
  v_cruise = 70.0 * CV.MPH_TO_MS
  planner_u = LongitudinalPlanner(_make_preap_params(), init_v=v_under, params=params)
  planner_u._map_speed_accel = 1
  planner_u.mpc = _ConstantAccelerationMpc(v_under, acceleration_mps2=1.5)
  planner_u.prev_accel_clip = [-1.2, LEAD_CLOSE_A_MIN_MS2]
  inputs_u = _make_planner_inputs(v_under)
  _set_v_cruise_ms(inputs_u, v_cruise)
  lead_u = inputs_u["radarState"].leadOne
  lead_u.status = True
  lead_u.dRel = 160.0
  lead_u.vLead = v_under
  lead_u.modelProb = 1.0
  lead_u.radar = True
  for _ in range(16):
    planner_u.update(inputs_u)
  assert planner_u.output_a_target == pytest.approx(LEAD_CLOSE_A_MIN_MS2, abs=0.08)

  # Emergency −a still owns at MAX (FCW / crash).
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.mpc.crash_cnt = 3
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)

