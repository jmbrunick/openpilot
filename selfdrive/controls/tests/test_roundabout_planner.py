"""Planner: RB funnel eases aTarget; a sharp corner / no flag does not."""
from types import SimpleNamespace

import numpy as np

from cereal import car, log, messaging
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.tests.test_tesla_preap_following import (
  _ConstantAccelerationMpc,
  _PlannerInputs,
)
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL, MODE_FOLLOW
from openpilot.selfdrive.mapd.roundabout import RB_V_MAX_MS, live_map_roundabout_hint, roundabout_ease_v_ms
from openpilot.selfdrive.modeld.constants import ModelConstants


class _RbParams:
  def get(self, key, return_default=False):
    if key == "NAPFollowDistance":
      return 4
    if key == "NAPFollowDistanceCity":
      return 4
    if key == "NAPFollowDistanceHwy":
      return 4
    if key == "NAPMapSpeedMode":
      return MODE_FOLLOW
    if key == "NAPMapSpeedOffsetMph":
      return 0
    if key == "NAPMapSpeedLookahead":
      return LOOKAHEAD_NORMAL
    if key == "NAPMapSpeedAccel":
      return 5
    raise AssertionError(key)

  def get_bool(self, key):
    if key == "NAPAdaptiveAccel":
      return False
    if key == "NAPHypermile":
      return False
    if key == "NAPHypermileHillClimb":
      return False
    if key == "NAPFollowDistanceSplitMigrated":
      return True
    raise AssertionError(key)

  def put(self, key, value):
    pass

  def put_bool(self, key, value):
    pass


def _preap_cp():
  params = car.CarParams.new_message()
  params.brand = "tesla"
  params.carFingerprint = "TESLA_MODEL_S_PREAP"
  params.openpilotLongitudinalControl = True
  params.pcmCruise = False
  params.steerRatio = 15.75
  params.wheelbase = 2.959
  return params


def _inputs(v_ego, v_cruise, *, approaching=False, on_rb=False, dist_m=90.0, rb_mph=20.0):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2
  md = messaging.new_message("liveMapDataNAP").liveMapDataNAP

  controls.longControlState = LongCtrlState.pid
  selfdrive.personality = log.LongitudinalPersonality.standard
  car_state.vEgo = v_ego
  car_state.vCruise = v_cruise * CV.MS_TO_KPH
  car_control.orientationNED = [0.0, 0.0, 0.0]
  model.position.x = (v_ego * np.array(ModelConstants.T_IDXS)).tolist()
  model.velocity.x = (v_ego * np.ones_like(ModelConstants.T_IDXS)).tolist()
  model.acceleration.x = np.zeros_like(ModelConstants.T_IDXS).tolist()
  model.meta.disengagePredictions.gasPressProbs = [1.0] * 6
  md.speedLimit = v_cruise
  md.speedLimitValid = True
  md.approachingRoundabout = bool(approaching)
  md.onRoundabout = bool(on_rb)
  md.roundaboutDistance = float(dist_m)
  md.roundaboutSpeedLimit = float(rb_mph) * CV.MPH_TO_MS

  return _PlannerInputs({
    "radarState": radar,
    "controlsState": controls,
    "selfdriveState": selfdrive,
    "carState": car_state,
    "carControl": car_control,
    "liveParameters": live_parameters,
    "modelV2": model,
    "liveMapDataNAP": md,
  })


def _run(approaching=False, on_rb=False, dist_m=90.0, mpc_a=0.4):
  v_ego = 49.0 * CV.MPH_TO_MS
  v_cruise = 50.0 * CV.MPH_TO_MS
  planner = LongitudinalPlanner(_preap_cp(), init_v=v_ego, params=_RbParams())
  planner.mpc = _ConstantAccelerationMpc(v_ego, mpc_a)
  inputs = _inputs(v_ego, v_cruise, approaching=approaching, on_rb=on_rb, dist_m=dist_m)
  for _ in range(8):
    planner.update(inputs)
  return planner.output_a_target


def test_funnel_triggers_speed_ease():
  a_rb = _run(approaching=True, dist_m=90.0, mpc_a=0.4)
  assert a_rb < -0.15, a_rb
  v49 = 49.0 * CV.MPH_TO_MS
  eased = roundabout_ease_v_ms(
    live_map_roundabout_hint(SimpleNamespace(
      onRoundabout=False, approachingRoundabout=True,
      roundaboutDistance=90.0, roundaboutSpeedLimit=20.0 * CV.MPH_TO_MS,
    )),
    v49, v49, LOOKAHEAD_NORMAL,
  )
  assert eased is not None and eased < v49
  assert eased <= RB_V_MAX_MS + 8.0  # still falling through the funnel


def test_sharp_corner_without_flag_does_not_ease():
  # Same 49→50 HUD, MPC +0.4 — the Willmar failure without an RB flag.
  a_corner = _run(approaching=False, on_rb=False, mpc_a=0.4)
  assert a_corner > 0.2, a_corner


def test_on_roundabout_holds_soft_target():
  a_on = _run(on_rb=True, dist_m=0.0, mpc_a=0.4)
  assert a_on < -0.15, a_on
