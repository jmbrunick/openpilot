"""Hypermile Hill Climb: grade hold, crest/downhill ease, planner wiring."""
import math
from pathlib import Path

import numpy as np
import pytest

from cereal import log, messaging
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.hill_climb import (
  CLIMB_EXTRA_MAX_MS2,
  CREST_EASE_MS2,
  DOWNHILL_EASE_MS2,
  GRAVITY_MS2,
  PARAM_HILL_CLIMB,
  PITCH_CLIMB_RAD,
  PITCH_CREST_RAD,
  PITCH_DOWN_RAD,
  apply_hill_climb,
  climb_authority_ms2,
  downhill_ease_ms2,
  grade_load_ms2,
  hill_climb_applies,
  read_hypermile_hill_climb,
)
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_APPROACH_A_MS2,
  lead_approach_need_m,
)
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalPlanSource,
  T_IDXS,
  get_T_FOLLOW,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  LongitudinalPlanner,
  get_max_accel,
)
from openpilot.selfdrive.controls.tests.test_hypermile import FakeParams
from openpilot.selfdrive.controls.tests.test_tesla_preap_following import (
  STOP_DISTANCE_M,
  _ConstantAccelerationMpc,
  _PlannerInputs,
  _make_preap_params,
)
from openpilot.selfdrive.mapd.constants import (
  LOOKAHEAD_EARLY,
  MODE_FOLLOW,
  map_accel_a_ms2,
)
from openpilot.selfdrive.mapd.map_speed_policy import map_track_accel_ms2
from openpilot.selfdrive.modeld.constants import ModelConstants


UPHILL_RAD = math.radians(4.0)  # existing full-loop fixture; clearly above 2°
DOWNHILL_RAD = math.radians(-4.0)
FLAT_RAD = 0.0
# Accel 1 + Early (Hypermile eco snap).
FLAT_ACCEL_1 = map_accel_a_ms2(LOOKAHEAD_EARLY, 1)


class _HillParams:
  def __init__(self, *, hypermile=False, hill_climb=True, map_mode=MODE_FOLLOW, accel=1, lookahead=LOOKAHEAD_EARLY):
    self.hypermile = hypermile
    self.hill_climb = hill_climb
    self.map_mode = map_mode
    self.accel = accel
    self.lookahead = lookahead

  def get(self, key, return_default=False):
    if key == "NAPFollowDistance":
      return 4
    if key == "NAPHypermileFollowLevel":
      return 3
    if key == "NAPMapSpeedMode":
      return self.map_mode
    if key == "NAPMapSpeedOffsetMph":
      return -5
    if key == "NAPMapSpeedLookahead":
      return self.lookahead
    if key == "NAPMapSpeedAccel":
      return self.accel
    raise AssertionError(key)

  def get_bool(self, key):
    if key == "NAPAdaptiveAccel":
      return True
    if key == "NAPHypermile":
      return self.hypermile
    if key == "NAPHypermileHillClimb":
      return self.hill_climb
    raise AssertionError(key)


def _planner_inputs(v_ego, v_cruise_ms, pitch):
  radar = messaging.new_message("radarState").radarState
  controls = messaging.new_message("controlsState").controlsState
  selfdrive = messaging.new_message("selfdriveState").selfdriveState
  car_state = messaging.new_message("carState").carState
  car_control = messaging.new_message("carControl").carControl
  live_parameters = messaging.new_message("liveParameters").liveParameters
  model = messaging.new_message("modelV2").modelV2

  controls.longControlState = LongCtrlState.pid
  selfdrive.personality = log.LongitudinalPersonality.standard
  car_state.vEgo = v_ego
  car_state.vCruise = v_cruise_ms * CV.MS_TO_KPH
  car_control.orientationNED = [0.0, float(pitch), 0.0]
  model.position.x = (v_ego * np.array(ModelConstants.T_IDXS)).tolist()
  model.velocity.x = (v_ego * np.ones_like(ModelConstants.T_IDXS)).tolist()
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


def _run_map_climb(hypermile, hill_climb, pitch, *, v_ego=20.0, v_cruise=31.29, mpc_a=0.0):
  params = _HillParams(hypermile=hypermile, hill_climb=hill_climb)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=mpc_a)
  planner.prev_accel_clip = [-1.2, get_max_accel(v_ego)]
  inputs = _planner_inputs(v_ego, v_cruise, pitch)
  planner.update(inputs)
  return planner


def test_default_on_and_inert_unless_hypermile():
  assert hill_climb_applies(True, False) is False
  assert hill_climb_applies(False, True) is False
  assert hill_climb_applies(True, True) is True
  assert read_hypermile_hill_climb(FakeParams(bools={PARAM_HILL_CLIMB: True})) is True
  assert read_hypermile_hill_climb(FakeParams(bools={PARAM_HILL_CLIMB: False})) is False
  # Missing key / test double without the param → On (keys.h default).
  assert read_hypermile_hill_climb(object()) is True


def test_uphill_under_max_raises_accel1_authority():
  flat = FLAT_ACCEL_1
  assert flat == pytest.approx(0.30, abs=1e-9)
  uphill = climb_authority_ms2(flat, UPHILL_RAD)
  assert uphill > flat
  expected = flat + min(CLIMB_EXTRA_MAX_MS2, GRAVITY_MS2 * math.sin(UPHILL_RAD))
  assert uphill == pytest.approx(expected)
  # 4° is ~0.68 m/s² of gravity — Accel 1 alone sags.
  assert grade_load_ms2(UPHILL_RAD) > flat
  assert PITCH_CLIMB_RAD == pytest.approx(math.radians(2.0))
  assert UPHILL_RAD > PITCH_CLIMB_RAD


def test_flat_and_gates_leave_cmd_alone():
  a_flat = 0.30
  kwargs = dict(
    prev_pitch_rad=0.0,
    v_ego_ms=20.0,
    v_cruise_ms=31.29,
    a_cmd=a_flat,
    in_deadband=False,
  )
  assert apply_hill_climb(pitch_rad=FLAT_RAD, hypermile_on=False, hill_climb_on=True, **kwargs) == a_flat
  assert apply_hill_climb(pitch_rad=UPHILL_RAD, hypermile_on=False, hill_climb_on=True, **kwargs) == a_flat
  assert apply_hill_climb(pitch_rad=UPHILL_RAD, hypermile_on=True, hill_climb_on=False, **kwargs) == a_flat
  assert apply_hill_climb(pitch_rad=FLAT_RAD, hypermile_on=True, hill_climb_on=True, **kwargs) == a_flat
  # Below the 2° gate: crown / IMU noise is not a climb.
  almost = PITCH_CLIMB_RAD - 1e-3
  assert apply_hill_climb(pitch_rad=almost, hypermile_on=True, hill_climb_on=True, **kwargs) == a_flat


def test_crest_and_downhill_ease_direction():
  hold = 0.0
  down = apply_hill_climb(
    pitch_rad=DOWNHILL_RAD,
    prev_pitch_rad=0.0,
    v_ego_ms=31.29,
    v_cruise_ms=31.29,
    a_cmd=hold,
    in_deadband=True,
    hypermile_on=True,
    hill_climb_on=True,
  )
  assert down < 0.0
  assert down >= -DOWNHILL_EASE_MS2 - 1e-9
  assert abs(down) < 0.55  # under Early map brake; not a Late bite

  crest = apply_hill_climb(
    pitch_rad=PITCH_CREST_RAD - 0.005,
    prev_pitch_rad=UPHILL_RAD,
    v_ego_ms=31.20,
    v_cruise_ms=31.29,
    a_cmd=hold,
    in_deadband=True,
    hypermile_on=True,
    hill_climb_on=True,
  )
  assert crest == pytest.approx(-CREST_EASE_MS2)
  assert downhill_ease_ms2(DOWNHILL_RAD) <= DOWNHILL_EASE_MS2
  assert downhill_ease_ms2(-PITCH_CLIMB_RAD) >= CREST_EASE_MS2

  # Under MAX on a downhill: lower +a, do not dump regen away from MAX.
  eased_up = apply_hill_climb(
    pitch_rad=DOWNHILL_RAD,
    prev_pitch_rad=0.0,
    v_ego_ms=20.0,
    v_cruise_ms=31.29,
    a_cmd=0.30,
    in_deadband=False,
    hypermile_on=True,
    hill_climb_on=True,
  )
  assert 0.0 <= eased_up < 0.30


def test_never_raises_max_or_fights_lead_brake():
  v_cruise = 31.29
  # At MAX: hold against gravity only — no Accel-1 leftover past MAX.
  hold = apply_hill_climb(
    pitch_rad=UPHILL_RAD,
    prev_pitch_rad=UPHILL_RAD,
    v_ego_ms=v_cruise,
    v_cruise_ms=v_cruise,
    a_cmd=0.0,
    in_deadband=True,
    hypermile_on=True,
    hill_climb_on=True,
  )
  extra = min(CLIMB_EXTRA_MAX_MS2, grade_load_ms2(UPHILL_RAD))
  assert hold == pytest.approx(extra)
  assert hold < climb_authority_ms2(FLAT_ACCEL_1, UPHILL_RAD)

  # Lead / MPC already braking: hill climb must not cancel it.
  lead_a = -LEAD_APPROACH_A_MS2
  assert apply_hill_climb(
    pitch_rad=UPHILL_RAD,
    prev_pitch_rad=UPHILL_RAD,
    v_ego_ms=20.0,
    v_cruise_ms=v_cruise,
    a_cmd=lead_a,
    in_deadband=False,
    hypermile_on=True,
    hill_climb_on=True,
  ) == lead_a

  # Downhill + already braking: keep the harder of lead/map vs light ease.
  hard = -2.0
  assert apply_hill_climb(
    pitch_rad=DOWNHILL_RAD,
    prev_pitch_rad=0.0,
    v_ego_ms=v_cruise,
    v_cruise_ms=v_cruise,
    a_cmd=hard,
    in_deadband=True,
    hypermile_on=True,
    hill_climb_on=True,
  ) == hard


def test_planner_gates_and_uphill_authority():
  v_ego = 20.0
  v_cruise = 31.29
  flat_a = map_track_accel_ms2(v_ego, v_cruise, FLAT_ACCEL_1)
  assert flat_a is not None and flat_a > 0.0

  off = _run_map_climb(False, True, UPHILL_RAD, v_ego=v_ego, v_cruise=v_cruise)
  hill_off = _run_map_climb(True, False, UPHILL_RAD, v_ego=v_ego, v_cruise=v_cruise)
  flat = _run_map_climb(True, True, FLAT_RAD, v_ego=v_ego, v_cruise=v_cruise)
  climb = _run_map_climb(True, True, UPHILL_RAD, v_ego=v_ego, v_cruise=v_cruise)

  assert off.output_a_target == pytest.approx(flat_a, abs=0.06)
  assert hill_off.output_a_target == pytest.approx(flat_a, abs=0.06)
  assert flat.output_a_target == pytest.approx(flat_a, abs=0.06)
  assert climb.output_a_target > flat.output_a_target + 0.20
  # Still under cruise safety clip; never a MAX raise (vCruise untouched).
  assert climb.output_a_target <= get_max_accel(v_ego) + 1e-9
  assert climb.output_a_target <= climb_authority_ms2(flat_a, UPHILL_RAD) + 1e-6


def test_planner_lead_still_wins_on_uphill():
  v_ego = 26.8
  v_lead = 22.4
  v_cruise = 31.29
  t_follow = get_T_FOLLOW(nap_follow_dist=4)
  d_follow = t_follow * v_lead + STOP_DISTANCE_M
  need = lead_approach_need_m(v_ego, v_lead, t_follow=t_follow)
  v_rel = v_ego - v_lead
  rel_need = (v_rel * v_rel) / (2.0 * LEAD_APPROACH_A_MS2)
  d_rel = d_follow + rel_need

  params = _HillParams(hypermile=True, hill_climb=True)
  planner = LongitudinalPlanner(_make_preap_params(), init_v=v_ego, params=params)
  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, get_max_accel(v_ego)]
  inputs = _planner_inputs(v_ego, v_cruise, UPHILL_RAD)
  lead = inputs["radarState"].leadOne
  lead.status = True
  lead.dRel = d_rel
  lead.vLead = v_lead
  planner.update(inputs)
  assert planner.output_a_target < 0.0
  assert planner.output_a_target == pytest.approx(-LEAD_APPROACH_A_MS2, abs=0.08)

  planner.mpc = _ConstantAccelerationMpc(v_ego, acceleration_mps2=-2.0)
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-2.0, abs=0.08)


def test_planner_crest_ease_near_max():
  v_cruise = 31.29
  planner = _run_map_climb(True, True, PITCH_CREST_RAD - 0.005, v_ego=v_cruise, v_cruise=v_cruise)
  # First frame has prev_pitch=0 so no crest; seed a climb then flatten.
  planner._hill_pitch = UPHILL_RAD
  inputs = _planner_inputs(v_cruise, v_cruise, PITCH_CREST_RAD - 0.005)
  planner.mpc = _ConstantAccelerationMpc(v_cruise, acceleration_mps2=0.0)
  planner.prev_accel_clip = [-1.2, get_max_accel(v_cruise)]
  planner.update(inputs)
  assert planner.output_a_target == pytest.approx(-CREST_EASE_MS2, abs=0.06)


def test_settings_and_docs_wire_hill_climb():
  root = Path(__file__).resolve().parents[3]
  tici = (root / "selfdrive/ui/layouts/settings/driving_mannerisms.py").read_text()
  mici = (root / "selfdrive/ui/mici/layouts/settings/driving_mannerisms.py").read_text()
  nap = (root / "selfdrive/ui/layouts/settings/nap.py").read_text()
  content = (root / "selfdrive/ui/layouts/settings/nap_content.py").read_text()
  keys = (root / "common/params_keys.h").read_text()
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  docs = (root / "docs-nap/hypermile.md").read_text()
  readme = (root / "docs-nap/README.md").read_text()
  releases = (root / "RELEASES.md").read_text()
  arch = (root / "docs-nap/architecture.md").read_text()

  assert "Hill Climb" in tici
  assert "NAP_HYPERMILE_HILL_CLIMB" in tici
  assert "hill climb" in mici
  assert "NAP_HYPERMILE_HILL_CLIMB" in mici
  assert "put_bool(NAP_HYPERMILE_HILL_CLIMB, True)" in nap
  assert "NAPHypermileHillClimb" in keys
  assert 'BOOL, "1"' in next(ln for ln in keys.splitlines() if '"NAPHypermileHillClimb"' in ln)
  assert "apply_hill_climb" in planner
  assert "maps-elevation lookahead" in planner
  assert "orientationNED" in planner
  assert "NAPHypermileHillClimb" in content
  assert "Hill Climb" in docs
  assert "maps-elevation lookahead" in docs.lower() or "not included" in docs.lower()
  assert "IMU pitch" in docs or "IMU-pitch" in docs
  assert "hill climb" in readme.lower() or "Hill Climb" in readme
  hm = next(p for p in releases.split("\n\n") if "Hill Climb" in p)
  assert "not included" in hm.lower() or "NOT included" in hm or "no maps-elevation" in hm.lower()
  assert "nap-release" in hm.lower()
  assert "Hill Climb" in arch or "hill climb" in arch
