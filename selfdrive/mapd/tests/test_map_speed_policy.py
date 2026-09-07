from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import MODE_CAP, MODE_DISPLAY, MODE_FOLLOW, MODE_OFF
from openpilot.selfdrive.mapd.map_speed_policy import (
  SOURCE_CRUISE, SOURCE_LEAD0, V_CRUISE_UNSET,
  apply_map_speed_kph, cap_planner_v_cruise_ms, longitudinal_obstacle_source,
)

# Keep in sync with long_mpc (avoid importing cereal/acados here).
_COMFORT_BRAKE = 2.5
_STOP_DISTANCE = 6.0
_T_FOLLOW = 1.45  # LongitudinalPersonality.standard


def _cruise_obstacle_m(v_cruise_ms: float) -> float:
  return (v_cruise_ms ** 2) / (2 * _COMFORT_BRAKE) + _T_FOLLOW * v_cruise_ms + _STOP_DISTANCE


def _lead_obstacle_m(d_rel: float, v_lead: float) -> float:
  return d_rel + (v_lead ** 2) / (2 * _COMFORT_BRAKE)


def test_off_and_display_never_change_driver_set():
  for mode in (MODE_OFF, MODE_DISPLAY):
    assert apply_map_speed_kph(100, 70, mode=mode, engaged=True, op_long_software_cruise=True) == 100


def test_cap_lowers_but_does_not_raise():
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(50, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 50


def test_follow_tracks_map_unless_override():
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(50, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True) == 70
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=True,
                             driver_override=True) == 100


def test_pcm_cruise_never_gets_control_overlay():
  # No-pedal stock CC: do not invent a parallel set-speed path
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=False) == 100
  assert apply_map_speed_kph(100, 70, mode=MODE_FOLLOW, engaged=True, op_long_software_cruise=False) == 100


def test_disengaged_or_unset_passthrough():
  assert apply_map_speed_kph(100, 70, mode=MODE_CAP, engaged=False, op_long_software_cruise=True) == 100
  assert apply_map_speed_kph(V_CRUISE_UNSET, 70, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == V_CRUISE_UNSET
  assert apply_map_speed_kph(100, None, mode=MODE_CAP, engaged=True, op_long_software_cruise=True) == 100


def test_offset_applies_to_map_target():
  offset = 5 * CV.MPH_TO_KPH
  out = apply_map_speed_kph(120, 70, mode=MODE_CAP, offset_kph=offset, engaged=True, op_long_software_cruise=True)
  assert abs(out - (70 + offset)) < 0.05


def test_planner_cap_is_down_only():
  lim = 25.0  # m/s
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_CAP) == lim
  assert cap_planner_v_cruise_ms(20.0, lim, mode=MODE_CAP) == 20.0
  assert cap_planner_v_cruise_ms(30.0, lim, mode=MODE_DISPLAY) == 30.0
  assert cap_planner_v_cruise_ms(30.0, None, mode=MODE_FOLLOW) == 30.0


def test_no_lead_tracks_map_capped_cruise():
  """No radar lead: fake far/fast lead, so cruise (map ceiling) is the tightest obstacle."""
  v_ego = 31.29  # ~70 mph
  v_cruise = cap_planner_v_cruise_ms(40.0, 31.29, mode=MODE_CAP)
  assert abs(v_cruise - 31.29) < 1e-6
  cruise = _cruise_obstacle_m(v_cruise)
  fake = _lead_obstacle_m(50.0, v_ego + 10.0)
  assert longitudinal_obstacle_source(fake, fake, cruise) == SOURCE_CRUISE


def test_slower_lead_still_commands_below_map_ceiling():
  """Close slower lead must win min(lead, cruise) regardless of map v_cruise."""
  lead = _lead_obstacle_m(40.0, 15.0)
  for v_cruise in (20.0, 31.29, 40.0):
    capped = cap_planner_v_cruise_ms(v_cruise, 31.29, mode=MODE_CAP)
    cruise = _cruise_obstacle_m(capped)
    assert longitudinal_obstacle_source(lead, 1e8, cruise) == SOURCE_LEAD0
    assert min(lead, cruise) == lead


def test_planner_and_mpc_keep_radar_after_map_cap():
  """Regression: map cap runs before mpc.update(radarState, v_cruise)."""
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  planner = (root / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  mpc = (root / "selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  cap_at = planner.find("cap_planner_v_cruise_ms")
  mpc_at = planner.find("self.mpc.update(sm['radarState'], v_cruise")
  assert 0 <= cap_at < mpc_at
  assert "np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])" in mpc
  assert "self.params[:,2] = np.min(x_obstacles, axis=1)" in mpc
