from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import MODE_CAP, MODE_DISPLAY, MODE_FOLLOW, MODE_OFF
from openpilot.selfdrive.mapd.map_speed_policy import (
  V_CRUISE_UNSET, apply_map_speed_kph, cap_planner_v_cruise_ms,
)


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
