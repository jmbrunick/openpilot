from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  blinker_pauses_lateral,
  lat_active_with_blinker_pause,
)
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN

TURN_SPEED = 15 * CV.MPH_TO_MS
HIGHWAY_SPEED = 30 * CV.MPH_TO_MS


def _lat_kwargs(**overrides):
  kwargs = dict(
    active=True,
    steer_fault_temporary=False,
    steer_fault_permanent=False,
    standstill=False,
    steer_at_standstill=False,
    left_blinker=False,
    right_blinker=False,
    v_ego=TURN_SPEED,
  )
  kwargs.update(overrides)
  return kwargs


def test_low_speed_one_lamp_pauses_lateral():
  assert TURN_SPEED < LANE_CHANGE_SPEED_MIN
  assert blinker_pauses_lateral(True, False, TURN_SPEED)
  assert blinker_pauses_lateral(False, True, TURN_SPEED)
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, v_ego=TURN_SPEED))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(right_blinker=True, v_ego=TURN_SPEED))


def test_highway_one_lamp_without_junction_does_not_pause():
  assert HIGHWAY_SPEED >= LANE_CHANGE_SPEED_MIN
  assert not blinker_pauses_lateral(True, False, HIGHWAY_SPEED)
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, v_ego=HIGHWAY_SPEED))


def test_highway_junction_on_blinker_side_pauses():
  assert blinker_pauses_lateral(True, False, HIGHWAY_SPEED, junction_on_blinker_side=True)
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=True, v_ego=HIGHWAY_SPEED, junction_on_blinker_side=True))


def test_hazards_do_not_pause():
  assert not blinker_pauses_lateral(True, True, TURN_SPEED)
  assert not blinker_pauses_lateral(True, True, HIGHWAY_SPEED, junction_on_blinker_side=True)
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, right_blinker=True, v_ego=TURN_SPEED))


def test_resume_when_lamp_clears():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, v_ego=TURN_SPEED))
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=False, right_blinker=False, v_ego=TURN_SPEED))


def test_long_path_unaffected_by_blinker_helper():
  paused = lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, active=True, v_ego=TURN_SPEED))
  resumed = lat_active_with_blinker_pause(**_lat_kwargs(active=True, v_ego=TURN_SPEED))
  assert paused is False
  assert resumed is True


def test_inactive_or_fault_still_blocks_lat():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(active=False, v_ego=HIGHWAY_SPEED))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_temporary=True, v_ego=HIGHWAY_SPEED))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_permanent=True, v_ego=HIGHWAY_SPEED))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(standstill=True, steer_at_standstill=False, v_ego=HIGHWAY_SPEED))
