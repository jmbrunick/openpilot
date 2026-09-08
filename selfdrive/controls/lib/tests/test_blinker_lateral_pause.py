from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  blinker_pauses_lateral,
  lat_active_with_blinker_pause,
)


def _lat_kwargs(**overrides):
  kwargs = dict(
    active=True,
    steer_fault_temporary=False,
    steer_fault_permanent=False,
    standstill=False,
    steer_at_standstill=False,
    left_blinker=False,
    right_blinker=False,
  )
  kwargs.update(overrides)
  return kwargs


def test_one_lamp_pauses_lateral():
  assert blinker_pauses_lateral(True, False)
  assert blinker_pauses_lateral(False, True)
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(right_blinker=True))


def test_hazards_do_not_pause():
  assert not blinker_pauses_lateral(True, True)
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, right_blinker=True))


def test_resume_when_lamp_clears():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True))
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=False, right_blinker=False))


def test_long_path_unaffected_by_blinker_helper():
  # Longitudinal uses CC.enabled / longActive, not this helper. A lamp must
  # not change the engaged/active inputs the long path already had.
  paused = lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, active=True))
  resumed = lat_active_with_blinker_pause(**_lat_kwargs(active=True))
  assert paused is False
  assert resumed is True


def test_inactive_or_fault_still_blocks_lat():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(active=False))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_temporary=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_permanent=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(standstill=True, steer_at_standstill=False))
