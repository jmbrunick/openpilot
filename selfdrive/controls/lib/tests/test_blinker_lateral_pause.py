from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
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


def test_hold_does_not_resume_while_hand_on_wheel():
  hold = BlinkerLateralHold()
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=True, steering_pressed=True, hold=hold))
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(steering_pressed=True, hold=hold))
  assert hold.holding


def test_hold_resumes_after_hand_release():
  hold = BlinkerLateralHold()
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=True, steering_pressed=True, hold=hold))
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(steering_pressed=True, hold=hold))
  assert lat_active_with_blinker_pause(**_lat_kwargs(hold=hold))
  assert not hold.holding


def test_hold_without_prior_blinker_does_not_pause():
  hold = BlinkerLateralHold()
  assert lat_active_with_blinker_pause(**_lat_kwargs(steering_pressed=True, hold=hold))
  assert not hold.holding


def test_hold_clears_when_not_engaged():
  hold = BlinkerLateralHold()
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=True, steering_pressed=True, hold=hold))
  assert not lat_active_with_blinker_pause(
    **_lat_kwargs(active=False, steering_pressed=True, hold=hold))
  assert not hold.holding
  assert lat_active_with_blinker_pause(**_lat_kwargs(steering_pressed=True, hold=hold))


def test_long_stays_active_inputs():
  paused = lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, active=True))
  resumed = lat_active_with_blinker_pause(**_lat_kwargs(active=True))
  assert paused is False
  assert resumed is True


def test_inactive_or_fault_still_blocks_lat():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(active=False))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_temporary=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_permanent=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(standstill=True, steer_at_standstill=False))
