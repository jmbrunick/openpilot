from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  ALC_ARM_SPEED_MIN,
  BlinkerLateralHold,
  LAMP_OFF_DEBOUNCE_S,
  blinker_pauses_lateral,
  lat_active_with_blinker_pause,
)
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN


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


def _lat_active(hold, *, left=False, right=False, pressed=False, active=True, dt=None,
                alc_active=False, v_ego=0.0, stalk_state=0):
  return lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=left, right_blinker=right, steering_pressed=pressed,
                  active=active, hold=hold, alc_active=alc_active, v_ego=v_ego,
                  stalk_state=stalk_state),
    dt=dt,
  )


def test_one_lamp_pauses_lateral():
  assert blinker_pauses_lateral(True, False)
  assert blinker_pauses_lateral(False, True)
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(right_blinker=True))


def test_hazards_do_not_pause():
  assert not blinker_pauses_lateral(True, True)
  assert lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True, right_blinker=True))
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, right=True)
  assert not hold.turn_active
  assert not hold.holding


def test_hazards_after_turn_do_not_keep_turn_active():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True)
  assert hold.turn_active
  assert _lat_active(hold, left=True, right=True)
  assert not hold.turn_active
  assert not hold.holding


def test_flash_gap_without_hand_keeps_pause():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True)
  # Typical Tesla off-period is ~0.3s. Stay paused with no steeringPressed.
  assert not _lat_active(hold, dt=0.4)
  assert hold.turn_active
  assert hold.holding
  assert not _lat_active(hold, left=True)
  assert not _lat_active(hold, dt=0.4)
  assert hold.turn_active


def test_one_second_dark_ends_turn_active():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True)
  assert not _lat_active(hold, dt=LAMP_OFF_DEBOUNCE_S - 0.01)
  assert hold.turn_active
  assert _lat_active(hold, dt=0.01)
  assert not hold.turn_active
  assert not hold.holding


def test_long_flashing_turn_stays_paused_without_hand():
  """Waiting in traffic with the blinker on must not drop the pause every blink."""
  hold = BlinkerLateralHold()
  dt = 0.01
  period = 0.66  # ~90 flashes/min
  on_s = 0.33
  for i in range(int(8.0 / dt)):
    left = ((i * dt) % period) < on_s
    assert not _lat_active(hold, left=left, dt=dt)
  assert hold.turn_active
  assert hold.holding


def test_justin_corner_does_not_auto_resume_on_lamp_off():
  """Turn-complete is 1s dark, not stalk-center. Hand-on then keeps the pause."""
  hold = BlinkerLateralHold()
  # 1. Single lamp: release lat. Long is not this helper.
  assert not _lat_active(hold, left=True)
  # 3. Hand-steer while the lamp is on: stay paused.
  assert not _lat_active(hold, left=True, pressed=True)
  # Flash gap with light/no hands: still the same turn.
  assert not _lat_active(hold, dt=0.4)
  assert hold.turn_active
  # 4. Turn complete (1s dark), hand still on: do not resume.
  assert not _lat_active(hold, pressed=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert not hold.turn_active
  assert hold.holding
  # 4. Hand released: resume lat by itself. No stalk pull.
  assert _lat_active(hold)
  assert not hold.holding


def test_hold_does_not_resume_while_hand_on_wheel():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True, pressed=True)
  assert not _lat_active(hold, pressed=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert hold.holding


def test_hold_resumes_after_hand_release():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True, pressed=True)
  assert not _lat_active(hold, pressed=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert _lat_active(hold)
  assert not hold.holding


def test_hold_without_prior_blinker_does_not_pause():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, pressed=True)
  assert not hold.holding


def test_hold_clears_when_not_engaged():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True, pressed=True)
  assert not _lat_active(hold, pressed=True, active=False)
  assert not hold.holding
  assert not hold.turn_active
  assert _lat_active(hold, pressed=True)


def test_lamp_off_without_hand_resumes_after_debounce():
  hold = BlinkerLateralHold()
  assert not _lat_active(hold, left=True)
  # One dark frame is a flash gap, not turn-complete.
  assert not _lat_active(hold)
  assert hold.turn_active
  assert _lat_active(hold, dt=LAMP_OFF_DEBOUNCE_S)
  assert not hold.turn_active


def test_inactive_or_fault_still_blocks_lat():
  assert not lat_active_with_blinker_pause(**_lat_kwargs(active=False))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_temporary=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(steer_fault_permanent=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(standstill=True, steer_at_standstill=False))


def test_alc_arm_speed_matches_desire_helper():
  assert ALC_ARM_SPEED_MIN == LANE_CHANGE_SPEED_MIN


def test_alc_active_flashing_lamps_do_not_pause_or_latch_turn():
  hold = BlinkerLateralHold()
  dt = 0.01
  period = 0.66
  on_s = 0.33
  for i in range(int(3.0 / dt)):
    left = ((i * dt) % period) < on_s
    assert _lat_active(hold, left=left, alc_active=True, dt=dt)
  assert not hold.turn_active
  assert not hold.holding
  assert hold.blocks_steer_disengage


def test_highway_stalk_tap_does_not_pause_before_alc_arms():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, v_ego=30.0, stalk_state=1)
  assert not hold.turn_active
  assert hold.blocks_steer_disengage


def test_alc_keep_alive_after_stalk_idle_does_not_pause():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, v_ego=30.0, stalk_state=1)
  # Stalk springs back; OP keeps flashing.
  assert _lat_active(hold, left=True, v_ego=30.0, stalk_state=0)
  assert _lat_active(hold, v_ego=30.0, dt=0.4)
  assert _lat_active(hold, left=True, v_ego=30.0)
  assert not hold.turn_active
  assert hold.blocks_steer_disengage


def test_alc_leftover_flashes_do_not_become_driver_turn():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, alc_active=True)
  # ALC finished; lamps still finishing a flash. Must not latch a turn.
  assert _lat_active(hold, left=True)
  assert not hold.turn_active
  assert _lat_active(hold, dt=0.4)
  assert not hold.turn_active
  assert _lat_active(hold, dt=LAMP_OFF_DEBOUNCE_S)
  assert not hold.blocks_steer_disengage


def test_driver_turn_without_alc_still_pauses():
  hold = BlinkerLateralHold()
  slow = 10 * CV.MPH_TO_MS
  assert not _lat_active(hold, left=True, v_ego=slow, stalk_state=1)
  assert hold.turn_active
  assert not _lat_active(hold, v_ego=slow, dt=0.4)
  assert hold.turn_active


def test_alc_grab_does_not_pause_lat():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, pressed=True, alc_active=True)
  assert not hold.turn_active
  assert hold.blocks_steer_disengage
