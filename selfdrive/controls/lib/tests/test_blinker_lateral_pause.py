from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
  LAMP_OFF_DEBOUNCE_S,
  blinker_pauses_lateral,
  blinker_turn_blocks_steering_disengage,
  lat_active_with_blinker_pause,
  preap_blinker_pause_hides_controls_mismatch,
  stalk_is_left_or_right,
)
from openpilot.selfdrive.controls.lib.stalk_tip_turn import (
  STALK_ALC_TURN_HOLD_S,
  STALK_TIP_HOLD_S,
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


def _lat_active(hold, *, left=False, right=False, pressed=False, active=True, dt=None,
                alc_active=False, v_ego=0.0, stalk_state=0, disengage=False,
                engaged=None):
  return lat_active_with_blinker_pause(
    **_lat_kwargs(left_blinker=left, right_blinker=right, steering_pressed=pressed,
                  active=active, hold=hold, alc_active=alc_active, v_ego=v_ego,
                  stalk_state=stalk_state, steering_disengage=disengage,
                  engaged=engaged),
    dt=dt,
  )


def test_one_lamp_pauses_lateral():
  assert blinker_pauses_lateral(True, False)
  assert blinker_pauses_lateral(False, True)
  assert not lat_active_with_blinker_pause(**_lat_kwargs(left_blinker=True))
  assert not lat_active_with_blinker_pause(**_lat_kwargs(right_blinker=True))


def test_blinker_turn_hard_gate_blocks_on_lamp_or_stalk_or_hold():
  hold = BlinkerLateralHold()
  assert stalk_is_left_or_right(1)
  assert stalk_is_left_or_right(2)
  assert not stalk_is_left_or_right(0)
  assert not stalk_is_left_or_right(3)
  assert blinker_turn_blocks_steering_disengage(True, False, 0, hold)
  assert blinker_turn_blocks_steering_disengage(False, True, 0, hold)
  assert blinker_turn_blocks_steering_disengage(False, False, 1, hold)
  assert blinker_turn_blocks_steering_disengage(False, False, 2, hold)
  assert not blinker_turn_blocks_steering_disengage(False, False, 0, hold)
  assert not blinker_turn_blocks_steering_disengage(True, True, 0, hold)
  _hold_past_tip(hold, v_ego=12.0)
  assert not _lat_active(hold, left=True, stalk_state=1)
  # Lamps dark and stalk idle: flash-latch hold still blocks.
  assert blinker_turn_blocks_steering_disengage(False, False, 0, hold)


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


def _tip_then_idle(hold, *, v_ego, stalk=1, left=True, dt=0.01):
  assert _lat_active(hold, left=left, v_ego=v_ego, stalk_state=stalk, dt=dt)
  assert _lat_active(hold, left=left, v_ego=v_ego, stalk_state=0, dt=dt)
  assert not hold.turn_active


def _hold_past_tip(hold, *, v_ego, stalk=1, left=True, dt=0.01):
  n = int(round(STALK_TIP_HOLD_S / dt))
  for _ in range(n - 1):
    assert _lat_active(hold, left=left, v_ego=v_ego, stalk_state=stalk, dt=dt)
  assert not _lat_active(hold, left=left, v_ego=v_ego, stalk_state=stalk, dt=dt)
  assert hold.turn_active


def _hold_past_alc_turn(hold, *, v_ego, stalk=1, left=True, dt=0.01, alc_active=True):
  n = int(round(STALK_ALC_TURN_HOLD_S / dt))
  for _ in range(n - 1):
    assert _lat_active(hold, left=left, v_ego=v_ego, stalk_state=stalk, dt=dt,
                       alc_active=alc_active)
  assert not _lat_active(hold, left=left, v_ego=v_ego, stalk_state=stalk, dt=dt,
                         alc_active=alc_active)
  assert hold.turn_active


def test_tip_then_idle_does_not_pause_at_low_or_highway_speed():
  for v in (10 * CV.MPH_TO_MS, 30.0):
    hold = BlinkerLateralHold()
    _tip_then_idle(hold, v_ego=v)
    assert _lat_active(hold, left=True, v_ego=v, stalk_state=0)
    assert not hold.turn_active
    assert hold.blocks_steer_disengage


def test_held_stalk_pauses_lat_at_highway_speed():
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=30.0)
  assert not _lat_active(hold, left=True, v_ego=30.0, stalk_state=1)
  assert hold.turn_active


def test_held_stalk_pauses_lat_at_low_speed():
  hold = BlinkerLateralHold()
  slow = 10 * CV.MPH_TO_MS
  _hold_past_tip(hold, v_ego=slow)
  assert hold.turn_active


def test_held_turn_pauses_even_if_alc_active():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  _hold_past_alc_turn(hold, v_ego=30.0, alc_active=True)
  assert not _lat_active(hold, left=True, alc_active=True, v_ego=30.0, stalk_state=1)
  assert hold.turn_active
  assert not hold._alc_keep


def test_same_direction_hold_under_1s_during_alc_does_not_pause():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  dt = 0.01
  n = int(round(0.90 / dt))
  for _ in range(n):
    assert _lat_active(hold, left=True, alc_active=True, v_ego=30.0, stalk_state=1, dt=dt)
  assert not hold.turn_active
  assert hold._alc_keep


def test_leftover_keep_alive_same_stalk_held_1s_becomes_turn():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  assert _lat_active(hold, left=True, alc_active=True, stalk_state=0)
  # ALC finished; leftover keep-alive still flashing.
  assert _lat_active(hold, left=True, alc_active=False, stalk_state=0)
  dt = 0.01
  for _ in range(int(round(0.90 / dt))):
    assert _lat_active(hold, left=True, alc_active=False, v_ego=30.0, stalk_state=1, dt=dt)
  assert not hold.turn_active
  for _ in range(int(round(0.15 / dt))):
    _lat_active(hold, left=True, alc_active=False, v_ego=30.0, stalk_state=1, dt=dt)
  assert not _lat_active(hold, left=True, alc_active=False, v_ego=30.0, stalk_state=1)
  assert hold.turn_active
  assert not hold._alc_keep


def test_alc_turn_then_dark_then_hand_release_resumes():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  _hold_past_alc_turn(hold, v_ego=30.0, alc_active=True)
  assert not _lat_active(hold, left=True, pressed=True, stalk_state=1)
  # Stalk released; lamps still on, then 1s dark with hand on, then release.
  assert not _lat_active(hold, left=True, pressed=True, stalk_state=0)
  assert not _lat_active(hold, pressed=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert not hold.turn_active
  assert hold.holding
  assert _lat_active(hold)
  assert not hold.holding


def test_alc_turn_pause_survives_stale_alc_active():
  """After a 1s ALC-to-turn, leftover alc_active must not grab the wheel again."""
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  _hold_past_alc_turn(hold, v_ego=30.0, alc_active=True)
  assert not _lat_active(hold, left=True, alc_active=True, stalk_state=0)
  assert hold.turn_active
  assert not hold._alc_keep


def test_opposite_tap_during_alc_does_not_pause():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  assert _lat_active(hold, left=True, alc_active=True, stalk_state=0)
  assert _lat_active(hold, right=True, alc_active=True, stalk_state=2, dt=0.01)
  assert _lat_active(hold, right=True, alc_active=True, stalk_state=0, dt=0.01)
  assert not hold.turn_active


def test_opposite_held_during_alc_pauses_at_tip_window():
  hold = BlinkerLateralHold()
  _tip_then_idle(hold, v_ego=30.0)
  assert _lat_active(hold, left=True, alc_active=True, stalk_state=0)
  dt = 0.01
  n = int(round(STALK_TIP_HOLD_S / dt))
  for _ in range(n - 1):
    assert _lat_active(hold, right=True, alc_active=True, v_ego=30.0, stalk_state=2, dt=dt)
  assert not _lat_active(hold, right=True, alc_active=True, v_ego=30.0, stalk_state=2, dt=dt)
  assert hold.turn_active


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


def test_stalk_tip_does_not_pause_before_alc_arms():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, v_ego=30.0, stalk_state=1)
  assert not hold.turn_active
  assert hold.blocks_steer_disengage


def test_alc_keep_alive_after_stalk_idle_does_not_pause():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, v_ego=30.0, stalk_state=1)
  # Tip complete: stalk springs back; OP keeps flashing.
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
  _hold_past_tip(hold, v_ego=slow)
  assert not _lat_active(hold, v_ego=slow, dt=0.4)
  assert hold.turn_active


def test_alc_grab_does_not_pause_lat():
  hold = BlinkerLateralHold()
  assert _lat_active(hold, left=True, pressed=True, alc_active=True)
  assert not hold.turn_active
  assert hold.blocks_steer_disengage


def test_preap_mismatch_hidden_only_on_preap_during_pause():
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=10.0)
  assert hold.blocks_steer_disengage
  assert preap_blinker_pause_hides_controls_mismatch(
    brand="tesla", fingerprint="TESLA_MODEL_S_PREAP",
    blocks_steer_disengage=hold.blocks_steer_disengage)
  assert not preap_blinker_pause_hides_controls_mismatch(
    brand="tesla", fingerprint="TESLA_MODEL_S_PREAP",
    blocks_steer_disengage=False)
  assert not preap_blinker_pause_hides_controls_mismatch(
    brand="toyota", fingerprint="TESLA_MODEL_S_PREAP",
    blocks_steer_disengage=True)


def test_held_flashing_corner_hides_mismatch_for_eight_seconds():
  """Justin's long blinker-held corner: lamps flash, hands-on oscillates.

  Python already keeps cruiseEnabled. The remaining full-cancel was
  controlsMismatch after ~2s of panda !controlsAllowed. The hold must
  stay blocking so selfdrived can hide that disagreement.
  """
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=12.0, stalk=1)
  dt = 0.01
  period = 0.66
  on_s = 0.33
  for i in range(int(8.0 / dt)):
    left = ((i * dt) % period) < on_s
    pressed = (i % 20) < 10
    assert not _lat_active(hold, left=left, pressed=pressed, stalk_state=1, dt=dt)
    assert hold.blocks_steer_disengage
    assert preap_blinker_pause_hides_controls_mismatch(
      brand="tesla", fingerprint="TESLA_MODEL_S_PREAP",
      blocks_steer_disengage=True)


def test_mismatch_not_hidden_after_turn_and_hand_release():
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=10.0)
  assert not _lat_active(hold, left=True, pressed=True, stalk_state=1)
  assert not _lat_active(hold, pressed=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert hold.holding
  assert _lat_active(hold)
  assert not hold.blocks_steer_disengage
  assert not preap_blinker_pause_hides_controls_mismatch(
    brand="tesla", fingerprint="TESLA_MODEL_S_PREAP",
    blocks_steer_disengage=hold.blocks_steer_disengage)


def test_faster_corner_hands_on_two_without_steering_pressed_stays_paused():
  """Justin's slightly faster turn: EPAS hands-on 2, torsion bar not yet pressed."""
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=14.0)
  dt = 0.01
  period = 0.66
  on_s = 0.33
  for i in range(int(4.0 / dt)):
    left = ((i * dt) % period) < on_s
    assert not _lat_active(hold, left=left, pressed=False, disengage=True,
                           stalk_state=1, dt=dt)
    assert hold.turn_active
    assert hold.blocks_steer_disengage


def test_high_torque_does_not_expire_latch_on_one_second_dark():
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=14.0)
  assert not _lat_active(hold, left=True, disengage=True, stalk_state=1)
  # Lamps dark for a full second while still wrenching — still the same turn.
  assert not _lat_active(hold, pressed=False, disengage=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert hold.turn_active
  assert hold.blocks_steer_disengage
  # Torque released, then 1s dark: turn ends and lat resumes by itself.
  assert _lat_active(hold, pressed=False, disengage=False, dt=LAMP_OFF_DEBOUNCE_S)
  assert not hold.turn_active
  assert not hold.blocks_steer_disengage


def test_hold_does_not_reset_when_active_drops_but_enabled_stays():
  hold = BlinkerLateralHold()
  _hold_past_tip(hold, v_ego=12.0)
  assert not _lat_active(hold, left=True, disengage=True, active=False, engaged=True)
  assert hold.turn_active
  assert hold.blocks_steer_disengage
