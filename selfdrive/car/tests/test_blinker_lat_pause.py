"""Blinker-lamp lateral pause: events, cruise, and Tesla FSM."""

from cereal import car, log
from opendbc.car.toyota.values import CAR as TOYOTA

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  LAMP_OFF_DEBOUNCE_S,
  blinker_pauses_lateral,
  preap_blinker_pause_hides_controls_mismatch,
)
from openpilot.selfdrive.selfdrived.events import ET, EVENTS


EventName = log.OnroadEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type


def _cp():
  # Common-event path only. Tesla pedal-calibration events are unrelated.
  cp = car.CarParams.new_message()
  cp.carFingerprint = TOYOTA.TOYOTA_RAV4
  cp.brand = "toyota"
  cp.pcmCruise = True
  cp.openpilotLongitudinalControl = False
  return cp


def _cs(*, left=False, right=False, steering_disengage=False, steering_pressed=False,
        cancel=False, cruise_enabled=True, v_ego=0.0, stalk=0):
  cs = car.CarState.new_message()
  cs.cruiseState.available = True
  cs.cruiseState.enabled = cruise_enabled
  cs.gearShifter = "drive"
  cs.leftBlinker = left
  cs.rightBlinker = right
  cs.steeringDisengage = steering_disengage
  cs.steeringPressed = steering_pressed
  cs.vEgo = v_ego
  cs.turnSignalStalkState = stalk
  if cancel:
    be = car.CarState.ButtonEvent.new_message()
    be.type = ButtonType.cancel
    be.pressed = True
    cs.buttonEvents = [be]
  return cs


def _events(cs, cs_prev=None, cse=None, cc=None):
  helper = cse or CarSpecificEvents(_cp())
  return helper.update(cs, cs_prev or _cs(), cc or car.CarControl.new_message())


def test_one_lamp_steering_does_not_user_disable():
  prev = _cs(left=True)
  events = _events(_cs(left=True, steering_disengage=True, steering_pressed=True), prev)
  assert EventName.steerDisengage not in events.names
  assert ET.USER_DISABLE not in EVENTS[EventName.steerOverride]


def test_hazards_still_allow_steer_disengage():
  prev = _cs(left=True, right=True)
  events = _events(_cs(left=True, right=True, steering_disengage=True), prev)
  assert EventName.steerDisengage in events.names


def test_no_lamp_steer_disengage_still_fires():
  events = _events(_cs(steering_disengage=True), _cs())
  assert EventName.steerDisengage in events.names


def test_stalk_cancel_still_fires_during_pause():
  events = _events(_cs(left=True, cancel=True), _cs(left=True))
  assert EventName.buttonCancel in events.names


def test_blinker_pause_predicate():
  assert blinker_pauses_lateral(True, False)
  assert blinker_pauses_lateral(False, True)
  assert not blinker_pauses_lateral(True, True)
  assert not blinker_pauses_lateral(False, False)


def test_lamp_off_hand_still_on_does_not_user_disable():
  cse = CarSpecificEvents(_cp())
  _events(_cs(left=True), _cs(left=True), cse=cse)
  # Lamp just cleared on the same frame the wheel is grabbed.
  events = _events(_cs(steering_disengage=True, steering_pressed=True),
                   _cs(left=True), cse=cse)
  assert EventName.steerDisengage not in events.names


def test_flash_gap_without_hand_does_not_user_disable():
  cse = CarSpecificEvents(_cp())
  _events(_cs(left=True), _cs(), cse=cse)
  # Dark between flashes, no steeringPressed, then a grab.
  _events(_cs(), _cs(left=True), cse=cse)
  events = _events(_cs(steering_disengage=True, steering_pressed=True), _cs(), cse=cse)
  assert EventName.steerDisengage not in events.names
  assert cse.blinker_lat_hold.turn_active


def test_hand_on_after_turn_complete_does_not_user_disable():
  cse = CarSpecificEvents(_cp())
  _events(_cs(left=True), _cs(), cse=cse)
  cse.blinker_lat_hold.update(False, False, True, dt=LAMP_OFF_DEBOUNCE_S)
  events = _events(_cs(steering_disengage=True, steering_pressed=True), _cs(), cse=cse)
  assert EventName.steerDisengage not in events.names
  assert cse.blinker_lat_hold.holding
  assert not cse.blinker_lat_hold.turn_active


def test_steer_disengage_after_hand_release_still_fires():
  cse = CarSpecificEvents(_cp())
  _events(_cs(left=True, steering_disengage=True, steering_pressed=True),
          _cs(left=True), cse=cse)
  # Turn complete (~1s dark) with the wheel released, then a new grab.
  cse.blinker_lat_hold.update(False, False, False, dt=LAMP_OFF_DEBOUNCE_S)
  events = _events(_cs(steering_disengage=True, steering_pressed=True), _cs(), cse=cse)
  assert EventName.steerDisengage in events.names


def test_alc_keep_alive_lamps_do_not_steer_disengage():
  cse = CarSpecificEvents(_cp())
  cc = car.CarControl.new_message()
  cc.leftBlinker = True
  _events(_cs(left=True), _cs(), cse=cse, cc=cc)
  events = _events(_cs(left=True, steering_disengage=True, steering_pressed=True),
                   _cs(left=True), cse=cse, cc=cc)
  assert EventName.steerDisengage not in events.names
  assert not cse.blinker_lat_hold.turn_active
  assert cse.blinker_lat_hold.blocks_steer_disengage


def test_highway_stalk_tap_lamps_do_not_steer_disengage():
  cse = CarSpecificEvents(_cp())
  events = _events(_cs(left=True, steering_disengage=True, steering_pressed=True,
                       v_ego=30.0, stalk=1), _cs(), cse=cse)
  assert EventName.steerDisengage not in events.names
  assert not cse.blinker_lat_hold.turn_active


def test_held_flashing_corner_keeps_block_and_hides_preap_mismatch():
  cse = CarSpecificEvents(_cp())
  dt = 0.01
  period = 0.66
  on_s = 0.33
  prev = _cs()
  for i in range(int(8.0 / dt)):
    left = ((i * dt) % period) < on_s
    pressed = (i % 20) < 10
    cs = _cs(left=left, steering_disengage=pressed, steering_pressed=pressed, stalk=1)
    events = _events(cs, prev, cse=cse)
    prev = cs
    assert EventName.steerDisengage not in events.names
    assert cse.blinker_lat_hold.blocks_steer_disengage
  assert preap_blinker_pause_hides_controls_mismatch(
    brand="tesla", fingerprint="TESLA_MODEL_S_PREAP",
    blocks_steer_disengage=cse.blinker_lat_hold.blocks_steer_disengage)


def test_faster_corner_disengage_without_steering_pressed_does_not_cancel():
  cse = CarSpecificEvents(_cp())
  prev = _cs()
  for _ in range(50):
    cs = _cs(left=True, stalk=1)
    _events(cs, prev, cse=cse)
    prev = cs
  assert cse.blinker_lat_hold.turn_active
  events = _events(_cs(left=True, stalk=1, steering_disengage=True, steering_pressed=False),
                   prev, cse=cse)
  assert EventName.steerDisengage not in events.names
  assert cse.blinker_lat_hold.blocks_steer_disengage
  # Flash-gap dark frames while hands-on 2 still must not USER_DISABLE.
  events = _events(_cs(stalk=1, steering_disengage=True, steering_pressed=False),
                   _cs(left=True, stalk=1, steering_disengage=True), cse=cse)
  assert EventName.steerDisengage not in events.names
  assert cse.blinker_lat_hold.turn_active
