"""Blinker-lamp lateral pause: events, cruise, and Tesla FSM."""

from cereal import car, log
from opendbc.car.toyota.values import CAR as TOYOTA

from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import blinker_pauses_lateral
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
        cancel=False, cruise_enabled=True):
  cs = car.CarState.new_message()
  cs.cruiseState.available = True
  cs.cruiseState.enabled = cruise_enabled
  cs.gearShifter = "drive"
  cs.leftBlinker = left
  cs.rightBlinker = right
  cs.steeringDisengage = steering_disengage
  cs.steeringPressed = steering_pressed
  if cancel:
    be = car.CarState.ButtonEvent.new_message()
    be.type = ButtonType.cancel
    be.pressed = True
    cs.buttonEvents = [be]
  return cs


def _events(cs, cs_prev=None, cse=None):
  helper = cse or CarSpecificEvents(_cp())
  return helper.update(cs, cs_prev or _cs(), car.CarControl.new_message())


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


def test_steer_disengage_after_hand_release_still_fires():
  cse = CarSpecificEvents(_cp())
  _events(_cs(left=True, steering_disengage=True, steering_pressed=True),
          _cs(left=True), cse=cse)
  _events(_cs(), _cs(left=True, steering_disengage=True, steering_pressed=True), cse=cse)
  events = _events(_cs(steering_disengage=True, steering_pressed=True), _cs(), cse=cse)
  assert EventName.steerDisengage in events.names
