"""Blinker-lamp lateral pause: events, cruise, and Tesla FSM."""

from cereal import car, log
from opendbc.car.toyota.values import CAR as TOYOTA

from openpilot.common.constants import CV
from openpilot.selfdrive.car.car_specific import CarSpecificEvents
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import blinker_pauses_lateral
from openpilot.selfdrive.selfdrived.events import ET, EVENTS

TURN_SPEED = 15 * CV.MPH_TO_MS
HIGHWAY_SPEED = 30 * CV.MPH_TO_MS


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
        cancel=False, cruise_enabled=True, v_ego=TURN_SPEED):
  cs = car.CarState.new_message()
  cs.cruiseState.available = True
  cs.cruiseState.enabled = cruise_enabled
  cs.gearShifter = "drive"
  cs.vEgo = v_ego
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


def _events(cs, cs_prev=None):
  return CarSpecificEvents(_cp()).update(cs, cs_prev or _cs(), car.CarControl.new_message())


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


def test_highway_blinker_steer_disengage_still_fires():
  events = _events(_cs(left=True, steering_disengage=True, v_ego=HIGHWAY_SPEED),
                   _cs(left=True, v_ego=HIGHWAY_SPEED))
  assert EventName.steerDisengage in events.names


def test_blinker_pause_predicate():
  assert blinker_pauses_lateral(True, False, TURN_SPEED)
  assert blinker_pauses_lateral(False, True, TURN_SPEED)
  assert not blinker_pauses_lateral(True, False, HIGHWAY_SPEED)
  assert not blinker_pauses_lateral(True, True, TURN_SPEED)
  assert not blinker_pauses_lateral(False, False, TURN_SPEED)
