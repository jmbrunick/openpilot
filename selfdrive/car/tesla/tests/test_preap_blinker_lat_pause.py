"""Pre-AP FSM: blinker-lamp pause must not tear down cruiseEnabled."""

from opendbc.car import Bus
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.common.constants import CV
from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import (
  _peek_blinker_lamps,
  install_blinker_lat_pause,
)

TURN_SPEED = 15 * CV.MPH_TO_MS
HIGHWAY_SPEED = 30 * CV.MPH_TO_MS


def _engaged():
  eng = PreAPEngagement(double_pull_enabled=False, double_pull_window_ms=750)
  eng.cruiseEnabled = True
  eng.enableLongControl = True
  eng.enableJustCC = False
  return eng


def test_tesla_fsm_keeps_cruise_on_steer_during_one_lamp():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_right_blinker = False
  eng._nap_v_ego = TURN_SPEED
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tesla_fsm_highway_blinker_without_junction_still_tears_down():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_right_blinker = False
  eng._nap_v_ego = HIGHWAY_SPEED
  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def test_tesla_fsm_hazards_still_tear_down():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_right_blinker = True
  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def test_tesla_fsm_resumes_without_new_engage_after_lamp_clears():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_right_blinker = False
  eng._nap_v_ego = TURN_SPEED
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  # Lamp off, hands still on this frame: must not rising-edge tear down.
  eng._nap_left_blinker = False
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tesla_fsm_no_lamp_still_tears_down():
  install_blinker_lat_pause()
  eng = _engaged()
  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def test_tesla_fsm_cancel_still_works_during_pause():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_v_ego = TURN_SPEED
  eng.process_buttons(
    cruise_buttons=CruiseButtons.CANCEL, prev_cruise_buttons=0,
    curr_time_ms=1000, v_ego=10.0, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


class _Parser:
  def __init__(self, left, right):
    self.vl = {"GTW_carState": {"BC_indicatorLStatus": int(left), "BC_indicatorRStatus": int(right)}}


def test_peek_uses_lamp_bits_not_stalk():
  left, right = _peek_blinker_lamps({Bus.chassis: _Parser(True, False)})
  assert left and not right
  left, right = _peek_blinker_lamps({Bus.chassis: _Parser(True, True)})
  assert left and right
  left, right = _peek_blinker_lamps({})
  assert not left and not right
