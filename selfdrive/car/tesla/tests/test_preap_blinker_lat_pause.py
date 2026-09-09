"""Pre-AP FSM: blinker-lamp pause must not tear down cruiseEnabled."""

from opendbc.car import Bus
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import (
  _peek_blinker_lamps,
  install_blinker_lat_pause,
)
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import LAMP_OFF_DEBOUNCE_S


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
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


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
  eng.process_buttons(
    cruise_buttons=CruiseButtons.CANCEL, prev_cruise_buttons=0,
    curr_time_ms=1000, v_ego=10.0, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=False)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def _buttons(eng, *, cruise_buttons=0, prev=0, t_ms=1000, brake=False):
  return eng.process_buttons(
    cruise_buttons=cruise_buttons, prev_cruise_buttons=prev,
    curr_time_ms=t_ms, v_ego=10.0, speed_units="KPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=brake)


def test_tesla_fsm_brake_drops_long_and_stays_off():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  _buttons(eng, brake=True, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, brake=False, t_ms=3000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tesla_fsm_blinker_brake_then_hand_then_stalk_for_long():
  """Justin sequence: lamp pauses lat, brake drops long (stays off),
  hand-steer does not kill cruise, lamp-off with hand on keeps cruise,
  hand release keeps cruise, stalk is required to restore long."""
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  _buttons(eng, brake=True, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, brake=False, t_ms=3000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng._nap_left_blinker = False
  eng._nap_steering_pressed = True
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tesla_fsm_holds_cruise_until_hand_release_after_lamp():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  eng._nap_left_blinker = False
  eng._nap_steering_pressed = True
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  # Turn complete (~1s dark), hand still on: cruise stays. Then release.
  eng._nap_lat_hold.update(False, False, True, engaged=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert not eng._nap_lat_hold.turn_active
  assert eng._nap_lat_hold.holding

  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def test_tesla_fsm_flash_gap_without_hand_keeps_cruise():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert eng.enableLongControl

  # Lamp dark between flashes, light/no hands, then a grab must not tear down.
  eng._nap_left_blinker = False
  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(False)
  assert eng._nap_lat_hold.turn_active
  assert eng.cruiseEnabled

  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tesla_fsm_1s_dark_then_steer_disengages():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled

  eng._nap_left_blinker = False
  eng._nap_lat_hold.update(False, False, False, engaged=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert not eng._nap_lat_hold.turn_active
  assert not eng._nap_lat_hold.holding

  eng.handle_steering_disengage(True)
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


def test_tesla_fsm_alc_lamps_do_not_latch_turn_or_drop_cruise():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_alc_active = True
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert not eng._nap_lat_hold.turn_active
  assert eng._nap_lat_hold.blocks_steer_disengage


def test_tesla_fsm_highway_stalk_tap_does_not_drop_cruise():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_stalk_state = 1
  eng._nap_v_ego = 30.0
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert not eng._nap_lat_hold.turn_active
