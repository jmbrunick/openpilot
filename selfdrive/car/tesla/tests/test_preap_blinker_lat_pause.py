"""Pre-AP FSM: blinker-lamp pause must not tear down cruiseEnabled.

A driver turn also drops enableLongControl. One SET after lamps/stalk go
idle restores long (double-pull first-pull is skipped). Tip ALC does not
drop long.
"""

from pathlib import Path

from opendbc.car import Bus
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import (
  _peek_blinker_lamps,
  _update_preap,
  hard_cancel_session,
  install_blinker_lat_pause,
  update_card_lat_handoff,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  EMERGENCY_DECEL_FRAMES,
  EMERGENCY_DECEL_MPS2,
  SOFT_YIELD_DEBOUNCE_FRAMES,
)
from openpilot.selfdrive.car.tesla import preap_blinker_lat_pause as pause_mod
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import LAMP_OFF_DEBOUNCE_S


def _engaged(*, double_pull=False):
  eng = PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)
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
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  # Lamp off, hands still on this frame: must not rising-edge tear down.
  # Long stays off until SET; cruise/lat latch is unchanged.
  eng._nap_left_blinker = False
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl


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
  assert not eng.enableLongControl

  eng._nap_left_blinker = False
  eng._nap_steering_pressed = True
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  # Turn complete (~1s dark), hand still on: cruise stays. Long stays off.
  # Lat resumes only after hand release (no SET). Then a later grab tears down.
  eng._nap_lat_hold.update(False, False, True, engaged=True, dt=LAMP_OFF_DEBOUNCE_S)
  assert not eng._nap_lat_hold.turn_active
  assert eng._nap_lat_hold.holding

  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl


def test_tesla_fsm_flash_gap_without_hand_keeps_cruise():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  # Lamp dark between flashes, light/no hands, then a grab must not tear down.
  eng._nap_left_blinker = False
  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(False)
  assert eng._nap_lat_hold.turn_active
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl


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


def test_tesla_fsm_faster_corner_hands_on_without_torque_threshold():
  """Hands-on 2 with torsion bar below STEER_THRESHOLD must not drop cruise."""
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_left_blinker = True
  eng._nap_steering_pressed = False
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng._nap_lat_hold.turn_active

  # Flash gap + 1s dark while still wrenching: still the same turn.
  eng._nap_left_blinker = False
  eng._nap_lat_hold.update(False, False, False, engaged=True,
                           steering_disengage=True, dt=LAMP_OFF_DEBOUNCE_S)
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng._nap_lat_hold.turn_active


def test_tesla_fsm_stalk_held_without_lamp_cache_keeps_cruise():
  """Hard gate: physical LEFT/RIGHT even if lamps/hold never latched."""
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_stalk_state = 1
  eng.handle_steering_disengage(True)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


class _FullParser:
  def __init__(self, *, left=False, right=False, stalk=1, hands=2, torque=0.0, speed=20.0):
    self.vl = {
      "GTW_carState": {"BC_indicatorLStatus": int(left), "BC_indicatorRStatus": int(right)},
      "EPAS_sysStatus": {"EPAS_torsionBarTorque": torque, "EPAS_handsOnLevel": hands},
      "STW_ACTN_RQ": {"TurnIndLvr_Stat": stalk},
      "ESP_B": {"ESP_vehicleSpeed": speed},
    }


def test_install_rewires_tesla_carstate_imported_update_preap():
  """card.py imports tesla.carstate before install; that binding must wrap."""
  from opendbc.car.tesla import carstate as tesla_carstate
  from opendbc.car.tesla.preap import carstate as preap_carstate

  install_blinker_lat_pause()
  assert tesla_carstate.update_preap is _update_preap
  assert preap_carstate.update_preap is _update_preap


def test_update_wrapper_feeds_handle_so_high_torque_keeps_cruise():
  """Live path: wrapper must set _nap_* before orig update calls handle."""
  install_blinker_lat_pause()
  eng = _engaged()
  called = {}

  def fake_orig(cs, parsers):
    called["yes"] = True
    cs.engagement.handle_steering_disengage(True)
    return "ok"

  real = pause_mod._ORIG_UPDATE
  pause_mod._ORIG_UPDATE = fake_orig
  try:
    cs = type("CS", (), {"engagement": eng})()
    parsers = {Bus.chassis: _FullParser(left=True, stalk=1, hands=2)}
    assert _update_preap(cs, parsers) == "ok"
    assert called["yes"]
    assert eng._nap_left_blinker
    assert eng._nap_stalk_state == 1
    assert eng.cruiseEnabled
    # First frame of LEFT+lamp is still the tip window; long drops once
    # the hold classifies a driver turn (0.40s) or lamps latch without a tip.
    assert eng.enableLongControl
  finally:
    pause_mod._ORIG_UPDATE = real


def _end_turn_latch(eng, *, pressed=False):
  """Advance past flash-latch (~1s dark). Lat may still be holding if pressed."""
  eng._nap_left_blinker = False
  eng._nap_right_blinker = False
  eng._nap_steering_pressed = pressed
  eng._nap_lat_hold.update(False, False, pressed, engaged=True, dt=LAMP_OFF_DEBOUNCE_S)
  eng.handle_steering_disengage(False)


def test_turn_blinker_drops_long_keeps_cruise():
  """Held/latched driver turn drops long in addition to pausing lat."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng.enableJustCC
  assert eng._nap_lat_hold.turn_active
  assert getattr(eng, "_nap_long_resume_pending", False)


def test_one_set_after_blinker_off_resumes_long_while_lat_paused():
  """After blinker/latch ends, one SET restores long even if lat is still paused."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(True)
  assert not eng.enableLongControl
  assert eng._nap_lat_hold.turn_active

  _end_turn_latch(eng, pressed=True)
  assert not eng._nap_lat_hold.turn_active
  assert eng._nap_lat_hold.holding
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=5000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert eng._nap_lat_hold.holding
  assert not getattr(eng, "_nap_long_resume_pending", False)


def test_one_set_after_blinker_off_resumes_long_when_lat_already_active():
  """After blinker/latch ends and hands are off, one SET still restores long."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert not eng.enableLongControl

  _end_turn_latch(eng, pressed=False)
  assert not eng._nap_lat_hold.turn_active
  assert not eng._nap_lat_hold.holding
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=5000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_set_during_active_turn_does_not_restore_long():
  """SET while a turn lamp is still showing must not bring long back."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert not eng.enableLongControl
  assert eng._nap_lat_hold.turn_active

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert getattr(eng, "_nap_long_resume_pending", False)


def test_one_set_after_lamps_dark_before_latch_expires():
  """Blinker off is enough for one SET. Do not wait out the ~1s flash latch."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert not eng.enableLongControl
  assert eng._nap_lat_hold.turn_active

  eng._nap_left_blinker = False
  eng.handle_steering_disengage(False)
  assert eng._nap_lat_hold.turn_active
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2500)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_tip_alc_does_not_drop_long():
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng._nap_alc_active = True
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert not eng._nap_lat_hold.turn_active


def test_stalk_tip_pending_does_not_drop_long():
  """LEFT/RIGHT inside the 0.40s tip window is ALC, not a turn."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_left_blinker = True
  eng._nap_stalk_state = 1
  eng.handle_steering_disengage(False)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert not eng._nap_lat_hold.turn_active


def test_double_pull_from_disengaged_still_needs_two_sets():
  """Resume-long one-pull must not skip initial double-pull engage."""
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1400)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def test_brake_drop_one_set_resumes_long_with_double_pull():
  """Same SET resume path after brake: one pull, not two."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  _buttons(eng, brake=True, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl

  _buttons(eng, brake=False, t_ms=3000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl


def _intent_then_brake(eng, *, a_ego, brake, frames=None, hands=1):
  if frames is None:
    frames = EMERGENCY_DECEL_FRAMES
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    update_card_lat_handoff(
      eng, engaged=True, lat_would_be_active=True,
      steering_torque=0.85, steering_rate_deg=20.0, hands_on_level=hands,
      brake_applied=False, a_ego=0.0, v_ego=15.0, param_on=True)
  canceled = False
  for _ in range(frames):
    canceled = update_card_lat_handoff(
      eng, engaged=bool(eng.cruiseEnabled), lat_would_be_active=True,
      steering_torque=0.2, steering_rate_deg=8.0, hands_on_level=hands,
      brake_applied=brake, a_ego=a_ego, v_ego=15.0, param_on=True)
  return canceled


def test_emergency_hard_brake_while_yielded_full_cancels_session():
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  canceled = _intent_then_brake(eng, a_ego=EMERGENCY_DECEL_MPS2, brake=True)
  assert canceled
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng.preap_cc_cancel_needed
  assert getattr(eng, "_nap_long_resume_pending", False) is False
  assert getattr(eng, "_nap_held_max_kph", None) is None


def test_light_brake_while_yielded_does_not_full_cancel():
  """Sticky-MAX silent pause stays the light-brake path."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  canceled = _intent_then_brake(eng, a_ego=-1.1, brake=True, frames=40)
  assert not canceled
  assert eng.cruiseEnabled
  _buttons(eng, brake=True, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert getattr(eng, "_nap_long_resume_pending", False)


def test_hard_cancel_session_is_not_silent_long_pause():
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  eng._nap_long_resume_pending = True
  eng._nap_held_max_kph = 88.5
  hard_cancel_session(eng)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl
  assert not getattr(eng, "_nap_long_resume_pending", False)
  assert getattr(eng, "_nap_held_max_kph", None) is None


def test_card_handoff_keeps_yield_during_driver_turn_blinker():
  """Soft-lat On must not treat lamp latch as lat-down (that forgot yield)."""
  install_blinker_lat_pause()
  eng = _engaged(double_pull=True)
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES):
    update_card_lat_handoff(
      eng, engaged=True, lat_would_be_active=True,
      steering_torque=0.85, steering_rate_deg=20.0, hands_on_level=1,
      brake_applied=False, a_ego=0.0, v_ego=15.0, param_on=True)
  assert eng._nap_lat_handoff._yielded
  # Driver-turn blinker + lat still requested: stay yielded, no identity reset.
  out = update_card_lat_handoff(
    eng, engaged=True, lat_would_be_active=True,
    steering_torque=0.2, steering_rate_deg=0.0, hands_on_level=1,
    brake_applied=False, a_ego=0.0, v_ego=15.0, param_on=True,
    blinker_paused=True)
  assert not out
  assert eng._nap_lat_handoff._yielded
  canceled = False
  for _ in range(EMERGENCY_DECEL_FRAMES):
    canceled = update_card_lat_handoff(
      eng, engaged=True, lat_would_be_active=True,
      steering_torque=0.2, steering_rate_deg=0.0, hands_on_level=1,
      brake_applied=True, a_ego=EMERGENCY_DECEL_MPS2, v_ego=15.0,
      param_on=True, blinker_paused=True)
  assert canceled
  assert not eng.cruiseEnabled


def test_card_update_preap_does_not_clear_lat_on_blinker_when_soft_lat_on():
  src = (Path(__file__).resolve().parents[4] /
         "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "lat_would_be_active=True" in src
  assert "hold.turn_active" in src
  assert "lat_would_be_active=not blinker_paused" not in src
