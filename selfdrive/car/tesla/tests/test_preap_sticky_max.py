"""Pre-AP sticky MAX: brake / turn long pause, one SET vs double SET.

Pedal mode. Soft lateral handoff is not this path.
"""

from openpilot.common.constants import CV
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import LAMP_OFF_DEBOUNCE_S
from openpilot.selfdrive.mapd.constants import MODE_FOLLOW, MODE_OFF
from openpilot.selfdrive.mapd.map_speed_policy import MapCruiseHold, decide_map_cruise
from openpilot.selfdrive.selfdrived.preap_regen import PreAPChimeState, update_preap_chimes


def _engaged(*, double_pull=True, pedal_kph=55.0 * CV.MPH_TO_KPH):
  eng = PreAPEngagement(double_pull_enabled=double_pull, double_pull_window_ms=750)
  eng.cruiseEnabled = True
  eng.enableLongControl = True
  eng.enableJustCC = False
  eng.pedal_speed_kph = float(pedal_kph)
  return eng


def _buttons(eng, *, cruise_buttons=0, prev=0, t_ms=1000, brake=False, v_ego=13.4):
  return eng.process_buttons(
    cruise_buttons=cruise_buttons, prev_cruise_buttons=prev,
    curr_time_ms=t_ms, v_ego=v_ego, speed_units="MPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=brake)


def _end_turn_latch(eng, *, pressed=False):
  eng._nap_left_blinker = False
  eng._nap_right_blinker = False
  eng._nap_steering_pressed = pressed
  eng._nap_lat_hold.update(False, False, pressed, engaged=True, dt=LAMP_OFF_DEBOUNCE_S)
  eng.handle_steering_disengage(False)


def test_brake_pause_keeps_held_max_and_session():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert getattr(eng, "_nap_long_resume_pending", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  assert abs(eng._nap_held_max_kph - held) < 1e-6


def test_one_set_at_standstill_does_not_take_long():
  """Stop + SET alone must not creep. Held MAX stays for a later gas touch."""
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert getattr(eng, "_nap_long_resume_pending", False)
  assert getattr(eng, "_nap_resume_wait_gas", False)
  assert not getattr(eng, "_nap_set_resume_long", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  assert abs(eng._nap_held_max_kph - held) < 1e-6


def test_one_set_at_standstill_then_gas_resumes_held_max():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert not eng.enableLongControl
  eng._nap_gas_pressed = True
  _buttons(eng, t_ms=4100, v_ego=0.0)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert not getattr(eng, "_nap_resume_wait_gas", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  assert abs(eng._nap_held_max_kph - held) < 1e-6


def test_one_set_while_rolling_resumes_without_gas():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=8.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=8.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=8.0)
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert not getattr(eng, "_nap_resume_wait_gas", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6


def test_standstill_set_with_gas_already_down_resumes():
  """SET and throttle on the same frame may take long."""
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  eng._nap_gas_pressed = True
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6


def test_double_set_at_standstill_still_takes_speed_now():
  """Second SET in the window at a stop is still forget-sticky / take now."""
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert getattr(eng, "_nap_resume_wait_gas", False)
  assert not eng.enableLongControl
  _buttons(eng, t_ms=4050, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4300, v_ego=0.0)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_take_speed_now", False)
  assert not getattr(eng, "_nap_resume_wait_gas", False)
  assert getattr(eng, "_nap_held_max_kph", None) is None


def test_hard_cancel_clears_standstill_resume_wait():
  from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import hard_cancel_session
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert getattr(eng, "_nap_resume_wait_gas", False)
  hard_cancel_session(eng)
  assert not getattr(eng, "_nap_resume_wait_gas", False)
  assert getattr(eng, "_nap_held_max_kph", None) is None


def test_one_set_after_brake_resumes_long_keeps_held_max():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000)
  _buttons(eng, brake=False, t_ms=3000, v_ego=13.4)  # ~30 mph
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=13.4)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert not getattr(eng, "_nap_set_take_speed_now", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6


def test_double_set_after_brake_forgets_held_and_takes_speed_now():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000)
  _buttons(eng, brake=False, t_ms=3000, v_ego=18.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=18.0)
  assert getattr(eng, "_nap_set_resume_long", False)
  _buttons(eng, t_ms=4050, v_ego=18.0)  # release
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4300, v_ego=18.0)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_take_speed_now", False)
  assert not getattr(eng, "_nap_set_resume_long", False)


def test_set_while_long_on_does_not_drop_long():
  """In-session first SET arms double-SET; it must not first-pull drop long."""
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2000)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert not getattr(eng, "_nap_set_take_speed_now", False)
  assert not getattr(eng, "_nap_set_resume_long", False)


def test_blinker_turn_one_set_resumes_held_max():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  assert not eng.enableLongControl
  assert eng.cruiseEnabled
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  _end_turn_latch(eng, pressed=True)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=5000, v_ego=10.0)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  assert eng._nap_lat_hold.holding


def test_tip_alc_does_not_use_long_pause_path():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  eng._nap_left_blinker = True
  eng._nap_alc_active = True
  eng.handle_steering_disengage(False)
  assert eng.enableLongControl
  assert not getattr(eng, "_nap_long_resume_pending", False)
  assert not getattr(eng, "_nap_set_resume_long", False)


def test_initial_double_pull_engage_is_take_speed_now():
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000, v_ego=24.6)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert not getattr(eng, "_nap_set_take_speed_now", False)
  _buttons(eng, t_ms=1050, v_ego=24.6)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1400, v_ego=24.6)
  assert eng.cruiseEnabled
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_take_speed_now", False)
  assert not getattr(eng, "_nap_set_resume_long", False)


def test_cancel_full_disengage_clears_held_max():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  eng = _engaged(pedal_kph=held)
  _buttons(eng, cruise_buttons=CruiseButtons.CANCEL, t_ms=2000)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl
  assert not getattr(eng, "_nap_long_resume_pending", False)
  assert getattr(eng, "_nap_held_max_kph", None) is None
  assert eng.pedal_speed_kph == 0.0


def test_steering_disengage_full_teardown_clears_held_max():
  install_blinker_lat_pause()
  eng = _engaged()
  eng.handle_steering_disengage(True)
  assert not eng.cruiseEnabled
  assert getattr(eng, "_nap_held_max_kph", None) is None


def _overlay(hold, eng, *, posted, mode=MODE_FOLLOW, traveled_kph=None, long_active=None):
  if long_active is None:
    long_active = bool(eng.enableLongControl)
  resume = bool(getattr(eng, "_nap_set_resume_long", False))
  take = bool(getattr(eng, "_nap_set_take_speed_now", False))
  raw = float(eng.pedal_speed_kph)
  if not long_active and hold.held_max_kph is not None:
    raw = float(hold.held_max_kph)
  return decide_map_cruise(
    hold, engaged=bool(eng.cruiseEnabled), mode=mode, raw_kph=raw,
    posted_kph=posted, engage_rising=False, now=0.0,
    take_speed_now=take, resume_held=resume,
    traveled_kph=traveled_kph, long_active=long_active,
  )


def test_fsm_plus_policy_brake_keeps_sticky_55_in_65():
  install_blinker_lat_pause()
  posted = 65 * CV.MPH_TO_KPH
  sticky = 55 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  eng = _engaged(pedal_kph=sticky)
  _buttons(eng, brake=True, t_ms=2000, v_ego=13.4)
  dec = _overlay(hold, eng, posted=posted, traveled_kph=30 * CV.MPH_TO_KPH)
  assert dec.sticky
  assert abs(dec.driver_kph - sticky) < 1e-6
  _buttons(eng, brake=False, t_ms=3000, v_ego=13.4)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=13.4)
  dec = _overlay(hold, eng, posted=posted, traveled_kph=30 * CV.MPH_TO_KPH)
  assert dec.sticky
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - sticky) < 1e-6


def test_fsm_plus_policy_no_map_double_set_uses_current_speed():
  install_blinker_lat_pause()
  held = 55 * CV.MPH_TO_KPH
  now_speed = 42 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=None,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=held,
  )
  eng = _engaged(pedal_kph=held)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2000, v_ego=now_speed * CV.KPH_TO_MS)
  _buttons(eng, t_ms=2050, v_ego=now_speed * CV.KPH_TO_MS)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2300, v_ego=now_speed * CV.KPH_TO_MS)
  dec = _overlay(hold, eng, posted=None, mode=MODE_OFF, traveled_kph=now_speed)
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - now_speed) < 1e-6


def test_fsm_plus_policy_mode_off_gravel_one_set_keeps_19():
  """Maps off, held 19, brake (ego 12), one SET then delayed engage_rising."""
  install_blinker_lat_pause()
  held = 19 * CV.MPH_TO_KPH
  ego = 12 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=None,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=held,
  )
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=ego * CV.KPH_TO_MS)
  dec = _overlay(hold, eng, posted=None, mode=MODE_OFF, traveled_kph=ego)
  assert abs(dec.driver_kph - held) < 1e-6
  assert abs(hold.held_max_kph - held) < 1e-6
  _buttons(eng, brake=False, t_ms=3000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=ego * CV.KPH_TO_MS)
  assert getattr(eng, "_nap_set_resume_long", False)
  assert not getattr(eng, "_nap_set_take_speed_now", False)
  assert abs(eng.pedal_speed_kph - held) < 1e-6
  # SET frame: resume_held, pedalLongActive not up yet (engage_rising False).
  dec = _overlay(hold, eng, posted=None, mode=MODE_OFF, traveled_kph=ego)
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - held) < 1e-6
  # Next card cycle: resume flag consumed, pedal authority rising.
  eng._nap_set_resume_long = False
  dec = decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=None,
    engage_rising=True, now=5.0, resume_held=False, long_active=True,
    traveled_kph=ego,
  )
  assert abs(hold.held_max_kph - held) < 1e-6
  assert abs(dec.driver_kph - held) < 1e-6
  if dec.seed_kph is not None:
    assert abs(dec.seed_kph - held) < 1e-6


def test_fsm_plus_policy_mode_off_stalk_then_resume_adjusted():
  install_blinker_lat_pause()
  start = 24 * CV.MPH_TO_KPH
  adjusted = 19 * CV.MPH_TO_KPH
  ego = 12 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=start, posted_kph=None,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=start,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=start, posted_kph=None,
    engage_rising=False, now=1.0, long_active=True,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=adjusted, posted_kph=None,
    engage_rising=False, now=2.0, long_active=True,
  )
  assert abs(hold.held_max_kph - adjusted) < 1e-6
  eng = _engaged(pedal_kph=adjusted)
  _buttons(eng, brake=True, t_ms=2000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, brake=False, t_ms=3000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=ego * CV.KPH_TO_MS)
  dec = _overlay(hold, eng, posted=None, mode=MODE_OFF, traveled_kph=ego)
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - adjusted) < 1e-6


def test_fsm_plus_policy_mode_off_double_set_after_pause_takes_traveled():
  install_blinker_lat_pause()
  held = 19 * CV.MPH_TO_KPH
  ego = 12 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_OFF, raw_kph=held, posted_kph=None,
    engage_rising=True, now=0.0, take_speed_now=True, traveled_kph=held,
  )
  eng = _engaged(pedal_kph=held)
  _buttons(eng, brake=True, t_ms=2000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, brake=False, t_ms=3000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, t_ms=4050, v_ego=ego * CV.KPH_TO_MS)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4300, v_ego=ego * CV.KPH_TO_MS)
  dec = _overlay(hold, eng, posted=None, mode=MODE_OFF, traveled_kph=ego)
  assert getattr(eng, "_nap_set_take_speed_now", False)
  assert dec.seed_kph is not None
  assert abs(dec.seed_kph - ego) < 1e-6


def test_fsm_plus_policy_maps_double_set_uses_posted():
  install_blinker_lat_pause()
  posted = 65 * CV.MPH_TO_KPH
  sticky = 55 * CV.MPH_TO_KPH
  hold = MapCruiseHold()
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=posted, posted_kph=posted,
    engage_rising=True, now=0.0,
  )
  decide_map_cruise(
    hold, engaged=True, mode=MODE_FOLLOW, raw_kph=sticky, posted_kph=posted,
    engage_rising=False, now=1.0, stalk_pressed=True,
  )
  eng = _engaged(pedal_kph=sticky)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2000)
  _buttons(eng, t_ms=2050)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=2300)
  dec = _overlay(hold, eng, posted=posted, traveled_kph=40 * CV.MPH_TO_KPH)
  assert not dec.sticky
  assert abs(dec.seed_kph - posted) < 1e-6


def _chime_for(eng, prev):
  return update_preap_chimes(
    lat_engaged=bool(eng.cruiseEnabled),
    long_engaged=bool(eng.enableLongControl),
    prev=prev,
  )


def test_brake_pause_does_not_chime_disengage_or_resume_fanfare():
  install_blinker_lat_pause()
  eng = _engaged()
  prev = PreAPChimeState(lat_engaged=True, long_engaged=True)
  _buttons(eng, brake=True, t_ms=2000)
  chimes, prev = _chime_for(eng, prev)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert not chimes.long_disengage
  assert not chimes.long_engage
  assert prev.long_paused

  _buttons(eng, brake=False, t_ms=3000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  chimes, prev = _chime_for(eng, prev)
  assert eng.enableLongControl
  assert getattr(eng, "_nap_set_resume_long", False)
  assert not chimes.long_engage
  assert not chimes.long_disengage
  assert not prev.long_paused


def test_blinker_turn_pause_does_not_chime_disengage():
  install_blinker_lat_pause()
  eng = _engaged()
  prev = PreAPChimeState(lat_engaged=True, long_engaged=True)
  eng._nap_left_blinker = True
  eng.handle_steering_disengage(False)
  chimes, prev = _chime_for(eng, prev)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert not chimes.long_disengage
  assert prev.long_paused


def test_cancel_after_engaged_still_chimes_long_disengage():
  install_blinker_lat_pause()
  eng = _engaged()
  prev = PreAPChimeState(lat_engaged=True, long_engaged=True)
  _buttons(eng, cruise_buttons=CruiseButtons.CANCEL, t_ms=2000)
  chimes, _ = _chime_for(eng, prev)
  assert not eng.cruiseEnabled
  assert not eng.enableLongControl
  assert chimes.long_disengage
  assert chimes.lat_disengage


def test_handoff_module_does_not_drop_long():
  """#71 soft lat handoff must not share the brake/turn long-pause path."""
  from pathlib import Path
  src = (Path(__file__).resolve().parents[4] / "selfdrive/controls/lib/driver_lateral_handoff.py").read_text()
  assert "_drop_longitudinal_keep_lateral" not in src
  assert "_nap_long_resume_pending" not in src
  assert "_nap_set_resume_long" not in src
  assert "emergency_cancel" in src
  pause = (Path(__file__).resolve().parents[4] /
           "selfdrive/car/tesla/preap_blinker_lat_pause.py").read_text()
  assert "hard_cancel_session" in pause
  assert "_drop_longitudinal_keep_lateral" in pause
  assert "RESUME_STANDSTILL_V_EGO" in pause
  assert "_nap_resume_wait_gas" in pause
