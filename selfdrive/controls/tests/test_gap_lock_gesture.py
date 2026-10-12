"""2 s engage-stalk hold. Long may be on, paused, or not yet taken.

In-session CANCEL TX does not drop lateral. opendbc safety
`tesla_preap_tx.h` applies `pcm_cruise_check(false)` for lever == CANCEL
only when `!controls_allowed`. The Python swallow keeps cruiseEnabled and
enableLongControl through the hold. The repeated CANCEL is only the
stock-CC spoof (`gap_lock_cancel_hold_tx`), and it does not reset the
spoofer delay.
"""

from pathlib import Path

from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.values import CruiseButtons

from openpilot.selfdrive.car.tesla.preap_blinker_lat_pause import install_blinker_lat_pause
from openpilot.selfdrive.controls.lib.gap_lock import gap_lock_arm_seq, gap_lock_cancel_hold_tx

ROOT = Path(__file__).resolve().parents[3]


def _engaged():
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng.cruiseEnabled = True
  eng.enableLongControl = True
  eng.enableJustCC = False
  eng.pedal_speed_kph = 88.5
  eng._nap_gap_lock_enabled_override = True
  eng._nap_can_valid = True
  return eng


def _buttons(eng, *, cruise_buttons=0, prev=0, t_ms=1000, brake=False, v_ego=13.4):
  return eng.process_buttons(
    cruise_buttons=cruise_buttons, prev_cruise_buttons=prev,
    curr_time_ms=t_ms, v_ego=v_ego, speed_units="MPH",
    use_pedal=True, pedal_long_allowed=True,
    long_control_allowed=True, real_brake_pressed=brake)


def _seq(eng) -> int:
  return gap_lock_arm_seq(eng)


def test_two_seconds_arms_once_and_keeps_long():
  install_blinker_lat_pause()
  eng = _engaged()
  speed = eng.pedal_speed_kph
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=10000)
  assert _seq(eng) == 0
  assert eng.stalk_pull_time_ms == 10000
  assert eng.enableLongControl and eng.cruiseEnabled
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=11900)
  assert _seq(eng) == 0
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=12000)
  assert _seq(eng) == 1
  assert eng.pedal_speed_kph == speed
  assert eng.enableLongControl and eng.cruiseEnabled
  assert not eng.preap_cc_cancel_needed
  assert eng._nap_gap_lock.cancel_hold
  assert eng._nap_gap_lock.holding
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=13000)
  assert _seq(eng) == 1
  _buttons(eng, cruise_buttons=0, prev=CruiseButtons.MAIN, t_ms=13100)
  assert not eng._nap_set_take_speed_now
  assert eng.enableLongControl and eng.cruiseEnabled
  assert eng.stalk_pull_time_ms == 10000
  assert not eng._nap_gap_lock.cancel_hold
  assert not eng._nap_gap_lock.holding
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=20000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=22000)
  assert _seq(eng) == 2


def test_release_before_two_seconds_does_not_add():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, prev=CruiseButtons.MAIN, t_ms=11000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=12000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=13900)
  assert _seq(eng) == 0


def test_cancel_hold_starts_at_half_a_second():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=10499)
  assert not eng._nap_gap_lock.cancel_hold
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=10500)
  assert eng._nap_gap_lock.cancel_hold
  assert eng.preap_last_cc_spoof_ms == 10500
  assert gap_lock_cancel_hold_tx(False, True, 10)
  assert not gap_lock_cancel_hold_tx(True, True, 10)
  assert not gap_lock_cancel_hold_tx(False, True, 11)
  assert not gap_lock_cancel_hold_tx(False, False, 10)


def test_disengaged_hold_arms_and_takes_long():
  """Rolling single pull engages lat+long; the 2 s hold still gap-locks."""
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng._nap_gap_lock_enabled_override = True
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000)
  assert eng.cruiseEnabled and eng.enableLongControl
  assert eng.stalk_pull_time_ms == 1000
  assert eng._nap_set_take_speed_now
  assert eng.pedal_speed_kph > 0
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=2999)
  assert _seq(eng) == 0
  assert eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=3000)
  assert _seq(eng) == 1
  assert eng.enableLongControl and eng.cruiseEnabled
  assert eng.pedal_speed_kph > 0


def test_disengaged_tap_engages_lat_and_long():
  """A tap from off while rolling is full engage, not lat-only."""
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng._nap_gap_lock_enabled_override = True
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000)
  assert eng.cruiseEnabled and eng.enableLongControl
  assert eng.stalk_pull_time_ms == 1000
  assert eng._nap_set_take_speed_now
  assert eng.pedal_speed_kph > 0
  _buttons(eng, prev=CruiseButtons.MAIN, t_ms=1500)
  assert eng.cruiseEnabled and eng.enableLongControl
  assert _seq(eng) == 0


def test_gap_lock_hold_from_off_at_a_stop_still_arms():
  """Foot-off pull at a stop waits for gas. A 2 s hold still takes long."""
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng._nap_gap_lock_enabled_override = True
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000, v_ego=0.0)
  assert eng.cruiseEnabled
  assert not eng.enableLongControl
  assert eng._nap_resume_wait_gas
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=2999, v_ego=0.0)
  assert _seq(eng) == 0
  assert not eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=3000, v_ego=0.0)
  assert _seq(eng) == 1
  assert eng.enableLongControl and eng.cruiseEnabled
  assert not eng._nap_resume_wait_gas


def test_resume_from_brake_hold_arms():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, brake=True, t_ms=2000)
  _buttons(eng, brake=False, t_ms=3000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  assert eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=5999)
  assert _seq(eng) == 0
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=6000)
  assert _seq(eng) == 1
  assert eng.enableLongControl


def test_one_pedal_resume_hold_arms():
  install_blinker_lat_pause()
  eng = _engaged()
  eng.enableLongControl = False
  eng._one_pedal_pause_latched = True
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000)
  assert eng.enableLongControl
  assert not eng._one_pedal_pause_latched
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=6000)
  assert _seq(eng) == 1
  assert eng.enableLongControl


def test_engage_while_gas_hold_arms():
  install_blinker_lat_pause()
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  eng._nap_gap_lock_enabled_override = True
  eng._nap_di_pedal_pos = 10.0
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=1000)
  assert eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=2999)
  assert _seq(eng) == 0
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=3000)
  assert _seq(eng) == 1
  assert eng.enableLongControl


def test_standstill_hold_arms_and_takes_long():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, brake=True, t_ms=2000, v_ego=0.0)
  _buttons(eng, brake=False, t_ms=3000, v_ego=0.0)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=4000, v_ego=0.0)
  assert not eng.enableLongControl
  assert eng._nap_resume_wait_gas
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=5999, v_ego=0.0)
  assert _seq(eng) == 0
  assert not eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=6000, v_ego=0.0)
  assert _seq(eng) == 1
  assert eng.enableLongControl
  assert not eng._nap_resume_wait_gas


def test_long_dropped_during_hold_still_arms():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  eng.enableLongControl = False
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=12000)
  assert _seq(eng) == 1
  assert eng.enableLongControl and eng.cruiseEnabled


def test_brake_during_hold_does_not_arm():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, brake=True, t_ms=12000)
  assert not eng.enableLongControl
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=13000)
  assert _seq(eng) == 0


def test_param_off_does_not_arm():
  install_blinker_lat_pause()
  eng = _engaged()
  eng._nap_gap_lock_enabled_override = False
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=13000)
  assert _seq(eng) == 0
  assert eng.enableLongControl


def test_cancel_and_can_invalid_abort():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, cruise_buttons=CruiseButtons.CANCEL, prev=CruiseButtons.MAIN, t_ms=11000)
  assert not eng.cruiseEnabled
  assert _seq(eng) == 0

  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  eng._nap_can_valid = False
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=CruiseButtons.MAIN, t_ms=13000)
  assert _seq(eng) == 0
  assert eng.enableLongControl


def test_second_pull_inside_the_window_is_still_take_speed_now():
  install_blinker_lat_pause()
  eng = _engaged()
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, t_ms=10000)
  _buttons(eng, prev=CruiseButtons.MAIN, t_ms=10100)
  _buttons(eng, cruise_buttons=CruiseButtons.MAIN, prev=0, t_ms=10200)
  assert eng._nap_set_take_speed_now
  assert eng.enableLongControl
  assert _seq(eng) == 0


def test_in_session_cancel_tx_does_not_drop_lateral():
  header = (ROOT / "opendbc_repo/opendbc/safety/modes/tesla_preap_tx.h").read_text()
  assert "(lever == 1) && !controls_allowed" in header
