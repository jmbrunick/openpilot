"""DAS_bodyControls wiper / high-low beam test. Default off matches stock teslacan."""
from opendbc.can import CANPacker
from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS

from openpilot.selfdrive.car.tesla.preap_body_controls import (
  BEAM_SETTING_HIGH,
  BEAM_SETTING_LOW,
  BEAM_SETTING_OFF,
  DAS_HIGH_BEAM_OFF,
  DAS_HIGH_BEAM_ON,
  DAS_HIGH_BEAM_UNDECIDED,
  DAS_WIPER_INTERMITTENT,
  DAS_WIPER_OFF,
  DAS_WIPER_ON,
  NAP_HIGH_LOW_BEAM,
  NAP_WIPER_SPEED,
  WIPER_SETTING_INTERMITTENT,
  WIPER_SETTING_OFF,
  WIPER_SETTING_ON,
  create_body_controls_message,
  das_high_low_beam_for_setting,
  das_wiper_speed_for_setting,
  install_body_controls_test,
  original_create_body_controls_message,
  register_nap_body_params,
)


def _tc():
  packer = CANPacker("tesla_preap")
  return TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})


def _wiper(dat: bytes) -> int:
  return (dat[0] >> 4) & 0x0F


def _headlight(dat: bytes) -> int:
  return dat[0] & 0x03


def _turn(dat: bytes) -> int:
  return dat[1] & 0x03


def _beam(dat: bytes) -> int:
  return (dat[1] >> 2) & 0x03


def test_setting_maps_to_das_values():
  assert das_wiper_speed_for_setting(WIPER_SETTING_OFF) == DAS_WIPER_OFF
  assert das_wiper_speed_for_setting(WIPER_SETTING_INTERMITTENT) == DAS_WIPER_INTERMITTENT
  assert das_wiper_speed_for_setting(WIPER_SETTING_ON) == DAS_WIPER_ON
  assert das_high_low_beam_for_setting(BEAM_SETTING_OFF) == DAS_HIGH_BEAM_UNDECIDED
  assert das_high_low_beam_for_setting(BEAM_SETTING_LOW) == DAS_HIGH_BEAM_OFF
  assert das_high_low_beam_for_setting(BEAM_SETTING_HIGH) == DAS_HIGH_BEAM_ON


def test_default_off_matches_stock_blinker_frame():
  tc = _tc()
  stock_fn = original_create_body_controls_message()
  for turn in (0, 1, 2):
    stock = stock_fn(tc, turn, 0, CANBUS.party, 1)
    test = create_body_controls_message(tc, turn, 0, CANBUS.party, 1, wiper_speed=0, high_low_beam=0)
    assert stock[0] == test[0] == 0x3E9
    assert stock[1] == test[1]
    assert stock[2] == test[2] == CANBUS.party


def test_wiper_on_sends_nonzero_speed_and_zero_when_off():
  tc = _tc()
  _, dat_int, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 2, wiper_speed=DAS_WIPER_INTERMITTENT, high_low_beam=0)
  _, dat_on, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 2, wiper_speed=DAS_WIPER_ON, high_low_beam=0)
  _, dat_off, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 2, wiper_speed=0, high_low_beam=0)
  assert _wiper(dat_int) == DAS_WIPER_INTERMITTENT
  assert _wiper(dat_on) == DAS_WIPER_ON
  assert _wiper(dat_off) == 0
  assert _headlight(dat_int) == _headlight(dat_on) == _headlight(dat_off) == 0


def test_high_low_beam_sends_low_or_high_not_flash():
  tc = _tc()
  _, dat_low, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 3, wiper_speed=0, high_low_beam=DAS_HIGH_BEAM_OFF)
  _, dat_high, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 3, wiper_speed=0, high_low_beam=DAS_HIGH_BEAM_ON)
  _, dat_off, _ = create_body_controls_message(
    tc, 0, 0, CANBUS.party, 3, wiper_speed=0, high_low_beam=0)
  assert _beam(dat_low) == DAS_HIGH_BEAM_OFF
  assert _beam(dat_high) == DAS_HIGH_BEAM_ON
  assert _beam(dat_off) == DAS_HIGH_BEAM_UNDECIDED
  assert _headlight(dat_low) == _headlight(dat_high) == 0


def test_blinker_field_preserved_with_wiper_and_beam():
  tc = _tc()
  _, dat_left, _ = create_body_controls_message(
    tc, 1, 0, CANBUS.party, 4, wiper_speed=DAS_WIPER_ON, high_low_beam=DAS_HIGH_BEAM_ON)
  _, dat_right, _ = create_body_controls_message(
    tc, 2, 0, CANBUS.party, 4, wiper_speed=DAS_WIPER_INTERMITTENT, high_low_beam=DAS_HIGH_BEAM_OFF)
  assert _turn(dat_left) == 1
  assert _turn(dat_right) == 2
  assert dat_left[2] & 0x0F == 1
  assert dat_right[2] & 0x0F == 1


def test_card_call_default_off_matches_stock(monkeypatch):
  monkeypatch.setattr(
    "openpilot.selfdrive.car.tesla.preap_body_controls.requested_das_wiper_speed", lambda: 0)
  monkeypatch.setattr(
    "openpilot.selfdrive.car.tesla.preap_body_controls.requested_das_high_low_beam", lambda: 0)
  tc = _tc()
  stock = original_create_body_controls_message()(tc, 1, 0, CANBUS.party, 5)
  # Same positional args carcontroller uses today.
  test = create_body_controls_message(tc, 1, 0, CANBUS.party, 5)
  assert stock[1] == test[1]


def test_card_call_settings_on_sets_fields_keeps_blinker(monkeypatch):
  monkeypatch.setattr(
    "openpilot.selfdrive.car.tesla.preap_body_controls.requested_das_wiper_speed",
    lambda: DAS_WIPER_INTERMITTENT)
  monkeypatch.setattr(
    "openpilot.selfdrive.car.tesla.preap_body_controls.requested_das_high_low_beam",
    lambda: DAS_HIGH_BEAM_ON)
  tc = _tc()
  _, dat, _ = create_body_controls_message(tc, 2, 0, CANBUS.party, 5)
  assert _wiper(dat) == DAS_WIPER_INTERMITTENT
  assert _beam(dat) == DAS_HIGH_BEAM_ON
  assert _turn(dat) == 2
  assert _headlight(dat) == 0


def test_install_and_register_defaults():
  register_nap_body_params()
  assert NAPParamKeys.WIPER_SPEED == NAP_WIPER_SPEED
  assert NAPParamKeys.HIGH_LOW_BEAM == NAP_HIGH_LOW_BEAM
  assert DEFAULTS[NAP_WIPER_SPEED] == 0
  assert DEFAULTS[NAP_HIGH_LOW_BEAM] == 0
  install_body_controls_test()
  tc = _tc()
  _, dat, _ = tc.create_body_controls_message(
    1, 0, CANBUS.party, 1, wiper_speed=DAS_WIPER_ON, high_low_beam=DAS_HIGH_BEAM_OFF)
  assert _wiper(dat) == DAS_WIPER_ON
  assert _beam(dat) == DAS_HIGH_BEAM_OFF
  assert _turn(dat) == 1
