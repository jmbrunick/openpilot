"""0x45 stalk wiper / high-beam test. Off matches today's forwarded stalk."""
from openpilot.selfdrive.car.tesla.preap_body_controls import (
  BEAM_SETTING_HIGH,
  BEAM_SETTING_LOW,
  BEAM_SETTING_OFF,
  NAP_HIGH_LOW_BEAM,
  NAP_WIPER_SPEED,
  STW_ACTN_RQ_ADDR,
  STW_HIBM_MASK,
  STW_HIGH_BEAM,
  STW_HIGH_BEAM_FLASH,
  STW_TURN_MASK,
  STW_TX_ECHO_BUS,
  STW_WASHER_SPRAY,
  STW_WIPER_BEAM_BYTE,
  STW_WIPER_ON,
  WIPER_SETTING_INTERMITTENT,
  WIPER_SETTING_OFF,
  WIPER_SETTING_ON,
  apply_stw_wiper_beam_nibbles,
  extra_stw_forward_needed,
  high_beam_test_requested,
  hibm_nibble,
  live_stw_counter,
  overlay_stw_wiper_beam,
  panda_can_drop_live_stw,
  register_nap_body_params,
  replace_relayed_stw,
  send_replaced_live_stw,
  stalk_test_active,
  wiper_test_requested,
)


def _rest() -> bytes:
  # Justin's parked capture, counter/checksum ignored for packing.
  return bytes.fromhex("00ff000000000000")


def _byte(dat: bytes) -> int:
  return dat[STW_WIPER_BEAM_BYTE]


def test_setting_maps_to_stalk_test_flags():
  assert not wiper_test_requested(WIPER_SETTING_OFF)
  assert wiper_test_requested(WIPER_SETTING_INTERMITTENT)
  assert wiper_test_requested(WIPER_SETTING_ON)
  assert not high_beam_test_requested(BEAM_SETTING_OFF)
  assert not high_beam_test_requested(BEAM_SETTING_LOW)
  assert high_beam_test_requested(BEAM_SETTING_HIGH)


def test_byte_packing_rest_wiper_high_both_never_spray():
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, False)) == STW_WIPER_ON
  assert _byte(apply_stw_wiper_beam_nibbles(rest, False, True)) == STW_HIGH_BEAM
  assert _byte(apply_stw_wiper_beam_nibbles(rest, True, True)) == (STW_WIPER_ON | STW_HIGH_BEAM)

  packed = [
    apply_stw_wiper_beam_nibbles(rest, False, False),
    apply_stw_wiper_beam_nibbles(rest, True, False),
    apply_stw_wiper_beam_nibbles(rest, False, True),
    apply_stw_wiper_beam_nibbles(rest, True, True),
  ]
  assert all(_byte(dat) != STW_WASHER_SPRAY for dat in packed)
  assert all((_byte(dat) & 0x0F) != STW_HIGH_BEAM_FLASH for dat in packed)
  assert all(dat[:2] == rest[:2] and dat[3:] == rest[3:] for dat in packed)


def test_off_leaves_real_stalk_nibbles_alone():
  held_wiper = bytes.fromhex("00ff100000000000")
  held_high = bytes.fromhex("00ff040000000000")
  held_both = bytes.fromhex("00ff140000000000")
  spray = bytes.fromhex("00ff200000000000")
  assert apply_stw_wiper_beam_nibbles(held_wiper, False, False) == held_wiper
  assert apply_stw_wiper_beam_nibbles(held_high, False, False) == held_high
  assert apply_stw_wiper_beam_nibbles(held_both, False, False) == held_both
  # Off must not force 0 over a stalk the driver is holding, including spray.
  assert apply_stw_wiper_beam_nibbles(spray, False, False) == spray
  # On replaces spray with wiper 1 instead of sending 2.
  assert _byte(apply_stw_wiper_beam_nibbles(spray, True, False)) == STW_WIPER_ON


def test_overlay_off_is_identity_including_crc():
  rest = _rest()
  assert overlay_stw_wiper_beam(rest, False, False, crc_fn=lambda _: 0xAA) == rest


def test_overlay_resigns_crc_only_when_changed():
  rest = _rest()
  out = overlay_stw_wiper_beam(rest, True, False, crc_fn=lambda payload: sum(payload) & 0xFF)
  assert _byte(out) == STW_WIPER_ON
  assert out[:7] != rest[:7]
  assert out[7] == (sum(out[:7]) & 0xFF)
  held = overlay_stw_wiper_beam(rest, False, True, crc_fn=lambda payload: sum(payload) & 0xFF)
  assert hibm_nibble(held) == STW_HIGH_BEAM
  assert held[7] == (sum(held[:7]) & 0xFF)


def test_high_keeps_sending_captured_00ff04_not_sna_or_rest():
  """Real stalk holds 00ff04. High must keep sending that, not a one-shot press."""
  rest = _rest()
  captured_high = bytes.fromhex("00ff04")
  assert rest[:3] == bytes.fromhex("00ff00")
  for _ in range(50):
    held = apply_stw_wiper_beam_nibbles(rest, False, True)
    assert held[:3] == captured_high
    assert hibm_nibble(held) == STW_HIGH_BEAM
    assert hibm_nibble(held) != 0
    assert hibm_nibble(held) != STW_HIGH_BEAM_FLASH
    assert (hibm_nibble(held) & STW_HIBM_MASK) != STW_HIBM_MASK  # not SNA
  off = apply_stw_wiper_beam_nibbles(rest, False, False)
  assert off == rest
  assert off[:3] == bytes.fromhex("00ff00")


def test_off_low_then_high_holds_4_again():
  rest = _rest()
  assert apply_stw_wiper_beam_nibbles(rest, False, False) == rest
  assert apply_stw_wiper_beam_nibbles(rest, False, high_beam_test_requested(BEAM_SETTING_LOW)) == rest
  held = apply_stw_wiper_beam_nibbles(rest, False, high_beam_test_requested(BEAM_SETTING_HIGH))
  assert hibm_nibble(held) == STW_HIGH_BEAM


def test_high_overlay_preserves_turn_indicator_bits():
  blinker = bytes.fromhex("00ff010000000000")
  held = apply_stw_wiper_beam_nibbles(blinker, False, True)
  assert _byte(held) & STW_TURN_MASK == 0x01
  assert hibm_nibble(held) == STW_HIGH_BEAM
  off = apply_stw_wiper_beam_nibbles(blinker, False, False)
  assert off == blinker


def test_wiper_overlay_stays_held_with_high_nibble_4():
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, True, False)
  assert _byte(held) == STW_WIPER_ON
  assert apply_stw_wiper_beam_nibbles(rest, True, False) == held
  both = apply_stw_wiper_beam_nibbles(held, True, True)
  assert _byte(both) == (STW_WIPER_ON | STW_HIGH_BEAM)
  assert _byte(both) & 0xF0 == STW_WIPER_ON
  assert hibm_nibble(both) == STW_HIGH_BEAM
  still = apply_stw_wiper_beam_nibbles(held, True, True)
  assert still == both


def test_stalk_test_active_is_settings_only():
  assert not stalk_test_active(False, False)
  assert stalk_test_active(True, False)
  assert stalk_test_active(False, True)


def test_extra_forward_only_when_on_and_no_existing_0x45():
  existing = [(STW_ACTN_RQ_ADDR, b"\x00" * 8, 0)]
  assert extra_stw_forward_needed([], 10, False, False) is False
  assert extra_stw_forward_needed([], 11, True, False) is False
  assert extra_stw_forward_needed(existing, 10, True, False) is False
  assert extra_stw_forward_needed([], 10, True, False) is True
  assert extra_stw_forward_needed([], 20, False, True) is True
  # Held High must TX between 10 Hz slots so bus-0 IDLE cannot sit unopposed.
  assert extra_stw_forward_needed([], 11, False, True) is True
  assert extra_stw_forward_needed([], 11, True, False) is False
  assert extra_stw_forward_needed(existing, 11, False, True) is False


def test_settings_copy_describes_held_4_and_cannot_drop_bus0_rest():
  from openpilot.selfdrive.ui.layouts.settings.nap_content import HIGH_LOW_BEAM_DESCRIPTION
  text = HIGH_LOW_BEAM_DESCRIPTION.lower()
  assert "00ff04" in text
  assert "nibble 4" in text
  assert "same counter" in text
  assert "cannot drop" in text
  assert "no harness relay" in text
  assert "bus 0" in text
  assert "00ff00" in text
  assert "second 0x45" in text
  assert "off/low" in text
  assert "flash" in text
  assert "pulse" not in text
  assert "sna" not in text


def test_panda_cannot_drop_live_stalk_rest():
  """candump after hold-4: bus 128 is 00ff04 (TX echo), bus 0 stays 00ff00."""
  assert panda_can_drop_live_stw() is False
  assert STW_TX_ECHO_BUS == 0x80
  assert STW_ACTN_RQ_ADDR == 0x45
  rest = _rest()
  held = apply_stw_wiper_beam_nibbles(rest, False, True)
  assert held[:3] == bytes.fromhex("00ff04")
  assert rest[:3] == bytes.fromhex("00ff00")


def test_preap_fwd_hook_already_blocks_0x45():
  """Legal forward/relay block for 0x45 already exists and is unconditional."""
  from pathlib import Path

  import opendbc
  src = Path(opendbc.__file__).resolve().parent / "safety/modes/tesla_preap.h"
  text = src.read_text()
  start = text.index("static bool tesla_preap_fwd_hook")
  end = text.index("safety_config tesla_preap_init")
  hook = text[start:end]
  assert "return true;" in hook
  assert "Block default 0↔2 forwarding" in hook or "Block default 0" in hook
  # No High-conditional block — panda safety has no NAP setting, and a
  # per-addr block would still not drop the live stalk on party bus 0.
  assert "NAPHighLowBeam" not in hook
  assert "BEAM_SETTING_HIGH" not in text


def test_register_defaults_stay_off():
  from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys
  register_nap_body_params()
  assert NAPParamKeys.WIPER_SPEED == NAP_WIPER_SPEED
  assert NAPParamKeys.HIGH_LOW_BEAM == NAP_HIGH_LOW_BEAM
  assert DEFAULTS[NAP_WIPER_SPEED] == 0
  assert DEFAULTS[NAP_HIGH_LOW_BEAM] == 0


def test_das_body_controls_stays_zero_when_settings_on():
  """DAS wiper/beam fields caused a controls mismatch. Leave them at 0."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  _, dat, _ = tc.create_body_controls_message(1, 0, CANBUS.party, 1)
  assert (dat[0] >> 4) & 0x0F == 0  # DAS_wiperSpeed
  assert (dat[1] >> 2) & 0x03 == 0  # DAS_highLowBeamDecision
  assert dat[0] & 0x03 == 0         # DAS_headlightRequest


def test_create_action_request_overlay_holds_4_and_valid_crc(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 3,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  addr, dat, bus = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  assert addr == STW_ACTN_RQ_ADDR == stock[0]
  assert bus == stock[2]
  assert _byte(dat) == (STW_WIPER_ON | STW_HIGH_BEAM)
  assert dat[6] & 0x07 == 3  # WprSw6Posn preserved
  assert dat[:2] == stock[1][:2]
  assert dat[3:7] == stock[1][3:7]
  assert dat[7] == tc.stw_crc(dat[:7])
  assert _byte(dat) != STW_WASHER_SPRAY
  # Holding High keeps nibble 4. Rest/IDLE and SNA are not what goes out.
  for _ in range(8):
    _, held, _ = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert _byte(held) == (STW_WIPER_ON | STW_HIGH_BEAM)
    assert hibm_nibble(held) == STW_HIGH_BEAM
    assert hibm_nibble(held) != 0
    assert (hibm_nibble(held) & STW_HIBM_MASK) != STW_HIBM_MASK
    assert held[7] == tc.stw_crc(held[:7])


class _FakeSpoofer:
  def __init__(self):
    self.sent = []

  def _send(self, CS, tesla_can, bus, button):
    msg = (STW_ACTN_RQ_ADDR, bytes([button]), bus)
    self.sent.append((button, msg))
    return msg


def test_stock_cc_overlay_forwards_once_and_never_a_second_0x45(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert fake.sent[0][0] == 0

  already = [(STW_ACTN_RQ_ADDR, b"\x00" * 8, 0)]
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: already)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert out == already
  assert fake.sent == []


def test_disengaged_idle_stalk_still_forwards_when_on(monkeypatch):
  """Car on, NAP not engaged, no stalk pull: On/High must still forward 0x45."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(
    cruiseEnabled=False,
    enableLongControl=False,
    enableJustCC=False,
    latActive=False,
    msg_stw_actn_req={"SpdCtrlLvr_Stat": 0},  # IDLE — no stalk pull
  )
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert fake.sent[0][0] == 0  # forwarded idle lever, not a cruise press


def test_high_holds_4_and_extra_forwards_until_off(monkeypatch):
  """Holding High keeps TXing 0x45 with nibble 4 so rest/IDLE cannot cancel."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 9})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  forwarded = 0
  for slot in range(1, 8):
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, slot * 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    forwarded += 1
  assert forwarded == 7
  # Between 10 Hz slots too — bus-0 IDLE cannot sit unopposed.
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 11, None, 0)
  assert len(out) == 1
  assert extra_stw_forward_needed([], 11, False, True) is True

  # Leave High — extra-forward stops, real stalk returns.
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 70, None, 0)
  assert out == []
  assert extra_stw_forward_needed([], 70, False, False) is False
  # Come back — hold 4 again, not a one-shot tap.
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 80, None, 0)
  assert len(out) == 1
  fake.sent.clear()
  out = body.stock_cc_update_with_overlay(fake, cs, 81, None, 0)
  assert len(out) == 1


def test_relayed_bus0_rest_is_replaced_with_held_4():
  """candump bus 0 00ff00 is the live stalk. Replace that payload in place."""
  rest = bytes.fromhex("00ff000000090e80")
  assert hibm_nibble(rest) == 0
  patched = replace_relayed_stw(rest, False, True, crc_fn=lambda payload: 0xAA)
  assert patched[:3] == bytes.fromhex("00ff04")
  assert hibm_nibble(patched) == STW_HIGH_BEAM
  assert hibm_nibble(patched) != 0
  assert hibm_nibble(patched) != STW_HIGH_BEAM_FLASH
  assert (hibm_nibble(patched) & STW_HIBM_MASK) != STW_HIBM_MASK
  # Same live frame — counter and neighboring bytes stay. Only HiBm + CRC.
  assert patched[:2] == rest[:2]
  assert patched[3:7] == rest[3:7]
  assert patched[7] == 0xAA
  # Off does not rewrite rest/IDLE.
  assert replace_relayed_stw(rest, False, False) == rest
  # Holding High keeps replacing with 4, not rest.
  for _ in range(8):
    again = replace_relayed_stw(rest, False, True)
    assert hibm_nibble(again) == STW_HIGH_BEAM
    assert again[:3] != rest[:3]


def test_live_stw_counter_is_not_plus_one():
  assert live_stw_counter({"MC_STW_ACTN_RQ": 9}) == 9
  assert live_stw_counter({"MC_STW_ACTN_RQ": 15}) == 15
  assert live_stw_counter({"MC_STW_ACTN_RQ": 0}) == 0


def test_send_replaced_live_stw_uses_live_counter_not_plus_one():
  from types import SimpleNamespace

  class _Rec:
    def __init__(self):
      self.counter = None

    def create_action_request(self, button, bus, counter, msg_stw=None):
      self.counter = counter
      return (STW_ACTN_RQ_ADDR, b"\x00" * 8, bus)

  rec = _Rec()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0, "MC_STW_ACTN_RQ": 9})
  out = send_replaced_live_stw(_FakeSpoofer(), cs, rec, 0)
  assert out[0] == STW_ACTN_RQ_ADDR
  assert rec.counter == 9  # live MC, not 10


def test_replace_relayed_rest_on_packed_stw_holds_4_same_mc():
  """Edit the packed live 0x45 — same MC, held 4, rest/IDLE gone, CRC re-signed."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 0,
    "HiBmLvr_Stat": 0,
  }
  _, rest, _ = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  assert rest[:3] == bytes.fromhex("00ff00")
  patched = replace_relayed_stw(rest, False, True, crc_fn=tc.stw_crc)
  assert patched[:3] == bytes.fromhex("00ff04")
  assert hibm_nibble(patched) == STW_HIGH_BEAM
  assert (patched[6] >> 4) & 0x0F == (rest[6] >> 4) & 0x0F  # MC nibble
  assert patched[7] == tc.stw_crc(patched[:7])
  assert patched[7] != rest[7]


def test_high_extra_forward_keeps_sending_00ff04(monkeypatch):
  """High extra-forward keeps 00ff04 on the live-counter 0x45. Never SNA or rest."""
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  orig = TeslaCANPreAP.create_action_request
  monkeypatch.setattr(TeslaCANPreAP, "create_action_request", body.create_action_request_with_overlay)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", orig)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 0,
    "HiBmLvr_Stat": 0,
  })
  for frame in range(20):
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, frame, tc, CANBUS.party)
    assert len(out) == 1
    addr, dat, bus = out[0]
    assert addr == STW_ACTN_RQ_ADDR
    assert bus == CANBUS.party
    assert (dat[6] >> 4) & 0x0F == 9
    assert dat[:3] == bytes.fromhex("00ff04")
    assert hibm_nibble(dat) == STW_HIGH_BEAM
    assert (hibm_nibble(dat) & STW_HIBM_MASK) != STW_HIBM_MASK
    assert dat[7] == tc.stw_crc(dat[:7])
    assert fake.sent == []  # High uses send_replaced_live_stw, not _send MC+1


def test_stock_cc_off_does_not_change_forwarding(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  assert body.stock_cc_update_with_overlay(fake, cs, 10, None, 0) == []
  assert fake.sent == []


def test_create_action_request_off_matches_stock(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 2,
  }
  stock = tc.create_action_request(CruiseButtons.SET_ACCEL, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  test = body.create_action_request_with_overlay(
    tc, CruiseButtons.SET_ACCEL, CANBUS.party, 6, msg_stw)
  assert test == stock
