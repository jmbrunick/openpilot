"""Pre-AP 0x45 collar INTERVAL3/4 parked spoof tests."""
from openpilot.selfdrive.car.tesla.preap_body_controls import (
  COLLAR_SETTING_3,
  COLLAR_SETTING_4,
  COLLAR_SETTING_OFF,
  NAP_WIPER_COLLAR,
  NAP_WIPER_SPEED,
  STW_ACTN_RQ_ADDR,
  STW_CANCEL_BURST_N,
  STW_COLLAR_MASK,
  STW_COLLAR_POSN_3,
  STW_COLLAR_POSN_4,
  STW_HIGH_BEAM,
  STW_TURN_MASK,
  STW_WASH_MASK,
  STW_WASHER_SPRAY,
  STW_WIPER_BEAM_BYTE,
  STW_WIPER_ON,
  WIPER_SETTING_AUTO,
  apply_stw_collar,
  collar_button_index,
  collar_hold_sends,
  collar_posn_for_setting,
  extra_stw_forward_needed,
  hibm_nibble,
  live_or_rest_stw,
  overlay_collar_on_can_msg,
  overlay_stw_collar,
  persist_collar_posn,
  put_wiper_collar_setting,
  read_wiper_collar_setting,
  reset_auto_gates,
  set_auto_gates,
  stw_collar_posn,
  stw_wash,
)


def _rest() -> bytes:
  return bytes.fromhex("00ff000000000000")


def _byte(dat: bytes) -> int:
  return dat[STW_WIPER_BEAM_BYTE]


class _FakeSpoofer:
  def __init__(self):
    self.sent = []

  def _send(self, CS, tesla_can, bus, button):
    msg = (STW_ACTN_RQ_ADDR, bytes([button]), bus)
    self.sent.append((button, msg))
    return msg


def test_settings_copy_describes_collar_experiment():
  from openpilot.selfdrive.ui.layouts.settings.nap_content import (
    WIPER_COLLAR_DESCRIPTION, WIPER_COLLAR_LABELS, WIPER_COLLAR_VALUES,
  )
  assert WIPER_COLLAR_VALUES == [0, 3, 4]
  assert WIPER_COLLAR_LABELS == ["Off", "Collar3", "Collar4"]
  text = WIPER_COLLAR_DESCRIPTION.lower()
  assert "interval3" in text or "wprsw6posn" in text
  assert "collar3" in text
  assert "collar4" in text
  assert "tipwipe" in text
  assert "spray" in text
  assert "10 ms" in text or "100 hz" in text
  assert "last-win" in text
  assert "flicker" in text
  assert "experiment" in text
  assert "gateway" in text
  assert "live" in text
  assert "crc" in text
  assert "src 128" in text
  assert "napwipercollar" in text
  assert "off" in text
  assert "auto" in text
  assert "wash" in text
  assert "0x45" in text


def test_collar_setting_maps_to_dbc_posn():
  assert collar_posn_for_setting(COLLAR_SETTING_OFF) is None
  assert collar_posn_for_setting(COLLAR_SETTING_3) == STW_COLLAR_POSN_3 == 3
  assert collar_posn_for_setting(COLLAR_SETTING_4) == STW_COLLAR_POSN_4 == 4
  # Raw DBC 3/4 if a tester puts those instead of UI indexes 1/2.
  assert collar_posn_for_setting(3) == STW_COLLAR_POSN_3
  assert collar_posn_for_setting(4) == STW_COLLAR_POSN_4
  assert collar_posn_for_setting(99) is None


def test_ui_collar3_index_1_persists_and_txs_posn_3(monkeypatch):
  """Justin cat NAPWiperCollar=1 while UI showed Collar3. Overlay must still TX 3.

  Persistent file is DBC 3 so `cat /data/params/d/NAPWiperCollar` shows 3.
  Legacy 1/2 still map. Button index for persisted 3 is Collar3, not Collar4.
  """
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body
  from openpilot.selfdrive.ui.layouts.settings.nap_content import WIPER_COLLAR_VALUES

  assert WIPER_COLLAR_VALUES[1] == STW_COLLAR_POSN_3 == 3
  assert persist_collar_posn(1) == persist_collar_posn(COLLAR_SETTING_3) == 3
  assert persist_collar_posn(2) == persist_collar_posn(COLLAR_SETTING_4) == 4
  assert persist_collar_posn(3) == 3
  assert persist_collar_posn(0) == 0
  assert collar_button_index(1) == collar_button_index(3) == 1
  assert collar_button_index(2) == collar_button_index(4) == 2
  assert collar_button_index(0) == 0
  # selected_index=min(len-1, setting) with setting=3 would light Collar4.
  assert min(len(WIPER_COLLAR_VALUES) - 1, 3) == 2
  assert collar_button_index(3) != min(len(WIPER_COLLAR_VALUES) - 1, 3)

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  cs = SimpleNamespace(msg_stw_actn_req=None)
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: 1)  # Justin's cat
  out = collar_hold_sends([], cs, tc, CANBUS.party)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert out[0][1][6] & 0x07 == 3
  assert stw_wash(out[0][1]) == 0


def test_collar_overlay_forces_posn_and_clears_wash_never_spray():
  rest = _rest()
  live_int1 = bytes.fromhex("00ff000000000100")  # WprSw6Posn=1, MC=0
  tipwipe = bytes.fromhex("00ff100000000100")
  spray = bytes.fromhex("00ff200000000100")
  high = bytes.fromhex("00ff040000000100")
  assert apply_stw_collar(rest, None) == rest
  assert apply_stw_collar(live_int1, None) == live_int1
  forced3 = apply_stw_collar(live_int1, 3)
  assert stw_collar_posn(forced3) == 3
  assert stw_wash(forced3) == 0
  assert _byte(forced3) != STW_WASHER_SPRAY
  assert _byte(forced3) != STW_WIPER_ON
  forced4 = apply_stw_collar(live_int1, 4)
  assert stw_collar_posn(forced4) == 4
  assert stw_wash(forced4) == 0
  cleared = apply_stw_collar(tipwipe, 3)
  assert stw_wash(cleared) == 0
  assert _byte(cleared) & STW_WASH_MASK == 0
  assert _byte(cleared) != STW_WIPER_ON
  assert _byte(cleared) != STW_WASHER_SPRAY
  nospray = apply_stw_collar(spray, 4)
  assert _byte(nospray) != STW_WASHER_SPRAY
  assert stw_wash(nospray) == 0
  # High-beam nibble and turn bits stay. Rear-wash bits stay (only WprWashSw_Psd).
  held_high = apply_stw_collar(high, 3)
  assert hibm_nibble(held_high) == STW_HIGH_BEAM
  blinker = bytes.fromhex("00ff010000000100")
  assert apply_stw_collar(blinker, 3)[STW_WIPER_BEAM_BYTE] & STW_TURN_MASK == 0x01
  # MC lives in byte 6 high nibble — do not clobber it. posn=1, MC=9 → 0x91.
  live_mc = bytes.fromhex("00ff000000009100")
  out = apply_stw_collar(live_mc, 3)
  assert (out[6] >> 4) & 0x0F == 9
  assert stw_collar_posn(out) == 3
  assert stw_collar_posn(live_int1) & STW_COLLAR_MASK == 1


def _sum_crc(payload):
  return sum(payload) & 0xFF


def _const_aa(_payload):
  return 0xAA


def test_collar_overlay_resigns_crc_only_when_changed():
  rest = _rest()
  assert overlay_stw_collar(rest, None, crc_fn=_const_aa) == rest
  out = overlay_stw_collar(rest, 3, crc_fn=_sum_crc)
  assert stw_collar_posn(out) == 3
  assert stw_wash(out) == 0
  assert out[7] == _sum_crc(out[:7])
  same = overlay_stw_collar(out, 3, crc_fn=_const_aa)
  assert same == out


def test_auto_overlay_uses_interval1_and_collar3_overrides(monkeypatch):
  """#162 camera Auto holds INTERVAL1; Collar3 overrides that to posn 3."""
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
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "_param_int", lambda key, default=0: (
    WIPER_SETTING_AUTO if key == NAP_WIPER_SPEED else default
  ))
  monkeypatch.setattr(body, "rain_wiper_needed", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: 0)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  set_auto_gates(True, "drive")
  try:
    addr, dat, bus = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert addr == STW_ACTN_RQ_ADDR == stock[0]
    assert _byte(dat) != STW_WIPER_ON
    assert stw_wash(dat) == 0
    assert stw_collar_posn(dat) == 1
    assert stw_collar_posn(stock[1]) == 2
    assert dat[7] == tc.stw_crc(dat[:7])
    monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: STW_COLLAR_POSN_3)
    addr, dat, bus = body.create_action_request_with_overlay(
      tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
    assert stw_collar_posn(dat) == 3
    assert stw_wash(dat) == 0
    assert _byte(dat) != STW_WIPER_ON
  finally:
    reset_auto_gates()


def test_create_action_request_collar3_forces_posn_wash0_live_mc(monkeypatch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 1,  # live 4-click Int1
    "WprWashSw_Psd": 1,  # live would be TIPWIPE if Int were on; experiment forces 0
    "HiBmLvr_Stat": 0,
  }
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_posn", lambda: 3)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  addr, dat, bus = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  assert addr == STW_ACTN_RQ_ADDR == stock[0]
  assert bus == stock[2]
  assert stw_collar_posn(dat) == 3
  assert stw_wash(dat) == 0
  assert _byte(dat) != STW_WIPER_ON
  assert _byte(dat) != STW_WASHER_SPRAY
  assert (dat[6] >> 4) & 0x0F == 9
  assert dat[:2] == stock[1][:2]
  assert dat[7] == tc.stw_crc(dat[:7])
  monkeypatch.setattr(body, "requested_collar_posn", lambda: 4)
  _, four, _ = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 9, msg_stw)
  assert stw_collar_posn(four) == 4
  assert stw_wash(four) == 0
  assert four[7] == tc.stw_crc(four[:7])


def test_collar_on_clears_int_tipwipe(monkeypatch):
  """Collar experiment is wash=0. Int TIPWIPE must not ride along."""
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS, CruiseButtons

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 2,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  }
  monkeypatch.setattr(body, "requested_wiper_test", lambda: True)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_posn", lambda: 3)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  _, dat, _ = body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  assert stw_collar_posn(dat) == 3
  assert stw_wash(dat) == 0
  assert _byte(dat) != STW_WIPER_ON
  assert _byte(dat) != STW_WASHER_SPRAY
  assert dat[7] == tc.stw_crc(dat[:7])


def test_collar_off_matches_stock_live_collar(monkeypatch):
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
  stock = tc.create_action_request(CruiseButtons.IDLE, CANBUS.party, 6, msg_stw)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_posn", lambda: None)
  monkeypatch.setattr(body, "_ORIG_CREATE_ACTION_REQUEST", TeslaCANPreAP.create_action_request)
  assert body.create_action_request_with_overlay(
    tc, CruiseButtons.IDLE, CANBUS.party, 6, msg_stw) == stock


def test_stock_cc_collar_extra_forwards_every_10ms_not_a_second_0x45(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_test", lambda: True)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 11, None, 0)
    assert len(out) == 1
    assert out[0][0] == STW_ACTN_RQ_ADDR
    already = [(STW_ACTN_RQ_ADDR, b"\x00" * 8, 0)]
    monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: already)
    out = body.stock_cc_update_with_overlay(fake, cs, 20, None, 0)
    assert out == already
  finally:
    reset_auto_gates()


def test_collar_to_off_sends_rest_cancel_burst_then_leaves_stalk(monkeypatch):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  reset_auto_gates()
  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={"SpdCtrlLvr_Stat": 0})
  on = {"v": True}
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_test", lambda: on["v"])
  monkeypatch.setattr(body, "requested_collar_posn", lambda: 3 if on["v"] else None)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, None, 0)
    assert len(out) == 1
    on["v"] = False
    forwarded = 0
    for frame in range(11, 11 + STW_CANCEL_BURST_N):
      fake.sent.clear()
      out = body.stock_cc_update_with_overlay(fake, cs, frame, None, 0)
      assert len(out) == 1
      assert out[0][0] == STW_ACTN_RQ_ADDR
      forwarded += 1
    assert forwarded == STW_CANCEL_BURST_N
    fake.sent.clear()
    out = body.stock_cc_update_with_overlay(fake, cs, 10 + STW_CANCEL_BURST_N * 10, None, 0)
    assert out == []
    assert extra_stw_forward_needed([], 10, False, False) is False
  finally:
    reset_auto_gates()


def test_stock_cc_collar_uses_live_mc_like_high_and_valid_crc(monkeypatch):
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
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "requested_collar_test", lambda: True)
  monkeypatch.setattr(body, "requested_collar_posn", lambda: 3)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  fake = _FakeSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 1,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  })
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(fake, cs, 10, tc, CANBUS.party)
    assert len(out) == 1
    addr, dat, bus = out[0]
    assert addr == STW_ACTN_RQ_ADDR
    assert bus == CANBUS.party
    assert stw_collar_posn(dat) == 3
    assert stw_wash(dat) == 0
    assert _byte(dat) != STW_WASHER_SPRAY
    assert (dat[6] >> 4) & 0x0F == 9  # High last-win: live MC, not MC+1
    assert dat[7] == tc.stw_crc(dat[:7])
    assert fake.sent == []  # Collar uses send_replaced_live_stw, not _send
  finally:
    reset_auto_gates()


def test_parked_collar3_tx_posn3_when_create_action_request_is_stock(monkeypatch):
  """Stalk Off + Collar3 must TX WprSw6Posn=3 even if overlay isn't installed.

  eae5beb extra-forward used send_replaced_live_stw / unpatched packer and
  TXed live Off (collar=0). Overlay on the packed TX is required. 10 Hz
  also loses to repeating bus-0 Off — hold every 10 ms like High.
  """
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  # Stock packer — do not patch TeslaCANPreAP.create_action_request.
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: COLLAR_SETTING_3)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  spoofer = StockCCSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 9,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,  # parked physical collar Off
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  })
  reset_auto_gates()
  try:
    stock = tc.create_action_request(0, CANBUS.party, 9, cs.msg_stw_actn_req)
    assert stw_collar_posn(stock[1]) == 0
    for frame in range(10, 30):
      out = body.stock_cc_update_with_overlay(spoofer, cs, frame, tc, CANBUS.party)
      assert len(out) == 1
      addr, dat, bus = out[0]
      assert addr == STW_ACTN_RQ_ADDR
      assert bus == CANBUS.party
      assert stw_collar_posn(dat) == 3
      assert stw_wash(dat) == 0
      assert _byte(dat) != STW_WIPER_ON
      assert _byte(dat) != STW_WASHER_SPRAY
      assert (dat[6] >> 4) & 0x0F == 9  # live MC last-win, like High
      assert dat[7] == tc.stw_crc(dat[:7])
  finally:
    reset_auto_gates()


def test_parked_collar4_tx_posn4_when_create_action_request_is_stock(monkeypatch):
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: COLLAR_SETTING_4)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])

  spoofer = StockCCSpoofer()
  cs = SimpleNamespace(msg_stw_actn_req={
    "SpdCtrlLvr_Stat": 0,
    "MC_STW_ACTN_RQ": 1,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,
    "WprWashSw_Psd": 0,
    "HiBmLvr_Stat": 0,
  })
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(spoofer, cs, 20, tc, CANBUS.party)
    assert len(out) == 1
    _, dat, _ = out[0]
    assert stw_collar_posn(dat) == 4
    assert stw_wash(dat) == 0
    assert (dat[6] >> 4) & 0x0F == 1
    assert dat[7] == tc.stw_crc(dat[:7])
  finally:
    reset_auto_gates()


def test_overlay_collar_on_can_msg_resigns_crc():
  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {
    "MC_STW_ACTN_RQ": 5,
    "CRC_STW_ACTN_RQ": 0,
    "DTR_Dist_Rq": 255,
    "VSL_Enbl_Rq": 1,
    "WprSw6Posn": 0,
    "WprWashSw_Psd": 0,
  }
  stock = tc.create_action_request(0, CANBUS.party, 6, msg_stw)
  assert stw_collar_posn(stock[1]) == 0
  forced = overlay_collar_on_can_msg(stock, tc, 3)
  assert forced[0] == stock[0]
  assert forced[2] == stock[2]
  assert stw_collar_posn(forced[1]) == 3
  assert stw_wash(forced[1]) == 0
  assert forced[1][7] == tc.stw_crc(forced[1][:7])
  assert overlay_collar_on_can_msg(stock, tc, None) is stock
  assert overlay_collar_on_can_msg(stock, None, 3) is stock


def test_ui_and_params_key_is_nap_wiper_collar():
  from pathlib import Path

  assert NAP_WIPER_COLLAR == "NAPWiperCollar"
  repo = Path(__file__).resolve().parents[4]
  keys = (repo / "common" / "params_keys.h").read_text()
  assert '"NAPWiperCollar"' in keys
  nap3x = (repo / "selfdrive" / "ui" / "layouts" / "settings" / "nap.py").read_text()
  assert "put_wiper_collar_setting" in nap3x
  assert "read_wiper_collar_setting" in nap3x
  assert "collar_button_index" in nap3x
  assert "put(NAPParamKeys.WIPER_COLLAR" not in nap3x
  assert "get(NAPParamKeys.WIPER_COLLAR" not in nap3x
  assert '"NAPWiperCollarStatus"' in keys
  mici = (repo / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "nap.py").read_text()
  assert "NAP_WIPER_COLLAR" in mici
  card = (repo / "selfdrive" / "car" / "card.py").read_text()
  assert "_publish_collar_hold" in card
  assert "collar_hold_sends" in card
  assert "TESLA_MODEL_S_PREAP" in card
  assert "elif getattr(self, \"_tesla_preap\"" not in card
  assert "write_collar_heartbeat" in card


def test_wprsw6posn_is_byte6_low3_justins_dump():
  """Live dump: collar in d[6]&7. DBC start bit 48, 3 bits LE."""
  rest = bytes.fromhex("00ff000000000000")
  int1 = bytes.fromhex("00ff000000000100")
  assert stw_collar_posn(rest) == rest[6] & 0x07 == 0
  assert stw_collar_posn(int1) == int1[6] & 0x07 == 1
  forced = apply_stw_collar(int1, 3)
  assert forced[6] & 0x07 == 3
  assert (forced[6] >> 4) & 0x0F == (int1[6] >> 4) & 0x0F


def test_collar_hold_sends_appends_when_idle_and_overlays_existing(monkeypatch):
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: COLLAR_SETTING_3)
  cs = SimpleNamespace(msg_stw_actn_req=None)  # parked, parser empty
  out = collar_hold_sends([], cs, tc, CANBUS.party)
  assert len(out) == 1
  addr, dat, bus = out[0]
  assert addr == STW_ACTN_RQ_ADDR
  assert dat[6] & 0x07 == 3
  assert stw_wash(dat) == 0
  assert dat[7] == tc.stw_crc(dat[:7])

  already = [tc.create_action_request(0, CANBUS.party, 4, live_or_rest_stw(None))]
  assert already[0][1][6] & 0x07 == 0
  out = collar_hold_sends(already, cs, tc, CANBUS.party)
  assert len(out) == 1
  assert out[0][1][6] & 0x07 == 3


def test_collar_hold_sends_when_tesla_can_is_none(monkeypatch):
  """Parked last-mile must pack 0x45 even if CI.CC.tesla_can is missing."""
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: STW_COLLAR_POSN_3)
  cs = SimpleNamespace(msg_stw_actn_req=None)
  out = collar_hold_sends([], cs, None, 0)
  assert len(out) == 1
  assert out[0][0] == STW_ACTN_RQ_ADDR
  assert out[0][1][6] & 0x07 == 3
  assert stw_wash(out[0][1]) == 0


def test_collar_status_writes_err_when_collar_off(monkeypatch, tmp_path):
  from types import SimpleNamespace

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  path = tmp_path / "NAPWiperCollar"
  status = tmp_path / "NAPWiperCollarStatus"

  class _Boom:
    def get(self, *args, **kwargs):
      raise RuntimeError("unknown key")

    def put(self, *args, **kwargs):
      raise RuntimeError("unknown key")

    def get_param_path(self, key=""):
      return str(tmp_path)

  monkeypatch.setattr(body, "_collar_file_paths", lambda: [str(path)])
  monkeypatch.setattr(body, "_get_params", lambda: _Boom())
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: 0)
  body._last_collar_status_t = 0.0
  collar_hold_sends([], SimpleNamespace(msg_stw_actn_req=None), None, 0)
  text = status.read_text()
  assert "collar=0" in text
  assert "err=collar_off" in text
  assert "tx=" in text


def test_sidecar_file_is_collar3_when_params_unknown(monkeypatch, tmp_path):
  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  path = tmp_path / "NAPWiperCollar"
  monkeypatch.setattr(body, "_collar_file_paths", lambda: [str(path)])

  class _Boom:
    def get(self, *args, **kwargs):
      raise RuntimeError("unknown key")

    def put(self, *args, **kwargs):
      raise RuntimeError("unknown key")

    def get_param_path(self, key=""):
      return str(tmp_path)

  monkeypatch.setattr(body, "_get_params", lambda: _Boom())
  put_wiper_collar_setting(COLLAR_SETTING_3)
  assert path.read_text() == "3"
  assert read_wiper_collar_setting() == STW_COLLAR_POSN_3
  assert collar_posn_for_setting(read_wiper_collar_setting()) == 3
  path.write_text("1")
  assert read_wiper_collar_setting() == STW_COLLAR_POSN_3
  body.migrate_wiper_collar_param()
  assert path.read_text() == "3"


def test_stock_cc_collar_txs_when_msg_stw_missing(monkeypatch):
  from types import SimpleNamespace

  from opendbc.can import CANPacker
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  from opendbc.car.tesla.values import CANBUS

  from openpilot.selfdrive.car.tesla import preap_body_controls as body

  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  monkeypatch.setattr(body, "read_wiper_collar_setting", lambda: COLLAR_SETTING_3)
  monkeypatch.setattr(body, "requested_wiper_test", lambda: False)
  monkeypatch.setattr(body, "requested_high_beam_test", lambda: False)
  monkeypatch.setattr(body, "_ORIG_STOCK_CC_UPDATE", lambda self, CS, frame, tesla_can, bus: [])
  reset_auto_gates()
  try:
    out = body.stock_cc_update_with_overlay(StockCCSpoofer(), SimpleNamespace(), 10, tc, CANBUS.party)
    assert len(out) == 1
    assert out[0][1][6] & 0x07 == 3
  finally:
    reset_auto_gates()
