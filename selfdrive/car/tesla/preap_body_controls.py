"""Default-off Pre-AP wiper / high-beam test on the forwarded stalk.

DAS_bodyControls DAS_wiperSpeed and DAS_highLowBeamDecision were ignored by
this pre-AP Model S and raised a controls mismatch. Those DAS fields stay 0
(stock teslacan blinker-only TX). This test rewrites the wiper/beam byte on
the 0x45 STW_ACTN_RQ frame NAP already forwards for stalk spoof.

Justin’s parked capture (ignore counter/checksum):
  rest        00ff00....  byte after ff is 0x00
  wipers on   00ff10....  high nibble of that byte is 1
  washer      00ff20....  do not send
  high beams  00ff04....  low nibble of that byte is 4

DBC HiBmLvr_Stat=1 is HIBM_ON_PSD (pressed), not a latched-on state.
On-car: holding nibble 4 at the 10 Hz forward slot retriggered high beams
instead of latching. Wiper nibble 1 can stay held (Int works). High is a
short press/release pulse on the rising edge of the setting, then the
forwarded stalk returns to the driver’s real nibble.

Off leaves the driver’s real stalk nibble alone (do not force 0). Wiper
On/Int holds high nibble 1. No rain model. No auto high-beam. Do not flash.
Do not inject a second 0x45 — overlay the existing forwarded frame and
recompute CRC the same way create_action_request already does.

Panda already allows TX of 0x45 on bus 0 (stalk spoof whitelist). The TX
hook does not gate 0x45 on controls_allowed — stock-CC engage already
sends this ID while disengaged. 0x3E9 DAS_bodyControls *is* gated; that
is why this test must not use DAS. Do not bypass safety if that ever
changes. Do not fake this through another ID.

This test must work with the car on and openpilot not engaged. It is not
gated on cruiseEnabled, latActive, or a stalk pull.

Known risk: pre-AP may ignore a spoofed stalk, or checksum/relay may fault.
This is a car test, not auto wipers or auto headlights.
"""

# Params / UI. 0 is off (today's forwarded stalk). Indexes, not raw DBC.
NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_HIGH_LOW_BEAM = "NAPHighLowBeam"

WIPER_SETTING_OFF = 0
WIPER_SETTING_INTERMITTENT = 1
WIPER_SETTING_ON = 2
BEAM_SETTING_OFF = 0
BEAM_SETTING_LOW = 1
BEAM_SETTING_HIGH = 2

STW_ACTN_RQ_ADDR = 0x45
STW_WIPER_BEAM_BYTE = 2
STW_WIPER_ON = 0x10
STW_WASHER_SPRAY = 0x20
STW_HIGH_BEAM = 0x04
STW_HIGH_BEAM_FLASH = 0x08
STW_FORWARD_SLOT = 10
# Two 10 Hz frames ≈ 200 ms: press seen, then release. Holding 4 forever
# retriggers the body toggle. One slot can be too short to look like a pull.
HIGH_BEAM_PULSE_SLOTS = 2

_ORIG_CREATE_ACTION_REQUEST = None
_ORIG_STOCK_CC_UPDATE = None
_installed = False
_high_beam_oneshot = None
_slot_high_overlay = False


def _tesla_can():
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  return TeslaCANPreAP


def _stock_cc():
  from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
  return StockCCSpoofer


def register_nap_body_params():
  """Expose the test keys on NAPParamKeys / DEFAULTS for settings reset."""
  from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys
  NAPParamKeys.WIPER_SPEED = NAP_WIPER_SPEED
  NAPParamKeys.HIGH_LOW_BEAM = NAP_HIGH_LOW_BEAM
  DEFAULTS[NAP_WIPER_SPEED] = WIPER_SETTING_OFF
  DEFAULTS[NAP_HIGH_LOW_BEAM] = BEAM_SETTING_OFF


def wiper_test_requested(setting: int) -> bool:
  """Int and On share the only captured wiper encoding (high nibble 1)."""
  return int(setting) in (WIPER_SETTING_INTERMITTENT, WIPER_SETTING_ON)


def high_beam_test_requested(setting: int) -> bool:
  """Only High spoofs. Low/Off leave the stalk — forcing 0 fights a held lever."""
  return int(setting) == BEAM_SETTING_HIGH


def high_beam_oneshot_step(setting_on: bool, prev_on: bool, remaining: int,
                           pulse_slots: int = HIGH_BEAM_PULSE_SLOTS) -> tuple[bool, bool, int]:
  """One-shot High pulse. Wipers stay held; beams must not.

  Rising edge of High starts a short press. Holding High does not retrigger.
  Off/Low then High again is the next trigger.
  Returns (overlay_this_slot, new_prev, new_remaining).
  """
  setting_on = bool(setting_on)
  if not setting_on:
    return False, False, 0
  if not prev_on:
    remaining = int(pulse_slots)
  remaining = int(remaining)
  if remaining > 0:
    return True, True, remaining - 1
  return False, True, 0


class HighBeamOneShot:
  def __init__(self, pulse_slots: int = HIGH_BEAM_PULSE_SLOTS):
    self.pulse_slots = pulse_slots
    self.prev = False
    self.remaining = 0

  def reset(self):
    self.prev = False
    self.remaining = 0

  def would_overlay(self, setting_on: bool) -> bool:
    overlay, _, _ = high_beam_oneshot_step(setting_on, self.prev, self.remaining, self.pulse_slots)
    return overlay

  def consume(self, setting_on: bool) -> bool:
    overlay, self.prev, self.remaining = high_beam_oneshot_step(
      setting_on, self.prev, self.remaining, self.pulse_slots)
    return overlay


def get_high_beam_oneshot() -> HighBeamOneShot:
  global _high_beam_oneshot
  if _high_beam_oneshot is None:
    _high_beam_oneshot = HighBeamOneShot()
  return _high_beam_oneshot


def reset_high_beam_oneshot():
  get_high_beam_oneshot().reset()
  global _slot_high_overlay
  _slot_high_overlay = False


def apply_stw_wiper_beam_nibbles(dat: bytes, wiper_on: bool, high_beam_on: bool) -> bytes:
  """Set captured stalk nibbles. Off leaves that nibble. Never writes spray."""
  if len(dat) <= STW_WIPER_BEAM_BYTE:
    return bytes(dat)
  out = bytearray(dat)
  b = out[STW_WIPER_BEAM_BYTE]
  if wiper_on:
    b = (b & 0x0F) | STW_WIPER_ON
  if high_beam_on:
    b = (b & 0xF0) | STW_HIGH_BEAM
  out[STW_WIPER_BEAM_BYTE] = b
  return bytes(out)


def stalk_test_active(wiper_on: bool | None = None, high_beam_on: bool | None = None) -> bool:
  """Settings only. Not gated on cruiseEnabled, latActive, or a stalk pull."""
  if wiper_on is None:
    wiper_on = requested_wiper_test()
  if high_beam_on is None:
    high_beam_on = requested_high_beam_test()
  return bool(wiper_on or high_beam_on)


def extra_stw_forward_needed(can_sends, frame: int, wiper_on: bool, high_beam_on: bool) -> bool:
  """One 0x45 per stock-cc slot when the test is on. Never a second frame.

  This is the parked / not-engaged path: stock-cc only TXes 0x45 on
  engage/cancel (a stalk pull). Wiper On/Int keeps forwarding so nibble 1
  can stay held. High only needs a forward during its short pulse — after
  that the real stalk stays on the bus so the beams can latch.
  """
  if not stalk_test_active(wiper_on, high_beam_on):
    return False
  if int(frame) % STW_FORWARD_SLOT != 0:
    return False
  return not any(msg[0] == STW_ACTN_RQ_ADDR for msg in can_sends)


def _param_int(key: str, default: int = 0) -> int:
  try:
    from openpilot.common.params import Params
    val = Params().get(key, return_default=True)
    return int(val) if val is not None else default
  except Exception:
    return default


def requested_wiper_test() -> bool:
  return wiper_test_requested(_param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF))


def requested_high_beam_test() -> bool:
  return high_beam_test_requested(_param_int(NAP_HIGH_LOW_BEAM, BEAM_SETTING_OFF))


def overlay_stw_wiper_beam(dat: bytes, wiper_on: bool, high_beam_on: bool, crc_fn=None) -> bytes:
  """Apply nibbles and resign CRC only when the payload changed."""
  new_dat = apply_stw_wiper_beam_nibbles(dat, wiper_on, high_beam_on)
  if new_dat == dat:
    return dat
  if crc_fn is None or len(new_dat) < 8:
    return new_dat
  out = bytearray(new_dat)
  out[7] = crc_fn(bytes(out[:7]))
  return bytes(out)


def create_action_request_with_overlay(self, button_to_press, bus, counter, msg_stw=None):
  """Forward the live stalk, then overlay the test nibbles on that same frame."""
  orig = _ORIG_CREATE_ACTION_REQUEST
  if orig is None:
    orig = _tesla_can().create_action_request
  addr, dat, out_bus = orig(self, button_to_press, bus, counter, msg_stw)
  dat = overlay_stw_wiper_beam(dat, requested_wiper_test(), _slot_high_overlay,
                               crc_fn=self.stw_crc)
  return addr, dat, out_bus


def stock_cc_update_with_overlay(self, CS, frame, tesla_can, can_bus_party):
  """Keep the single 0x45 TX path. When the test is on, forward if idle this slot.

  Does not read cruiseEnabled, latActive, or CC.enabled. A parked car with
  NAP not engaged and the stalk at rest is enough. High only extra-forwards
  during its one-shot pulse; wipers keep forwarding while held.
  """
  global _slot_high_overlay
  orig = _ORIG_STOCK_CC_UPDATE
  if orig is None:
    orig = _stock_cc().update
  wiper = requested_wiper_test()
  high_setting = requested_high_beam_test()
  if int(frame) % STW_FORWARD_SLOT == 0:
    # Tick once per 10 Hz slot even if we do not TX, so Off→High retriggers.
    _slot_high_overlay = get_high_beam_oneshot().consume(high_setting)
  can_sends = orig(self, CS, frame, tesla_can, can_bus_party)
  if extra_stw_forward_needed(can_sends, frame, wiper, _slot_high_overlay):
    msg_stw = getattr(CS, "msg_stw_actn_req", None)
    if msg_stw is not None:
      button = int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0)
      sent = self._send(CS, tesla_can, can_bus_party, button)
      if sent is not None:
        can_sends.append(sent)
  return can_sends


def install_body_controls_test():
  """Wire NAP Wipers & Lights settings to the forwarded 0x45 stalk byte.

  Does not patch DAS_bodyControls — those wiper/beam fields stay 0.
  """
  global _installed, _ORIG_CREATE_ACTION_REQUEST, _ORIG_STOCK_CC_UPDATE
  register_nap_body_params()
  if _installed:
    return
  tesla_can = _tesla_can()
  stock_cc = _stock_cc()
  _ORIG_CREATE_ACTION_REQUEST = tesla_can.create_action_request
  _ORIG_STOCK_CC_UPDATE = stock_cc.update
  tesla_can.create_action_request = create_action_request_with_overlay
  stock_cc.update = stock_cc_update_with_overlay
  _installed = True
