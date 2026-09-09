"""Pre-AP wiper / high-beam on the forwarded stalk.

DAS_bodyControls DAS_wiperSpeed and DAS_highLowBeamDecision were ignored by
this pre-AP Model S and raised a controls mismatch. Those DAS fields stay 0
(stock teslacan blinker-only TX). This rewrites the wiper/beam byte on
the 0x45 STW_ACTN_RQ frame NAP already forwards for stalk spoof.

Justin’s parked capture (ignore counter/checksum):
  rest        00ff00....  byte after ff is 0x00
  wipers on   00ff10....  high nibble of that byte is 1
  washer      00ff20....  do not send
  high beams  00ff04....  low nibble of that byte is 4

DBC HiBmLvr_Stat is a 2-bit enum, not a latched-on state:
  0 IDLE, 1 HIBM_ON_PSD (nibble 4), 2 HIBM_FLSH_ON_PSD (nibble 8), 3 SNA.
There is no separate low-beam press. candump src 0 is the live stalk RX;
src 128 is our TX echo (returned | 0x80) — not a panda 0↔2 relay. The body
hears both on party bus 0. A pulse-then-SNA extra TX lost to repeating
bus-0 IDLE. The real stalk continuously sends 00ff04 (HIBM_ON_PSD) while
high beams are held — not a one-shot press.

While High is selected, keep sending 00ff04 (nibble 4 held) on the
live-counter in-place replacement of 0x45 so the body keeps seeing
high-beam pressed and bus-0 IDLE cannot last-win as a cancel. Off/Low
return the real stalk. Do not pulse 4 then drop to SNA or rest.

Off leaves the driver’s real stalk nibble alone (do not force 0). Wiper
On/Int holds high nibble 1. Auto holds that same nibble 1 only while a
rain/wiper-need signal is set, then releases the real stalk when dry.
Default Off — Auto is opt-in. No spray. No auto high-beam. Do not flash.
Do not inject a second 0x45 — overlay the existing forwarded frame and
recompute CRC the same way create_action_request already does.

Panda already allows TX of 0x45 on bus 0 (stalk spoof whitelist). The TX
hook does not gate 0x45 on controls_allowed — stock-CC engage already
sends this ID while disengaged. 0x3E9 DAS_bodyControls *is* gated; that
is why this must not use DAS. Do not bypass safety if that ever
changes. Do not fake this through another ID.

This must work with the car on and openpilot not engaged. It is not
gated on cruiseEnabled, latActive, or a stalk pull.

Known risk: pre-AP may still see the real stalk rest on bus 0. Int already
wins when held, so Auto uses that same hold, not a pulse.
"""

# Params / UI. 0 is off (today's forwarded stalk). Indexes, not raw DBC.
NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_HIGH_LOW_BEAM = "NAPHighLowBeam"
NAP_RAIN_NEEDED = "NAPRainNeeded"

WIPER_SETTING_OFF = 0
WIPER_SETTING_INTERMITTENT = 1
WIPER_SETTING_ON = 2
WIPER_SETTING_AUTO = 3
BEAM_SETTING_OFF = 0
BEAM_SETTING_LOW = 1
BEAM_SETTING_HIGH = 2

# Camera rain head (modelV2.meta.rainProb and aliases). Stay dry without it.
RAIN_PROB_ON = 0.5
_CAMERA_RAIN_KEYS = ("rainProb", "rainingProb", "precipProb", "wiperNeedProb")

STW_ACTN_RQ_ADDR = 0x45
STW_WIPER_BEAM_BYTE = 2
STW_WIPER_ON = 0x10
STW_WASHER_SPRAY = 0x20
STW_TURN_MASK = 0x03  # TurnIndLvr_Stat. Do not touch — blinker lat-pause.
STW_HIBM_MASK = 0x0C  # HiBmLvr_Stat bits 2-3 of the captured byte.
STW_HIGH_BEAM = 0x04  # HIBM_ON_PSD — held while High is selected
STW_HIGH_BEAM_FLASH = 0x08  # HIBM_FLSH_ON_PSD — never send
STW_FORWARD_SLOT = 10

_ORIG_CREATE_ACTION_REQUEST = None
_ORIG_STOCK_CC_UPDATE = None
_installed = False
_rain_needed_override = None
_model_sm = None
_model_sm_failed = False


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


def wiper_test_requested(setting: int, rain_needed: bool = False) -> bool:
  """Int and On hold nibble 1. Auto holds it only while rain/wiper-need is set."""
  s = int(setting)
  if s in (WIPER_SETTING_INTERMITTENT, WIPER_SETTING_ON):
    return True
  if s == WIPER_SETTING_AUTO:
    return bool(rain_needed)
  return False


def high_beam_test_requested(setting: int) -> bool:
  """Only High spoofs. Low/Off leave the stalk — forcing 0 fights a held lever."""
  return int(setting) == BEAM_SETTING_HIGH


def hibm_nibble(dat: bytes) -> int:
  if len(dat) <= STW_WIPER_BEAM_BYTE:
    return 0
  return dat[STW_WIPER_BEAM_BYTE] & STW_HIBM_MASK


def apply_stw_wiper_beam_nibbles(dat: bytes, wiper_on: bool, high_beam_on: bool) -> bytes:
  """Set captured stalk nibbles. Off leaves that nibble. Never writes spray.

  High holds HIBM_ON_PSD (4) for as long as the setting is High. Only HiBm
  bits are touched — turn-indicator bits stay for blinker lat-pause.
  """
  if len(dat) <= STW_WIPER_BEAM_BYTE:
    return bytes(dat)
  out = bytearray(dat)
  b = out[STW_WIPER_BEAM_BYTE]
  if wiper_on:
    b = (b & 0x0F) | STW_WIPER_ON
  if high_beam_on:
    b = (b & ~STW_HIBM_MASK) | STW_HIGH_BEAM
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
  """One 0x45 when the test is on. Never a second frame.

  Parked / not-engaged: stock-cc only TXes 0x45 on engage/cancel. Wiper
  On/Int extra-forwards on the 10 Hz slot so nibble 1 stays held. Auto
  uses that same 10 Hz hold while rain/wiper-need is set. High
  extra-forwards every 10 ms so held nibble 4 can last-win against
  repeating bus-0 IDLE.
  """
  if not stalk_test_active(wiper_on, high_beam_on):
    return False
  if not high_beam_on and int(frame) % STW_FORWARD_SLOT != 0:
    return False
  return not any(msg[0] == STW_ACTN_RQ_ADDR for msg in can_sends)


def live_stw_counter(msg_stw) -> int:
  """Use the live/relayed MC. +1 builds a second competing 0x45."""
  if not msg_stw:
    return 0
  return int(msg_stw.get("MC_STW_ACTN_RQ", 0) or 0)


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


def replace_relayed_stw(dat: bytes, wiper_on: bool, high_beam_on: bool, crc_fn=None) -> bytes:
  """Edit the live/relayed 0x45 payload. Do not invent a second frame."""
  return overlay_stw_wiper_beam(dat, wiper_on, high_beam_on, crc_fn=crc_fn)


def send_replaced_live_stw(spoofer, CS, tesla_can, bus):
  """TX the live stalk with nibbles patched, same MC as bus 0 RX."""
  msg_stw = getattr(CS, "msg_stw_actn_req", None)
  if msg_stw is None:
    return None
  button = int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0)
  if tesla_can is None:
    return spoofer._send(CS, tesla_can, bus, button)
  return tesla_can.create_action_request(button, bus, live_stw_counter(msg_stw), msg_stw)


def _param_int(key: str, default: int = 0) -> int:
  try:
    from openpilot.common.params import Params
    val = Params().get(key, return_default=True)
    return int(val) if val is not None else default
  except Exception:
    return default


def _param_bool(key: str, default: bool = False) -> bool:
  try:
    from openpilot.common.params import Params
    return bool(Params().get_bool(key))
  except Exception:
    return default


def set_rain_wiper_needed(needed: bool | None) -> None:
  """Tests inject the rain/wiper-need signal. None returns to live sources."""
  global _rain_needed_override
  _rain_needed_override = None if needed is None else bool(needed)


def _rain_prob_from_model(model_v2) -> float | None:
  """Read a camera rain / wiper-need head if this fork's model publishes one."""
  if model_v2 is None:
    return None
  meta = model_v2
  if isinstance(model_v2, dict):
    meta = model_v2.get("meta", model_v2)
  else:
    meta = getattr(model_v2, "meta", model_v2)
  for name in _CAMERA_RAIN_KEYS:
    if isinstance(meta, dict):
      val = meta.get(name)
    else:
      val = getattr(meta, name, None)
    if val is None:
      continue
    try:
      return float(val)
    except (TypeError, ValueError):
      continue
  return None


def camera_rain_needed_from_model(model_v2) -> bool:
  """True when the camera rain / wiper-need probability is at or above the on threshold."""
  prob = _rain_prob_from_model(model_v2)
  if prob is None:
    return False
  return prob >= RAIN_PROB_ON


def _ensure_model_sm():
  """Lazy modelV2 reader. Fail closed (dry) if messaging is unavailable."""
  global _model_sm, _model_sm_failed
  if _model_sm_failed:
    return None
  if _model_sm is None:
    try:
      import cereal.messaging as messaging
      _model_sm = messaging.SubMaster(["modelV2"])
    except Exception:
      _model_sm_failed = True
      return None
  try:
    _model_sm.update(0)
  except Exception:
    return None
  return _model_sm


def camera_rain_needed() -> bool:
  """Live camera rain / wiper-need from modelV2. Dry if the head is absent."""
  try:
    sm = _ensure_model_sm()
    if sm is None:
      return False
    seen = getattr(sm, "seen", None)
    valid = getattr(sm, "valid", None)
    if seen is not None and not seen["modelV2"]:
      return False
    if valid is not None and not valid["modelV2"]:
      return False
    return camera_rain_needed_from_model(sm["modelV2"])
  except Exception:
    return False


def rain_wiper_needed() -> bool:
  """Rain / wiper-need for Auto. Default dry so Auto does not wipe every drive.

  Order: test override, NAPRainNeeded param (NAP rain path), camera rain head.
  """
  if _rain_needed_override is not None:
    return bool(_rain_needed_override)
  if _param_bool(NAP_RAIN_NEEDED, False):
    return True
  return camera_rain_needed()


def requested_wiper_test() -> bool:
  setting = _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF)
  if int(setting) == WIPER_SETTING_AUTO:
    return rain_wiper_needed()
  return wiper_test_requested(setting)


def requested_high_beam_test() -> bool:
  return high_beam_test_requested(_param_int(NAP_HIGH_LOW_BEAM, BEAM_SETTING_OFF))


def create_action_request_with_overlay(self, button_to_press, bus, counter, msg_stw=None):
  """Forward the live stalk, then overlay the test nibbles on that same frame."""
  orig = _ORIG_CREATE_ACTION_REQUEST
  if orig is None:
    orig = _tesla_can().create_action_request
  addr, dat, out_bus = orig(self, button_to_press, bus, counter, msg_stw)
  dat = replace_relayed_stw(dat, requested_wiper_test(), requested_high_beam_test(),
                            crc_fn=self.stw_crc)
  return addr, dat, out_bus


def stock_cc_update_with_overlay(self, CS, frame, tesla_can, can_bus_party):
  """Keep the single 0x45 TX path. When the test is on, forward if idle this slot.

  Does not read cruiseEnabled, latActive, or CC.enabled. A parked car with
  NAP not engaged and the stalk at rest is enough. High extra-forwards
  every 10 ms with held nibble 4 on the live-counter frame; wipers keep
  forwarding on the 10 Hz slot.
  """
  orig = _ORIG_STOCK_CC_UPDATE
  if orig is None:
    orig = _stock_cc().update
  wiper = requested_wiper_test()
  high_setting = requested_high_beam_test()
  can_sends = orig(self, CS, frame, tesla_can, can_bus_party)
  if extra_stw_forward_needed(can_sends, frame, wiper, high_setting):
    msg_stw = getattr(CS, "msg_stw_actn_req", None)
    if msg_stw is not None:
      if high_setting:
        # Same MC as the bus-0 RX rest — edit that frame, do not +1 a second 0x45.
        sent = send_replaced_live_stw(self, CS, tesla_can, can_bus_party)
      else:
        sent = self._send(CS, tesla_can, can_bus_party,
                          int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0))
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
