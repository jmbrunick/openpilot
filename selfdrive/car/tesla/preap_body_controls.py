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
On/Int holds high nibble 1. Auto holds that same nibble 1 only when all
of: setting is Auto, the vehicle is on, gear is Drive or Reverse, and
the 3X road camera sees a rainy or icy/frosted windshield (unwarped
ROAD Y). Park and Neutral never Auto-wipe, even with the car on. Release
the real stalk when the glass looks clear or gear leaves Drive/Reverse.
Default Off — Auto is opt-in. No spray. No auto high-beam. Do not flash.
Do not inject a second 0x45 — overlay the existing forwarded frame and
recompute CRC the same way create_action_request already does.

Panda already allows TX of 0x45 on bus 0 (stalk spoof whitelist). The TX
hook does not gate 0x45 on controls_allowed — stock-CC engage already
sends this ID while disengaged. 0x3E9 DAS_bodyControls *is* gated; that
is why this must not use DAS. Do not bypass safety if that ever
changes. Do not fake this through another ID.

Auto is not gated on cruiseEnabled, latActive, or a stalk pull. Int/On
do not use the camera or gear gate.

Known risk: pre-AP may still see the real stalk rest on bus 0. Int already
wins when held, so Auto uses that same hold, not a pulse.
"""

# Params / UI. 0 is off (today's forwarded stalk). Indexes, not raw DBC.
import time

NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_HIGH_LOW_BEAM = "NAPHighLowBeam"

WIPER_SETTING_OFF = 0
WIPER_SETTING_INTERMITTENT = 1
WIPER_SETTING_ON = 2
WIPER_SETTING_AUTO = 3
BEAM_SETTING_OFF = 0
BEAM_SETTING_LOW = 1
BEAM_SETTING_HIGH = 2

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
_live_cs = None
_vehicle_on_override = None
_gear_override = None
# Names for Drive/Reverse. Pre-AP DI_torque2 is DI_GEAR_D / DI_GEAR_R.
_DRIVE_GEARS = (
  "drive", "reverse", "d", "r",
  "di_gear_d", "di_gear_r", "di_gear_drive", "di_gear_reverse",
)
# cereal/opendbc GearShifter.drive=2 reverse=4. Tesla DI_gear R=2 D=4 — both allowed.
_DRIVE_INTS = frozenset({2, 4})
_AUTO_DEBUG_S = 1.0
_last_auto_log_t = 0.0
_last_gear_src = "none"


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
  """Int and On hold nibble 1. Auto holds it only while the glass is not clear.

  Gear / vehicle-on are applied in requested_wiper_test, not here, so packing
  tests can still check the nibble without a CarState.
  """
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
  uses that same 10 Hz hold while the glass looks rainy or icy. High
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
    if val is None:
      return default
    if isinstance(val, (bytes, bytearray)):
      val = val.decode("utf-8", errors="ignore").strip()
    return int(val)
  except Exception:
    return default


def set_rain_wiper_needed(needed: bool | None) -> None:
  """Tests inject the rain/ice/wiper-need signal. None returns to the camera."""
  global _rain_needed_override
  _rain_needed_override = None if needed is None else bool(needed)


def update_live_car_state(cs) -> None:
  """Stock-cc update publishes gear / vehicle-on for Auto."""
  global _live_cs
  _live_cs = cs


def set_auto_gates(vehicle_on: bool | None = None, gear=None) -> None:
  """Tests inject vehicle-on and gear. None leaves that field on live CS."""
  global _vehicle_on_override, _gear_override
  _vehicle_on_override = None if vehicle_on is None else bool(vehicle_on)
  _gear_override = gear


def reset_auto_gates() -> None:
  global _live_cs, _vehicle_on_override, _gear_override, _last_gear_src
  _live_cs = None
  _vehicle_on_override = None
  _gear_override = None
  _last_gear_src = "none"


def _gear_name(gear) -> str:
  if gear is None:
    return ""
  name = getattr(gear, "name", None)
  if isinstance(name, str) and name:
    return name.rsplit(".", 1)[-1].lower()
  token = str(gear).rsplit(".", 1)[-1].lower().strip()
  if token:
    return token
  return ""


def _gear_int(gear):
  if isinstance(gear, bool):
    return None
  if isinstance(gear, int):
    return int(gear)
  try:
    if hasattr(gear, "value") and not isinstance(gear, str):
      return int(gear.value)
  except Exception:
    pass
  try:
    return int(gear)
  except Exception:
    return None


def _gear_present(gear) -> bool:
  """True if a gear value was found. GearShifter.unknown is 0 — still present."""
  return gear is not None and gear != ""


def _cs_gear(cs) -> tuple[object, str]:
  """Live Pre-AP stock-cc CS is the inner parser: Drive lives on CS.out.gearShifter.

  structs.CarState.gearShifter is published on CS.out, not on the Tesla
  CarState object StockCCSpoofer.update receives. Reading only
  CS.gearShifter was None in Drive, so Auto never wiped.
  """
  if cs is None:
    return None, "none"
  gear = getattr(cs, "gearShifter", None)
  if _gear_present(gear):
    return gear, "cs"
  out = getattr(cs, "out", None)
  if out is not None:
    gear = getattr(out, "gearShifter", None)
    if _gear_present(gear):
      return gear, "out"
  return None, "none"


def vehicle_is_on() -> bool:
  """Onroad CarState is only published while the vehicle is on."""
  if _vehicle_on_override is not None:
    return bool(_vehicle_on_override)
  return _live_cs is not None


def in_drive_gear() -> bool:
  """Drive or Reverse. Park, Neutral, and unknown do not Auto-wipe."""
  global _last_gear_src
  if _gear_override is not None:
    gear, src = _gear_override, "override"
  else:
    gear, src = _cs_gear(_live_cs)
  _last_gear_src = src
  if not _gear_present(gear):
    return False
  try:
    from opendbc.car import structs
    gs = structs.CarState.GearShifter
    if gear in (gs.drive, gs.reverse):
      return True
    gi = _gear_int(gear)
    if gi is not None and gi in (int(gs.drive), int(gs.reverse)):
      return True
  except Exception:
    pass
  try:
    from cereal import car
    gs = car.CarState.GearShifter
    if gear in (gs.drive, gs.reverse):
      return True
  except Exception:
    pass
  if _gear_name(gear) in _DRIVE_GEARS:
    return True
  gi = _gear_int(gear)
  return gi in _DRIVE_INTS if gi is not None else False


def rain_wiper_needed() -> bool:
  """Rainy or icy/frosted windshield latch. Default clear so Auto does not wipe every drive.

  Order: test override, then the 3X ROAD camera near-glass check.
  """
  if _rain_needed_override is not None:
    return bool(_rain_needed_override)
  try:
    from openpilot.selfdrive.car.tesla.preap_windshield_rain import windshield_rain_needed
    return bool(windshield_rain_needed())
  except Exception:
    return False


def _auto_status_line(setting: int, on: bool, drive: bool, rain: bool, wipe: bool) -> str:
  if _gear_override is not None:
    gear, src = _gear_override, "override"
  else:
    gear, src = _cs_gear(_live_cs)
  rain_bits = "rain=0"
  try:
    from openpilot.selfdrive.car.tesla import preap_windshield_rain as rainmod
    d = rainmod._detector
    if d is not None:
      rain_bits = (
        "hold=%d ema=%.2f score=%.2f bokeh=%.2f sparse=%.1f connected=%d failed=%d "
        "frames=%d stream=%s err=%s" % (
          int(d.hold), d.ema, d.last_score, d.last_bokeh, d.last_sparse,
          int(d.connected), int(d._failed), d.n_frames, d.stream, d.last_err or "-",
        )
      )
  except Exception:
    rain_bits = "rain=err"
  gi = _gear_int(gear)
  raw = "-" if gi is None else str(gi)
  return (
    "nap wiper auto setting=%d on=%d gear=%s gear_src=%s raw=%s drive=%d rain=%d "
    "wipe=%d installed=%d %s" % (
      int(setting), int(on), _gear_name(gear) or "-", src, raw, int(drive), int(rain),
      int(wipe), int(_installed), rain_bits,
    )
  )


def _put_wiper_status(line: str) -> None:
  """Write NAPWiperRainStatus so `cat /data/params/d/NAPWiperRainStatus` always has gates.

  Rain _debug no longer writes this key. Non-blocking so the 100 Hz car
  thread does not hitch; this is the only writer.
  """
  try:
    from openpilot.common.params import Params
    Params().put("NAPWiperRainStatus", line, block=False)
  except TypeError:
    try:
      from openpilot.common.params import Params
      Params().put("NAPWiperRainStatus", line)
    except Exception:
      pass
  except Exception:
    pass


def _log_auto_status(setting: int, on: bool, drive: bool, rain: bool, wipe: bool) -> None:
  global _last_auto_log_t
  now = time.monotonic()
  if now - _last_auto_log_t < _AUTO_DEBUG_S:
    return
  _last_auto_log_t = now
  line = _auto_status_line(setting, on, drive, rain, wipe)
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.info("%s", line)
  except Exception:
    pass
  _put_wiper_status(line)


def requested_wiper_test() -> bool:
  setting = _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF)
  if int(setting) == WIPER_SETTING_AUTO:
    on = vehicle_is_on()
    drive = in_drive_gear()
    rain = False
    try:
      rain = rain_wiper_needed()
    except Exception:
      rain = False
    wipe = bool(on and drive and rain)
    _log_auto_status(setting, on, drive, rain, wipe)
    return wipe
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

  Does not read cruiseEnabled, latActive, or CC.enabled. High extra-forwards
  every 10 ms with held nibble 4 on the live-counter frame; wipers keep
  forwarding on the 10 Hz slot. Auto reads gear from this CS: Park/Neutral
  release the stalk even if the glass still looks wet.
  """
  orig = _ORIG_STOCK_CC_UPDATE
  if orig is None:
    orig = _stock_cc().update
  update_live_car_state(CS)
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
  DAS_turnIndicatorRequest stays on teslacan.create_body_controls_message,
  driven by CC.leftBlinker / rightBlinker while ALC is armed or in progress.
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
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.info("nap body controls overlay installed (0x45 wiper/beam)")
  except Exception:
    pass
  _put_wiper_status("nap wiper auto setting=- on=0 gear=- gear_src=none raw=- drive=0 rain=0 wipe=0 installed=1 waiting")
