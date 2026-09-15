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

Off leaves the driver’s real stalk nibble alone (do not force 0) unless
Int/On just dropped to Off — then extra-forward rest so the body cancels.
Wiper On/Int holds high nibble 1. Auto holds that same nibble 1 only when
all of: setting is Auto, the vehicle is on, gear is Drive or Reverse, and
the 3X road camera sees a rainy or icy/frosted windshield (unwarped
ROAD Y). Park and Neutral never Auto-wipe, even with the car on.

Pre-AP latches ~32 s intermittent from nibble 1. Bus-0 rest does not
cancel that. Stopping the extra-forward when rain/hold drops leaves the
body wiping forever (status wipe=0 is honest — we are not commanding
wipe, but we also are not canceling). While Auto is selected and wipe is
False, still extra-forward 0x45 at the 10 Hz slot with the high nibble
cleared (real stalk rest) so the body gets an explicit cancel. On the
falling edge of wipe, send several rest frames immediately (do not wait
for the next 10 Hz slot). Park/Neutral stay wipe=0 and use that same
cancel if we had been wiping. Do not force wipe on dry glass. Off that
never Auto-wiped still leaves the stalk alone.

Dry overcast must not acquire or keep HOLD. Off is the escape. Default
Off — Auto is opt-in. No spray. No auto high-beam. Do not flash.
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

Collar3/Collar4 is a parked experiment on the same 0x45. Justin's 2014
4-click collar sends Off=0 Int1=1 Int2=2 Low=5 High=6 and never 3 or 4.
DBC WprSw6Posn INTERVAL3=3 INTERVAL4=4 is “unused on 4-click collar.”
Hypothesis: 3/4 are rain Auto. Overlay WprSw6Posn=3 or 4 and force
WprWashSw_Psd=0 (no TIPWIPE, no WASH). Live stalk Off TXes collar=0 on
bus 0 continuously; 10 Hz Int hold loses that last-win. Collar3/4 brute-
force holds like High: extra-forward every card frame (~100 Hz) on the
live MC, overlay 3/4 on the packed TX (even if create_action_request is
unpatched), resign CRC. Flicker vs the real stalk is acceptable on this
parked experiment; longer term Justin will use the ESP32 column gateway
(same as headlights) — do not build that here. Never a second 0x45 in
the same tick. Off extra-forwards live collar so the force drops. Do
not weaken panda safety — 0x45 is already whitelisted. Camera Auto
(#159) is unchanged while this setting is Off.
"""

# Params / UI. 0 is off (today's forwarded stalk). Indexes, not raw DBC.
import os
import threading
import time

NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_HIGH_LOW_BEAM = "NAPHighLowBeam"
NAP_WIPER_COLLAR = "NAPWiperCollar"

WIPER_SETTING_OFF = 0
WIPER_SETTING_INTERMITTENT = 1
WIPER_SETTING_ON = 2
WIPER_SETTING_AUTO = 3
BEAM_SETTING_OFF = 0
BEAM_SETTING_LOW = 1
BEAM_SETTING_HIGH = 2
COLLAR_SETTING_OFF = 0
COLLAR_SETTING_3 = 1
COLLAR_SETTING_4 = 2

STW_ACTN_RQ_ADDR = 0x45
STW_WIPER_BEAM_BYTE = 2
STW_WIPER_ON = 0x10
STW_WASHER_SPRAY = 0x20
STW_TURN_MASK = 0x03  # TurnIndLvr_Stat. Do not touch — blinker lat-pause.
STW_HIBM_MASK = 0x0C  # HiBmLvr_Stat bits 2-3 of the captured byte.
STW_HIGH_BEAM = 0x04  # HIBM_ON_PSD — held while High is selected
STW_HIGH_BEAM_FLASH = 0x08  # HIBM_FLSH_ON_PSD — never send
STW_WASH_MASK = 0x30  # WprWashSw_Psd bits 4-5. 0=NPSD 1=TIPWIPE 2=WASH.
STW_COLLAR_BYTE = 6
STW_COLLAR_MASK = 0x07  # WprSw6Posn bits 0-2. MC lives in the high nibble.
STW_COLLAR_POSN_3 = 3  # INTERVAL3 — unused on the 4-click collar
STW_COLLAR_POSN_4 = 4  # INTERVAL4 — unused on the 4-click collar
STW_FORWARD_SLOT = 10
# Falling-edge rest: several 10 ms frames so Pre-AP drops latched Int now.
STW_CANCEL_BURST_N = 8
# Synthesize rest 0x45 when parked parser has no STW yet. WprSw6Posn is
# d[6]&7 (DBC start bit 48, 3 bits LE) — Justin's live dump.
REST_STW_ACTN = {
  "SpdCtrlLvr_Stat": 0,
  "MC_STW_ACTN_RQ": 0,
  "CRC_STW_ACTN_RQ": 0,
  "DTR_Dist_Rq": 255,
  "VSL_Enbl_Rq": 1,
  "WprSw6Posn": 0,
  "WprWashSw_Psd": 0,
  "HiBmLvr_Stat": 0,
  "TurnIndLvr_Stat": 0,
}

_ORIG_CREATE_ACTION_REQUEST = None
_ORIG_STOCK_CC_UPDATE = None
_installed = False
_rain_needed_override = None
_live_cs = None
_vehicle_on_override = None
_gear_override = None
_cereal_gear_override = None
_cereal_gear_forced = False
_cereal_sm = None
# Names for Drive/Reverse. Pre-AP DI_torque2 is DI_GEAR_D / DI_GEAR_R.
_DRIVE_GEARS = (
  "drive", "reverse", "d", "r",
  "di_gear_d", "di_gear_r", "di_gear_drive", "di_gear_reverse",
)
_PARK_NEUTRAL_GEARS = (
  "park", "neutral", "p", "n",
  "di_gear_p", "di_gear_n", "di_gear_park", "di_gear_neutral",
)
# cereal/opendbc GearShifter.drive=2 reverse=4. Tesla DI_gear R=2 D=4 — both allowed.
_DRIVE_INTS = frozenset({2, 4})
# GearShifter.park=1 neutral=3. Not 0 — unknown must fall through to cereal.
_PARK_NEUTRAL_INTS = frozenset({1, 3})
_GEAR_ATTRS = (
  "gearShifter", "gear_shifter", "gear", "shifter",
  "DI_gear", "di_gear",
)
_AUTO_DEBUG_S = 1.0
# First Auto stock_cc must not connect ROAD during engage (modeld + GIL).
RAIN_HELPER_START_DELAY_S = 2.5
_last_auto_log_t = 0.0
_last_status_put_t = 0.0
_last_status_gate = None
_last_gear_src = "none"
_last_wiper_req = False
_wiper_cancel_burst = 0
_last_collar_on = False
_collar_cancel_burst = 0
_last_collar_status_t = 0.0
_params = None
_rain_mod = None
_rain_import_started = False
_auto_since_t = 0.0
_GEAR_SHIFTER_ENUMS = None


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
  NAPParamKeys.WIPER_COLLAR = NAP_WIPER_COLLAR
  DEFAULTS[NAP_WIPER_SPEED] = WIPER_SETTING_OFF
  DEFAULTS[NAP_HIGH_LOW_BEAM] = BEAM_SETTING_OFF
  DEFAULTS[NAP_WIPER_COLLAR] = COLLAR_SETTING_OFF
  try:
    migrate_wiper_collar_param()
  except Exception:
    pass


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


def apply_stw_wiper_beam_nibbles(dat: bytes, wiper_on: bool, high_beam_on: bool,
                                clear_wiper: bool = False) -> bytes:
  """Set captured stalk nibbles. Off leaves that nibble. Never writes spray.

  High holds HIBM_ON_PSD (4) for as long as the setting is High. Only HiBm
  bits are touched — turn-indicator bits stay for blinker lat-pause.
  Auto dry / wipe-release cancel sets clear_wiper so the high nibble is
  rest 0, not a leftover 1 that would keep Pre-AP intermittent.
  """
  if len(dat) <= STW_WIPER_BEAM_BYTE:
    return bytes(dat)
  out = bytearray(dat)
  b = out[STW_WIPER_BEAM_BYTE]
  if wiper_on:
    b = (b & 0x0F) | STW_WIPER_ON
  elif clear_wiper:
    b = b & 0x0F
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


def extra_stw_forward_needed(can_sends, frame: int, wiper_on: bool, high_beam_on: bool,
                             wiper_cancel: bool = False, cancel_now: bool = False,
                             collar_on: bool = False, collar_cancel: bool = False) -> bool:
  """One 0x45 when holding a nibble or sending rest-cancel. Never a second frame.

  Parked / not-engaged: stock-cc only TXes 0x45 on engage/cancel. Wiper
  On/Int extra-forwards on the 10 Hz slot so nibble 1 stays held. Auto
  uses that same 10 Hz hold while the glass looks rainy or icy, and the
  same 10 Hz slot with nibble NOT held while Auto is selected and dry so
  Pre-AP drops latched intermittent. Falling-edge cancel_now does not
  wait for the slot. High extra-forwards every 10 ms so held nibble 4
  can last-win against repeating bus-0 IDLE. Collar3/4 brute-force holds
  every card frame (~100 Hz, live-MC extra-forward) so bus-0 live Off
  cannot overwrite 3/4; flicker vs the real stalk is acceptable until
  the ESP32 column gateway. Off bursts live collar (no force) then
  leaves the stalk. candump src 0 is the live stalk; our TX echo is
  src 128 (returned | 0x80).
  """
  if not (stalk_test_active(wiper_on, high_beam_on) or wiper_cancel or collar_on or collar_cancel):
    return False
  if not high_beam_on and not collar_on and not cancel_now and int(frame) % STW_FORWARD_SLOT != 0:
    return False
  return not any(msg[0] == STW_ACTN_RQ_ADDR for msg in can_sends)


def live_stw_counter(msg_stw) -> int:
  """Use the live/relayed MC. +1 builds a second competing 0x45."""
  if not msg_stw:
    return 0
  return int(msg_stw.get("MC_STW_ACTN_RQ", 0) or 0)


def overlay_stw_wiper_beam(dat: bytes, wiper_on: bool, high_beam_on: bool, crc_fn=None,
                           clear_wiper: bool = False) -> bytes:
  """Apply nibbles and resign CRC only when the payload changed."""
  new_dat = apply_stw_wiper_beam_nibbles(dat, wiper_on, high_beam_on, clear_wiper=clear_wiper)
  if new_dat == dat:
    return dat
  if crc_fn is None or len(new_dat) < 8:
    return new_dat
  out = bytearray(new_dat)
  out[7] = crc_fn(bytes(out[:7]))
  return bytes(out)


def replace_relayed_stw(dat: bytes, wiper_on: bool, high_beam_on: bool, crc_fn=None,
                        clear_wiper: bool = False) -> bytes:
  """Edit the live/relayed 0x45 payload. Do not invent a second frame."""
  return overlay_stw_wiper_beam(dat, wiper_on, high_beam_on, crc_fn=crc_fn, clear_wiper=clear_wiper)


def collar_posn_for_setting(setting: int) -> int | None:
  """Map NAPWiperCollar to DBC WprSw6Posn. Off leaves the live collar.

  Persist 0/3/4 (cat /data/params/d/NAPWiperCollar shows 3 for Collar3).
  Legacy UI wrote 1=Collar3, 2=Collar4 — still accepted.
  """
  s = int(setting)
  if s in (COLLAR_SETTING_3, STW_COLLAR_POSN_3):
    return STW_COLLAR_POSN_3
  if s in (COLLAR_SETTING_4, STW_COLLAR_POSN_4):
    return STW_COLLAR_POSN_4
  return None


def persist_collar_posn(setting: int) -> int:
  """Value written to NAPWiperCollar: 0, 3, or 4."""
  posn = collar_posn_for_setting(setting)
  return 0 if posn is None else int(posn)


def collar_button_index(setting: int) -> int:
  """Off=0 Collar3=1 Collar4=2, from persisted 0/1/2/3/4."""
  posn = persist_collar_posn(setting)
  if posn == STW_COLLAR_POSN_3:
    return 1
  if posn == STW_COLLAR_POSN_4:
    return 2
  return 0


def stw_collar_posn(dat: bytes) -> int:
  if len(dat) <= STW_COLLAR_BYTE:
    return 0
  return dat[STW_COLLAR_BYTE] & STW_COLLAR_MASK


def stw_wash(dat: bytes) -> int:
  """WprWashSw_Psd: 0 NPSD, 1 TIPWIPE, 2 WASH, 3 SNA."""
  if len(dat) <= STW_WIPER_BEAM_BYTE:
    return 0
  return (dat[STW_WIPER_BEAM_BYTE] & STW_WASH_MASK) >> 4


def apply_stw_collar(dat: bytes, posn: int | None) -> bytes:
  """Force WprSw6Posn and WprWashSw_Psd=0. Off is identity. Never spray.

  Keeps MC (high nibble of byte 6), turn, high-beam, and rear-wash bits.
  Collar3/4 is the rain-Auto experiment — no TIPWIPE 0x10.
  """
  if posn is None or len(dat) <= STW_COLLAR_BYTE:
    return bytes(dat)
  out = bytearray(dat)
  out[STW_COLLAR_BYTE] = (out[STW_COLLAR_BYTE] & ~STW_COLLAR_MASK) | (int(posn) & STW_COLLAR_MASK)
  if len(out) > STW_WIPER_BEAM_BYTE:
    out[STW_WIPER_BEAM_BYTE] = out[STW_WIPER_BEAM_BYTE] & ~STW_WASH_MASK
  return bytes(out)


def overlay_stw_collar(dat: bytes, posn: int | None, crc_fn=None) -> bytes:
  """Apply collar force and resign CRC only when the payload changed."""
  new_dat = apply_stw_collar(dat, posn)
  if new_dat == dat:
    return dat
  if crc_fn is None or len(new_dat) < 8:
    return new_dat
  out = bytearray(new_dat)
  out[7] = crc_fn(bytes(out[:7]))
  return bytes(out)


def overlay_collar_on_can_msg(msg, tesla_can, posn: int | None):
  """Force WprSw6Posn / wash=0 on a packed 0x45 TX and resign CRC.

  Idle extra-forward must TX collar even when TeslaCANPreAP.create_action_request
  is the stock packer (install missed, or this instance is unpatched).
  Identity when posn is None, tesla_can is missing, or dat is too short.
  """
  if msg is None or posn is None or tesla_can is None:
    return msg
  try:
    addr, dat, bus = msg[0], msg[1], msg[2]
  except (TypeError, IndexError, ValueError):
    return msg
  if int(addr) != STW_ACTN_RQ_ADDR:
    return msg
  crc_fn = getattr(tesla_can, "stw_crc", None)
  if crc_fn is None:
    return msg
  new_dat = overlay_stw_collar(bytes(dat), int(posn), crc_fn=crc_fn)
  if new_dat == bytes(dat):
    return msg
  return (addr, new_dat, bus)


def build_collar_hold_msg(CS, tesla_can, bus=0):
  """Pack one 0x45 with WprSw6Posn=3/4 wash=0. Justin: collar is d[6]&7."""
  posn = requested_collar_posn()
  if posn is None or tesla_can is None:
    return None
  msg_stw = live_or_rest_stw(CS)
  button = int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0)
  sent = tesla_can.create_action_request(button, bus, live_stw_counter(msg_stw), msg_stw)
  return overlay_collar_on_can_msg(sent, tesla_can, posn)


def collar_hold_sends(can_sends, CS, tesla_can, bus=0):
  """Last-mile collar TX. Overlay every outgoing 0x45 or append one.

  Card calls this after apply (onroad) and from step when apply does not
  run (parked / not initialized). Never a second 0x45 in the same tick.
  """
  posn = requested_collar_posn()
  if posn is None:
    return list(can_sends)
  out = []
  had = False
  for msg in can_sends:
    try:
      addr = int(msg[0])
    except Exception:
      out.append(msg)
      continue
    if addr == STW_ACTN_RQ_ADDR:
      msg = overlay_collar_on_can_msg(msg, tesla_can, posn)
      had = True
    out.append(msg)
  if not had:
    built = build_collar_hold_msg(CS, tesla_can, bus)
    if built is not None:
      out.append(built)
  _note_collar_tx(posn, out, appended=not had)
  return out


def send_replaced_live_stw(spoofer, CS, tesla_can, bus):
  """TX the live stalk with nibbles patched, same MC as bus 0 RX."""
  msg_stw = live_or_rest_stw(CS)
  button = int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0)
  if tesla_can is None:
    return spoofer._send(CS, tesla_can, bus, button)
  return tesla_can.create_action_request(button, bus, live_stw_counter(msg_stw), msg_stw)


def live_or_rest_stw(CS) -> dict:
  """Parked extra-forward must not die if msg_stw_actn_req is missing."""
  msg = getattr(CS, "msg_stw_actn_req", None) if CS is not None else None
  if isinstance(msg, dict) and msg:
    return msg
  return dict(REST_STW_ACTN)


def _get_params():
  """Reuse one Params handle. Constructing Params() every 10 ms lagged card."""
  global _params
  if _params is None:
    from openpilot.common.params import Params
    _params = Params()
  return _params


def _collar_file_paths() -> list[str]:
  """Sidecar files so Collar3 survives if params_pyx wasn't rebuilt with NAPWiperCollar."""
  paths = []
  try:
    root = _get_params().get_param_path("")
    if root:
      paths.append(os.path.join(root, NAP_WIPER_COLLAR))
  except Exception:
    pass
  paths.append("/data/params/d/" + NAP_WIPER_COLLAR)
  out = []
  for path in paths:
    if path and path not in out:
      out.append(path)
  return out


def put_wiper_collar_setting(setting: int) -> None:
  """UI and card share this. File is DBC 0/3/4 so `cat .../NAPWiperCollar` shows 3."""
  s = persist_collar_posn(setting)
  try:
    _get_params().put(NAP_WIPER_COLLAR, s)
  except Exception:
    pass
  payload = str(s)
  for path in _collar_file_paths():
    try:
      parent = os.path.dirname(path)
      if parent:
        os.makedirs(parent, exist_ok=True)
      with open(path, "w", encoding="utf-8") as f:
        f.write(payload)
    except Exception:
      continue


def _collar_file_read() -> str | None:
  for path in _collar_file_paths():
    try:
      with open(path, encoding="utf-8") as f:
        txt = f.read().strip()
      if txt:
        return txt
    except Exception:
      continue
  return None


def read_wiper_collar_setting() -> int:
  """Prefer Params; if that is 0/unknown, honor a sidecar file write from the UI."""
  param_val = None
  try:
    raw = _get_params().get(NAP_WIPER_COLLAR, return_default=True)
    if raw is not None:
      param_val = int(raw)
  except Exception:
    param_val = None
  if param_val not in (None, COLLAR_SETTING_OFF):
    return persist_collar_posn(param_val)
  txt = _collar_file_read()
  if txt:
    try:
      file_val = int(txt)
      if file_val:
        return persist_collar_posn(file_val)
    except (TypeError, ValueError):
      pass
  return COLLAR_SETTING_OFF


def migrate_wiper_collar_param() -> None:
  """Rewrite legacy UI 1/2 to DBC 3/4 so cat shows 3 and mici does not pin Off."""
  try:
    val = read_wiper_collar_setting()
  except Exception:
    return
  if val not in (STW_COLLAR_POSN_3, STW_COLLAR_POSN_4):
    return
  raw_i = None
  try:
    raw = _get_params().get(NAP_WIPER_COLLAR, return_default=True)
    if raw is not None:
      raw_i = int(raw)
  except Exception:
    raw_i = None
  file_txt = _collar_file_read()
  if raw_i == val and file_txt == str(val):
    return
  put_wiper_collar_setting(val)


def _note_collar_tx(posn, can_sends, appended=False) -> None:
  """~1 Hz proof line. Do not stomp Auto's NAPWiperRainStatus."""
  global _last_collar_status_t
  now = time.monotonic()
  if now - _last_collar_status_t < 0.8:
    return
  _last_collar_status_t = now
  packed = None
  for msg in can_sends or ():
    try:
      if int(msg[0]) == STW_ACTN_RQ_ADDR:
        packed = bytes(msg[1])
        break
    except Exception:
      continue
  d6 = "-"
  if packed is not None and len(packed) > STW_COLLAR_BYTE:
    d6 = str(packed[STW_COLLAR_BYTE] & STW_COLLAR_MASK)
  line = "collar=%d tx=%d d6=%s src=128 installed=%d file=%s appended=%d" % (
    int(posn or 0),
    1 if packed is not None else 0,
    d6,
    int(_installed),
    _collar_file_read() or "-",
    int(bool(appended)),
  )
  try:
    _get_params().put("NAPWiperCollarStatus", line, block=False)
  except TypeError:
    try:
      _get_params().put("NAPWiperCollarStatus", line)
    except Exception:
      pass
  except Exception:
    pass
  if _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF) != WIPER_SETTING_AUTO:
    _put_wiper_status(line)
  for path in _collar_file_paths():
    status_path = os.path.join(os.path.dirname(path), "NAPWiperCollarStatus")
    try:
      parent = os.path.dirname(status_path)
      if parent:
        os.makedirs(parent, exist_ok=True)
      with open(status_path, "w", encoding="utf-8") as f:
        f.write(line + "\n")
      break
    except Exception:
      continue


def _param_int(key: str, default: int = 0) -> int:
  try:
    val = _get_params().get(key, return_default=True)
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


def set_cereal_gear(gear) -> None:
  """Tests inject cereal carState.gearShifter. None means cereal is empty."""
  global _cereal_gear_override, _cereal_gear_forced
  _cereal_gear_forced = True
  _cereal_gear_override = gear


def reset_auto_gates() -> None:
  global _live_cs, _vehicle_on_override, _gear_override, _last_gear_src, _last_wiper_req
  global _cereal_gear_override, _cereal_gear_forced, _wiper_cancel_burst
  global _last_collar_on, _collar_cancel_burst
  global _last_auto_log_t, _last_status_put_t, _last_status_gate, _auto_since_t
  global _last_collar_status_t
  _live_cs = None
  _vehicle_on_override = None
  _gear_override = None
  _cereal_gear_override = None
  _cereal_gear_forced = False
  _last_gear_src = "none"
  _last_wiper_req = False
  _wiper_cancel_burst = 0
  _last_collar_on = False
  _collar_cancel_burst = 0
  _last_auto_log_t = 0.0
  _last_status_put_t = 0.0
  _last_status_gate = None
  _last_collar_status_t = 0.0
  _auto_since_t = 0.0


def _gear_name(gear) -> str:
  """Token for Drive/Reverse. capnp _DynamicEnum has .name=None and str()=='drive'."""
  if gear is None:
    return ""
  tokens = []
  try:
    tokens.append(str(gear))
  except Exception:
    pass
  name = getattr(gear, "name", None)
  if isinstance(name, str) and name:
    tokens.append(name)
  for raw in tokens:
    token = str(raw).rsplit(".", 1)[-1].lower().strip()
    if token and token not in ("none", "null"):
      return token
  return ""


def _gear_int(gear):
  if isinstance(gear, bool):
    return None
  if isinstance(gear, int):
    return int(gear)
  for attr in ("raw",):
    try:
      v = getattr(gear, attr, None)
      if isinstance(v, int) and not isinstance(v, bool):
        return int(v)
    except Exception:
      pass
  try:
    return int(gear)
  except Exception:
    return None


def _gear_present(gear) -> bool:
  """True if a gear value was found. GearShifter.unknown is 0 — still present."""
  return gear is not None and gear != ""


def _gear_type_name(gear) -> str:
  if gear is None:
    return "none"
  try:
    return type(gear).__name__
  except Exception:
    return "?"


def _is_drive_or_reverse(gear) -> bool:
  """Drive/Reverse. Token first: cereal _DynamicEnum != structs.GearShifter.drive."""
  if not _gear_present(gear):
    return False
  if _gear_name(gear) in _DRIVE_GEARS:
    return True
  gi = _gear_int(gear)
  if gi in _DRIVE_INTS:
    return True
  for gs in _gear_shifter_enums():
    try:
      if gear == gs.drive or gear == gs.reverse:
        return True
    except Exception:
      pass
    try:
      if gi is not None and gi in (int(gs.drive), int(gs.reverse)):
        return True
    except Exception:
      pass
    for side in (gs.drive, gs.reverse):
      try:
        if _gear_name(gear) == _gear_name(side):
          return True
      except Exception:
        pass
  return False


def _is_park_or_neutral(gear) -> bool:
  """Known Park/Neutral. Unknown 0 is missing, not Park."""
  if not _gear_present(gear):
    return False
  name = _gear_name(gear)
  if name in _PARK_NEUTRAL_GEARS:
    return True
  if name in ("unknown", "invalid", "di_gear_invalid", "sna"):
    return False
  gi = _gear_int(gear)
  if gi in _PARK_NEUTRAL_INTS and name not in _DRIVE_GEARS:
    return True
  for gs in _gear_shifter_enums():
    for side_name in ("park", "neutral"):
      side = getattr(gs, side_name, None)
      if side is None:
        continue
      try:
        if gear == side:
          return True
      except Exception:
        pass
      try:
        if _gear_name(gear) == _gear_name(side):
          return True
      except Exception:
        pass
  return False


def _gear_known(gear) -> bool:
  """Drive/Reverse/Park/Neutral. unknown 0 is not known — fall through to cereal."""
  return _is_drive_or_reverse(gear) or _is_park_or_neutral(gear)


def _gear_shifter_enums():
  """Import once. Re-importing opendbc/cereal on the 10 ms path hit the GIL."""
  global _GEAR_SHIFTER_ENUMS
  if _GEAR_SHIFTER_ENUMS is not None:
    return _GEAR_SHIFTER_ENUMS
  found = []
  try:
    from opendbc.car import structs
    found.append(structs.CarState.GearShifter)
  except Exception:
    pass
  try:
    from cereal import car
    gs = car.CarState.GearShifter
    if gs not in found:
      found.append(gs)
  except Exception:
    pass
  _GEAR_SHIFTER_ENUMS = tuple(found)
  return _GEAR_SHIFTER_ENUMS


def _attr_gear(obj, attr: str):
  if obj is None:
    return None
  try:
    if isinstance(obj, dict):
      return obj.get(attr)
    return getattr(obj, attr, None)
  except Exception:
    return None


def _host_gears(host, src: str) -> list[tuple[object, str]]:
  found = []
  if host is None:
    return found
  for attr in _GEAR_ATTRS:
    gear = _attr_gear(host, attr)
    if _gear_present(gear):
      found.append((gear, src if attr == "gearShifter" else f"{src}.{attr}"))
  if isinstance(host, dict):
    for key in ("DI_gear", "gearShifter"):
      if key in host and _gear_present(host[key]):
        found.append((host[key], src))
  return found


def _cs_gear(cs) -> tuple[object, str]:
  """Live stock-cc CS is the inner parser. Drive is on CS.out / cereal carState.

  Prefer a candidate that already looks like Drive/Reverse so an inner
  unknown default cannot hide CS.out (cereal _DynamicEnum, str='drive').
  Unknown 0 is not a gear — caller may fall through to cereal.
  """
  if cs is None:
    return None, "none"
  candidates = []
  candidates.extend(_host_gears(cs, "cs"))
  out = _attr_gear(cs, "out")
  if out is not None:
    candidates.extend(_host_gears(out, "out"))
  for key in ("msg_di_torque2", "DI_torque2"):
    msg = _attr_gear(cs, key)
    if isinstance(msg, dict):
      g = msg.get("DI_gear")
      if _gear_present(g):
        candidates.append((g, "di_torque2"))
  for gear, src in candidates:
    if _is_drive_or_reverse(gear):
      return gear, src
  for gear, src in candidates:
    if _is_park_or_neutral(gear):
      return gear, src
  return None, "none"


def _read_cereal_gear() -> tuple[object, str]:
  """Published carState.gearShifter. Justin's probe: drive _DynamicEnum."""
  if _cereal_gear_forced:
    if _gear_present(_cereal_gear_override):
      return _cereal_gear_override, "cereal"
    return None, "none"
  global _cereal_sm
  try:
    if _cereal_sm is None:
      import cereal.messaging as messaging
      _cereal_sm = messaging.SubMaster(["carState"])
    _cereal_sm.update(0)
    cs = _cereal_sm["carState"]
    gear = _attr_gear(cs, "gearShifter")
    if not _gear_present(gear):
      gear = _attr_gear(cs, "gear")
    if _gear_known(gear):
      return gear, "cereal"
  except Exception:
    pass
  return None, "none"


def _resolved_gear() -> tuple[object, str]:
  """CS aliases first. Cereal carState when inner CS has no known gear."""
  if _gear_override is not None:
    return _gear_override, "override"
  gear, src = _cs_gear(_live_cs)
  if _gear_known(gear):
    return gear, src
  cg, csrc = _read_cereal_gear()
  if _gear_known(cg):
    return cg, csrc
  if _gear_present(gear):
    return gear, src
  return cg, csrc


def vehicle_is_on() -> bool:
  """Onroad CarState is only published while the vehicle is on."""
  if _vehicle_on_override is not None:
    return bool(_vehicle_on_override)
  return _live_cs is not None


def in_drive_gear() -> bool:
  """Drive or Reverse. Park, Neutral, and unknown do not Auto-wipe."""
  global _last_gear_src
  gear, src = _resolved_gear()
  _last_gear_src = src
  return _is_drive_or_reverse(gear)


def rain_wiper_needed() -> bool:
  """Rainy or icy/frosted windshield latch. Default clear so Auto does not wipe every drive.

  Order: test override, then the 3X ROAD camera near-glass check.
  """
  if _rain_needed_override is not None:
    return bool(_rain_needed_override)
  try:
    rain = _rain_module()
    if rain is None:
      return False
    return bool(rain.windshield_rain_needed())
  except Exception:
    return False


def _rain_module():
  """Already-imported rain module only. Never import numpy on the card RT thread."""
  global _rain_mod
  if _rain_mod is not None:
    return _rain_mod
  import sys
  m = sys.modules.get("openpilot.selfdrive.car.tesla.preap_windshield_rain")
  if m is not None:
    _rain_mod = m
  return _rain_mod


def _preimport_rain_module() -> None:
  """Background numpy import. stock_cc.update must not pay this on CTRL_HIGH."""
  global _rain_mod
  try:
    from openpilot.selfdrive.car.tesla import preap_windshield_rain as rain
    _rain_mod = rain
  except Exception:
    pass


def _kick_rain_import() -> None:
  global _rain_import_started
  if _rain_import_started:
    return
  _rain_import_started = True
  threading.Thread(target=_preimport_rain_module, name="nap-wiper-import", daemon=True).start()


def _sync_rain_helper() -> None:
  """VisionIpc + numpy only while Wipers = Auto. Off/Int/On must not recv ROAD."""
  global _auto_since_t
  if _rain_needed_override is not None:
    return
  setting = _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF)
  rain = _rain_module()
  if int(setting) != WIPER_SETTING_AUTO:
    _auto_since_t = 0.0
    if rain is not None:
      try:
        rain.stop_windshield_rain_helper()
      except Exception:
        pass
    return
  now = time.monotonic()
  if _auto_since_t <= 0.0:
    _auto_since_t = now
  if rain is None:
    _kick_rain_import()
    return
  if now - _auto_since_t < float(RAIN_HELPER_START_DELAY_S):
    return
  try:
    rain.ensure_windshield_rain_helper()
  except Exception:
    pass


def _auto_status_line(setting: int, on: bool, drive: bool, rain: bool, wipe: bool) -> str:
  gear, src = _resolved_gear()
  rain_bits = "rain=0"
  try:
    from openpilot.selfdrive.car.tesla import preap_windshield_rain as rainmod
    d = rainmod._detector
    if d is not None:
      now = time.monotonic()
      age_ms = (now - d._last_frame_t) * 1000.0 if d._last_frame_t else -1.0
      pulse_s = float(getattr(rainmod, "WIPE_PULSE_S", 0.0))
      wait_s = float(getattr(rainmod, "CLEAR_WAIT_S", 0.0))
      wipe_t0 = float(getattr(d, "_wipe_t0", 0.0) or 0.0)
      wait_t0 = float(getattr(d, "_wait_t0", 0.0) or 0.0)
      pulse_left = max(0.0, pulse_s - (now - wipe_t0)) if d.hold and wipe_t0 else 0.0
      wait_left = max(0.0, wait_s - (now - wait_t0)) if wait_t0 else 0.0
      rain_bits = (
        f"hold={int(d.hold)} holdn={int(getattr(d, '_hold_n', 0))}/{int(getattr(rainmod, 'MIN_HOLD_N', 0))} "
        + f"warm={int(getattr(d, '_warm_n', 0))}/{int(getattr(rainmod, 'WARMUP_N', 0))} "
        + f"ema={d.ema:.2f} score={d.last_score:.2f} bokeh={d.last_bokeh:.2f} blob={d.last_blob:.2f} "
        + f"speckle={d.last_speckle:.3f} sparse={d.last_sparse:.1f} struct={d.last_structure:.3f} sat={d.last_sat:.3f} "
        + f"clear={int(getattr(d, '_clear_n', 0))}/{int(getattr(rainmod, 'CLEAR_RELEASE_N', 0))} "
        + f"heavy={int(d.last_score >= float(getattr(rainmod, 'HEAVY_ON', 2.2)))} "
        + f"pulse={pulse_left:.1f}/{pulse_s:.1f} wait={wait_left:.1f}/{wait_s:.1f} "
        + f"connected={int(d.connected)} failed={int(d._failed)} frames={d.n_frames} stream={d.stream} "
        + f"helper={int(d.helper_alive)} period_s={float(getattr(rainmod, 'SCORE_PERIOD_S', 0)):.1f} "
        + f"hz={float(getattr(rainmod, 'SCORE_HZ', 0)):.1f} age_ms={age_ms:.0f} err={d.last_err or '-'}"
      )
  except Exception:
    rain_bits = "rain=err"
  gi = _gear_int(gear)
  raw = "-" if gi is None else str(gi)
  return (
    f"nap wiper auto setting={int(setting)} on={int(on)} gear={_gear_name(gear) or '-'} gear_src={src} "
    + f"gear_type={_gear_type_name(gear)} raw={raw} drive={int(drive)} rain={int(rain)} wipe={int(wipe)} "
    + f"cancel={int(wiper_rest_tx_needed(wipe))} "
    + f"installed={int(_installed)} {rain_bits}"
  )


def _put_wiper_status(line: str) -> None:
  """Write NAPWiperRainStatus. Callers must rate-limit — this is the expensive put."""
  try:
    _get_params().put("NAPWiperRainStatus", line, block=False)
  except TypeError:
    try:
      _get_params().put("NAPWiperRainStatus", line)
    except Exception:
      pass
  except Exception:
    pass


def _log_auto_status(setting: int, on: bool, drive: bool, rain: bool, wipe: bool) -> None:
  """Params.put / cloudlog at 1 Hz, or immediately on Auto gate changes. Not every 10 ms."""
  global _last_auto_log_t, _last_status_put_t, _last_status_gate
  gate = (int(setting), bool(on), bool(drive), bool(rain), bool(wipe))
  now = time.monotonic()
  if gate == _last_status_gate and now - _last_status_put_t < _AUTO_DEBUG_S:
    return
  _last_status_gate = gate
  _last_status_put_t = now
  line = _auto_status_line(setting, on, drive, rain, wipe)
  if now - _last_auto_log_t >= _AUTO_DEBUG_S:
    _last_auto_log_t = now
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.info("%s", line)
    except Exception:
      pass
  _put_wiper_status(line)


def wiper_rest_tx_needed(wiper_on: bool | None = None) -> bool:
  """Auto dry, or a wipe 1→0 burst, still extra-forwards rest. Off-never-wiped does not."""
  if wiper_on is None:
    wiper_on = _last_wiper_req
  if wiper_on:
    return False
  if _wiper_cancel_burst > 0:
    return True
  return _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF) == WIPER_SETTING_AUTO


def _arm_wiper_cancel(prev_wiper: bool, wiper_on: bool) -> None:
  global _last_wiper_req, _wiper_cancel_burst
  if prev_wiper and not wiper_on:
    _wiper_cancel_burst = STW_CANCEL_BURST_N
  _last_wiper_req = bool(wiper_on)


def _note_wiper_cancel_frame() -> None:
  global _wiper_cancel_burst
  if _wiper_cancel_burst > 0:
    _wiper_cancel_burst -= 1


def requested_collar_posn() -> int | None:
  return collar_posn_for_setting(read_wiper_collar_setting())


def requested_collar_test() -> bool:
  global _last_collar_on
  on = requested_collar_posn() is not None
  _last_collar_on = on
  return on


def collar_rest_tx_needed(collar_on: bool | None = None) -> bool:
  """Collar3/4 → Off bursts live stalk so the forced posn drops. Off-never-forced does not."""
  if collar_on is None:
    collar_on = _last_collar_on
  if collar_on:
    return False
  return _collar_cancel_burst > 0


def _arm_collar_cancel(prev_collar: bool, collar_on: bool) -> None:
  global _last_collar_on, _collar_cancel_burst
  if prev_collar and not collar_on:
    _collar_cancel_burst = STW_CANCEL_BURST_N
  _last_collar_on = bool(collar_on)


def _note_collar_cancel_frame() -> None:
  global _collar_cancel_burst
  if _collar_cancel_burst > 0:
    _collar_cancel_burst -= 1


def requested_wiper_test() -> bool:
  global _last_wiper_req
  setting = _param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF)
  _sync_rain_helper()
  if int(setting) == WIPER_SETTING_AUTO:
    on = vehicle_is_on()
    drive = in_drive_gear()
    rain = False
    try:
      rain = rain_wiper_needed()
    except Exception:
      rain = False
    wipe = bool(on and drive and rain)
    _last_wiper_req = wipe
    _log_auto_status(setting, on, drive, rain, wipe)
    return wipe
  wipe = wiper_test_requested(setting)
  _last_wiper_req = wipe
  return wipe


def requested_high_beam_test() -> bool:
  return high_beam_test_requested(_param_int(NAP_HIGH_LOW_BEAM, BEAM_SETTING_OFF))


def create_action_request_with_overlay(self, button_to_press, bus, counter, msg_stw=None):
  """Forward the live stalk, then overlay the test nibbles on that same frame."""
  orig = _ORIG_CREATE_ACTION_REQUEST
  if orig is None:
    orig = _tesla_can().create_action_request
  addr, dat, out_bus = orig(self, button_to_press, bus, counter, msg_stw)
  wiper = requested_wiper_test()
  dat = replace_relayed_stw(dat, wiper, requested_high_beam_test(),
                            crc_fn=self.stw_crc, clear_wiper=wiper_rest_tx_needed(wiper))
  dat = overlay_stw_collar(dat, requested_collar_posn(), crc_fn=self.stw_crc)
  return addr, dat, out_bus


def stock_cc_update_with_overlay(self, CS, frame, tesla_can, can_bus_party):
  """Keep the single 0x45 TX path. When the test is on, forward if idle this slot.

  Does not read cruiseEnabled, latActive, or CC.enabled. High extra-forwards
  every 10 ms with held nibble 4 on the live-counter frame; wipers keep
  forwarding on the 10 Hz slot. Auto dry / wipe-release extra-forwards
  rest (cleared high nibble) on that same slot so Pre-AP drops latched
  intermittent; a wipe 1→0 burst does not wait for the slot. Auto reads
  gear from this CS: Park/Neutral stay wipe=0 and still cancel if we had
  been wiping. Primes the ROAD VisionIpc helper only while Auto so poll()
  does not recv on this CTRL_HIGH thread. Off/Int/On stop the helper.
  Collar3/4 brute-force extra-forwards every card frame (~100 Hz) like
  High (live MC, in-place replace) with WprSw6Posn forced and
  WprWashSw_Psd=0 so bus-0 Off cannot last-win. Flicker vs the real
  stalk is acceptable on this parked experiment; do not build the
  ESP32 column gateway here. Off bursts live collar then leaves the
  stalk. Our TX echo is candump src 128, not src 0 (live stalk).
  """
  orig = _ORIG_STOCK_CC_UPDATE
  if orig is None:
    orig = _stock_cc().update
  update_live_car_state(CS)
  _sync_rain_helper()
  prev_wiper = _last_wiper_req
  wiper = requested_wiper_test()
  _arm_wiper_cancel(prev_wiper, wiper)
  prev_collar = _last_collar_on
  collar = requested_collar_test()
  _arm_collar_cancel(prev_collar, collar)
  high_setting = requested_high_beam_test()
  wiper_cancel = wiper_rest_tx_needed(wiper)
  collar_cancel = collar_rest_tx_needed(collar)
  cancel_now = bool((wiper_cancel and _wiper_cancel_burst > 0) or
                    (collar_cancel and _collar_cancel_burst > 0))
  can_sends = orig(self, CS, frame, tesla_can, can_bus_party)
  had_stw = any(msg[0] == STW_ACTN_RQ_ADDR for msg in can_sends)
  if extra_stw_forward_needed(can_sends, frame, wiper, high_setting,
                              wiper_cancel=wiper_cancel, cancel_now=cancel_now,
                              collar_on=collar, collar_cancel=collar_cancel):
    msg_stw = getattr(CS, "msg_stw_actn_req", None)
    if msg_stw is None and (collar or high_setting):
      msg_stw = live_or_rest_stw(CS)
    if msg_stw is not None:
      if high_setting or collar:
        # Same MC as the bus-0 RX rest — edit that frame, do not +1 a second 0x45.
        # Collar must last-win like High: live Off repeats collar=0 on bus 0.
        sent = send_replaced_live_stw(self, CS, tesla_can, can_bus_party)
      else:
        # Int hold (10 Hz) and collar Off-cancel burst.
        sent = self._send(CS, tesla_can, can_bus_party,
                          int(msg_stw.get("SpdCtrlLvr_Stat", 0) or 0))
      if sent is not None:
        # Pack collar on the TX itself. create_action_request overlay is not
        # enough — if that method is unpatched, live-MC replace would TX Off
        # (collar=0) and the 0x45 monitor would not move.
        sent = overlay_collar_on_can_msg(sent, tesla_can, requested_collar_posn())
        can_sends.append(sent)
        had_stw = True
  if had_stw:
    if wiper_cancel:
      _note_wiper_cancel_frame()
    if collar_cancel:
      _note_collar_cancel_frame()
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
    cloudlog.info("nap body controls overlay installed (0x45 wiper/beam/collar)")
  except Exception:
    pass
  _put_wiper_status("nap wiper auto setting=- on=0 gear=- gear_src=none raw=- drive=0 rain=0 wipe=0 installed=1 waiting")
