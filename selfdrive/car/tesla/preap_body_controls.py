"""Optional Pre-AP DAS_bodyControls wiper and high/low-beam test.

NAP already sends DAS_bodyControls (0x3E9) for the lane-change blinker.
teslacan forces DAS_wiperSpeed, DAS_headlightRequest, and
DAS_highLowBeamDecision to 0. This module replaces that builder so a
driver-facing NAP setting can request a wiper speed or a high/low beam
decision. Default is off: those fields stay 0 and the frame matches
today's blinker-only TX.

Known risk: pre-AP body controllers often ignore Autopilot wiper/beam
requests. This is a car test, not a promise it works.

Do not command defrost. The only defrost signal on this bus
(MCU_frontDefrostReq_das) is the car telling the 3X that defrost was
requested.
"""

from opendbc.car.tesla.preap.nap_params import DEFAULTS, NAPParamKeys

# Params / UI. 0 is off (today's TX). Values are setting indexes, not raw DBC.
NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_HIGH_LOW_BEAM = "NAPHighLowBeam"

# DAS_wiperSpeed: 0 off, 1-14 speeds. No rain model — the setting is the request.
# Intermittent = slowest non-zero (1). On = mid continuous (8), not max (14).
WIPER_SETTING_OFF = 0
WIPER_SETTING_INTERMITTENT = 1
WIPER_SETTING_ON = 2
DAS_WIPER_OFF = 0
DAS_WIPER_INTERMITTENT = 1
DAS_WIPER_ON = 8

# DAS_highLowBeamDecision: 0 undecided, 1 off/low, 2 on/high.
# DAS_headlightRequest is headlights on/off, not dimming — leave it 0.
BEAM_SETTING_OFF = 0
BEAM_SETTING_LOW = 1
BEAM_SETTING_HIGH = 2
DAS_HIGH_BEAM_UNDECIDED = 0
DAS_HIGH_BEAM_OFF = 1
DAS_HIGH_BEAM_ON = 2

_ORIG_CREATE_BODY_CONTROLS = None
_installed = False


def _tesla_can():
  from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
  return TeslaCANPreAP


def original_create_body_controls_message():
  """Stock teslacan builder (blinker only, wiper/beam forced 0)."""
  global _ORIG_CREATE_BODY_CONTROLS
  if _ORIG_CREATE_BODY_CONTROLS is None:
    _ORIG_CREATE_BODY_CONTROLS = _tesla_can().create_body_controls_message
  return _ORIG_CREATE_BODY_CONTROLS


def register_nap_body_params():
  """Expose the test keys on NAPParamKeys / DEFAULTS for settings reset."""
  NAPParamKeys.WIPER_SPEED = NAP_WIPER_SPEED
  NAPParamKeys.HIGH_LOW_BEAM = NAP_HIGH_LOW_BEAM
  DEFAULTS[NAP_WIPER_SPEED] = WIPER_SETTING_OFF
  DEFAULTS[NAP_HIGH_LOW_BEAM] = BEAM_SETTING_OFF


def das_wiper_speed_for_setting(setting: int) -> int:
  if setting == WIPER_SETTING_INTERMITTENT:
    return DAS_WIPER_INTERMITTENT
  if setting == WIPER_SETTING_ON:
    return DAS_WIPER_ON
  return DAS_WIPER_OFF


def das_high_low_beam_for_setting(setting: int) -> int:
  if setting == BEAM_SETTING_LOW:
    return DAS_HIGH_BEAM_OFF
  if setting == BEAM_SETTING_HIGH:
    return DAS_HIGH_BEAM_ON
  return DAS_HIGH_BEAM_UNDECIDED


def _param_int(key: str, default: int = 0) -> int:
  try:
    from openpilot.common.params import Params
    val = Params().get(key, return_default=True)
    return int(val) if val is not None else default
  except Exception:
    return default


def requested_das_wiper_speed() -> int:
  return das_wiper_speed_for_setting(_param_int(NAP_WIPER_SPEED, WIPER_SETTING_OFF))


def requested_das_high_low_beam() -> int:
  return das_high_low_beam_for_setting(_param_int(NAP_HIGH_LOW_BEAM, BEAM_SETTING_OFF))


def create_body_controls_message(self, turn, hazard, bus, counter,
                                 wiper_speed=None, high_low_beam=None):
  """Build DAS_bodyControls (0x3E9). Blinker fields stay as teslacan today.

  wiper_speed / high_low_beam: raw DAS values. None reads the NAP settings
  (default 0). DAS_headlightRequest stays 0 — it is headlights on/off, not dimming.

  Known risk: pre-AP body controllers often ignore Autopilot wiper/beam
  requests. This is a car test, not a promise it works.
  """
  if wiper_speed is None:
    wiper_speed = requested_das_wiper_speed()
  if high_low_beam is None:
    high_low_beam = requested_das_high_low_beam()
  wiper_speed = max(0, min(14, int(wiper_speed)))
  high_low_beam = max(0, min(2, int(high_low_beam)))

  values = {
    "DAS_headlightRequest": 0,
    "DAS_hazardLightRequest": hazard,
    "DAS_wiperSpeed": wiper_speed,
    "DAS_turnIndicatorRequest": turn,
    "DAS_highLowBeamDecision": high_low_beam,
    "DAS_highLowBeamOffReason": 0,
    "DAS_turnIndicatorRequestReason": 1 if turn > 0 else 0,
    "DAS_bodyControlsCounter": counter,
    "DAS_bodyControlsChecksum": 0,
  }
  from opendbc.car.tesla.values import CANBUS
  data = self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)[1]
  values["DAS_bodyControlsChecksum"] = self.checksum(0x3E9, data[:7])
  return self.packers[CANBUS.party].make_can_msg("DAS_bodyControls", bus, values)


def install_body_controls_test():
  """Patch teslacan so card TX honors the NAP wiper / beam settings."""
  global _installed
  register_nap_body_params()
  if _installed:
    return
  tesla_can = _tesla_can()
  original_create_body_controls_message()
  tesla_can.create_body_controls_message = create_body_controls_message
  _installed = True
