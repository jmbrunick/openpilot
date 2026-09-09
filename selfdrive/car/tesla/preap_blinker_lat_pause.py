"""Keep Pre-AP cruiseEnabled while a blinker lamp pauses lateral.

opendbc's handle_steering_disengage tears down the FSM on hands-on ≥ 2.
GTW lamp bits flash, so BlinkerLateralHold latches turn-active through
those gaps. During that turn we have already released steering, so a
wheel input must not drop cruiseEnabled / enableLongControl. After ~1s
of continuous dark, keep that suppression until torque is released so a
finishing hand-steer does not fully disengage NAP. Stalk cancel is unchanged.

ALC keep-alive flashes are not a driver turn: do not latch turn-active and
do not tear down cruise. DAS_bodyControls turn-indicator TX stays in
teslacan / carcontroller from CC.leftBlinker; this helper does not touch it.

This follows the same install-from-card pattern as preap_body_controls.
"""

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import BlinkerLateralHold

_ORIG_HANDLE = None
_ORIG_UPDATE = None
_installed = False


def _peek_blinker_lamps(can_parsers):
  try:
    from opendbc.car import Bus
    gtw = can_parsers[Bus.chassis].vl["GTW_carState"]
    return gtw["BC_indicatorLStatus"] == 1, gtw["BC_indicatorRStatus"] == 1
  except Exception:
    return False, False


def _peek_steering_pressed(can_parsers):
  try:
    from opendbc.car import Bus
    from opendbc.car.tesla.values import STEER_THRESHOLD
    epas = can_parsers[Bus.chassis].vl["EPAS_sysStatus"]
    return abs(epas["EPAS_torsionBarTorque"]) > STEER_THRESHOLD
  except Exception:
    return False


def _peek_stalk_and_speed(can_parsers):
  try:
    from opendbc.car import Bus
    stw = can_parsers[Bus.chassis].vl["STW_ACTN_RQ"]
    stalk = int(stw.get("TurnIndLvr_Stat", 0) or 0)
    if stalk == 3:  # SNA
      stalk = 0
    v_kph = can_parsers[Bus.chassis].vl["ESP_B"]["ESP_vehicleSpeed"]
    return stalk, float(v_kph) * CV.KPH_TO_MS
  except Exception:
    return 0, 0.0


def _hold_for(engagement):
  hold = getattr(engagement, "_nap_lat_hold", None)
  if hold is None:
    hold = BlinkerLateralHold()
    engagement._nap_lat_hold = hold
  return hold


def _hold_kwargs(engagement, dt=None):
  kwargs = dict(
    alc_active=bool(getattr(engagement, "_nap_alc_active", False)),
    v_ego=float(getattr(engagement, "_nap_v_ego", 0.0) or 0.0),
    stalk_state=int(getattr(engagement, "_nap_stalk_state", 0) or 0),
  )
  if dt is not None:
    kwargs["dt"] = dt
  return kwargs


def _handle_steering_disengage(self, steering_disengage):
  left = getattr(self, "_nap_left_blinker", False)
  right = getattr(self, "_nap_right_blinker", False)
  pressed = bool(getattr(self, "_nap_steering_pressed", False) or steering_disengage)
  hold = _hold_for(self)
  # dt=0: _update_preap already advanced the dark timer this cycle.
  hold.update(
    left, right, pressed, engaged=bool(getattr(self, "cruiseEnabled", False)),
    **_hold_kwargs(self, dt=0.0))
  if hold.blocks_steer_disengage:
    # Keep prev in sync so lamp-off / hand-release is not a rising edge.
    self.prev_steering_disengage = steering_disengage
    return
  return _ORIG_HANDLE(self, steering_disengage)


def _update_preap(cs, can_parsers):
  left, right = _peek_blinker_lamps(can_parsers)
  pressed = _peek_steering_pressed(can_parsers)
  stalk, v_ego = _peek_stalk_and_speed(can_parsers)
  engagement = getattr(cs, "engagement", None)
  if engagement is not None:
    engagement._nap_left_blinker = left
    engagement._nap_right_blinker = right
    engagement._nap_steering_pressed = pressed
    engagement._nap_stalk_state = stalk
    engagement._nap_v_ego = v_ego
    _hold_for(engagement).update(
      left, right, pressed, engaged=bool(getattr(engagement, "cruiseEnabled", False)),
      **_hold_kwargs(engagement))
  return _ORIG_UPDATE(cs, can_parsers)


def install_blinker_lat_pause():
  """Patch Pre-AP engagement so a lamp-on turn does not tear down cruise."""
  global _installed, _ORIG_HANDLE, _ORIG_UPDATE
  if _installed:
    return
  from opendbc.car.tesla.preap import carstate as preap_carstate
  from opendbc.car.tesla.preap.engagement import PreAPEngagement

  _ORIG_HANDLE = PreAPEngagement.handle_steering_disengage
  _ORIG_UPDATE = preap_carstate.update_preap
  PreAPEngagement.handle_steering_disengage = _handle_steering_disengage
  preap_carstate.update_preap = _update_preap
  _installed = True
