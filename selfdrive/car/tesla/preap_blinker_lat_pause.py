"""Keep Pre-AP cruiseEnabled while a blinker lamp pauses lateral.

opendbc's handle_steering_disengage tears down the FSM on hands-on ≥ 2.
During a lamp-on turn we have already released steering, so a wheel input
must not drop cruiseEnabled / enableLongControl. Stalk cancel is unchanged.

This follows the same install-from-card pattern as preap_body_controls.
"""

from openpilot.selfdrive.controls.lib.blinker_lateral_pause import blinker_pauses_lateral

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


def _peek_v_ego(can_parsers):
  try:
    from opendbc.car import Bus
    from opendbc.car.common.conversions import Conversions as CV
    return float(can_parsers[Bus.chassis].vl["ESP_B"]["ESP_vehicleSpeed"]) * CV.KPH_TO_MS
  except Exception:
    return 0.0


def _handle_steering_disengage(self, steering_disengage):
  if blinker_pauses_lateral(getattr(self, "_nap_left_blinker", False),
                            getattr(self, "_nap_right_blinker", False),
                            getattr(self, "_nap_v_ego", 0.0),
                            getattr(self, "_nap_junction_on_blinker_side", False)):
    # Keep prev in sync so lamp-off with hands still on is not a rising edge.
    self.prev_steering_disengage = steering_disengage
    return
  return _ORIG_HANDLE(self, steering_disengage)


def _update_preap(cs, can_parsers):
  left, right = _peek_blinker_lamps(can_parsers)
  engagement = getattr(cs, "engagement", None)
  if engagement is not None:
    engagement._nap_left_blinker = left
    engagement._nap_right_blinker = right
    engagement._nap_v_ego = _peek_v_ego(can_parsers)
    # liveMapDataNAP has no junction-side flag; stay false.
    engagement._nap_junction_on_blinker_side = False
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
