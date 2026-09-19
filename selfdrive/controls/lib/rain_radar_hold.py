"""Rain-sensing radar-hold for Pre-AP longitudinal lead selection.

Evening rain on Scallywag (route 1c95345a3286a5db|000000df--467073c363)
flapped leadOne radar↔vision: vision-only dRel steps ~7.75 m mean vs
radar-associated 0.48 m, often with modelProb ≥ 0.9 and a −16 to −32 m
range error. Radar track 806 stayed smooth whenever association held.

NAPWiperSpeed==3 is the Auto / rain-sensing stalk setting — not a proof
that it is raining right now. While that mode is selected, prefer a live
radar-associated lead (proactive while Auto can wipe). Off / Int / On
leave stock fusion unchanged. Do not require NAPWiperRainStatus rain=1.

Gate (live Params):
  radar_prefer = NAPWiperSpeed == 3
"""
from __future__ import annotations

import math
from typing import Any

# Keep in sync with preap_body_controls. 3 is Auto / rain-sensing On.
NAP_WIPER_SPEED = "NAPWiperSpeed"
WIPER_SETTING_AUTO = 3

# Hold last radar lead this many model frames after the track ID disappears.
RAIN_RADAR_LOST_HOLD_FRAMES = 8
# Vision-only bar while rain-sensing is On. Secondary — dig flaps were already ≥ 0.9.
RAIN_VISION_ONLY_MIN_PROB = 0.90
# In-lane / cut-in gates (radar frame, not vision).
RAIN_MIN_DREL_M = 0.5
RAIN_INLANE_YREL_M = 2.5
RAIN_INCUMBENT_MAX_YREL_M = 4.0
RAIN_CUT_IN_GAP_M = 8.0


def _decode_param(val: Any) -> Any:
  if isinstance(val, (bytes, bytearray)):
    return val.decode("utf-8", errors="ignore")
  return val


def wiper_is_auto(wiper_speed: Any) -> bool:
  """True when rain-sensing / Auto is selected. Not 'it is raining'."""
  try:
    return int(wiper_speed) == WIPER_SETTING_AUTO
  except (TypeError, ValueError):
    return False


def rain_sensing_on(wiper_speed: Any) -> bool:
  """Mode gate for radar-prefer. rain=1 / score are not required."""
  return wiper_is_auto(wiper_speed)


def read_wiper_speed(params: Any) -> int:
  try:
    val = _decode_param(params.get(NAP_WIPER_SPEED, return_default=True))
    if val is None or val == "":
      return 0
    return int(val)
  except Exception:
    return 0


class RainRadarGate:
  """Live Params rain-sensing mode gate. Tests inject params or set_override()."""

  def __init__(self, params: Any = None):
    self._params = params
    self._override: bool | None = None
    self._params_failed = False

  def set_override(self, raining: bool | None) -> None:
    self._override = None if raining is None else bool(raining)

  def _get_params(self) -> Any:
    if self._params is not None or self._params_failed:
      return self._params
    try:
      from openpilot.common.params import Params
      self._params = Params()
    except Exception:
      self._params_failed = True
      self._params = None
    return self._params

  def update(self) -> bool:
    if self._override is not None:
      return self._override
    params = self._get_params()
    if params is None:
      return False
    return rain_sensing_on(read_wiper_speed(params))


def radar_hold_kinematics_ok(track: Any, max_yrel: float = RAIN_INCUMBENT_MAX_YREL_M) -> bool:
  try:
    d_rel = float(track.dRel)
    y_rel = float(track.yRel)
  except (TypeError, ValueError, AttributeError):
    return False
  if not math.isfinite(d_rel) or not math.isfinite(y_rel):
    return False
  return d_rel > RAIN_MIN_DREL_M and abs(y_rel) <= max_yrel


def closest_inlane_radar(tracks: dict[int, Any]) -> Any | None:
  candidates = [
    track for track in tracks.values()
    if radar_hold_kinematics_ok(track, RAIN_INLANE_YREL_M)
  ]
  if not candidates:
    return None
  return min(candidates, key=lambda track: track.dRel)


def pick_rain_radar_track(associated: Any | None, tracks: dict[int, Any],
                          incumbent_id: int | None) -> Any | None:
  """Prefer a live radar lead while rain-sensing is On.

  Hold the incumbent through vision mismatch. Switch only for a much
  closer in-lane radar cut-in, or when the incumbent is gone.
  """
  incumbent = tracks.get(incumbent_id) if incumbent_id is not None else None
  if incumbent is not None and not radar_hold_kinematics_ok(incumbent):
    incumbent = None
  inlane = closest_inlane_radar(tracks)

  if incumbent is not None and inlane is not None and inlane.identifier != incumbent.identifier:
    if incumbent.dRel - inlane.dRel >= RAIN_CUT_IN_GAP_M:
      return inlane

  if incumbent is not None:
    return incumbent
  if associated is not None:
    return associated
  return inlane
