"""Rain-sensing radar-hold for Pre-AP longitudinal lead selection.

Evening rain on Scallywag (route 1c95345a3286a5db|000000df--467073c363)
flapped leadOne radar↔vision: vision-only dRel steps ~7.75 m mean vs
radar-associated 0.48 m, often with modelProb ≥ 0.9 and a −16 to −32 m
range error. Radar track 806 stayed smooth whenever association held.

NAPWiperSpeed==3 is the Auto / rain-sensing stalk setting — not a proof
that it is raining right now. While that mode is selected, prefer a live
radar-associated lead (proactive while Auto can wipe). Off / Int / On
leave stock fusion unchanged. Do not require NAPWiperRainStatus rain=1.

Hold only tracks that still pass the travel-path / oncoming gates
(`radar_path_gate`). Auto rain mode must not lock roadside signs or
opposing-lane traffic as leadOne.

Gate (live Params):
  radar_prefer = NAPWiperSpeed == 3
"""
from __future__ import annotations

from typing import Any, Sequence

from openpilot.selfdrive.controls.lib.radar_path_gate import (
  MIN_DREL_M,
  PATH_HALF_WIDTH_M,
  PATH_INCUMBENT_HALF_WIDTH_M,
  radar_follow_ok,
)

# Keep in sync with preap_body_controls. 3 is Auto / rain-sensing On.
NAP_WIPER_SPEED = "NAPWiperSpeed"
WIPER_SETTING_AUTO = 3

# Hold last radar lead this many model frames after the track ID disappears.
RAIN_RADAR_LOST_HOLD_FRAMES = 8
# Vision-only bar while rain-sensing is On. Secondary — dig flaps were already ≥ 0.9.
RAIN_VISION_ONLY_MIN_PROB = 0.90
# Path / oncoming gates (travel path, not raw radar yRel). The old 2.5 / 4.0
# yRel windows swallowed left roadside signs and opposing-lane traffic.
RAIN_MIN_DREL_M = MIN_DREL_M
RAIN_INLANE_YREL_M = PATH_HALF_WIDTH_M
RAIN_INCUMBENT_MAX_YREL_M = PATH_INCUMBENT_HALF_WIDTH_M
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


def radar_hold_kinematics_ok(track: Any, max_yrel: float = RAIN_INCUMBENT_MAX_YREL_M,
                             v_ego: float = 0.0,
                             path_x: Sequence[float] | None = None,
                             path_y: Sequence[float] | None = None) -> bool:
  """Live rain-hold candidate. Path + oncoming; yRel-only is not enough."""
  return radar_follow_ok(track, v_ego, path_x, path_y, max_lat=max_yrel)


def closest_inlane_radar(tracks: dict[int, Any], v_ego: float = 0.0,
                         path_x: Sequence[float] | None = None,
                         path_y: Sequence[float] | None = None) -> Any | None:
  candidates = [
    track for track in tracks.values()
    if radar_hold_kinematics_ok(track, RAIN_INLANE_YREL_M, v_ego, path_x, path_y)
  ]
  if not candidates:
    return None
  return min(candidates, key=lambda track: track.dRel)


def pick_rain_radar_track(associated: Any | None, tracks: dict[int, Any],
                          incumbent_id: int | None, v_ego: float = 0.0,
                          path_x: Sequence[float] | None = None,
                          path_y: Sequence[float] | None = None) -> Any | None:
  """Prefer a live radar lead while rain-sensing is On.

  Hold the incumbent through vision mismatch only if it still sits on
  the travel path and is not oncoming. Switch only for a much closer
  in-path radar cut-in, or when the incumbent is gone. Do not latch
  off-path signs or opposing-lane traffic.
  """
  incumbent = tracks.get(incumbent_id) if incumbent_id is not None else None
  if incumbent is not None and not radar_hold_kinematics_ok(
      incumbent, RAIN_INCUMBENT_MAX_YREL_M, v_ego, path_x, path_y):
    incumbent = None
  if associated is not None and not radar_hold_kinematics_ok(
      associated, RAIN_INLANE_YREL_M, v_ego, path_x, path_y):
    associated = None
  inlane = closest_inlane_radar(tracks, v_ego, path_x, path_y)

  if incumbent is not None and inlane is not None and inlane.identifier != incumbent.identifier:
    if incumbent.dRel - inlane.dRel >= RAIN_CUT_IN_GAP_M:
      return inlane

  if incumbent is not None:
    return incumbent
  if associated is not None:
    return associated
  return inlane
