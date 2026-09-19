"""Rain-sensing radar-hold for Pre-AP longitudinal lead selection.

Evening rain on Scallywag (route 1c95345a3286a5db|000000df--467073c363)
flapped leadOne radar↔vision: vision-only dRel steps ~7.75 m mean vs
radar-associated 0.48 m, often with modelProb ≥ 0.9 and a −16 to −32 m
range error. Radar track 806 stayed smooth whenever association held.

e1 `1c95345a3286a5db|000000e1--b993674371` (tip 638f5f7d4, Auto=3)
then showed the hold latching off-path STAT / oncoming as leadOne
(mp ≪ 0.15, aTarget=−3.5). #199 look-ahead is not implicated.

NAPWiperSpeed==3 is the Auto / rain-sensing stalk setting — not a proof
that it is raining right now. While that mode is selected, hold a live
**path-valid radar association** through wet-vision range flaps. Off /
Int / On leave stock fusion unchanged. Do not require rain=1.

This is not “prefer any Bosch track in a wide FOV.” Radar used for long
must sit on the same model driving path vision uses for lead-in-path
(`modelV2.position`). Rain-hold only latches a track that is (or was)
associated to that path and still passes the path / oncoming gates.
Do not acquire an unassociated closest-in-lane radar (left signs,
oncoming) as leadOne.

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
# Path / oncoming gates. e1 #201 used 2.5 / 4.0 yRel and no vLead / path
# check — LEFT STAT at +2.2…+3.9 m and EP_2059 near-edge opposing at
# +2.23…+2.48 became leadOne. Associated / incumbent use 2.0 m (Justin
# 2.5→~2.0). Unassociated closest-in-lane is not acquired.
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
  """Hold a path-valid radar *association* while rain-sensing is On.

  Not a wide-FOV radar prefer. A track is eligible only if it is the
  current vision-associated lead (on the OP path) or the incumbent
  association still on that path. Unassociated closest-in-lane radar
  is not acquired — that is how left signs / oncoming became leadOne.
  Switch only for a much closer *associated* path cut-in.
  """
  incumbent = tracks.get(incumbent_id) if incumbent_id is not None else None
  if incumbent is not None and not radar_hold_kinematics_ok(
      incumbent, RAIN_INCUMBENT_MAX_YREL_M, v_ego, path_x, path_y):
    incumbent = None
  # Vision already path-nominated this track. Use the incumbent width so a
  # 1.6–2.0 m associated lead is not dropped; oncoming / off-path still fail.
  if associated is not None and not radar_hold_kinematics_ok(
      associated, RAIN_INCUMBENT_MAX_YREL_M, v_ego, path_x, path_y):
    associated = None

  if associated is not None and incumbent is not None:
    if associated.identifier != incumbent.identifier:
      if incumbent.dRel - associated.dRel >= RAIN_CUT_IN_GAP_M:
        return associated
      return incumbent
    return associated
  if associated is not None:
    return associated
  return incumbent
