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
# EP_2059 (e1 segs 13–17): #201 RAIN_INLANE=2.5 latched near-edge
# oncoming 771/802 at |yRel| 2.23–2.48 (0.02–0.27 m inside 2.5).
# Justin: 2.5 → 2.0 m (~6.6 ft) + reject vLead < 0. Path association
# still required. Clean 21:02/21:03 stayed |yRel| > 2.5. Incumbent
# 2.0 (not 4.0) so a latched near-edge cannot walk to y=3.98.
RAIN_MIN_DREL_M = MIN_DREL_M
RAIN_INLANE_YREL_M = 2.0
RAIN_INCUMBENT_MAX_YREL_M = PATH_INCUMBENT_HALF_WIDTH_M
RAIN_CUT_IN_GAP_M = 8.0
# Incumbent-only hold beyond this needs a confident vision lead (range may
# flap). 20:59:40 STAT 84–123 m and 20:49:10 track 321 at 99.3 m / mp 0.007
# must not regen. Live path association is enough at any range. Original
# rain dig: track 806 at 93.8 m + modelProb 0.978 still holds.
RAIN_FAR_HOLD_DREL_M = 70.0
RAIN_FAR_HOLD_MIN_PROB = 0.50


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


def _track_id(obj: Any) -> int | None:
  if obj is None:
    return None
  if isinstance(obj, dict):
    val = obj.get("identifier", obj.get("radarTrackId"))
  else:
    val = getattr(obj, "identifier", None)
    if val is None:
      val = getattr(obj, "radarTrackId", None)
  try:
    return int(val)
  except (TypeError, ValueError):
    return None


def _track_drel(obj: Any) -> float | None:
  if obj is None:
    return None
  if isinstance(obj, dict):
    val = obj.get("dRel")
  else:
    val = getattr(obj, "dRel", None)
  try:
    f = float(val)
  except (TypeError, ValueError):
    return None
  return f


def rain_far_hold_ok(track: Any, associated: Any | None = None,
                     vision_prob: float = 1.0) -> bool:
  """Far incumbent-only hold needs confident vision; live association is enough.

  Does not blanket-reject on-path stationary. A path-associated stopped
  car at 90 m stays valid. Unassociated low-prob furniture at 84–123 m does not.
  """
  if track is None:
    return False
  if _track_id(associated) is not None and _track_id(associated) == _track_id(track):
    return True
  d_rel = _track_drel(track)
  if d_rel is None:
    return False
  try:
    prob = float(vision_prob)
  except (TypeError, ValueError):
    prob = 0.0
  if d_rel > RAIN_FAR_HOLD_DREL_M and prob < RAIN_FAR_HOLD_MIN_PROB:
    return False
  return True


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
                          path_y: Sequence[float] | None = None,
                          vision_prob: float = 1.0) -> Any | None:
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
      chosen = associated if incumbent.dRel - associated.dRel >= RAIN_CUT_IN_GAP_M else incumbent
    else:
      chosen = associated
  elif associated is not None:
    chosen = associated
  else:
    chosen = incumbent
  if chosen is not None and not rain_far_hold_ok(chosen, associated, vision_prob):
    return None
  return chosen
