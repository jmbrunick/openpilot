"""Early radar lead: a fast-closing, moving, in-path radar track the camera
has not matched yet.

Oct 23 17:28 US 212: radar saw the slow car from 155 m closing ~9 m/s, but
the camera lead read ~58 mph against a real ~36 mph, so the association
velocity gate refused the track until ~55 m and the car then sat on the
regen floor. Two narrow cases now take the radar track as leadOne:

  * camera speed disagrees: the vision lead is at long range and agrees
    with the track in distance and lateral, only its speed is off, and
    radar says slower (the direction that brakes, never one that pulls);
  * lone track: no lead at all, the track alone.

Both need the same gates, held continuously for EARLY_PERSIST_S: ego at
speed, a measured track, inside a tight path corridor, moving in our
direction (bridges, signs and stopped cars read ~0 m/s and never pass),
closing fast, and inside a range / time-to-contact window. The follow law
then starts its far, gentle close earlier.
"""
from typing import Any
from collections.abc import Sequence

from openpilot.selfdrive.controls.lib.radar_path_gate import path_lateral_m, track_is_oncoming

EARLY_MIN_V_EGO_MS = 15.0
EARLY_MIN_D_M = 40.0
EARLY_MAX_D_M = 170.0
EARLY_MIN_V_TRACK_MS = 6.0  # moving with us: stationary returns never qualify
EARLY_MIN_CLOSE_MS = 4.0
EARLY_MAX_TTC_S = 20.0
EARLY_PATH_HALF_WIDTH_M = 1.2
EARLY_PERSIST_S = 1.5
# Camera-disagree match: vision must be far and agree in d / lateral.
EARLY_VISION_MIN_D_M = 50.0
EARLY_VISION_D_GATE_M = 12.0
EARLY_VISION_LAT_GATE_M = 2.0
EARLY_VISION_MIN_DV_MS = 3.0  # radar slower than vision by at least this
# Once taken, the track is held on looser gates so the lead does not drop
# the moment the close is speed-matched.
EARLY_HOLD_MIN_D_M = 15.0
EARLY_HOLD_HALF_WIDTH_M = 2.0


def early_track_gates(track: Any, v_ego: float, path_x: Sequence[float] | None,
                      path_y: Sequence[float] | None, held: bool = False) -> bool:
  """Instantaneous gates for one radar track (no persistence). `held` is
  the looser keep gate for the track already taken."""
  try:
    d = float(track.dRel)
    v_rel = float(track.vRel)
    measured = bool(track.measured)
  except (AttributeError, TypeError, ValueError):
    return False
  if not measured or float(v_ego) < EARLY_MIN_V_EGO_MS:
    return False
  if not ((EARLY_HOLD_MIN_D_M if held else EARLY_MIN_D_M) <= d <= EARLY_MAX_D_M):
    return False
  v_track = float(v_ego) + v_rel
  if v_track < EARLY_MIN_V_TRACK_MS or track_is_oncoming(track, v_ego):
    return False
  if not held:
    if v_rel > -EARLY_MIN_CLOSE_MS:
      return False
    if d / max(-v_rel, 1e-3) > EARLY_MAX_TTC_S:
      return False
  if path_x is None or path_y is None or len(path_x) == 0:
    return False  # no path, no early lead
  lat = path_lateral_m(track, path_x, path_y)
  half = EARLY_HOLD_HALF_WIDTH_M if held else EARLY_PATH_HALF_WIDTH_M
  return lat is not None and abs(lat) <= half


class EarlyRadarLead:
  """Per-track persistence of the early gates."""

  def __init__(self):
    self._since: dict[int, float] = {}
    self.held_id: int | None = None

  def update(self, t: float, tracks: dict[int, Any], v_ego: float,
             path_x: Sequence[float] | None, path_y: Sequence[float] | None) -> list[Any]:
    """Tracks that have held every gate for EARLY_PERSIST_S, nearest first.
    A held track that still passes the keep gate comes first."""
    ok = []
    since = {}
    held = None
    for tid, tr in tracks.items():
      if tid == self.held_id and early_track_gates(tr, v_ego, path_x, path_y, held=True):
        held = tr
        since[tid] = self._since.get(tid, float(t))
        continue
      if early_track_gates(tr, v_ego, path_x, path_y):
        since[tid] = self._since.get(tid, float(t))
        if float(t) - since[tid] >= EARLY_PERSIST_S - 1e-6:
          ok.append(tr)
    self._since = since
    if held is None:
      self.held_id = None
    ok.sort(key=lambda tr: float(tr.dRel))
    return ([held] if held is not None else []) + ok

  def hold(self, track_id: int | None) -> None:
    """The association took this track (None: it took something else)."""
    self.held_id = track_id

  def reset(self) -> None:
    self._since = {}
    self.held_id = None


def vision_disagree_match(early: list[Any], v_ego: float, vision_d: float, vision_y: float,
                          vision_v: float, vision_prob: float) -> Any | None:
  """A qualified track that is the far vision lead except for its speed,
  and slower than vision says. None otherwise."""
  if vision_prob <= 0.5 or vision_d < EARLY_VISION_MIN_D_M:
    return None
  for tr in early:
    if abs(float(tr.dRel) - vision_d) > EARLY_VISION_D_GATE_M:
      continue
    if abs(float(tr.yRel) - vision_y) > EARLY_VISION_LAT_GATE_M:
      continue
    if float(v_ego) + float(tr.vRel) <= vision_v - EARLY_VISION_MIN_DV_MS:
      return tr
  return None
