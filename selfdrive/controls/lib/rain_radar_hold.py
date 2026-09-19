"""Path-gated radar prefer for Pre-AP longitudinal lead selection.

Default operating mode — not rain-Auto-only. Prefer a live **path-valid
radar association** for long lead when:

  1. NAP → Radar Settings → Radar Enabled is On (`NAPRadarEnabled`)
  2. radar is reliable (no hardware fault / stream timeout / erratic
     track kinematics)

NAPWiperSpeed==3 was a temporary rain gate. It is not required.

If radar is disabled, unhealthy, or the track is not path-associated,
fall back to stock vision-radar fusion. Off-path / oncoming are always
rejected. On-path stationary is kept.

Evening rain on Scallywag (route 1c95345a3286a5db|000000df--467073c363)
flapped leadOne radar↔vision: vision-only dRel steps ~7.75 m mean vs
radar-associated 0.48 m. Radar track 806 stayed smooth when associated.

e1 `1c95345a3286a5db|000000e1--b993674371` (tip 638f5f7d4) then showed
the old rain-hold latching off-path STAT / oncoming as leadOne
(mp ≪ 0.15, aTarget=−3.5). Path + vLead gates stay. #199 is not
implicated.

This is not “prefer any Bosch track in a wide FOV.” Radar used for long
must sit on the same model driving path vision uses for lead-in-path
(`modelV2.position`):
  on path + stationary → KEEP (stopped lead / pedestrian)
  off path + stationary → REJECT (signs, gas station)
  oncoming / opposing → REJECT
"""
from __future__ import annotations

from typing import Any, Sequence

from openpilot.selfdrive.controls.lib.radar_path_gate import (
  MIN_DREL_M,
  PATH_INCUMBENT_HALF_WIDTH_M,
  radar_follow_ok,
)

# Keep in sync with preap_body_controls / nap_params.
NAP_WIPER_SPEED = "NAPWiperSpeed"
WIPER_SETTING_AUTO = 3
NAP_RADAR_ENABLED = "NAPRadarEnabled"
NAP_RADAR_IGNORE_HW_FAIL = "NAPRadarIgnoreHwFail"

# Hold last radar lead this many model frames after the track ID disappears.
RAIN_RADAR_LOST_HOLD_FRAMES = 8
# Vision-only bar while path-gated prefer is active. Dig flaps were already ≥ 0.9.
RAIN_VISION_ONLY_MIN_PROB = 0.90

# Reliability: hardware / timeout trip immediately. Erratic kinematics
# need a short streak. Recover after this many consecutive good samples
# so a single clean frame cannot chatter the alert / prefer latch.
RELIABLE_FAIL_FRAMES = 4
RELIABLE_OK_FRAMES = 16
RELIABLE_DROPOUT_S = 0.50
RELIABLE_YREL_JUMP_M = 4.0
RELIABLE_DREL_JUMP_M = 25.0
RELIABLE_MIN_VEGO_MS = 5.0
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
# Path of travel is the filter. On-path stationary (stopped car /
# pedestrian) is KEEP. Off-path STAT (signs, gas station) fails the
# path gate. Do not blanket-reject vLead≈0.


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
  """Legacy Auto-wiper helper. Not the prefer gate — prefer is default."""
  return wiper_is_auto(wiper_speed)


def read_wiper_speed(params: Any) -> int:
  try:
    val = _decode_param(params.get(NAP_WIPER_SPEED, return_default=True))
    if val is None or val == "":
      return 0
    return int(val)
  except Exception:
    return 0


def _param_bool(params: Any, key: str, default: bool = False) -> bool:
  if params is None:
    return default
  try:
    if hasattr(params, "get_bool"):
      return bool(params.get_bool(key))
    val = params.get(key, return_default=True)
    if isinstance(val, (bytes, bytearray)):
      val = val.decode("utf-8", errors="ignore")
    if val is None or val == "":
      return default
    return bool(int(val)) if not isinstance(val, bool) else bool(val)
  except Exception:
    return default


def radar_errors_unhealthy(errors: Any, ignore_hw_fail: bool = False) -> bool:
  """Hardware / CAN faults. Ignore-HW-fail only masks radarFault."""
  if errors is None:
    return False

  def _flag(name: str) -> bool:
    if isinstance(errors, dict):
      return bool(errors.get(name, False))
    return bool(getattr(errors, name, False))

  if _flag("canError") or _flag("radarUnavailableTemporary"):
    return True
  if _flag("radarFault") and not ignore_hw_fail:
    return True
  return False


def _track_xy(track: Any) -> tuple[float, float] | None:
  try:
    d_rel = float(track.dRel)
    y_rel = float(track.yRel)
  except (TypeError, ValueError, AttributeError):
    return None
  if not (d_rel == d_rel and y_rel == y_rel):  # NaN
    return None
  return d_rel, y_rel


class RadarReliability:
  """Practical Bosch health for path-gated prefer.

  Immediate trip: CAN/fault bits, measurement timeout, empty table while
  moving. Streak trip: same-ID |ΔyRel| / |ΔdRel| jumps that look like a
  glitching table. Recover after RELIABLE_OK_FRAMES clean samples so the
  HUD / prefer latch does not chatter.
  """

  def __init__(self):
    self.healthy = True
    self.reason = ""
    self._bad_streak = 0
    self._good_streak = 0
    self._last_xy: dict[int, tuple[float, float]] = {}
    self._override: bool | None = None

  def set_override(self, healthy: bool | None) -> None:
    self._override = None if healthy is None else bool(healthy)

  def reset(self) -> None:
    self.healthy = True
    self.reason = ""
    self._bad_streak = 0
    self._good_streak = 0
    self._last_xy = {}

  def update(self, tracks: dict[int, Any] | None = None,
             errors: Any = None, v_ego: float = 0.0,
             timed_out: bool = False, ignore_hw_fail: bool = False) -> bool:
    if self._override is not None:
      self.healthy = self._override
      self.reason = "" if self.healthy else "override"
      return self.healthy

    bad, reason = self._sample_bad(tracks or {}, errors, v_ego, timed_out,
                                   ignore_hw_fail)
    if bad:
      self._bad_streak += 1
      self._good_streak = 0
      immediate = reason in ("timeout", "fault", "dropout")
      if immediate or self._bad_streak >= RELIABLE_FAIL_FRAMES:
        self.healthy = False
        self.reason = reason
    else:
      self._good_streak += 1
      self._bad_streak = 0
      if self._good_streak >= RELIABLE_OK_FRAMES:
        self.healthy = True
        self.reason = ""
    return self.healthy

  def _sample_bad(self, tracks: dict[int, Any], errors: Any, v_ego: float,
                  timed_out: bool, ignore_hw_fail: bool) -> tuple[bool, str]:
    if timed_out:
      self._last_xy = {}
      return True, "timeout"
    if radar_errors_unhealthy(errors, ignore_hw_fail=ignore_hw_fail):
      return True, "fault"

    snap: dict[int, tuple[float, float]] = {}
    for tid, track in tracks.items():
      xy = _track_xy(track)
      if xy is None:
        continue
      try:
        snap[int(tid)] = xy
      except (TypeError, ValueError):
        continue

    if (float(v_ego) >= RELIABLE_MIN_VEGO_MS and not snap and
        self._last_xy):
      self._last_xy = {}
      return True, "dropout"

    erratic = False
    for tid, (d_rel, y_rel) in snap.items():
      prev = self._last_xy.get(tid)
      if prev is None:
        continue
      if abs(y_rel - prev[1]) > RELIABLE_YREL_JUMP_M:
        erratic = True
        break
      if abs(d_rel - prev[0]) > RELIABLE_DREL_JUMP_M:
        erratic = True
        break
    self._last_xy = snap
    if erratic:
      return True, "erratic"
    return False, ""


class RadarPreferGate:
  """Default path-gated prefer. Requires Radar Enabled + reliability.

  NAPWiperSpeed is not consulted. Tests inject params, set_override(),
  or set_enabled_override().
  """

  def __init__(self, params: Any = None):
    self._params = params
    self._override: bool | None = None
    self._enabled_override: bool | None = None
    self._params_failed = False
    self.enabled = True
    self.reliable = True
    self.ignore_hw_fail = False

  def set_override(self, prefer: bool | None) -> None:
    """Test hook. True/False forces prefer; None uses enabled+health."""
    self._override = None if prefer is None else bool(prefer)

  def set_enabled_override(self, enabled: bool | None) -> None:
    self._enabled_override = None if enabled is None else bool(enabled)

  def set_reliable(self, ok: bool) -> None:
    self.reliable = bool(ok)

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

  def _read_enabled(self) -> bool:
    if self._enabled_override is not None:
      return bool(self._enabled_override)
    params = self._get_params()
    if params is None:
      # Unit tests / params unavailable: prefer is the default mode.
      return True
    return _param_bool(params, NAP_RADAR_ENABLED, default=False)

  def read_ignore_hw_fail(self) -> bool:
    self.ignore_hw_fail = _param_bool(self._get_params(), NAP_RADAR_IGNORE_HW_FAIL, default=False)
    return self.ignore_hw_fail

  def update(self) -> bool:
    self.ignore_hw_fail = self.read_ignore_hw_fail()
    self.enabled = self._read_enabled()
    if self._override is not None:
      return bool(self._override)
    return self.enabled and self.reliable

  @property
  def fallback_alert(self) -> bool:
    """Enabled but prefer dropped because radar is unhealthy. Not Off."""
    if self._override is False:
      return False
    return bool(self.enabled) and not bool(self.reliable)


# Back-compat name used by older tests / RadarD wiring.
RainRadarGate = RadarPreferGate


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
  """Hold a path-valid radar *association* while prefer is active.

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
