"""Rain-gate radar-hold for Pre-AP longitudinal lead selection.

Evening rain on Scallywag (route 1c95345a3286a5db|000000df--467073c363)
flapped leadOne radar↔vision: vision-only dRel steps ~7.75 m mean vs
radar-associated 0.48 m, often with modelProb ≥ 0.9 and a −16 to −32 m
range error. Radar track 806 stayed smooth whenever association held.

When the Auto-wiper rain gate is on, keep a live radar-associated lead
through that vision mismatch. Dry / non-Auto fusion is unchanged.

Rain gate (live Params — Connect qlogs often lack NAPWiperRainStatus):
  primary: NAPWiperSpeed == 3 (Auto) and status contains rain=1
  strengthen: acq=1, or score >= acquire for the published sens=
  fallback if status is missing / rain=err: Auto and a recent wipe=1
  never: vision-only wetness, collar/TX alone, or InitData-only
"""
from __future__ import annotations

import math
import re
import time
from typing import Any

# Keep in sync with preap_body_controls / preap_windshield_rain.
NAP_WIPER_SPEED = "NAPWiperSpeed"
NAP_WIPER_RAIN_STATUS = "NAPWiperRainStatus"
WIPER_SETTING_AUTO = 3

# Mid acquire is 4.5; sens 0–4 matches Auto Wipers ACQUIRE_SENSITIVITY_SCALE.
ACQUIRE_ON = 4.5
ACQUIRE_SENSITIVITY_SCALE = (0.58, 0.76, 1.00, 1.10, 1.20)

# Hold last radar lead this many model frames after the track ID disappears.
RAIN_RADAR_LOST_HOLD_FRAMES = 8
# Vision-only bar while raining. Secondary — dig flaps were already ≥ 0.9.
RAIN_VISION_ONLY_MIN_PROB = 0.90
# Fallback latch after the last wipe=1 while status is missing / err.
RAIN_WIPE_HOLD_S = 8.0
# In-lane / cut-in gates (radar frame, not vision).
RAIN_MIN_DREL_M = 0.5
RAIN_INLANE_YREL_M = 2.5
RAIN_INCUMBENT_MAX_YREL_M = 4.0
RAIN_CUT_IN_GAP_M = 8.0

_FIELD_RE = re.compile(r"(?:^|[\s])([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


def _decode_param(val: Any) -> Any:
  if isinstance(val, (bytes, bytearray)):
    return val.decode("utf-8", errors="ignore")
  return val


def _status_fields(status: str | None) -> dict[str, str]:
  if not status:
    return {}
  # First occurrence wins so the main-line rain=/wipe=/acq= beat rain=err bits.
  fields: dict[str, str] = {}
  for key, raw in _FIELD_RE.findall(status):
    fields.setdefault(key, raw)
  return fields


def parse_status_int(status: str | None, key: str) -> int | None:
  raw = _status_fields(status).get(key)
  if raw is None:
    return None
  try:
    return int(raw)
  except (TypeError, ValueError):
    return None


def parse_status_float(status: str | None, key: str) -> float | None:
  raw = _status_fields(status).get(key)
  if raw is None:
    return None
  try:
    return float(raw)
  except (TypeError, ValueError):
    return None


def status_has_rain(status: str | None) -> bool:
  return parse_status_int(status, "rain") == 1


def status_has_acq(status: str | None) -> bool:
  return parse_status_int(status, "acq") == 1


def status_has_wipe(status: str | None) -> bool:
  return parse_status_int(status, "wipe") == 1


def status_score_at_acquire(status: str | None) -> bool:
  """Optional strengthen: score= at/above acquire for the published sens=."""
  score = parse_status_float(status, "score")
  if score is None:
    return False
  sens = parse_status_int(status, "sens")
  if sens is None:
    sens = 2
  sens = max(0, min(len(ACQUIRE_SENSITIVITY_SCALE) - 1, int(sens)))
  return score >= ACQUIRE_ON * ACQUIRE_SENSITIVITY_SCALE[sens]


def status_rain_usable(status: str | None) -> bool:
  """True when the live line has a parseable rain=0/1 (not missing / rain=err)."""
  if not status or not str(status).strip():
    return False
  rain = _status_fields(status).get("rain")
  return rain in ("0", "1")


def wiper_is_auto(wiper_speed: Any) -> bool:
  try:
    return int(wiper_speed) == WIPER_SETTING_AUTO
  except (TypeError, ValueError):
    return False


def evaluate_rain_follow_gate(wiper_speed: Any, status: str | None, now: float,
                              last_wipe_t: float | None,
                              wipe_hold_s: float = RAIN_WIPE_HOLD_S) -> tuple[bool, float | None]:
  """Return (rain_gate, updated_last_wipe_t).

  Primary / strengthen only fire on a usable status. Fallback is Auto plus
  a recent wipe=1 latch — not Auto alone, not collar/TX.
  """
  wipe_t = now if status_has_wipe(status) else last_wipe_t
  if not wiper_is_auto(wiper_speed):
    return False, wipe_t

  if status_rain_usable(status):
    wet = status_has_rain(status) or status_has_acq(status) or status_score_at_acquire(status)
    return wet, wipe_t

  recent_wipe = wipe_t is not None and (now - wipe_t) <= wipe_hold_s
  return recent_wipe, wipe_t


def read_wiper_speed(params: Any) -> int:
  try:
    val = _decode_param(params.get(NAP_WIPER_SPEED, return_default=True))
    if val is None or val == "":
      return 0
    return int(val)
  except Exception:
    return 0


def read_wiper_rain_status(params: Any) -> str | None:
  try:
    val = _decode_param(params.get(NAP_WIPER_RAIN_STATUS))
  except Exception:
    return None
  if val is None:
    return None
  text = str(val).strip()
  return text or None


class RainRadarGate:
  """Live Params rain gate. Tests inject params or set_override()."""

  def __init__(self, params: Any = None, now_fn=time.monotonic):
    self._params = params
    self._now_fn = now_fn
    self._override: bool | None = None
    self._last_wipe_t: float | None = None
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
    now = float(self._now_fn())
    gate, self._last_wipe_t = evaluate_rain_follow_gate(
      read_wiper_speed(params),
      read_wiper_rain_status(params),
      now,
      self._last_wipe_t,
    )
    return gate


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
  """Prefer a live radar lead while raining.

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
