"""Hypermile: eco preset for Pre-AP NAP.

Not a second long controller. Eco snap / Step Down / Hill Climb only.
Stock Follow Distance 1–7 (`NAPFollowDistance`) is shared — Hypermile
does not own follow levels or remap the stalk.

Settings → NAP → Driving Mannerisms → Hypermile (default Off).
"""
from __future__ import annotations

import json

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.map_speed_policy import (
  is_cruise_stalk_hold_step,
  is_cruise_stalk_tip_step,
)

# Params
PARAM_HYPERMILE = "NAPHypermile"
PARAM_FOLLOW_DISTANCE = "NAPFollowDistance"
PARAM_SAVED = "NAPHypermileSaved"
PARAM_STEP_DOWN = "NAPHypermileStepDown"

# Mileage defer: same 50→80 posted scale as eco, larger drop. Does not
# stack with eco. Hard cap — never more than this under posted.
STEP_DOWN_MPH = 15.0

# Stock Follow Distance 1–7 (Driving Mannerisms + stalk behind a lead).
# Same param whether Hypermile is On or Off. Includes closest 1.
FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7
FOLLOW_DISTANCE_DEFAULT = 4
NAP_FOLLOW_DISTANCE_RANGE = range(FOLLOW_DISTANCE_MIN, FOLLOW_DISTANCE_MAX + 1)

# Eco snap. Comfort-biased efficiency — not maximum regen bite.
# Early + light map ease (lookahead Early = 0.55 m/s², starts farther out)
# and a lazy climb (Accel 1). Late lookahead (1.20 m/s²) is the wrong trade:
# it holds speed then dumps regen. Safety / MPC hard brake unchanged.
# Soft-lat / DM / blinker / stock 1–7 follow are not snapped.
# Map offset is *not* snapped to a flat −5 (that forced town 30→25).
# Live eco offset is posted-scaled: 0 at/under 50, −8 at 80, cap −8 above.
ECO_ADAPTIVE_ACCEL = True
ECO_MAP_MODE_FOLLOW = 3  # NAPMapSpeedMode Follow
ECO_MAP_MODE_CAP = 2
ECO_MAP_OFFSET_MPH = -8  # highway cap; interpolated from 50→80 posted
ECO_OFFSET_START_MPH = 50.0  # posted at/below: no eco drop
ECO_OFFSET_FULL_MPH = 80.0  # posted at/above: full −8
ECO_MAP_LOOKAHEAD_EARLY = 3  # NAPMapSpeedLookahead Early
ECO_MAP_ACCEL = 1  # laziest Follow climb (lower peak a / jerk)

SNAPSHOT_KEYS = (
  "NAPAdaptiveAccel",
  "NAPMapSpeedMode",
  "NAPMapSpeedOffsetMph",
  "NAPMapSpeedLookahead",
  "NAPMapSpeedAccel",
)


def clamp_follow_distance(level) -> int:
  try:
    return max(FOLLOW_DISTANCE_MIN, min(FOLLOW_DISTANCE_MAX, int(level)))
  except (TypeError, ValueError):
    return FOLLOW_DISTANCE_DEFAULT


def effective_nap_follow_dist(is_preap: bool, nap_follow_dist) -> int | None:
  """Follow index the planner/MPC should use, or None to fall back to personality.

  Always stock `NAPFollowDistance` 1–7 on Pre-AP. Hypermile does not
  override (no 1–5 band, no ≤50 mph far-gap force).
  """
  if not is_preap:
    return None
  if nap_follow_dist in NAP_FOLLOW_DISTANCE_RANGE:
    return int(nap_follow_dist)
  return None


def follow_distance_hud_text(level: int) -> str:
  return f"Follow Distance: {clamp_follow_distance(level)}"


def poll_follow_distance_hud(prev, current) -> tuple[int | None, bool]:
  """Seed-then-announce Follow Distance HUD.

  First successful read only stores the baseline (no toast on process
  start). Every later stalk/settings change of the 1–7 slider announces.
  """
  if current is None:
    return prev, False
  try:
    cur = clamp_follow_distance(current)
  except (TypeError, ValueError):
    return prev, False
  if prev is None:
    return cur, False
  try:
    return cur, cur != int(prev)
  except (TypeError, ValueError):
    return cur, True


def _get_int(params, key: str, default: int) -> int:
  try:
    raw = params.get(key, return_default=True)
    if raw is None or raw == "":
      return default
    if isinstance(raw, bytes):
      raw = raw.decode("utf-8", errors="ignore")
    return int(raw)
  except Exception:
    return default


def _get_bool(params, key: str, default: bool = False) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return default


def _get_str(params, key: str) -> str:
  try:
    raw = params.get(key, return_default=True)
  except Exception:
    return ""
  if raw is None:
    return ""
  if isinstance(raw, bytes):
    return raw.decode("utf-8", errors="ignore")
  return str(raw)


def read_hypermile_params(params) -> bool:
  """Hypermile On. Unknown / test doubles → Off."""
  return _get_bool(params, PARAM_HYPERMILE, False)


def read_follow_distance(params) -> int:
  """Stock Follow Distance 1–7 (`NAPFollowDistance`). Default 4."""
  return clamp_follow_distance(_get_int(params, PARAM_FOLLOW_DISTANCE, FOLLOW_DISTANCE_DEFAULT))


def read_hypermile_step_down(params) -> bool:
  """Opt-in mileage defer. Default Off. Inert unless Hypermile is On."""
  return _get_bool(params, PARAM_STEP_DOWN, False)


def step_down_applies(hypermile_on: bool, step_down_on: bool) -> bool:
  return bool(hypermile_on) and bool(step_down_on)


def _posted_mph(posted_kph: float | None) -> float | None:
  if posted_kph is None:
    return None
  try:
    pk = float(posted_kph)
  except (TypeError, ValueError):
    return None
  if pk <= 0:
    return None
  return pk * CV.KPH_TO_MPH


def maps_posted_known(posted_kph: float | None, maps_posted: bool | None = None) -> bool:
  """True only with maps present and a real OSM/posted limit.

  Same spirit as sticky MAX: do not invent posted when maps are off,
  unmatched, or speedLimit is unknown / non-positive.
  """
  if maps_posted is False:
    return False
  return _posted_mph(posted_kph) is not None


def posted_scale_offset_mph(posted_mph: float | None, full_offset_mph: float) -> float:
  """0 at/under 50, linear to full_offset at 80, cap above 80.

  Unknown / non-positive posted → 0 (do not invent a town drop).
  """
  if posted_mph is None:
    return 0.0
  try:
    posted = float(posted_mph)
  except (TypeError, ValueError):
    return 0.0
  if posted <= ECO_OFFSET_START_MPH:
    return 0.0
  if posted >= ECO_OFFSET_FULL_MPH:
    return float(full_offset_mph)
  span = ECO_OFFSET_FULL_MPH - ECO_OFFSET_START_MPH
  t = (float(posted) - ECO_OFFSET_START_MPH) / span
  return float(full_offset_mph) * t


def eco_map_offset_mph(posted_mph: float | None) -> float:
  """Posted-scaled Hypermile eco offset (mph). Town stays posted.

  ≤50 → 0; 80 → −8; >80 → −8 cap; linear 50→80.
  """
  return posted_scale_offset_mph(posted_mph, ECO_MAP_OFFSET_MPH)


def step_down_offset_mph(posted_mph: float | None) -> float:
  """Posted-scaled Step Down offset (mph). Same breakpoints as eco, −15 at 80.

  ≤50 → 0; 80 → −15; >80 → −15 cap. Replaces eco — does not stack.
  """
  return posted_scale_offset_mph(posted_mph, -STEP_DOWN_MPH)


def map_target_offset_kph(
  map_offset_kph: float,
  *,
  hypermile_on: bool,
  step_down_on: bool,
  posted_kph: float | None = None,
  maps_posted: bool | None = None,
) -> float:
  """Offset added to raw OSM posted for Cap/Follow.

  Hypermile eco / Step Down apply only when maps are present and the
  OSM/posted limit is known. Maps off, no match, or unknown posted → 0
  (no invented drop). Hypermile On + Step Down On: posted-scaled −15 at
  80 (0 at/under 50), replacing eco so 80→65. Step Down Off: eco
  (≤50 → 0, 80+ → −8). Hypermile Off: keep map_offset_kph.
  """
  if hypermile_on and not maps_posted_known(posted_kph, maps_posted):
    return 0.0
  posted_mph = _posted_mph(posted_kph)
  if step_down_applies(hypermile_on, step_down_on):
    return step_down_offset_mph(posted_mph) * CV.MPH_TO_KPH
  if hypermile_on:
    return eco_map_offset_mph(posted_mph) * CV.MPH_TO_KPH
  return float(map_offset_kph)


def stepped_map_target_kph(
  raw_posted_kph: float,
  *,
  hypermile_on: bool,
  step_down_on: bool,
  map_offset_kph: float = 0.0,
) -> float:
  """Map MAX target from raw posted (kph). Step-down only lowers, ≤15 mph under."""
  raw = float(raw_posted_kph)
  off = map_target_offset_kph(
    map_offset_kph, hypermile_on=hypermile_on, step_down_on=step_down_on,
    posted_kph=raw,
  )
  target = raw + off
  if step_down_applies(hypermile_on, step_down_on):
    floor = raw - STEP_DOWN_MPH * CV.MPH_TO_KPH
    target = min(raw, max(floor, target))
  return target


def read_snapshot_values(params) -> dict:
  return {
    "NAPAdaptiveAccel": _get_bool(params, "NAPAdaptiveAccel", True),
    "NAPMapSpeedMode": _get_int(params, "NAPMapSpeedMode", 0),
    "NAPMapSpeedOffsetMph": _get_int(params, "NAPMapSpeedOffsetMph", 0),
    "NAPMapSpeedLookahead": _get_int(params, "NAPMapSpeedLookahead", 2),
    "NAPMapSpeedAccel": _get_int(params, "NAPMapSpeedAccel", 5),
  }


def eco_preset_from(current: dict) -> dict:
  """Comfort-biased eco knobs. Keep Cap if already Cap; Off/Display → Follow.

  Always Early lookahead + Accel 1. Do not keep Late (late hard regen).
  Do not write NAPMapSpeedOffsetMph — a flat −5 drops town 30→25.
  Live eco offset is posted-scaled in map_target_offset_kph. Soft-lat / DM
  not included.
  """
  mode = int(current.get("NAPMapSpeedMode", 0) or 0)
  if mode not in (ECO_MAP_MODE_CAP, ECO_MAP_MODE_FOLLOW):
    mode = ECO_MAP_MODE_FOLLOW
  return {
    "NAPAdaptiveAccel": ECO_ADAPTIVE_ACCEL,
    "NAPMapSpeedMode": mode,
    "NAPMapSpeedLookahead": ECO_MAP_LOOKAHEAD_EARLY,
    "NAPMapSpeedAccel": ECO_MAP_ACCEL,
  }


def _put_values(params, values: dict) -> None:
  for key, value in values.items():
    if isinstance(value, bool):
      params.put_bool(key, value)
    else:
      params.put(key, int(value))


def apply_hypermile_toggle(params, want_on: bool) -> bool:
  """Snap eco on rising On; restore snapshot on falling Off. Idempotent.

  Returns the resulting On state. Soft-lat / DM / blinker / stock 1–7
  Follow Distance stay untouched (not snapped, not restored).
  """
  was_on = _get_bool(params, PARAM_HYPERMILE, False)
  if want_on and not was_on:
    current = read_snapshot_values(params)
    params.put(PARAM_SAVED, json.dumps(current, separators=(",", ":")))
    _put_values(params, eco_preset_from(current))
    params.put_bool(PARAM_HYPERMILE, True)
    return True
  if (not want_on) and was_on:
    raw = _get_str(params, PARAM_SAVED)
    if raw:
      try:
        saved = json.loads(raw)
        if isinstance(saved, dict):
          restore = {k: saved[k] for k in SNAPSHOT_KEYS if k in saved}
          if restore:
            _put_values(params, restore)
      except (TypeError, ValueError, json.JSONDecodeError):
        pass
    try:
      params.remove(PARAM_SAVED)
    except Exception:
      params.put(PARAM_SAVED, "")
    params.put_bool(PARAM_HYPERMILE, False)
    return False
  params.put_bool(PARAM_HYPERMILE, bool(want_on))
  return bool(want_on)


def stalk_adjusts_follow(*, has_lead: bool) -> bool:
  """Stalk tip remaps stock Follow Distance 1–7 when a radar lead is present.

  Same whether Hypermile is On or Off. Full press (5 mph) still steps MAX.
  No lead: leave Pre-AP stalk as MAX / cruise-speed adjust.
  """
  return bool(has_lead)


def step_follow_distance(level: int, closer: bool) -> int:
  """Stalk up = closer (toward 1). Stalk down = farther (toward 7)."""
  delta = -1 if closer else 1
  return clamp_follow_distance(int(level) + delta)


def stalk_is_closer(button_closer: bool | None, raw_delta_kph: float | None) -> bool | None:
  """True=closer (RES+/up), False=farther (RES−/down), None=not a stalk edge."""
  if button_closer is True:
    return True
  if button_closer is False:
    return False
  if raw_delta_kph is None:
    return None
  if abs(float(raw_delta_kph)) < 0.3:
    return None
  return float(raw_delta_kph) > 0.0


def detect_follow_stalk(
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
) -> tuple[bool, bool | None, float | None]:
  """(is_follow_stalk, closer, undo_raw_kph). Does not write params.

  Lead + tip (~1 mph / ~1 kph / MPH_TO_KPH): remap Follow Distance and
  undo that frame's MAX. Lead + full press (~5 mph / 5 kph / 5×kph):
  leave MAX; not a follow stalk. Pedal delta magnitude is the source of
  truth when present. Button-event-only edges (no clear 5 mph hold
  delta) still count as a tip / bump — Pre-AP buttonEvents do not
  distinguish tip vs hold.
  """
  if not stalk_adjusts_follow(has_lead=has_lead):
    return False, None, None
  have_raw = raw_kph is not None and prev_raw_kph is not None
  # Full press: keep MAX +5/−5. Do not remap Follow Distance.
  if have_raw and is_cruise_stalk_hold_step(prev_raw_kph, raw_kph):
    return False, None, None
  delta = None
  if have_raw and is_cruise_stalk_tip_step(prev_raw_kph, raw_kph):
    delta = float(raw_kph) - float(prev_raw_kph)
  closer = stalk_is_closer(button_closer, delta)
  if closer is None:
    return False, None, None
  undo = float(prev_raw_kph) if prev_raw_kph is not None else None
  return True, closer, undo


def persist_follow_distance(params, closer: bool) -> int:
  """Step and write `NAPFollowDistance` so Driving Mannerisms updates live."""
  level = read_follow_distance(params)
  new_level = step_follow_distance(level, closer)
  if new_level != level:
    params.put(PARAM_FOLLOW_DISTANCE, int(new_level))
  return new_level


def consume_follow_stalk(
  params,
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
  apply: bool = True,
) -> tuple[int | None, float | None]:
  """If this stalk edge is a Follow Distance tip, persist it and undo MAX.

  Returns (new_or_same_level, undo_raw_kph). undo_raw_kph is the previous
  MAX so card can write pedal_speed back. (None, None) means leave stalk
  as MAX adjust (no lead, or a 5 mph full press). apply=False detects
  and returns the current level without writing (card uses this during
  the 0.25 s stalk cooldown).
  """
  is_stalk, closer, undo = detect_follow_stalk(
    has_lead=has_lead,
    button_closer=button_closer,
    raw_kph=raw_kph,
    prev_raw_kph=prev_raw_kph,
  )
  if not is_stalk:
    return None, None
  if apply:
    return persist_follow_distance(params, bool(closer)), undo
  return read_follow_distance(params), undo


def button_event_closer(button_events) -> bool | None:
  """Parse CarState.buttonEvents: accelCruise → closer, decelCruise → farther.

  Press edge only. A tap that also emits a release must not step twice.
  """
  for be in button_events or []:
    try:
      if not bool(getattr(be, "pressed", True)):
        continue
      typ = getattr(be, "type", None)
      name = str(getattr(typ, "name", typ)).lower().replace("_", "")
      if "accelcruise" in name or name in ("accel", "resaccel", "setaccel"):
        return True
      if "decelcruise" in name or name in ("decel", "decelset"):
        return False
    except Exception:
      continue
  return None
