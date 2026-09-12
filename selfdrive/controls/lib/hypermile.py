"""Hypermile: eco preset + speed-split follow for Pre-AP NAP.

Phase 1+2. Not a second long controller. Publishes an effective stock
follow-distance index (1–7) into the existing MPC / lead-approach path.

Settings → NAP → Driving Mannerisms → Hypermile (default Off).
"""
from __future__ import annotations

import json

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.map_speed_policy import is_cruise_stalk_step

# Params
PARAM_HYPERMILE = "NAPHypermile"
PARAM_FOLLOW_LEVEL = "NAPHypermileFollowLevel"
PARAM_SAVED = "NAPHypermileSaved"
PARAM_STEP_DOWN = "NAPHypermileStepDown"

# Mileage defer: same 50→80 posted scale as eco, larger drop. Does not
# stack with eco. Hard cap — never more than this under posted.
STEP_DOWN_MPH = 15.0

# Stalk-selected band while Hypermile is On.
FOLLOW_LEVEL_MIN = 1
FOLLOW_LEVEL_MAX = 5
FOLLOW_LEVEL_DEFAULT = 3

# Stock NAPFollowDistance 1–7 → T_FOLLOW 0.7 … 1.9 s.
# Hypermile 1–5 maps onto stock 2–6 so high-speed draft is tighter than the
# low-speed far gap, but never stock 1 (0.7 s bumper-draft).
HYPERMILE_TO_STOCK = (2, 3, 4, 5, 6)
SAFE_FLOOR_STOCK = 2  # 0.9 s
LOW_SPEED_STOCK = 7   # 1.9 s — far gap below the split
NAP_FOLLOW_DISTANCE_RANGE = range(1, 8)

# vEgo ≤ this uses the far gap (cut stop-and-go). Above: stalk 1–5.
SPLIT_MPH = 50.0
SPLIT_MS = SPLIT_MPH * CV.MPH_TO_MS

# Eco snap. Comfort-biased efficiency — not maximum regen bite.
# Early + light map ease (lookahead Early = 0.55 m/s², starts farther out)
# and a lazy climb (Accel 1). Late lookahead (1.20 m/s²) is the wrong trade:
# it holds speed then dumps regen. Safety / MPC hard brake unchanged.
# Soft-lat / DM / blinker / stock 1–7 follow are not touched.
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


def clamp_follow_level(level) -> int:
  try:
    return max(FOLLOW_LEVEL_MIN, min(FOLLOW_LEVEL_MAX, int(level)))
  except (TypeError, ValueError):
    return FOLLOW_LEVEL_DEFAULT


def hypermile_level_to_stock(level: int) -> int:
  """Stalk 1–5 → stock follow index, never below the safe floor."""
  idx = clamp_follow_level(level) - 1
  stock = HYPERMILE_TO_STOCK[idx]
  return max(SAFE_FLOOR_STOCK, int(stock))


def effective_nap_follow_dist(
  is_preap: bool,
  nap_follow_dist,
  hypermile_on: bool,
  hypermile_level: int,
  v_ego_ms: float,
) -> int | None:
  """Follow index the planner/MPC should use, or None to fall back to personality."""
  if not is_preap:
    return None
  if hypermile_on:
    if float(v_ego_ms) <= SPLIT_MS:
      return LOW_SPEED_STOCK
    return hypermile_level_to_stock(hypermile_level)
  if nap_follow_dist in NAP_FOLLOW_DISTANCE_RANGE:
    return int(nap_follow_dist)
  return None


def follow_level_hud_text(level: int) -> str:
  return f"Hypermile: Follow {clamp_follow_level(level)}"


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


def read_hypermile_params(params) -> tuple[bool, int]:
  """(on, level 1–5). Unknown / test doubles → Off, 3."""
  return _get_bool(params, PARAM_HYPERMILE, False), clamp_follow_level(
    _get_int(params, PARAM_FOLLOW_LEVEL, FOLLOW_LEVEL_DEFAULT)
  )


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
) -> float:
  """Offset added to raw OSM posted for Cap/Follow.

  Hypermile On + Step Down On: posted-scaled −15 at 80 (0 at/under 50),
  replacing eco so 80→65 (not 57). Hypermile On + Step Down Off: live
  posted-scaled eco (≤50 → 0, 80+ → −8). Unknown posted → 0. Hypermile
  Off: step-down is inert; keep map_offset_kph.
  """
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

  Returns the resulting On state. Soft-lat / DM / blinker / 1–7 follow
  stay untouched. Follow level 1–5 is kept across Off so the next drive
  resumes the last stalk choice.
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


def stalk_adjusts_follow(*, hypermile_on: bool, has_lead: bool) -> bool:
  """Stalk +/- remaps 1–5 only while Hypermile is On and a lead is present.

  No lead: leave Pre-AP stalk as MAX / cruise-speed adjust.
  """
  return bool(hypermile_on) and bool(has_lead)


def step_hypermile_level(level: int, closer: bool) -> int:
  """Stalk up = closer (toward 1). Stalk down = farther (toward 5)."""
  delta = -1 if closer else 1
  return clamp_follow_level(int(level) + delta)


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


def detect_hypermile_stalk(
  params,
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
) -> tuple[bool, bool | None, float | None]:
  """(is_follow_stalk, closer, undo_raw_kph). Does not write params."""
  on, _level = read_hypermile_params(params)
  if not stalk_adjusts_follow(hypermile_on=on, has_lead=has_lead):
    return False, None, None
  delta = None
  if raw_kph is not None and prev_raw_kph is not None and is_cruise_stalk_step(prev_raw_kph, raw_kph):
    delta = float(raw_kph) - float(prev_raw_kph)
  closer = stalk_is_closer(button_closer, delta)
  if closer is None:
    return False, None, None
  undo = float(prev_raw_kph) if prev_raw_kph is not None else None
  return True, closer, undo


def persist_follow_level(params, closer: bool) -> int:
  _on, level = read_hypermile_params(params)
  new_level = step_hypermile_level(level, closer)
  if new_level != level:
    params.put(PARAM_FOLLOW_LEVEL, int(new_level))
  return new_level


def consume_hypermile_stalk(
  params,
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
  apply: bool = True,
) -> tuple[int | None, float | None]:
  """If this stalk edge is a follow-level step, persist it and undo MAX.

  Returns (new_or_same_level, undo_raw_kph). undo_raw_kph is the previous
  MAX so card can write pedal_speed back. (None, None) means leave stalk
  as MAX adjust. apply=False detects and returns the would-be level
  without writing (card uses this during the 0.25 s stalk cooldown).
  """
  is_stalk, closer, undo = detect_hypermile_stalk(
    params,
    has_lead=has_lead,
    button_closer=button_closer,
    raw_kph=raw_kph,
    prev_raw_kph=prev_raw_kph,
  )
  if not is_stalk:
    return None, None
  if apply:
    return persist_follow_level(params, bool(closer)), undo
  _on, level = read_hypermile_params(params)
  return level, undo


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
