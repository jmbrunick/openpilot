"""Pre-AP stalk +/- remaps stock Follow Distance when a radar lead is present.

Not Hypermile. Writes the same NAPFollowDistance 1–7 as
Settings → NAP → Driving Mannerisms → Follow Distance. Stalk up = closer
(toward 1). Stalk down = farther (toward 7). No lead: leave stalk as MAX.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.map_speed_policy import is_cruise_stalk_step

PARAM_FOLLOW = "NAPFollowDistance"
FOLLOW_MIN = 1
FOLLOW_MAX = 7
FOLLOW_DEFAULT = 4
# Tesla stalk can emit a button edge and a 1/5 mph pedal step. Hold repeats.
STALK_COOLDOWN_S = 0.25


def clamp_follow_distance(level: int | None) -> int:
  try:
    return max(FOLLOW_MIN, min(FOLLOW_MAX, int(level)))
  except (TypeError, ValueError):
    return FOLLOW_DEFAULT


def read_follow_distance(params) -> int:
  try:
    raw = params.get(PARAM_FOLLOW, return_default=True)
  except Exception:
    raw = FOLLOW_DEFAULT
  if raw is None:
    raw = FOLLOW_DEFAULT
  return clamp_follow_distance(raw)


def follow_distance_hud_text(level: int | None) -> str:
  return f"Follow Distance: {clamp_follow_distance(level)}"


def stalk_adjusts_follow(*, has_lead: bool) -> bool:
  """Stalk +/- remaps 1–7 only while a radar lead is present.

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
  params,
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
) -> tuple[bool, bool | None, float | None]:
  """(is_follow_stalk, closer, undo_raw_kph). Does not write params."""
  if not stalk_adjusts_follow(has_lead=has_lead):
    return False, None, None
  delta = None
  if raw_kph is not None and prev_raw_kph is not None and is_cruise_stalk_step(prev_raw_kph, raw_kph):
    delta = float(raw_kph) - float(prev_raw_kph)
  closer = stalk_is_closer(button_closer, delta)
  if closer is None:
    return False, None, None
  undo = float(prev_raw_kph) if prev_raw_kph is not None else None
  return True, closer, undo


def persist_follow_distance(params, closer: bool) -> int:
  level = read_follow_distance(params)
  new_level = step_follow_distance(level, closer)
  if new_level != level:
    params.put(PARAM_FOLLOW, int(new_level))
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
  """If this stalk edge is a follow-distance step, persist it and undo MAX.

  Returns (new_or_same_level, undo_raw_kph). undo_raw_kph is the previous
  MAX so card can write pedal_speed back. (None, None) means leave stalk
  as MAX adjust. apply=False detects and returns the current level
  without writing (card uses this during the 0.25 s stalk cooldown).
  """
  is_stalk, closer, undo = detect_follow_stalk(
    params,
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
