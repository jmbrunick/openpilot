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
PARAM_FOLLOW_HUD_PENDING = "NAPFollowHudPending"
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


# Pre-AP STW_ACTN_RQ.SpdCtrlLvr_Stat / CruiseButtons. buttonEvents collapse
# tip and 2nd detent to the same accelCruise/decelCruise, so detent is the
# hardware source of truth when present.
CRUISE_STALK_IDLE = 0
CRUISE_STALK_UP_2ND = 4    # UP_2ND / RES_ACCEL_2ND  +5
CRUISE_STALK_DN_2ND = 8    # DN_2ND / DECEL_2ND      −5
CRUISE_STALK_UP_1ST = 16   # UP_1ST / RES_ACCEL      +1
CRUISE_STALK_DN_1ST = 32   # DN_1ST / DECEL_SET      −1


def cruise_detent_closer(detent: int | None) -> bool | None:
  if detent in (CRUISE_STALK_UP_1ST, CRUISE_STALK_UP_2ND):
    return True
  if detent in (CRUISE_STALK_DN_1ST, CRUISE_STALK_DN_2ND):
    return False
  return None


def cruise_detent_is_tip(detent: int | None) -> bool:
  return detent in (CRUISE_STALK_UP_1ST, CRUISE_STALK_DN_1ST)


def cruise_detent_is_hold(detent: int | None) -> bool:
  return detent in (CRUISE_STALK_UP_2ND, CRUISE_STALK_DN_2ND)


def cruise_detent_is_idle(detent: int | None) -> bool:
  return detent == CRUISE_STALK_IDLE


class FollowStalkGesture:
  """Classify Pre-AP cruise-stalk tip vs full press, like ALC tip-vs-hold.

  A physical full press always walks through first detent. Follow Distance
  is committed only when the lever returns to IDLE after a first-detent
  tip that never hit 2nd detent (or a clear 5 mph hold step). The tip
  frame's 1 mph MAX is undone immediately so 2nd detent's +5/−5 applies
  from the original set. buttonEvents alone are not a tip.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.pending_closer: bool | None = None
    self.saw_hold = False
    self.undid = False

  @property
  def is_pending(self) -> bool:
    return self.pending_closer is not None and not self.saw_hold

  def update(
    self,
    *,
    has_lead: bool,
    detent: int | None = None,
    button_closer: bool | None = None,
    button_released: bool = False,
    raw_kph: float | None = None,
    prev_raw_kph: float | None = None,
  ) -> tuple[bool, bool | None, float | None]:
    """(commit_follow, closer, undo_raw_kph). Does not write params."""
    if not stalk_adjusts_follow(has_lead=has_lead):
      self.reset()
      return False, None, None

    have_raw = raw_kph is not None and prev_raw_kph is not None
    hold_step = have_raw and is_cruise_stalk_hold_step(prev_raw_kph, raw_kph)
    tip_step = have_raw and is_cruise_stalk_tip_step(prev_raw_kph, raw_kph)
    is_hold = cruise_detent_is_hold(detent) or hold_step
    is_tip = cruise_detent_is_tip(detent) or tip_step
    closer = cruise_detent_closer(detent)
    if closer is None:
      delta = (float(raw_kph) - float(prev_raw_kph)) if tip_step else None
      closer = stalk_is_closer(button_closer, delta)

    if is_hold:
      self.saw_hold = True
      self.pending_closer = None
      return False, None, None

    # Second press while already pending, no detent / 5 mph: 2nd detent.
    if (
      self.pending_closer is not None
      and button_closer is not None
      and detent is None
      and not tip_step
    ):
      self.saw_hold = True
      self.pending_closer = None
      return False, None, None

    undo = None
    if is_tip and not self.saw_hold:
      if closer is not None:
        self.pending_closer = closer
      if tip_step and not self.undid:
        undo = float(prev_raw_kph)
        self.undid = True

    if (
      not is_tip
      and not is_hold
      and button_closer is not None
      and self.pending_closer is None
      and not self.saw_hold
    ):
      self.pending_closer = button_closer

    is_idle = cruise_detent_is_idle(detent) or (detent is None and button_released)
    if is_idle:
      if self.pending_closer is not None and not self.saw_hold:
        closer_out = self.pending_closer
        self.reset()
        return True, closer_out, None
      self.reset()
      return False, None, None

    return False, None, undo


def detect_follow_stalk(
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
  detent: int | None = None,
  button_released: bool = False,
  gesture: FollowStalkGesture | None = None,
) -> tuple[bool, bool | None, float | None]:
  """(commit_follow, closer, undo_raw_kph). Does not write params.

  Lead + first-detent tip (~1 mph / 1 kph): undo that frame's MAX and
  commit Follow only after the lever returns to IDLE without a 2nd
  detent. Lead + full press (2nd detent / ~5 mph): leave MAX; never a
  follow stalk, even if the press passed through first detent.
  Pre-AP buttonEvents do not distinguish tip vs hold — do not treat a
  button-only press as a completed tip.
  """
  g = gesture if gesture is not None else FollowStalkGesture()
  return g.update(
    has_lead=has_lead,
    detent=detent,
    button_closer=button_closer,
    button_released=button_released,
    raw_kph=raw_kph,
    prev_raw_kph=prev_raw_kph,
  )


def persist_follow_distance(params, closer: bool) -> int:
  """Step and write `NAPFollowDistance` so Driving Mannerisms updates live.

  Always sets NAPFollowHudPending so a tip at 1 or 7 still toasts the
  current level (value-only poll would miss a no-op write).
  """
  level = read_follow_distance(params)
  new_level = step_follow_distance(level, closer)
  if new_level != level:
    params.put(PARAM_FOLLOW_DISTANCE, int(new_level))
  try:
    params.put_bool(PARAM_FOLLOW_HUD_PENDING, True)
  except Exception:
    pass
  return new_level


def consume_follow_stalk(
  params,
  *,
  has_lead: bool,
  button_closer: bool | None,
  raw_kph: float | None,
  prev_raw_kph: float | None,
  apply: bool = True,
  detent: int | None = None,
  button_released: bool = False,
  gesture: FollowStalkGesture | None = None,
) -> tuple[int | None, float | None]:
  """If this stalk edge completes a Follow Distance tip, persist it.

  Returns (new_or_same_level, undo_raw_kph). undo_raw_kph is the previous
  MAX on the tip-press frame so card can write pedal_speed back. Commit
  (level not None) happens on IDLE after a tip-only gesture. (None, None)
  means leave stalk as MAX (no lead, full press, or tip still held).
  apply=False detects without writing (0.25 s stalk cooldown).
  """
  is_stalk, closer, undo = detect_follow_stalk(
    has_lead=has_lead,
    button_closer=button_closer,
    raw_kph=raw_kph,
    prev_raw_kph=prev_raw_kph,
    detent=detent,
    button_released=button_released,
    gesture=gesture,
  )
  if not is_stalk:
    return None, undo
  if apply:
    return persist_follow_distance(params, bool(closer)), undo
  return read_follow_distance(params), undo


def _button_event_name(be) -> str:
  typ = getattr(be, "type", None)
  return str(getattr(typ, "name", typ)).lower().replace("_", "")


def _is_cruise_adjust_button(name: str) -> bool:
  return (
    "accelcruise" in name or "decelcruise" in name
    or name in ("accel", "resaccel", "setaccel", "decel", "decelset")
  )


def button_event_closer(button_events) -> bool | None:
  """Parse CarState.buttonEvents: accelCruise → closer, decelCruise → farther.

  Press edge only. A tap that also emits a release must not step twice.
  Pre-AP maps both tip and 2nd detent to these types — not a tip vs hold.
  """
  for be in button_events or []:
    try:
      if not bool(getattr(be, "pressed", True)):
        continue
      name = _button_event_name(be)
      if "accelcruise" in name or name in ("accel", "resaccel", "setaccel"):
        return True
      if "decelcruise" in name or name in ("decel", "decelset"):
        return False
    except Exception:
      continue
  return None


def button_event_released(button_events) -> bool:
  """True on accel/decelCruise release. Used when raw detent is unavailable."""
  for be in button_events or []:
    try:
      if bool(getattr(be, "pressed", True)):
        continue
      if _is_cruise_adjust_button(_button_event_name(be)):
        return True
    except Exception:
      continue
  return False
