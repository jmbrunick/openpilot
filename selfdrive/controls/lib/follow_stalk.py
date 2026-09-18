"""Pre-AP stalk tip remaps stock Follow Distance when a radar lead is present.

Writes the same NAPFollowDistance 1–7 as Settings → NAP → Driving
Mannerisms → Follow Distance. A first-detent tip (1 mph / 1 kph): stalk
up = closer (toward 1), down = farther (toward 7), undo that frame's
MAX, commit Follow on return to IDLE. A full press (2nd detent / 5 mph)
keeps MAX. No lead: leave stalk as MAX.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd.map_speed_policy import (
  is_cruise_stalk_hold_step,
  is_cruise_stalk_tip_step,
)

from openpilot.selfdrive.controls.lib.follow_distance import (
  PARAM_FOLLOW,
  PARAM_FOLLOW_CITY,
  PARAM_FOLLOW_HWY,
  follow_band_is_highway,
  migrate_follow_distance_params,
)
PARAM_FOLLOW_HUD_PENDING = "NAPFollowHudPending"
FOLLOW_MIN = 1
FOLLOW_MAX = 7
FOLLOW_DEFAULT = 4
# Tesla stalk can emit a button edge and a 1/5 mph pedal step. Hold repeats.
STALK_COOLDOWN_S = 0.25

# Pre-AP STW_ACTN_RQ.SpdCtrlLvr_Stat / CruiseButtons. buttonEvents collapse
# tip and 2nd detent to the same accelCruise/decelCruise, so detent is the
# hardware source of truth when present.
CRUISE_STALK_IDLE = 0
CRUISE_STALK_UP_2ND = 4    # UP_2ND / RES_ACCEL_2ND  +5
CRUISE_STALK_DN_2ND = 8    # DN_2ND / DECEL_2ND      −5
CRUISE_STALK_UP_1ST = 16   # UP_1ST / RES_ACCEL      +1
CRUISE_STALK_DN_1ST = 32   # DN_1ST / DECEL_SET      −1


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


def stalk_adjusts_follow(*, has_lead: bool) -> bool:
  """Stalk tip remaps 1–7 only while a radar lead is present.

  Full press (5 mph) still steps MAX. No lead: leave Pre-AP stalk as
  MAX / cruise-speed adjust.
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
  params,
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
  del params
  g = gesture if gesture is not None else FollowStalkGesture()
  return g.update(
    has_lead=has_lead,
    detent=detent,
    button_closer=button_closer,
    button_released=button_released,
    raw_kph=raw_kph,
    prev_raw_kph=prev_raw_kph,
  )


def persist_follow_distance(params, closer: bool, v_ego=None, v_cruise=None,
                            has_lead=False, engaged=False) -> int:
  """Step city or highway Follow Distance (and HUD NAPFollowDistance).

  With v_ego, the active band is stepped (48/52 ego, or hwy-from-30 when
  MAX > 50 with a lead). Without speed (tests), all three keys stay in
  sync. Always request the HUD (including at 1/7).
  """
  migrate_follow_distance_params(params)
  if v_ego is not None:
    key = PARAM_FOLLOW_HWY if follow_band_is_highway(
      v_ego, v_cruise=v_cruise, engaged=engaged, has_lead=has_lead,
    ) else PARAM_FOLLOW_CITY
    try:
      raw = params.get(key, return_default=True)
    except Exception:
      raw = None
    level = clamp_follow_distance(raw)
    new_level = step_follow_distance(level, closer)
    if new_level != level:
      params.put(key, int(new_level))
    params.put(PARAM_FOLLOW, int(new_level))
  else:
    level = read_follow_distance(params)
    new_level = step_follow_distance(level, closer)
    if new_level != level:
      params.put(PARAM_FOLLOW, int(new_level))
    try:
      params.put(PARAM_FOLLOW_CITY, int(new_level))
      params.put(PARAM_FOLLOW_HWY, int(new_level))
    except Exception:
      pass
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
  v_ego=None,
) -> tuple[int | None, float | None]:
  """If this stalk edge completes a Follow Distance tip, persist it.

  Returns (new_or_same_level, undo_raw_kph). undo_raw_kph is the previous
  MAX on the tip-press frame so card can write pedal_speed back. Commit
  (level not None) happens on IDLE after a tip-only gesture. apply=False
  detects without writing (0.25 s stalk cooldown).
  """
  is_stalk, closer, undo = detect_follow_stalk(
    params,
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
    return persist_follow_distance(params, bool(closer), v_ego=v_ego), undo
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
