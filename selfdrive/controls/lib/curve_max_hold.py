"""Snapshot HUD MAX / sticky hold at curve entry; restore after the bend.

Through a sharp curve, temporary slowing (planner `limit_accel_in_turns`
and/or a comfort lat-accel cruise cap) may lower published MAX so the car
takes an appropriate corner speed. That lower value must not permanently
rebase sticky MAX or the Cap/Follow map target.

Hypermile eco −5 (posted 60 → steady target 55) is unchanged. If MAX was
60 before the bend (sticky hold or that displayed set), restore 60 — not
the eco target the curve path would otherwise land on.

GPS / OSM flicker on a bend used to look like posted `a` → `b`, wipe
sticky, then restore to posted+offset (55). Freeze that rebase while the
curve is active; after lat accel / steer are straight-ish, put MAX back.
A posted change that is still there after exit is a real new zone and
is allowed to rebase.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.map_speed_policy import V_CRUISE_UNSET, posted_limits_same

# Same a_y formula as longitudinal_planner.limit_accel_in_turns.
CURVE_ENTER_LAT_MS2 = 1.30
CURVE_EXIT_LAT_MS2 = 0.65
CURVE_ENTER_STEER_DEG = 14.0
CURVE_EXIT_STEER_DEG = 8.0
CURVE_EXIT_HOLD_S = 0.45
# Comfortable corner lat accel for the temporary HUD/planner cap.
CURVE_COMFORT_LAT_MS2 = 2.00
# Ignore parking-lot / standstill wheel spin.
CURVE_MIN_V_EGO_MS = 5.0
# Do not invent a crawl MAX in a hairpin.
CURVE_SPEED_FLOOR_MS = 6.0


def steer_lat_accel_ms2(v_ego_ms: float, angle_steers_deg: float,
                        steer_ratio: float, wheelbase: float) -> float:
  """Unsigned lateral accel from steer angle (same model as limit_accel_in_turns)."""
  sr = float(steer_ratio) if steer_ratio and steer_ratio > 1e-3 else 15.75
  wb = float(wheelbase) if wheelbase and wheelbase > 1e-3 else 2.959
  a_y = (float(v_ego_ms) ** 2) * float(angle_steers_deg) * CV.DEG_TO_RAD / (sr * wb)
  return abs(a_y)


def curve_speed_ms(v_ego_ms: float, angle_steers_deg: float,
                   steer_ratio: float, wheelbase: float,
                   a_lat: float = CURVE_COMFORT_LAT_MS2) -> float | None:
  """Comfort v from current steer. None when not a meaningful bend."""
  if v_ego_ms < CURVE_MIN_V_EGO_MS or a_lat <= 0:
    return None
  ang = abs(float(angle_steers_deg))
  if ang < 1e-3:
    return None
  sr = float(steer_ratio) if steer_ratio and steer_ratio > 1e-3 else 15.75
  wb = float(wheelbase) if wheelbase and wheelbase > 1e-3 else 2.959
  kappa = ang * CV.DEG_TO_RAD / (sr * wb)
  if kappa <= 1e-6:
    return None
  v = math.sqrt(float(a_lat) / kappa)
  if v < CURVE_SPEED_FLOOR_MS:
    return CURVE_SPEED_FLOOR_MS
  return v


def is_sharp_curve(v_ego_ms: float, angle_steers_deg: float,
                   steer_ratio: float, wheelbase: float, *,
                   active: bool) -> bool:
  """Hysteresis: enter on lat accel or steer; stay until both are quiet."""
  if v_ego_ms < CURVE_MIN_V_EGO_MS:
    return False
  a_y = steer_lat_accel_ms2(v_ego_ms, angle_steers_deg, steer_ratio, wheelbase)
  steer = abs(float(angle_steers_deg))
  if active:
    return a_y >= CURVE_EXIT_LAT_MS2 or steer >= CURVE_EXIT_STEER_DEG
  return a_y >= CURVE_ENTER_LAT_MS2 or steer >= CURVE_ENTER_STEER_DEG


def _valid_max_kph(kph: float | None) -> bool:
  if kph is None:
    return False
  return 0.0 < float(kph) < float(V_CRUISE_UNSET)


@dataclass
class CurveMaxSnapshot:
  hud_max_kph: float
  held_max_kph: float | None
  sticky_set_kph: float | None
  posted_kph: float | None


@dataclass
class CurveMaxDecision:
  """One card.py cycle after the map overlay computed HUD MAX."""
  hud_kph: float
  restore_seed_kph: float | None
  freeze_posted: bool
  active: bool


class CurveMaxHold:
  """Lat-based snapshot / restore of pre-curve MAX."""

  def __init__(self) -> None:
    self.snapshot: CurveMaxSnapshot | None = None
    self.active: bool = False
    self._exit_s: float = 0.0

  def reset(self) -> None:
    self.snapshot = None
    self.active = False
    self._exit_s = 0.0

  def lat_curving(self, v_ego_ms: float, angle_steers_deg: float,
                  steer_ratio: float, wheelbase: float) -> bool:
    return is_sharp_curve(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase, active=self.active,
    )

  def should_freeze_posted(self, v_ego_ms: float, angle_steers_deg: float,
                           steer_ratio: float, wheelbase: float) -> bool:
    """True while in/exiting a curve — do not treat OSM flicker as posted b."""
    return bool(self.active or self.lat_curving(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase,
    ))

  def capture(self, *, hud_max_kph: float,
              held_max_kph: float | None,
              sticky_set_kph: float | None,
              posted_kph: float | None) -> None:
    hud = float(hud_max_kph)
    if not _valid_max_kph(hud):
      return
    self.snapshot = CurveMaxSnapshot(
      hud_max_kph=hud,
      held_max_kph=float(held_max_kph) if _valid_max_kph(held_max_kph) else None,
      sticky_set_kph=float(sticky_set_kph) if _valid_max_kph(sticky_set_kph) else None,
      posted_kph=float(posted_kph) if _valid_max_kph(posted_kph) else None,
    )
    self.active = True
    self._exit_s = 0.0

  def begin_cycle(
    self,
    hold,
    *,
    last_hud_kph: float,
    posted_kph: float | None,
    v_ego_ms: float,
    angle_steers_deg: float,
    steer_ratio: float,
    wheelbase: float,
    engaged: bool,
    take_speed_now: bool = False,
    dt: float = 0.01,
  ) -> tuple[float | None, bool]:
    """Before decide_map_cruise: snapshot pre-curve MAX, freeze posted flicker.

    Returns (posted_kph for policy, freeze). Snapshot posted is the last
    known zone (`hold.last_posted_kph`), not this frame's possibly-flickered
    match. The exit frame is not frozen so a posted that persisted through
    the bend can rebase.
    """
    if not engaged or take_speed_now:
      self.reset()
      return posted_kph, False
    lat_now = self.lat_curving(v_ego_ms, angle_steers_deg, steer_ratio, wheelbase)
    if self.active and not lat_now and (self._exit_s + float(dt)) + 1e-9 >= CURVE_EXIT_HOLD_S:
      return posted_kph, False
    freeze = bool(self.active or lat_now)
    if freeze and self.snapshot is None:
      posted_snap = getattr(hold, "last_posted_kph", None)
      if not _valid_max_kph(posted_snap):
        posted_snap = posted_kph
      hud_snap = last_hud_kph
      if not _valid_max_kph(hud_snap):
        if _valid_max_kph(getattr(hold, "sticky_set_kph", None)):
          hud_snap = float(hold.sticky_set_kph)
        elif _valid_max_kph(getattr(hold, "held_max_kph", None)):
          hud_snap = float(hold.held_max_kph)
      self.capture(
        hud_max_kph=float(hud_snap) if _valid_max_kph(hud_snap) else 0.0,
        held_max_kph=getattr(hold, "held_max_kph", None),
        sticky_set_kph=getattr(hold, "sticky_set_kph", None),
        posted_kph=posted_snap,
      )
    if freeze and self.snapshot is not None:
      return self.policy_posted_kph(posted_kph), True
    return posted_kph, freeze

  def policy_posted_kph(self, posted_kph: float | None) -> float | None:
    """Posted value decide_map_cruise should see while the curve is frozen."""
    if self.snapshot is not None and self.snapshot.posted_kph is not None:
      return float(self.snapshot.posted_kph)
    return posted_kph

  def protect_hold(self, hold) -> None:
    """Undo a downward sticky/held write the curve path invented."""
    snap = self.snapshot
    if snap is None or not self.active:
      return
    if snap.held_max_kph is not None:
      hold.held_max_kph = float(snap.held_max_kph)
    if snap.sticky_set_kph is not None:
      hold.sticky_set_kph = float(snap.sticky_set_kph)
    if snap.posted_kph is not None:
      hold.last_posted_kph = float(snap.posted_kph)

  def restore_hold(self, hold) -> float | None:
    """Write snapshot back onto hold. Returns HUD MAX to publish, or None."""
    snap = self.snapshot
    if snap is None:
      return None
    self.protect_hold(hold)
    return float(snap.hud_max_kph)

  def finish(
    self,
    *,
    hud_kph: float,
    hold,
    posted_kph: float | None,
    v_ego_ms: float,
    angle_steers_deg: float,
    steer_ratio: float,
    wheelbase: float,
    engaged: bool,
    stalk_pressed: bool = False,
    take_speed_now: bool = False,
    dt: float,
  ) -> CurveMaxDecision:
    """Apply temp curve cap or restore. Call after decide_map_cruise + overlay."""
    if not engaged or take_speed_now:
      self.reset()
      return CurveMaxDecision(float(hud_kph), None, False, False)

    lat_now = self.lat_curving(v_ego_ms, angle_steers_deg, steer_ratio, wheelbase)

    if lat_now and not self.active:
      # Fallback if begin_cycle did not run. Prefer hold (pre-flicker).
      last_hud = float(hud_kph)
      if _valid_max_kph(getattr(hold, "sticky_set_kph", None)):
        last_hud = float(hold.sticky_set_kph)
      elif _valid_max_kph(getattr(hold, "held_max_kph", None)):
        last_hud = float(hold.held_max_kph)
      posted_snap = getattr(hold, "last_posted_kph", None)
      if not _valid_max_kph(posted_snap):
        posted_snap = posted_kph
      self.capture(
        hud_max_kph=last_hud,
        held_max_kph=getattr(hold, "held_max_kph", None),
        sticky_set_kph=getattr(hold, "sticky_set_kph", None),
        posted_kph=posted_snap,
      )

    if not self.active:
      return CurveMaxDecision(float(hud_kph), None, False, False)

    will_exit = (not lat_now) and (self._exit_s + float(dt)) + 1e-9 >= CURVE_EXIT_HOLD_S
    if stalk_pressed:
      posted_snap = posted_kph
      if self.snapshot is not None and self.snapshot.posted_kph is not None:
        posted_snap = self.snapshot.posted_kph
      self.capture(
        hud_max_kph=float(hud_kph),
        held_max_kph=float(hud_kph) if _valid_max_kph(hud_kph) else getattr(hold, "held_max_kph", None),
        sticky_set_kph=float(hud_kph) if getattr(hold, "sticky_set_kph", None) is not None else None,
        posted_kph=posted_snap,
      )
    elif not will_exit:
      self.protect_hold(hold)

    if lat_now:
      self._exit_s = 0.0
      capped = _temp_curve_hud(
        float(hud_kph), v_ego_ms, angle_steers_deg, steer_ratio, wheelbase,
      )
      return CurveMaxDecision(capped, None, True, True)

    self._exit_s += float(dt)
    if not will_exit:
      capped = _temp_curve_hud(
        float(hud_kph), v_ego_ms, angle_steers_deg, steer_ratio, wheelbase,
      )
      return CurveMaxDecision(capped, None, True, True)

    # Straight-ish long enough. Same posted zone → restore pre-curve MAX.
    # A new posted that persisted through the bend is a real rebase.
    same_zone = (
      self.snapshot is not None
      and (
        self.snapshot.posted_kph is None
        or posted_kph is None
        or posted_limits_same(self.snapshot.posted_kph, posted_kph)
      )
    )
    if same_zone:
      restored = self.restore_hold(hold)
      self.reset()
      if restored is not None:
        return CurveMaxDecision(float(restored), float(restored), False, False)
    self.reset()
    return CurveMaxDecision(float(hud_kph), None, False, False)


def _temp_curve_hud(hud_kph: float, v_ego_ms: float, angle_steers_deg: float,
                    steer_ratio: float, wheelbase: float) -> float:
  """Lower published MAX for the bend only. Never raise. Never below floor."""
  v_curve = curve_speed_ms(v_ego_ms, angle_steers_deg, steer_ratio, wheelbase)
  if v_curve is None:
    return float(hud_kph)
  curve_kph = float(v_curve) * CV.MS_TO_KPH
  return min(float(hud_kph), curve_kph)
