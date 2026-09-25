"""Snapshot HUD MAX / sticky hold at curve entry; restore after the bend.

Through a sharp curve, temporary slowing (planner `limit_accel_in_turns`
and/or a comfort lat-accel cruise cap) may lower published MAX so the car
takes an appropriate corner speed. That lower value must not permanently
rebase sticky MAX or the Cap/Follow map target.

Hypermile eco is the live posted-scaled Cap/Follow target. If MAX was
60 before the bend (sticky hold or that displayed set), restore 60 — not
the live eco / posted target the curve path would otherwise land on.

GPS / OSM flicker on a bend used to look like posted `a` → `b`, wipe
sticky, then restore to posted+offset. Freeze that rebase while the
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
# Light smoothing so the cap is not a raw frame of curvature / yaw.
CURVE_LAT_TAU_S = 0.40
# Mid-curve the cap may drop when the bend actually tightens, but it
# must not rise. Steering jitter used to pump MAX both ways. The first
# fraction of a second still tracks the smoothed estimate so one noisy
# frame is not frozen in.
CURVE_CAP_TIGHTEN_KPH = 2.0
CURVE_CAP_SETTLE_S = 0.45
# Exit raises MAX toward the snapshot at about an Accel-3 rate unless
# card passes the live map Accel envelope.
CURVE_RESTORE_A_DEFAULT_MS2 = 0.40


def steer_lat_accel_ms2(v_ego_ms: float, angle_steers_deg: float,
                        steer_ratio: float, wheelbase: float) -> float:
  """Unsigned lateral accel from steer angle (same model as limit_accel_in_turns)."""
  sr = float(steer_ratio) if steer_ratio and steer_ratio > 1e-3 else 15.75
  wb = float(wheelbase) if wheelbase and wheelbase > 1e-3 else 2.959
  a_y = (float(v_ego_ms) ** 2) * float(angle_steers_deg) * CV.DEG_TO_RAD / (sr * wb)
  return abs(a_y)


def cornering_lat_accel_ms2(v_ego_ms: float, angle_steers_deg: float,
                            steer_ratio: float, wheelbase: float, *,
                            curvature: float | None = None,
                            yaw_rate: float | None = None) -> float:
  """Unsigned lateral accel from real cornering, else the steer model.

  Curvature is the vehicle-model / desired path curvature (learned steer
  ratio, tire stiffness, angle offset). Yaw rate × v is the same quantity
  when that curvature is not available. Raw steering angle skips the angle
  offset, uses the stock steer ratio, and ignores understeer, so it reads
  high. The steer model remains only when both signals are missing.
  """
  v = float(v_ego_ms)
  if curvature is not None and math.isfinite(float(curvature)):
    return abs(float(curvature)) * v * v
  if yaw_rate is not None and math.isfinite(float(yaw_rate)):
    return abs(float(yaw_rate) * v)
  return steer_lat_accel_ms2(v, angle_steers_deg, steer_ratio, wheelbase)


def curve_speed_from_lat(v_ego_ms: float, a_y: float | None,
                         a_lat: float = CURVE_COMFORT_LAT_MS2) -> float | None:
  """Comfort speed for a measured lateral accel. None when not a bend."""
  if a_y is None or v_ego_ms < CURVE_MIN_V_EGO_MS or a_lat <= 0:
    return None
  ay = float(a_y)
  if ay < 0.05:
    return None
  v = float(v_ego_ms) * math.sqrt(float(a_lat) / ay)
  if v < CURVE_SPEED_FLOOR_MS:
    return CURVE_SPEED_FLOOR_MS
  return v


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
                   active: bool, lat_override: float | None = None) -> bool:
  """Hysteresis: enter on lat accel or steer; stay until both are quiet."""
  if v_ego_ms < CURVE_MIN_V_EGO_MS:
    return False
  if lat_override is None:
    a_y = steer_lat_accel_ms2(v_ego_ms, angle_steers_deg, steer_ratio, wheelbase)
  else:
    a_y = float(lat_override)
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
    self._ay_raw: float | None = None
    self._ay_s: float | None = None
    self._begin_ran: bool = False
    self._cap_kph: float | None = None
    self._cap_age: float = 0.0
    self._release_kph: float | None = None
    self._restore_a: float = CURVE_RESTORE_A_DEFAULT_MS2
    self._last_dt: float = 0.01

  def reset(self) -> None:
    self.snapshot = None
    self.active = False
    self._exit_s = 0.0
    self._ay_raw = None
    self._ay_s = None
    self._begin_ran = False
    self._cap_kph = None
    self._cap_age = 0.0
    self._release_kph = None

  def _observe(self, v_ego_ms: float, angle_steers_deg: float,
               steer_ratio: float, wheelbase: float, dt: float, *,
               curvature: float | None, yaw_rate: float | None,
               update: bool) -> None:
    """Sample cornering once per card cycle and smooth it for the cap."""
    if not update and self._ay_raw is not None:
      return
    raw = cornering_lat_accel_ms2(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase,
      curvature=curvature, yaw_rate=yaw_rate,
    )
    self._ay_raw = raw
    self._last_dt = max(0.0, float(dt))
    if self._ay_s is None:
      self._ay_s = raw
      return
    tau = CURVE_LAT_TAU_S
    alpha = self._last_dt / (tau + self._last_dt) if tau > 0.0 else 1.0
    self._ay_s = self._ay_s + alpha * (raw - self._ay_s)

  def lat_curving(self, v_ego_ms: float, angle_steers_deg: float,
                  steer_ratio: float, wheelbase: float) -> bool:
    return is_sharp_curve(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase, active=self.active,
      lat_override=self._ay_raw,
    )

  def _held_cap_kph(self, hud_kph: float, v_ego_ms: float) -> float:
    """Cap from smoothed cornering. May drop; does not rise mid-bend."""
    v_curve = curve_speed_from_lat(v_ego_ms, self._ay_s)
    if v_curve is None:
      proposed = float(hud_kph)
    else:
      proposed = min(float(hud_kph), float(v_curve) * CV.MS_TO_KPH)
    if self._cap_kph is None:
      self._cap_kph = proposed
      self._cap_age = 0.0
    else:
      self._cap_age += self._last_dt
      if self._cap_age < CURVE_CAP_SETTLE_S:
        self._cap_kph = proposed
      elif proposed < self._cap_kph - CURVE_CAP_TIGHTEN_KPH:
        self._cap_kph = proposed
    return min(float(hud_kph), float(self._cap_kph))

  def _step_release(self, target_kph: float, dt: float) -> tuple[float, bool]:
    """Ramp HUD MAX up toward the pre-curve snapshot. Returns (kph, done)."""
    target = float(target_kph)
    if self._release_kph is None:
      start = self._cap_kph if self._cap_kph is not None else target
      self._release_kph = min(float(start), target)
    rate = self._restore_a if self._restore_a > 0.0 else CURVE_RESTORE_A_DEFAULT_MS2
    step = rate * CV.MS_TO_KPH * max(0.0, float(dt))
    nxt = min(target, float(self._release_kph) + step)
    done = nxt >= target - 0.05
    self._release_kph = target if done else nxt
    return float(self._release_kph), done

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
    curvature: float | None = None,
    yaw_rate: float | None = None,
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
    self._observe(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase, dt,
      curvature=curvature, yaw_rate=yaw_rate, update=True,
    )
    self._begin_ran = True
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
    curvature: float | None = None,
    yaw_rate: float | None = None,
    restore_a_ms2: float | None = None,
  ) -> CurveMaxDecision:
    """Apply temp curve cap or restore. Call after decide_map_cruise + overlay."""
    if not engaged or take_speed_now:
      self.reset()
      return CurveMaxDecision(float(hud_kph), None, False, False)

    if restore_a_ms2 is not None and float(restore_a_ms2) > 0.0:
      self._restore_a = float(restore_a_ms2)
    self._observe(
      v_ego_ms, angle_steers_deg, steer_ratio, wheelbase, dt,
      curvature=curvature, yaw_rate=yaw_rate, update=not self._begin_ran,
    )
    self._begin_ran = False
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
      self._release_kph = None
      capped = self._held_cap_kph(float(hud_kph), v_ego_ms)
      return CurveMaxDecision(capped, None, True, True)

    self._exit_s += float(dt)
    if not will_exit:
      capped = self._held_cap_kph(float(hud_kph), v_ego_ms)
      return CurveMaxDecision(capped, None, True, True)

    # Straight-ish long enough. Same posted zone → ramp MAX back to the
    # snapshot. A new posted that persisted through the bend is a real rebase.
    same_zone = (
      self.snapshot is not None
      and (
        self.snapshot.posted_kph is None
        or posted_kph is None
        or posted_limits_same(self.snapshot.posted_kph, posted_kph)
      )
    )
    if same_zone and self.snapshot is not None:
      target = float(self.snapshot.hud_max_kph)
      ramped, done = self._step_release(target, dt)
      if not done:
        self.protect_hold(hold)
        return CurveMaxDecision(ramped, None, False, True)
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
