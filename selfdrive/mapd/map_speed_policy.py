"""Select cruise/MAX set speed from the existing driver target and an OSM limit.

This is not a second longitudinal controller. It only returns the kph value that
card.py already publishes as CS.vCruise / vCruiseCluster (HUD "MAX").
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  ACCEL_DEFAULT, ACCEL_MAX, ACCEL_MIN, DRIVER_OVERRIDE_S, LOOKAHEAD_NORMAL, LOOKAHEAD_OFF,
  LOOKAHEAD_TUNING, MANUAL_SET_EPS_KPH, MIN_DECREASE_MS, MODE_CAP, MODE_DISPLAY,
  MODE_FOLLOW, MODE_OFF, POSTED_LIMIT_EPS_KPH, TRACK_DEADBAND_MS, TRACK_TAPER_MS,
  map_accel_a_ms2, map_brake_a_ms2,
)

# Keep in sync with openpilot.selfdrive.car.cruise (avoid importing cereal here).
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255


def _clamp_accel_level(level: int) -> int:
  return max(ACCEL_MIN, min(ACCEL_MAX, int(level)))


def read_map_speed_params(params) -> tuple[int, float, int, int]:
  """Return (mode, offset_kph, lookahead, accel). Unknown keys → off / 0 / normal / 5."""
  try:
    mode = int(params.get("NAPMapSpeedMode", return_default=True) or 0)
    offset_mph = float(params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    lookahead = int(params.get("NAPMapSpeedLookahead", return_default=True) or LOOKAHEAD_NORMAL)
    if lookahead not in LOOKAHEAD_TUNING:
      lookahead = LOOKAHEAD_NORMAL
    accel = _clamp_accel_level(int(params.get("NAPMapSpeedAccel", return_default=True) or ACCEL_DEFAULT))
    return mode, offset_mph * CV.MPH_TO_KPH, lookahead, accel
  except Exception:
    return MODE_OFF, 0.0, LOOKAHEAD_NORMAL, ACCEL_DEFAULT


def anticipatory_limit_ms(
  current_ms: float,
  next_ms: float,
  next_dist_m: float,
  v_ego_ms: float,
  lookahead: int,
  accel: int = ACCEL_DEFAULT,
) -> float | None:
  """Decrease-only cruise ceiling for an upcoming lower limit, or None.

  Uses comfort decel + margin so MAX eases down and the car is near the new
  limit as GPS enters that way. A higher limit ahead never raises the ceiling;
  Follow still raises when the current match is the faster way.
  """
  if lookahead <= LOOKAHEAD_OFF or lookahead not in LOOKAHEAD_TUNING:
    return None
  if current_ms <= 0 or next_ms <= 0 or next_dist_m <= 0:
    return None
  if next_ms >= current_ms - MIN_DECREASE_MS:
    return None
  _a_base, margin_m, horizon_m = LOOKAHEAD_TUNING[lookahead]
  if horizon_m <= 0:
    return None
  # Brake / anticipatory decreases are locked at Accel 5. `accel` only
  # scales Follow speed-up (MAX rising); keep the arg for call-site compat.
  del accel
  a_comfort = map_brake_a_ms2(lookahead)
  if a_comfort <= 0:
    return None
  v0 = max(float(v_ego_ms), float(current_ms), 0.0)
  vt = float(next_ms)
  if v0 <= vt:
    return None
  need_m = (v0 * v0 - vt * vt) / (2.0 * a_comfort) + margin_m
  need_m = min(need_m, horizon_m)
  if next_dist_m > need_m:
    return None
  # Smooth profile: v² = vt² + 2 a d  so MAX falls as the sign approaches.
  v_cmd = math.sqrt(max(0.0, vt * vt + 2.0 * a_comfort * float(next_dist_m)))
  return max(vt, min(float(current_ms), v_cmd))


def effective_map_limit_ms(
  current_ms: float | None,
  next_ms: float = 0.0,
  next_dist_m: float = 0.0,
  v_ego_ms: float = 0.0,
  lookahead: int = LOOKAHEAD_NORMAL,
  accel: int = ACCEL_DEFAULT,
) -> float | None:
  """Posted limit, optionally eased down for a closer lower limit ahead."""
  if current_ms is None or current_ms <= 0:
    return None
  anticipated = anticipatory_limit_ms(
    float(current_ms), float(next_ms or 0.0), float(next_dist_m or 0.0),
    float(v_ego_ms or 0.0), lookahead, accel,
  )
  if anticipated is None:
    return float(current_ms)
  return min(float(current_ms), anticipated)


def map_slew_a_ms2(prev_ms: float, target_ms: float, lookahead: int, accel_level: int) -> float:
  """Slew rate for HUD/planner MAX: brake-locked a on decreases, Accel 1–10 on rises."""
  if float(target_ms) < float(prev_ms):
    return map_brake_a_ms2(lookahead)
  return map_accel_a_ms2(lookahead, accel_level)


def slew_map_speed_ms(prev_ms: float, target_ms: float, dt: float, a_ms2: float) -> float:
  """Rate-limit map-driven MAX (m/s) so a new limit does not cliff the HUD."""
  if dt <= 0 or a_ms2 <= 0:
    return float(target_ms)
  max_dv = float(a_ms2) * float(dt)
  delta = float(target_ms) - float(prev_ms)
  if abs(delta) <= max_dv:
    return float(target_ms)
  return float(prev_ms) + math.copysign(max_dv, delta)


def map_track_decel_ms2(v_ego_ms: float, v_cruise_ms: float, a_comfort: float) -> float | None:
  """Comfort decel (negative m/s²) to track a lower map MAX, or None.

  LongitudinalMpc's cruise column is a virtual lead at
  get_safe_obstacle_distance(v_ego) with V_EGO_COST=0. Holding 70 mph for
  10 s after MAX drops to 45 never violates that obstacle, so aTarget stays
  ~0 unless we command this. Lead/MPC may still request more braking via min().
  Callers must pass Accel-5 comfort a (map_brake_a_ms2); Accel 1–10 does not
  change brake rate.
  """
  if a_comfort <= 0 or v_ego_ms <= 0 or v_cruise_ms <= 0:
    return None
  dv = float(v_ego_ms) - float(v_cruise_ms)
  if dv <= TRACK_DEADBAND_MS:
    return None
  span = max(1e-6, TRACK_TAPER_MS - TRACK_DEADBAND_MS)
  scale = min(1.0, (dv - TRACK_DEADBAND_MS) / span)
  return -float(a_comfort) * scale


def map_track_accel_ms2(v_ego_ms: float, v_cruise_ms: float, a_comfort: float) -> float | None:
  """Comfort accel cap (positive m/s²) when catching a higher Follow MAX, or None.

  min() with MPC so Accel 1–10 limits how hard we climb; a slower lead can
  still command negative a.
  """
  if a_comfort <= 0 or v_ego_ms <= 0 or v_cruise_ms <= 0:
    return None
  dv = float(v_cruise_ms) - float(v_ego_ms)
  if dv <= TRACK_DEADBAND_MS:
    return None
  span = max(1e-6, TRACK_TAPER_MS - TRACK_DEADBAND_MS)
  scale = min(1.0, (dv - TRACK_DEADBAND_MS) / span)
  return float(a_comfort) * scale


def posted_limits_same(a_kph: float | None, b_kph: float | None) -> bool:
  if a_kph is None or b_kph is None:
    return False
  return abs(float(a_kph) - float(b_kph)) < POSTED_LIMIT_EPS_KPH


def is_manual_set_change(prev_kph: float, cur_kph: float) -> bool:
  """True when set speed moved (not engage 0↔set). Includes ego jumps."""
  if prev_kph in (0, V_CRUISE_UNSET) or cur_kph <= 0 or cur_kph >= V_CRUISE_UNSET:
    return False
  return abs(float(cur_kph) - float(prev_kph)) > MANUAL_SET_EPS_KPH


def is_cruise_stalk_step(prev_kph: float, cur_kph: float) -> bool:
  """True if delta matches Tesla pedal stalk +/- (1 or 5 mph / kph).

  An ego / DI_digitalSpeed jump is not a stalk step and must not arm
  sticky or the Follow 10s timer.
  """
  if not is_manual_set_change(prev_kph, cur_kph):
    return False
  delta = abs(float(cur_kph) - float(prev_kph))
  steps = (1.0, 5.0, CV.MPH_TO_KPH, 5.0 * CV.MPH_TO_KPH)
  return any(abs(delta - step) < 0.55 for step in steps)


def _ref_posted_kph(hold: MapCruiseHold, posted_kph: float | None) -> float | None:
  if posted_kph is not None and posted_kph > 0:
    return float(posted_kph)
  if hold.last_posted_kph is not None and hold.last_posted_kph > 0:
    return float(hold.last_posted_kph)
  return None


def is_below_posted(set_kph: float, posted_kph: float | None) -> bool:
  if posted_kph is None or posted_kph <= 0:
    return False
  return float(set_kph) < float(posted_kph) - MANUAL_SET_EPS_KPH


def is_above_posted(set_kph: float, posted_kph: float | None) -> bool:
  if posted_kph is None or posted_kph <= 0:
    return False
  return float(set_kph) > float(posted_kph) + MANUAL_SET_EPS_KPH


@dataclass
class MapCruiseHold:
  """Engage-seed + sticky manual set while the posted OSM limit is unchanged.

  Below-limit sticky and the Follow raise-above 10s timer are separate:
  `follow_override_until` must never clear or replace a below-limit hold.
  """
  last_posted_kph: float | None = None
  last_raw_kph: float = V_CRUISE_UNSET
  policy_kph: float | None = None
  sticky_set_kph: float | None = None
  follow_override_until: float = 0.0

  def reset(self) -> None:
    self.last_posted_kph = None
    self.last_raw_kph = V_CRUISE_UNSET
    self.policy_kph = None
    self.sticky_set_kph = None
    self.follow_override_until = 0.0

  def clear_sticky(self) -> None:
    self.sticky_set_kph = None


@dataclass
class MapCruiseDecision:
  driver_kph: float
  follow_override: bool
  seed_kph: float | None
  sticky: bool


def _sticky_decision(hold: MapCruiseHold, mode: int, posted_kph: float | None) -> MapCruiseDecision:
  held = float(hold.sticky_set_kph)
  if mode == MODE_CAP and posted_kph is not None and posted_kph > 0:
    held = min(held, float(posted_kph))
  hold.policy_kph = held
  # follow_override True so apply_map_speed cannot Follow-raise if a caller
  # ignores `sticky` and only passes the override flag.
  return MapCruiseDecision(held, True, held, True)


def decide_map_cruise(
  hold: MapCruiseHold,
  *,
  engaged: bool,
  mode: int,
  raw_kph: float,
  posted_kph: float | None,
  engage_rising: bool,
  now: float,
  stalk_pressed: bool | None = None,
) -> MapCruiseDecision:
  """Engage seed + sticky hold. posted_kph is OSM current maxspeed + offset.

  `engaged` must be pedal-long active (not lateral-only). `stalk_pressed`
  is extra (button edge). A 1/5 mph pedal_speed step always counts as a
  stalk; an ego jump does not. Do not treat `stalk_pressed=False` as
  "ignore pedal_speed" — pre-AP button events are not reliable.

  Returns the driver-set to overlay, whether Follow should hold that set,
  and an optional pedal_speed write-back for seed / sticky only.
  """
  if (not engaged) or mode not in (MODE_CAP, MODE_FOLLOW):
    hold.reset()
    return MapCruiseDecision(raw_kph, False, None, False)

  posted_ok = posted_kph is not None and posted_kph > 0
  # Button OR a real stalk-sized pedal step. Never OR-in an ego jump.
  manual = is_cruise_stalk_step(hold.last_raw_kph, raw_kph) or bool(stalk_pressed)
  hold.last_raw_kph = raw_kph

  if engage_rising and posted_ok:
    hold.sticky_set_kph = None
    hold.follow_override_until = 0.0
    hold.last_posted_kph = posted_kph
    hold.last_raw_kph = float(posted_kph)
    hold.policy_kph = float(posted_kph)
    return MapCruiseDecision(float(posted_kph), False, float(posted_kph), False)

  if posted_ok and hold.last_posted_kph is not None and not posted_limits_same(hold.last_posted_kph, posted_kph):
    # New posted limit b: drop a-5 hold and resume Cap/Follow at b.
    hold.sticky_set_kph = None
    hold.follow_override_until = 0.0
    hold.last_posted_kph = posted_kph
    hold.last_raw_kph = float(posted_kph)
    hold.policy_kph = float(posted_kph)
    return MapCruiseDecision(float(posted_kph), False, float(posted_kph), False)

  if posted_ok and hold.last_posted_kph is None:
    hold.last_posted_kph = posted_kph
    if hold.sticky_set_kph is not None and not is_below_posted(hold.sticky_set_kph, posted_kph):
      hold.sticky_set_kph = None
      hold.follow_override_until = 0.0
      hold.last_raw_kph = float(posted_kph)
      hold.policy_kph = float(posted_kph)
      return MapCruiseDecision(float(posted_kph), False, float(posted_kph), False)

  if posted_ok:
    hold.last_posted_kph = posted_kph

  ref_posted = _ref_posted_kph(hold, posted_kph)

  if manual:
    hold.policy_kph = float(raw_kph)
    if is_below_posted(raw_kph, ref_posted) or ref_posted is None:
      # Sticky below-limit (or no sign yet): never arm DRIVER_OVERRIDE_S.
      # Timeout must not Follow-raise back to `a` after a stalk to a-5.
      hold.sticky_set_kph = float(raw_kph)
      hold.follow_override_until = 0.0
    else:
      hold.sticky_set_kph = None
      if mode == MODE_FOLLOW and is_above_posted(raw_kph, ref_posted):
        hold.follow_override_until = now + DRIVER_OVERRIDE_S
      else:
        hold.follow_override_until = 0.0

  if hold.policy_kph is None:
    hold.policy_kph = float(raw_kph)

  # Promote a below-limit set off the 10s timer if anything armed it.
  if hold.sticky_set_kph is None and is_below_posted(float(hold.policy_kph), ref_posted):
    if hold.follow_override_until > 0.0 or manual:
      hold.sticky_set_kph = float(hold.policy_kph)
      hold.follow_override_until = 0.0

  if hold.sticky_set_kph is not None:
    return _sticky_decision(hold, mode, posted_kph)

  override = mode == MODE_FOLLOW and now < hold.follow_override_until
  if override and is_below_posted(float(hold.policy_kph), ref_posted):
    hold.sticky_set_kph = float(hold.policy_kph)
    hold.follow_override_until = 0.0
    return _sticky_decision(hold, mode, posted_kph)

  return MapCruiseDecision(float(hold.policy_kph), override, None, False)


def apply_map_speed_kph(
  driver_set_kph: float,
  map_limit_kph: float | None,
  *,
  mode: int,
  offset_kph: float = 0.0,
  engaged: bool = False,
  op_long_software_cruise: bool = False,
  driver_override: bool = False,
  v_min: float = V_CRUISE_MIN,
  v_max: float = V_CRUISE_MAX,
) -> float:
  """Return the kph target for vCruise / HUD MAX.

  Control (cap/follow) only runs when NAP owns software cruise — pre-AP pedal
  mode (`openpilotLongitudinalControl` and not `pcmCruise`). Display and off
  never change the driver/stock target.
  """
  if driver_set_kph <= 0 or driver_set_kph >= V_CRUISE_UNSET:
    return driver_set_kph

  driver = float(driver_set_kph)
  if mode in (MODE_OFF, MODE_DISPLAY) or not engaged:
    return driver
  if not op_long_software_cruise:
    return driver
  if map_limit_kph is None or map_limit_kph <= 0:
    return driver

  map_target = max(v_min, min(v_max, float(map_limit_kph) + float(offset_kph)))

  if mode == MODE_CAP:
    return min(driver, map_target)

  if mode == MODE_FOLLOW:
    # driver_override is the raise-above-limit 10s hold only. Below-limit
    # sticky is applied by decide_map_cruise before this function runs.
    if driver_override:
      return max(v_min, min(v_max, driver))
    return map_target

  return driver


def cap_planner_v_cruise_ms(
  v_cruise_ms: float,
  map_limit_ms: float | None,
  *,
  mode: int,
  offset_ms: float = 0.0,
) -> float:
  """Ceiling for planner v_cruise only.

  Does not touch radarState. LongitudinalMpc.update still stacks
  [lead0, lead1, cruise_obstacle(v_cruise)] and constrains to min(...),
  so a slower lead still commands below this ceiling.
  """
  if mode not in (MODE_CAP, MODE_FOLLOW):
    return v_cruise_ms
  if map_limit_ms is None or map_limit_ms <= 0:
    return v_cruise_ms
  return min(v_cruise_ms, float(map_limit_ms) + float(offset_ms))


# Same column order as LongitudinalMpc.update:
# x_obstacles = [lead0, lead1, cruise_obstacle(v_cruise)]
SOURCE_LEAD0 = 0
SOURCE_LEAD1 = 1
SOURCE_CRUISE = 2


def longitudinal_obstacle_source(lead0_m: float, lead1_m: float, cruise_m: float) -> int:
  """Return which MPC obstacle is tightest at t=0 (0=lead0, 1=lead1, 2=cruise).

  Map speed only changes the cruise column. A closer/slower lead must still win.
  """
  stacked = (float(lead0_m), float(lead1_m), float(cruise_m))
  return min(range(3), key=lambda i: stacked[i])
