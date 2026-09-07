"""Select cruise/MAX set speed from the existing driver target and an OSM limit.

This is not a second longitudinal controller. It only returns the kph value that
card.py already publishes as CS.vCruise / vCruiseCluster (HUD "MAX").
"""
from __future__ import annotations

import math

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import (
  LOOKAHEAD_NORMAL, LOOKAHEAD_OFF, LOOKAHEAD_TUNING, MIN_DECREASE_MS,
  MODE_CAP, MODE_DISPLAY, MODE_FOLLOW, MODE_OFF,
)

# Keep in sync with openpilot.selfdrive.car.cruise (avoid importing cereal here).
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255


def read_map_speed_params(params) -> tuple[int, float, int]:
  """Return (mode, offset_kph, lookahead). Unknown keys → off / 0 / normal."""
  try:
    mode = int(params.get("NAPMapSpeedMode", return_default=True) or 0)
    offset_mph = float(params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    lookahead = int(params.get("NAPMapSpeedLookahead", return_default=True) or LOOKAHEAD_NORMAL)
    if lookahead not in LOOKAHEAD_TUNING:
      lookahead = LOOKAHEAD_NORMAL
    return mode, offset_mph * CV.MPH_TO_KPH, lookahead
  except Exception:
    return MODE_OFF, 0.0, LOOKAHEAD_NORMAL


def anticipatory_limit_ms(
  current_ms: float,
  next_ms: float,
  next_dist_m: float,
  v_ego_ms: float,
  lookahead: int,
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
  a_comfort, margin_m, horizon_m = LOOKAHEAD_TUNING[lookahead]
  if a_comfort <= 0 or horizon_m <= 0:
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
) -> float | None:
  """Posted limit, optionally eased down for a closer lower limit ahead."""
  if current_ms is None or current_ms <= 0:
    return None
  anticipated = anticipatory_limit_ms(
    float(current_ms), float(next_ms or 0.0), float(next_dist_m or 0.0),
    float(v_ego_ms or 0.0), lookahead,
  )
  if anticipated is None:
    return float(current_ms)
  return min(float(current_ms), anticipated)


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
