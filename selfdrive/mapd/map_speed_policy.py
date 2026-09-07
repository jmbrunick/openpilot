"""Select cruise/MAX set speed from the existing driver target and an OSM limit.

This is not a second longitudinal controller. It only returns the kph value that
card.py already publishes as CS.vCruise / vCruiseCluster (HUD "MAX").
"""
from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import MODE_CAP, MODE_FOLLOW, MODE_OFF, MODE_DISPLAY

# Keep in sync with openpilot.selfdrive.car.cruise (avoid importing cereal here).
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255


def read_map_speed_params(params) -> tuple[int, float]:
  """Return (mode, offset_kph). Unknown keys → off / 0 (pre-rebuild params)."""
  try:
    mode = int(params.get("NAPMapSpeedMode", return_default=True) or 0)
    offset_mph = float(params.get("NAPMapSpeedOffsetMph", return_default=True) or 0)
    return mode, offset_mph * CV.MPH_TO_KPH
  except Exception:
    return MODE_OFF, 0.0


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
  """Safety net: never command the long planner above the map limit in cap/follow."""
  if mode not in (MODE_CAP, MODE_FOLLOW):
    return v_cruise_ms
  if map_limit_ms is None or map_limit_ms <= 0:
    return v_cruise_ms
  return min(v_cruise_ms, float(map_limit_ms) + float(offset_ms))
