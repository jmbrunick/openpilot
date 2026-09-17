"""City vs highway Follow Distance (Mannerisms 1–7 setpoints).

Two params: NAPFollowDistanceCity (<~50 mph) and NAPFollowDistanceHwy
(>~50 mph). Migrates from the single NAPFollowDistance slider. Available
whenever Mannerisms FD is used — not gated on Hypermile.

While long is engaged and a radar lead is valid, t_follow slews between
the two setpoints (no step at 50). Dropping into city, ego is slightly
slower than the lead so the gap *opens* to city FD. Rising onto highway,
t_follow creeps toward hwy FD. Hysteresis around 50 mph.
"""
from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_CLOSE_OPENING_A_MS2,
  NAP_T_FOLLOW,
  nap_t_follow,
)

PARAM_FOLLOW = "NAPFollowDistance"
PARAM_FOLLOW_CITY = "NAPFollowDistanceCity"
PARAM_FOLLOW_HWY = "NAPFollowDistanceHwy"
PARAM_FOLLOW_MIGRATED = "NAPFollowDistanceSplitMigrated"

FOLLOW_MIN = 1
FOLLOW_MAX = 7
FOLLOW_DEFAULT = 4
NAP_FOLLOW_DISTANCE_RANGE = range(FOLLOW_MIN, FOLLOW_MAX + 1)

# ~50 mph split with hysteresis so chatter at 49–51 does not flip setpoints.
FOLLOW_SPLIT_MPH = 50.0
FOLLOW_CITY_ENTER_MPH = 48.0
FOLLOW_HWY_ENTER_MPH = 52.0
# While opening to a farther city FD: ~3% slower than lead (small % more slowing).
FOLLOW_OPEN_SLOW_FRAC = 0.03
# t_follow slew (seconds of headway per second). Opening is a bit faster
# than highway creep so the city gap actually grows; creep stays lazy.
FOLLOW_T_SLEW_OPEN_PER_S = 0.08
FOLLOW_T_SLEW_CREEP_PER_S = 0.04


def clamp_follow_distance(level: int | None) -> int:
  try:
    return max(FOLLOW_MIN, min(FOLLOW_MAX, int(level)))
  except (TypeError, ValueError):
    return FOLLOW_DEFAULT


def _get_int(params, key, default=FOLLOW_DEFAULT) -> int:
  try:
    raw = params.get(key, return_default=True)
  except Exception:
    return default
  if raw is None or raw == "":
    return default
  return clamp_follow_distance(raw)


def _get_bool(params, key) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def migrate_follow_distance_params(params) -> tuple[int, int]:
  """Copy legacy NAPFollowDistance into city/hwy once, then read both.

  Returns `(city, hwy)` in 1–7. Always available (not Hypermile).
  """
  legacy = _get_int(params, PARAM_FOLLOW)
  if not _get_bool(params, PARAM_FOLLOW_MIGRATED):
    try:
      params.put(PARAM_FOLLOW_CITY, int(legacy))
      params.put(PARAM_FOLLOW_HWY, int(legacy))
      params.put_bool(PARAM_FOLLOW_MIGRATED, True)
    except Exception:
      pass
    return legacy, legacy
  city = _get_int(params, PARAM_FOLLOW_CITY, default=legacy)
  hwy = _get_int(params, PARAM_FOLLOW_HWY, default=legacy)
  return city, hwy


def follow_band_is_highway(v_ego, prev_highway=None) -> bool:
  """Hysteresis around 50 mph. `prev_highway` None seeds at the 50 split."""
  mph = float(v_ego) * CV.MS_TO_MPH if v_ego is not None else FOLLOW_SPLIT_MPH
  if prev_highway is None:
    return mph >= FOLLOW_SPLIT_MPH
  if prev_highway and mph < FOLLOW_CITY_ENTER_MPH:
    return False
  if (not prev_highway) and mph > FOLLOW_HWY_ENTER_MPH:
    return True
  return bool(prev_highway)


def follow_open_a_ms2(v_ego, v_lead):
  """Slightly slower than lead while the city gap is still opening, or None."""
  if v_ego is None or v_lead is None:
    return None
  vt = max(0.0, float(v_lead))
  if vt <= 0.0:
    return None
  v_des = vt * (1.0 - FOLLOW_OPEN_SLOW_FRAC)
  dv = float(v_ego) - v_des
  if dv <= 0.0:
    return None
  return -min(LEAD_CLOSE_OPENING_A_MS2, dv * 0.20)


class FollowDistanceBlend:
  """Slew t_follow between city and hwy setpoints; extra −a while opening."""

  def __init__(self):
    self.highway = None
    self.t_follow = None
    self.city = FOLLOW_DEFAULT
    self.hwy = FOLLOW_DEFAULT
    self.active_dist = FOLLOW_DEFAULT

  def read_setpoints(self, params):
    self.city, self.hwy = migrate_follow_distance_params(params)

  def update(self, v_ego, dt, *, engaged, has_lead, v_lead=None):
    self.highway = follow_band_is_highway(v_ego, self.highway)
    target_dist = self.hwy if self.highway else self.city
    self.active_dist = target_dist if target_dist in NAP_FOLLOW_DISTANCE_RANGE else FOLLOW_DEFAULT
    target_t = nap_t_follow(self.active_dist)
    if target_t is None:
      target_t = NAP_T_FOLLOW[FOLLOW_DEFAULT - 1]
    if self.t_follow is None:
      self.t_follow = float(target_t)

    opening = False
    if engaged and has_lead:
      if target_t > self.t_follow + 1e-6:
        step = FOLLOW_T_SLEW_OPEN_PER_S * max(0.0, float(dt))
        self.t_follow = min(float(target_t), self.t_follow + step)
        opening = self.t_follow < float(target_t) - 1e-4
      elif target_t < self.t_follow - 1e-6:
        step = FOLLOW_T_SLEW_CREEP_PER_S * max(0.0, float(dt))
        self.t_follow = max(float(target_t), self.t_follow - step)
      else:
        self.t_follow = float(target_t)
    else:
      self.t_follow = float(target_t)

    extra_a = follow_open_a_ms2(v_ego, v_lead) if opening else None
    return self.t_follow, self.active_dist, extra_a
