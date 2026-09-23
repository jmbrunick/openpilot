"""City vs highway Follow Distance (Mannerisms 1–7 setpoints).

Two params: NAPFollowDistanceCity (<~50 mph) and NAPFollowDistanceHwy
(>~50 mph). Migrates from the single NAPFollowDistance slider. Available
whenever Mannerisms FD is used — not gated on Hypermile.

While long is engaged and a radar lead is valid, t_follow slews between
the two setpoints (no step at 50). Dropping into city, ego is slightly
slower than the lead so the gap *opens* to city FD. Rising onto highway,
t_follow creeps toward hwy FD.

Default band is ego-speed hysteresis around 50 mph (enter hwy > 52,
return to city < 48). Highway intent: when a lead is present, long is
engaged, and MAX / set cruise speed is > 50 mph, apply highway FD once
ego > ~30 mph (enter > 30, exit < 28) — do not wait for 48/52. MAX ≤ 50
stays on city FD. If MAX drops below 48, blend back toward city FD.
t_follow slew (not a step) holds the band change.

A stalk bump does not use that live band. `follow_stalk_band_is_highway`
picks which param the tip writes: City below 30 mph, Highway above 50 mph,
and 30–50 mph follows MAX (above 50 → Highway, so a bump while still
accelerating under a highway set speed stays on Highway).
"""
from __future__ import annotations

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_CLOSE_OPENING_A_MS2,
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
# MAX / set speed > 50 mph is highway intent. Drop intent below 48 so a
# 49–51 MAX does not chatter city↔hwy. Once intent is on (lead + long
# engaged), start hwy FD from ~30 mph — not the 48/52 ego gate.
FOLLOW_HWY_INTENT_MPH = 50.0
FOLLOW_HWY_INTENT_EXIT_MPH = 48.0
FOLLOW_HWY_INTENT_EGO_MPH = 30.0
FOLLOW_HWY_INTENT_EGO_EXIT_MPH = 28.0
# Stalk-write gates. Not the 48/52 live-follow hysteresis above.
# City only below 30. Highway above 50, or in 30–50 when MAX is above 50.
STALK_CITY_BELOW_MPH = 30.0
STALK_HWY_ABOVE_MPH = 50.0
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


def parse_follow_distance(level) -> int | None:
  """1–7, or None when unset / invalid (planner then uses personality)."""
  try:
    n = int(level)
  except (TypeError, ValueError):
    return None
  return n if n in NAP_FOLLOW_DISTANCE_RANGE else None


def _get_bool(params, key) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def migrate_follow_distance_params(params) -> tuple[int | None, int | None]:
  """Copy legacy NAPFollowDistance into city/hwy once, then read both.

  Returns `(city, hwy)` in 1–7, or None when that band is invalid so the
  planner can keep personality t_follow (napFollowDistance 0). Always
  available (not Hypermile).
  """
  try:
    raw_legacy = params.get(PARAM_FOLLOW, return_default=True)
  except Exception:
    raw_legacy = None
  legacy = parse_follow_distance(raw_legacy)
  if not _get_bool(params, PARAM_FOLLOW_MIGRATED):
    seed = legacy if legacy is not None else FOLLOW_DEFAULT
    try:
      params.put(PARAM_FOLLOW_CITY, int(seed))
      params.put(PARAM_FOLLOW_HWY, int(seed))
      params.put_bool(PARAM_FOLLOW_MIGRATED, True)
    except Exception:
      pass
    return seed, seed
  try:
    raw_city = params.get(PARAM_FOLLOW_CITY, return_default=True)
  except Exception:
    raw_city = None
  try:
    raw_hwy = params.get(PARAM_FOLLOW_HWY, return_default=True)
  except Exception:
    raw_hwy = None
  city = parse_follow_distance(raw_city)
  hwy = parse_follow_distance(raw_hwy)
  if city is None:
    city = legacy
  if hwy is None:
    hwy = legacy
  return city, hwy


def follow_highway_intent(v_cruise, prev_intent=None):
  """MAX / set speed highway intent.

  True when MAX > 50 mph, False when MAX ≤ 50 (exit below 48). None when
  MAX is unknown so callers keep the ego 48/52 band. Compare in m/s so a
  50 mph set speed does not flip True via mph round-trip float.
  """
  if v_cruise is None:
    return prev_intent
  v = float(v_cruise)
  enter = FOLLOW_HWY_INTENT_MPH * CV.MPH_TO_MS
  exit_v = FOLLOW_HWY_INTENT_EXIT_MPH * CV.MPH_TO_MS
  if prev_intent is None:
    return v > enter
  if prev_intent and v < exit_v:
    return False
  if (not prev_intent) and v > enter:
    return True
  return bool(prev_intent)


def follow_ego_highway_intent(v_ego, prev_highway=None) -> bool:
  """Hwy FD once ego > ~30 mph. Exit below 28 so 29–31 does not chatter."""
  v = 0.0 if v_ego is None else float(v_ego)
  enter = FOLLOW_HWY_INTENT_EGO_MPH * CV.MPH_TO_MS
  exit_v = FOLLOW_HWY_INTENT_EGO_EXIT_MPH * CV.MPH_TO_MS
  if prev_highway is None:
    return v > enter
  if prev_highway and v < exit_v:
    return False
  if (not prev_highway) and v > enter:
    return True
  return bool(prev_highway)


def follow_stalk_band_is_highway(v_ego, v_cruise=None) -> bool:
  """True when a stalk bump should step Highway Follow Distance.

  Highway when ego is above 50 mph, or when ego is in 30–50 mph and
  MAX / set speed is above 50 mph (still coming up to a highway MAX).
  City when ego is below 30 mph, and in the 30–50 band when MAX is at
  or under 50 or unknown. A non-positive cruise reading is unknown, so
  it cannot force City at highway speed.

  Live follow (48/52, and hwy-from-~30 with a lead and MAX > 50) stays
  on `follow_band_is_highway`.
  """
  city_below = STALK_CITY_BELOW_MPH * CV.MPH_TO_MS
  hwy_above = STALK_HWY_ABOVE_MPH * CV.MPH_TO_MS
  try:
    v = 0.0 if v_ego is None else float(v_ego)
  except (TypeError, ValueError):
    v = 0.0
  if v < city_below:
    return False
  if v > hwy_above:
    return True
  try:
    if v_cruise is None:
      return False
    vmax = float(v_cruise)
  except (TypeError, ValueError):
    return False
  if vmax <= 0.0:
    return False
  return vmax > hwy_above


def published_cruise_ms(v_cruise_kph, unset_kph: float = 255.0) -> float | None:
  """HUD MAX in kph → m/s for the stalk band, or None if it is not a set speed.

  0 and the unset sentinel are not a city MAX. CarState.vCruise is 0 until
  card publishes the helper at the end of the frame.
  """
  try:
    v_kph = float(v_cruise_kph)
    unset = float(unset_kph)
  except (TypeError, ValueError):
    return None
  if not (0.0 < v_kph < unset):
    return None
  return v_kph * CV.KPH_TO_MS


def follow_band_is_highway(v_ego, prev_highway=None, *, v_cruise=None,
                          engaged=False, has_lead=False, intent=None,
                          prev_intent=None) -> bool:
  """City vs hwy Follow Distance band.

  Default: ego-speed hysteresis around 50 mph (48/52). When long is
  engaged, a lead is present, and MAX > 50 mph, apply hwy FD once ego
  > ~30 mph (28/30 hysteresis) — do not wait for 48/52. MAX ≤ 50 stays
  on city FD; unknown MAX keeps 48/52.
  """
  if intent is None:
    intent = follow_highway_intent(v_cruise, prev_intent)
  if engaged and has_lead:
    if intent is True:
      return follow_ego_highway_intent(v_ego, prev_highway)
    if intent is False:
      return False
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
    self.intent = None
    self.t_follow = None
    self.city = FOLLOW_DEFAULT
    self.hwy = FOLLOW_DEFAULT
    self.active_dist = FOLLOW_DEFAULT

  def read_setpoints(self, params):
    self.city, self.hwy = migrate_follow_distance_params(params)

  def update(self, v_ego, dt, *, engaged, has_lead, v_lead=None, v_cruise=None):
    self.intent = follow_highway_intent(v_cruise, self.intent)
    self.highway = follow_band_is_highway(
      v_ego, self.highway, v_cruise=v_cruise, engaged=engaged,
      has_lead=has_lead, intent=self.intent,
    )
    target_dist = self.hwy if self.highway else self.city
    if target_dist not in NAP_FOLLOW_DISTANCE_RANGE:
      self.active_dist = None
      return None, None, None
    self.active_dist = target_dist
    target_t = nap_t_follow(self.active_dist)
    if target_t is None:
      self.active_dist = None
      return None, None, None
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
