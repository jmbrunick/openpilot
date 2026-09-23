"""City vs highway Follow Distance (Mannerisms 1–7 setpoints).

Two params: NAPFollowDistanceCity and NAPFollowDistanceHwy. Migrates
from the single NAPFollowDistance slider. Available whenever Mannerisms
FD is used — not gated on Hypermile.

Band selection is MAX-primary, for both the live blend and a stalk bump:

- City when MAX / set cruise speed is under 50 mph
- Highway when MAX is 50 mph or higher (exactly 50 is Highway)
- Traveled speed only when MAX is unset, invalid, or 0

A hard brake that drops ego under 50 while MAX stays at 75 stays on
Highway follow. Ego speed does not open the city gap under a highway MAX.

While long is engaged and a radar lead is valid, t_follow still slews
between the two setpoints (no step when the band changes). Dropping into
city, ego is slightly slower than the lead so the gap *opens* to city FD.
Rising onto highway, t_follow creeps toward hwy FD.

Ego hysteresis (enter hwy above 52, return to city below 48) remains
only for that no-MAX fallback. Ego speed chatters through 49–51; a set
MAX does not, so the MAX cut is a hard 50 with no extra hold. The slew
above is what smooths a real band change — it does not pick the band.
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

# MAX / set speed cut. Highway at 50 mph and above. City under 50.
# Ego fallback (no MAX) still uses 48/52 so 49–51 mph of traveled
# speed does not flap the setpoint. A set MAX is not that noisy.
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


def follow_set_max_ms(v_cruise) -> float | None:
  """MAX in m/s, or None when it is unset, invalid, or not positive.

  0 is not a city set speed. Callers that already filtered the 255 kph
  unset sentinel (`published_cruise_ms`) pass None.
  """
  try:
    if v_cruise is None:
      return None
    v = float(v_cruise)
  except (TypeError, ValueError):
    return None
  if not (v > 0.0):
    return None
  return v


def follow_max_is_highway(v_cruise) -> bool | None:
  """Highway when MAX ≥ 50 mph, City when MAX < 50, None when no MAX.

  Exactly 50 mph is Highway. Compare in m/s so a 50 mph set does not
  miss the cut on a kph round-trip. No extra hold under 50: a 49 mph
  MAX is City even if the previous band was Highway.
  """
  v = follow_set_max_ms(v_cruise)
  if v is None:
    return None
  return v >= FOLLOW_SPLIT_MPH * CV.MPH_TO_MS


def follow_ego_band_is_highway(v_ego, prev_highway=None) -> bool:
  """Traveled-speed band. Only used when MAX is not available.

  No previous band: Highway at 50 mph and above. With a previous band,
  hold Highway down through 49 and City up through 51 (drop below 48,
  rise above 52) so ego chatter at the cut does not flap setpoints.
  """
  try:
    v = 0.0 if v_ego is None else float(v_ego)
  except (TypeError, ValueError):
    v = 0.0
  mph = v * CV.MS_TO_MPH
  if prev_highway is None:
    return mph >= FOLLOW_SPLIT_MPH
  if prev_highway and mph < FOLLOW_CITY_ENTER_MPH:
    return False
  if (not prev_highway) and mph > FOLLOW_HWY_ENTER_MPH:
    return True
  return bool(prev_highway)


def follow_stalk_band_is_highway(v_ego, v_cruise=None) -> bool:
  """True when a stalk bump should step Highway Follow Distance.

  Same MAX-primary cut as live follow: Highway when MAX ≥ 50 mph
  (including while ego is still coming up to that set speed, and
  including exactly 50). City when MAX is under 50, even if ego is
  already above 50. Traveled speed is the fallback only when MAX is
  unset, invalid, or 0 — a zero cruise reading must not force City
  once ego is on the highway side of the fallback cut.
  """
  band = follow_max_is_highway(v_cruise)
  if band is not None:
    return band
  return follow_ego_band_is_highway(v_ego, prev_highway=None)


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

  MAX-primary: Highway when set speed ≥ 50 mph, City when it is under
  50, whether or not a lead is present and whether or not long is
  engaged. Ego speed is not part of that choice — decelerating under a
  highway MAX stays Highway. `engaged` / `has_lead` / `intent` are
  accepted so older callers keep working; they do not pick the band.
  The t_follow slew in `FollowDistanceBlend` still runs only while long
  is engaged with a lead.

  No MAX (unset / invalid / 0): ego fallback, with 48/52 hysteresis
  when a previous band is known.
  """
  _ = (engaged, has_lead, intent, prev_intent)
  band = follow_max_is_highway(v_cruise)
  if band is not None:
    return band
  return follow_ego_band_is_highway(v_ego, prev_highway)


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

  def update(self, v_ego, dt, *, engaged, has_lead, v_lead=None, v_cruise=None):
    self.highway = follow_band_is_highway(v_ego, self.highway, v_cruise=v_cruise)
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
