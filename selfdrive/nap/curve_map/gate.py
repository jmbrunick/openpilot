"""Trust gate for the map curve preview (NAP, longitudinal only).

The map only speaks when the car is demonstrably on the mapped road:
  1. the preview's own match is tight (distance, heading) and fresh,
  2. mapd's matched way (liveMapDataNAP.wayId) is the way the preview starts on,
  3. where the camera can judge (near field), the map's curvature agrees
     with the camera's. A disagreement drops trust at once; agreement
     earns it back slowly.
Trust in [0, 1] scales every map point's weight.
"""
from __future__ import annotations

MATCH_MAX_M = 12.0
MATCH_HDG_MAX_DEG = 20.0
MAPD_MATCH_MAX_M = 15.0
MAX_AGE_S = 2.5
NEAR_T = (0.8, 2.0)          # s ahead where the camera's curvature is trusted
CAM_K_MIN = 0.003            # 1/m: below this the camera says "nearly straight"
RATIO_LO, RATIO_HI = 0.3, 3.0
MAP_BEND_K = 0.006           # map says "bend" here...
CAM_STRAIGHT_K = 0.0015      # ...while the camera says straight -> mismatch
RISE_1_S = 0.5
MISMATCH_HOLD_S = 3.0


def _k_at(pts, d: float) -> float | None:
  if not pts or d < pts[0][0] or d > pts[-1][0]:
    return None
  for (d0, k0), (d1, k1) in zip(pts[:-1], pts[1:], strict=True):
    if d1 >= d:
      f = 0.0 if d1 <= d0 else (d - d0) / (d1 - d0)
      return k0 + f * (k1 - k0)
  return pts[-1][1]


def match_ok(age_s: float, match_m: float, match_hdg_deg: float, ways,
             mapd_way: int, mapd_match_m: float) -> bool:
  """ways: the preview's walked way ids, start first (mapd may still hold the previous way at a join)."""
  if age_s > MAX_AGE_S or match_m > MATCH_MAX_M or match_hdg_deg > MATCH_HDG_MAX_DEG:
    return False
  if mapd_way and (mapd_match_m > MAPD_MATCH_MAX_M or int(mapd_way) not in ways):
    return False
  return True


def near_agrees(v: float, ts, kf, map_pts) -> bool | None:
  """None when the near field has nothing to compare; else whether map and camera agree."""
  ks_c, ks_m = [], []
  for t, k in zip(ts, kf, strict=True):
    if NEAR_T[0] <= float(t) <= NEAR_T[1]:
      km = _k_at(map_pts, float(t) * v)
      if km is not None:
        ks_c.append(abs(float(k)))
        ks_m.append(km)
  if not ks_c:
    return None
  kc, km = max(ks_c), max(ks_m)
  if kc >= CAM_K_MIN:
    return RATIO_LO <= km / kc <= RATIO_HI
  if km >= MAP_BEND_K and kc < CAM_STRAIGHT_K:
    return False
  return True


class MapTrust:
  def __init__(self) -> None:
    self.trust = 0.0
    self._hold = 0.0

  def update(self, dt: float, ok_match: bool, agrees: bool | None) -> float:
    if not ok_match:
      self.trust = 0.0
      return 0.0
    if agrees is False:
      self.trust = 0.0
      self._hold = MISMATCH_HOLD_S
      return 0.0
    if self._hold > 0.0:
      self._hold -= dt
      return self.trust
    self.trust = min(1.0, self.trust + RISE_1_S * dt)
    return self.trust
