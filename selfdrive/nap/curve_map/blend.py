"""Merge map curve points into curve follow's path (NAP, longitudinal only).

One calculation: map points are appended to the camera path as extra
virtual-lead candidates. A map point counts fully from MAP_FULL_S ahead and
fades to nothing by MAP_ZERO_S, where the camera path sets the final speed.
"""
from __future__ import annotations

MAP_FULL_S = 4.0
MAP_ZERO_S = 2.5
MAP_WEIGHT = 1.0
MAP_K_SCALE = 1.0


def _fade(t: float) -> float:
  if t >= MAP_FULL_S:
    return 1.0
  if t <= MAP_ZERO_S:
    return 0.0
  x = (t - MAP_ZERO_S) / (MAP_FULL_S - MAP_ZERO_S)
  return x * x * (3.0 - 2.0 * x)


def merge_map_points(v_ego: float, ts, ds, ks, weights, map_pts):
  """map_pts: [(distance ahead m, |kappa|)]. Returns extended (ts, ds, ks, weights)."""
  v = max(1.0, float(v_ego))
  ts2, ds2, ks2, w2 = list(ts), list(ds), list(ks), list(weights)
  for d, k in map_pts:
    d = float(d)
    if d <= 0.0:
      continue
    t = d / v
    w = MAP_WEIGHT * _fade(t)
    if w <= 0.0:
      continue
    ts2.append(t)
    ds2.append(d)
    ks2.append(abs(float(k)) * MAP_K_SCALE)
    w2.append(w)
  return ts2, ds2, ks2, w2
