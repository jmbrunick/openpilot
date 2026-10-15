"""Merge map curve points into curve follow's path (NAP, longitudinal only).

One calculation: map points are appended to the camera path as extra
virtual-lead candidates. A map point counts fully from MAP_FULL_S ahead and
fades to nothing by MAP_ZERO_S, where the camera path sets the final speed.
Every map weight is also scaled by the trust gate (gate.py).
"""
from __future__ import annotations

from openpilot.selfdrive.nap.curve_map.gate import MapTrust, match_ok, near_agrees

MAP_FULL_S = 4.0
MAP_ZERO_S = 2.5
MAP_K_SCALE = 1.0


def _fade(t: float) -> float:
  if t >= MAP_FULL_S:
    return 1.0
  if t <= MAP_ZERO_S:
    return 0.0
  x = (t - MAP_ZERO_S) / (MAP_FULL_S - MAP_ZERO_S)
  return x * x * (3.0 - 2.0 * x)


def merge_map_points(v_ego: float, ts, ds, ks, weights, map_pts, trust: float = 1.0):
  """map_pts: [(distance ahead m, |kappa|)]. Returns extended (ts, ds, ks, weights)."""
  v = max(1.0, float(v_ego))
  ts2, ds2, ks2, w2 = list(ts), list(ds), list(ks), list(weights)
  if trust <= 0.0:
    return ts2, ds2, ks2, w2
  for d, k in map_pts:
    d = float(d)
    if d <= 0.0:
      continue
    t = d / v
    w = float(trust) * _fade(t)
    if w <= 0.0:
      continue
    ts2.append(t)
    ds2.append(d)
    ks2.append(abs(float(k)) * MAP_K_SCALE)
    w2.append(w)
  return ts2, ds2, ks2, w2


class MapFeed:
  """The latest preview, moved along by the distance driven since it was taken."""

  def __init__(self) -> None:
    self.gate = MapTrust()
    self.clear()

  def clear(self) -> None:
    self.pts: list[tuple[float, float]] = []
    self.ways: tuple[int, ...] = ()
    self.match_m = 1e9
    self.match_hdg = 180.0
    self.age = 1e9
    self.travel = 0.0
    self.trust = 0.0

  def set(self, preview) -> None:
    """preview: preview.Preview or None (no tile / no match)."""
    if preview is None:
      self.clear()
      return
    self.pts = list(preview.pts)
    self.ways = tuple(preview.ways)
    self.match_m, self.match_hdg = preview.match_m, preview.match_hdg_deg
    self.age, self.travel = 0.0, 0.0

  def merge(self, dt: float, v_ego: float, ts, ds, kf, weights, mapd_way: int = 0, mapd_match_m: float = 0.0):
    pts = [(d - self.travel, k) for d, k in self.pts if d - self.travel > -30.0]
    ok = bool(pts) and match_ok(self.age, self.match_m, self.match_hdg, self.ways, mapd_way, mapd_match_m)
    agrees = near_agrees(float(v_ego), ts, kf, pts) if ok else None
    self.trust = self.gate.update(dt, ok, agrees)
    self.age += dt
    self.travel += max(0.0, float(v_ego)) * dt
    return merge_map_points(v_ego, ts, ds, kf, weights, pts, self.trust)
