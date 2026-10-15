"""Map curve preview: the road's curvature ahead from the NAP curve pack (longitudinal only).

Walks the mapped road from the car's position in its travel direction, way
to way (straightest continuation at a junction: it never invents a turn),
out to a distance horizon, and returns [(distance_m, |kappa|)] every ~10 m.
Pure sqlite reads on the tile packs; meant to run ~1 Hz off the control
loop (see worker.py).
"""
from __future__ import annotations

import math
import os
import sqlite3

from openpilot.selfdrive.nap.curve_map.pack import EARTH_R, SMOOTH_M, tile_key, tile_path

MATCH_M = 20.0
MATCH_HDG_DEG = 45.0
JOIN_M = 15.0
JOIN_HDG_DEG = 50.0
_DEG_M = 111320.0


def _hd(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def _dist(lat1, lon1, lat2, lon2) -> float:
  x = math.radians(lon2 - lon1) * EARTH_R * math.cos(math.radians(0.5 * (lat1 + lat2)))
  y = math.radians(lat2 - lat1) * EARTH_R
  return math.hypot(x, y)


class MapCurvePreview:
  def __init__(self, tile_dir: str) -> None:
    self.tile_dir = tile_dir
    self._cons: dict[tuple[int, int], sqlite3.Connection | None] = {}
    self._last: tuple[sqlite3.Connection, int, int, int] | None = None  # con, way, seq, dir

  def _con(self, lat: float, lon: float) -> sqlite3.Connection | None:
    key = tile_key(lat, lon)
    if key not in self._cons:
      p = tile_path(self.tile_dir, key)
      if not os.path.isfile(p):
        return None
      self._cons[key] = sqlite3.connect(f"file:{p}?mode=ro", uri=True, check_same_thread=False)
    return self._cons[key]

  def has_tile(self, lat: float, lon: float) -> bool:
    return self._con(lat, lon) is not None

  @staticmethod
  def _near(con, lat, lon, r_m):
    dl = r_m / _DEG_M
    dn = r_m / (_DEG_M * max(0.2, math.cos(math.radians(lat))))
    return con.execute(
      "SELECT pts.way_id, pts.seq, pts.lat, pts.lon, pts.hdg FROM pts_rtree JOIN pts ON pts.id = pts_rtree.id " +
      "WHERE pts_rtree.min_lat >= ? AND pts_rtree.max_lat <= ? AND pts_rtree.min_lon >= ? AND pts_rtree.max_lon <= ?",
      (lat - dl, lat + dl, lon - dn, lon + dn)).fetchall()

  @staticmethod
  def _way(con, way_id):
    return con.execute("SELECT lat, lon, hdg, k FROM pts WHERE way_id = ? ORDER BY seq", (way_id,)).fetchall()

  def query(self, lat: float, lon: float, bearing_deg: float, horizon_m: float) -> list[tuple[float, float]] | None:
    """[(distance ahead m, |kappa|)] or None when the road cannot be matched."""
    con = self._con(lat, lon)
    if con is None:
      return None
    best = None
    for way_id, seq, plat, plon, hdg in self._near(con, lat, lon, MATCH_M):
      d = _dist(lat, lon, plat, plon)
      if d > MATCH_M:
        continue
      for direction, h in ((1, hdg), (-1, (hdg + 180.0) % 360.0)):
        dh = _hd(h, bearing_deg)
        if dh <= MATCH_HDG_DEG:
          score = d + 0.3 * dh
          if best is None or score < best[0]:
            best = (score, way_id, seq, direction)
    if best is None:
      return None
    _, way_id, seq, direction = best
    out: list[tuple[float, float]] = []
    pts = self._way(con, way_id)
    i = seq
    s = 0.0
    plat, plon = lat, lon
    seen = {way_id}
    pending_join, join_s = 0.0, 0.0
    while s < horizon_m:
      i += direction
      if 0 <= i < len(pts):
        la, lo, _, k = pts[i]
        s += _dist(plat, plon, la, lo)
        plat, plon = la, lo
        kk = abs(float(k))
        if pending_join > 0.0 and s - join_s <= 0.5 * SMOOTH_M:
          kk += pending_join
        out.append((s, kk))
        continue
      # end of way: straightest continuation
      end_h = pts[i - direction][2] if direction == 1 else (pts[i - direction][2] + 180.0) % 360.0
      nxt = None
      for w2, sq2, _la, _lo, hdg2 in self._near(con, plat, plon, JOIN_M):
        if w2 in seen:
          continue
        n2 = con.execute("SELECT max(seq) FROM pts WHERE way_id = ?", (w2,)).fetchone()[0]
        for d2, h2, ok in ((1, hdg2, sq2 == 0), (-1, (hdg2 + 180.0) % 360.0, sq2 == n2)):
          if not ok:
            continue
          dh = _hd(h2, end_h)
          if dh <= JOIN_HDG_DEG and (nxt is None or dh < nxt[0]):
            nxt = (dh, w2, sq2, d2)
      if nxt is None:
        break
      dh, way_id, sq2, direction = nxt
      seen.add(way_id)
      pts = self._way(con, way_id)
      i = sq2 - direction  # the loop steps onto sq2 first
      # the turn at the shared node belongs to neither way: spread it over SMOOTH_M
      join_k = math.radians(dh) / SMOOTH_M
      if out and join_k > 0.0:
        out[-1] = (out[-1][0], out[-1][1] + join_k)
      pending_join, join_s = join_k, s
      continue
    return out
