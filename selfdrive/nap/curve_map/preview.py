"""Map curve preview: the road's curvature ahead from the NAP curve pack (longitudinal only).

Walks the mapped road from the car's position in its travel direction, way
to way (straightest continuation at a junction: it never invents a turn),
out to a distance horizon, and returns the walked road's curvature
(geometry.curvature) as [(distance_m, |kappa|)] every OUT_STEP_M, plus the
match quality the gate needs. Pure sqlite reads on the tile packs; runs
~1 Hz in a plannerd worker thread (worker.py), never on the control loop.
"""
from __future__ import annotations

import math
import os
import sqlite3
from dataclasses import dataclass, field

from openpilot.selfdrive.nap.curve_map.geometry import curvature
from openpilot.selfdrive.nap.curve_map.pack import EARTH_R, STEP_M, tile_key, tile_path

MATCH_M = 20.0
MATCH_HDG_DEG = 45.0
JOIN_M = 8.0
JOIN_HDG_DEG = 50.0
OUT_STEP_M = 10.0
BACK_M = 30.0  # road behind the car, so the near-field curvature has both chords
_DEG_M = 111320.0


def _hd(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def _dist(lat1, lon1, lat2, lon2) -> float:
  x = math.radians(lon2 - lon1) * EARTH_R * math.cos(math.radians(0.5 * (lat1 + lat2)))
  y = math.radians(lat2 - lat1) * EARTH_R
  return math.hypot(x, y)


@dataclass
class Preview:
  pts: list[tuple[float, float]] = field(default_factory=list)  # (distance ahead m, |kappa|)
  way_id: int = 0
  match_m: float = 1e9
  match_hdg_deg: float = 180.0
  ways: tuple[int, ...] = ()


class MapCurvePreview:
  def __init__(self, tile_dir: str) -> None:
    self.tile_dir = tile_dir
    self._cons: dict[tuple[int, int], sqlite3.Connection | None] = {}

  def _con(self, lat: float, lon: float) -> sqlite3.Connection | None:
    key = tile_key(lat, lon)
    if key not in self._cons:
      p = tile_path(self.tile_dir, key)
      self._cons[key] = sqlite3.connect(f"file:{p}?mode=ro", uri=True, check_same_thread=False) if os.path.isfile(p) else None
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
    return con.execute("SELECT lat, lon, hdg FROM pts WHERE way_id = ? ORDER BY seq", (way_id,)).fetchall()

  def _walk(self, con, way_id, seq, direction, length):
    """[(lat, lon)] along the road from (way, seq) in `direction` for `length` m, and the ways used."""
    pts = self._way(con, way_id)
    out = [pts[seq][:2]]
    used = [way_id]
    s = 0.0
    i = seq
    while s < length:
      i += direction
      if 0 <= i < len(pts):
        s += _dist(out[-1][0], out[-1][1], pts[i][0], pts[i][1])
        out.append(pts[i][:2])
        continue
      last = pts[i - direction]
      end_h = last[2] if direction == 1 else (last[2] + 180.0) % 360.0
      nxt = None
      for w2, sq2, _la, _lo, hdg2 in self._near(con, last[0], last[1], JOIN_M):
        if w2 in used:
          continue
        n2 = con.execute("SELECT max(seq) FROM pts WHERE way_id = ?", (w2,)).fetchone()[0]
        for d2, h2, ok in ((1, hdg2, sq2 == 0), (-1, (hdg2 + 180.0) % 360.0, sq2 == n2)):
          dh = _hd(h2, end_h)
          if ok and dh <= JOIN_HDG_DEG and (nxt is None or dh < nxt[0]):
            nxt = (dh, w2, sq2, d2)
      if nxt is None:
        break
      _, way_id, sq2, direction = nxt
      used.append(way_id)
      pts = self._way(con, way_id)
      i = sq2  # shared node: already in `out`
    return out, used

  def query(self, lat: float, lon: float, bearing_deg: float, horizon_m: float) -> Preview | None:
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
            best = (score, way_id, seq, direction, d, dh)
    if best is None:
      return None
    _, way_id, seq, direction, d_m, dh = best
    ahead, used = self._walk(con, way_id, seq, direction, horizon_m)
    behind, _ = self._walk(con, way_id, seq, -direction, BACK_M)
    line = behind[::-1] + ahead[1:]
    n0 = len(behind) - 1
    lat0, lon0 = line[n0]
    c = math.cos(math.radians(lat0))
    xy = [(math.radians(b - lon0) * EARTH_R * c, math.radians(a - lat0) * EARTH_R) for a, b in line]
    k = curvature(xy, STEP_M)
    out = []
    s, nxt_out = 0.0, 0.0
    for i in range(n0, len(xy)):
      if i > n0:
        s += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
      if s >= nxt_out:
        out.append((s, abs(k[i])))
        nxt_out += OUT_STEP_M
    return Preview(pts=out, way_id=int(way_id), match_m=float(d_m), match_hdg_deg=float(dh), ways=tuple(used))
