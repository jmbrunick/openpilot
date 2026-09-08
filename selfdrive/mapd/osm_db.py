"""Offline OSM speed-limit SQLite (R-tree) for comma 3X.

Schema is NAP-owned. Data is OpenStreetMap (ODbL). Query path is GPS → nearest
heading-aligned way with an explicit maxspeed tag. Ways with maxspeed_ms=0 are
junction geometry only (cross streets with no posted tag) and never LIMIT.
"""
from __future__ import annotations

import math
import os
import sqlite3
import struct
from dataclasses import dataclass

from openpilot.selfdrive.mapd.constants import (
  HEADING_ALIGN_DEG,
  INTERSECTION_LOOKAHEAD_M,
  JUNCTION_CLUSTER_M,
  JUNCTION_RADIUS_M,
  LOOKAHEAD_M,
  LOOKAHEAD_MAX_M,
  MAX_MATCH_DISTANCE_M,
  SEARCH_PAD_DEG,
  TURN_MAX_DEG,
  TURN_MIN_DEG,
  osm_sign_lead_m,
)

EARTH_R = 6371000.0
_COORDS_HDR = struct.Struct("<I")
_COORD_F64 = struct.Struct("<dd")
_COORD_F32 = struct.Struct("<ff")
# Drop vertices closer than this when inserting (keeps US extract under a Release asset).
SIMPLIFY_TOL_M = 12.0


@dataclass(frozen=True)
class SpeedLimitMatch:
  speed_limit_ms: float
  way_id: int
  road_name: str
  highway: str
  distance_m: float
  next_speed_limit_ms: float = 0.0
  next_distance_m: float = 0.0
  coords: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class IntersectionAhead:
  """Nearest turn junction along the matched OSM way. Distances are meters."""
  distance_m: float
  has_left: bool
  has_right: bool
  left_speed_ms: float = 0.0
  right_speed_ms: float = 0.0


def _pack_coords(coords: list[tuple[float, float]]) -> bytes:
  # float32 is ~1 m at US longitudes — enough for MAX_MATCH_DISTANCE_M.
  buf = bytearray(_COORDS_HDR.pack(len(coords)))
  for lat, lon in coords:
    buf += _COORD_F32.pack(float(lat), float(lon))
  return bytes(buf)


def _unpack_coords(blob: bytes) -> list[tuple[float, float]]:
  n = _COORDS_HDR.unpack_from(blob)[0]
  rest = len(blob) - _COORDS_HDR.size
  if rest == n * _COORD_F32.size:
    fmt, step = _COORD_F32, _COORD_F32.size
  elif rest == n * _COORD_F64.size:
    fmt, step = _COORD_F64, _COORD_F64.size
  else:
    return []
  coords = []
  off = _COORDS_HDR.size
  for _ in range(n):
    lat, lon = fmt.unpack_from(blob, off)
    coords.append((float(lat), float(lon)))
    off += step
  return coords


def simplify_coords(coords: list[tuple[float, float]], tol_m: float = SIMPLIFY_TOL_M) -> list[tuple[float, float]]:
  """Douglas-Peucker in local meters. Always keeps endpoints."""
  if len(coords) <= 2 or tol_m <= 0:
    return coords
  lat0, lon0 = coords[0]

  def xy(p: tuple[float, float]) -> tuple[float, float]:
    return _local_xy(p[0], p[1], lat0, lon0)

  pts = [xy(c) for c in coords]
  keep = [False] * len(coords)
  keep[0] = keep[-1] = True
  stack = [(0, len(coords) - 1)]
  while stack:
    i, j = stack.pop()
    ax, ay = pts[i]
    bx, by = pts[j]
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    max_d = -1.0
    max_k = -1
    for k in range(i + 1, j):
      px, py = pts[k]
      if ab2 < 1e-6:
        d = math.hypot(px - ax, py - ay)
      else:
        t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        d = math.hypot(ax + t * abx - px, ay + t * aby - py)
      if d > max_d:
        max_d = d
        max_k = k
    if max_k >= 0 and max_d > tol_m:
      keep[max_k] = True
      stack.append((i, max_k))
      stack.append((max_k, j))
  return [c for c, k in zip(coords, keep, strict=True) if k]


def _wrap_heading_delta(a: float, b: float) -> float:
  d = abs(a - b) % 360.0
  return min(d, 360.0 - d)


def _signed_heading_delta(from_deg: float, to_deg: float) -> float:
  """[-180, 180]: positive is clockwise (right) from current heading."""
  return ((float(to_deg) - float(from_deg) + 180.0) % 360.0) - 180.0


def _turn_sides(current_hdg: float, way_hdg: float) -> tuple[bool, bool]:
  """Left/right turn possible onto a bidirectional way from current_hdg."""
  left = right = False
  for h in (float(way_hdg), (float(way_hdg) + 180.0) % 360.0):
    d = _signed_heading_delta(current_hdg, h)
    ad = abs(d)
    if TURN_MIN_DEG <= ad <= TURN_MAX_DEG:
      if d < 0.0:
        left = True
      else:
        right = True
  return left, right


def _interp_ll(a: tuple[float, float], b: tuple[float, float], t: float) -> tuple[float, float]:
  return (float(a[0]) + t * (float(b[0]) - float(a[0])),
          float(a[1]) + t * (float(b[1]) - float(a[1])))


def _closest_on_way(
  lat: float, lon: float, coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> tuple[int, float, float, float] | None:
  """(seg_index, t in [0,1], dist_m, segment_heading_deg) or None."""
  if len(coords) < 2:
    return None
  best = 1e12
  best_i = 0
  best_t = 0.0
  best_hdg = 0.0
  px, py = 0.0, 0.0
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 < 1e-6 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dist = math.hypot(ax + t * abx - px, ay + t * aby - py)
    if dist < best:
      best = dist
      best_i = i
      best_t = t
      best_hdg = _bearing_deg(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
  return best_i, best_t, float(best), float(best_hdg)


def _ahead_segments(
  lat: float, lon: float, bearing_deg: float,
  coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> list[tuple[tuple[float, float], tuple[float, float]]] | None:
  """Polyline segments from the closest point toward the heading-ahead end."""
  hit = _closest_on_way(lat, lon, coords)
  if hit is None:
    return None
  best_i, best_t, _, best_hdg = hit
  start = _interp_ll(coords[best_i], coords[best_i + 1], best_t)
  with_digitization = _wrap_heading_delta(bearing_deg, best_hdg) <= 90.0
  segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
  if with_digitization:
    segs.append((start, coords[best_i + 1]))
    for k in range(best_i + 1, len(coords) - 1):
      segs.append((coords[k], coords[k + 1]))
  else:
    segs.append((start, coords[best_i]))
    for k in range(best_i - 1, -1, -1):
      segs.append((coords[k + 1], coords[k]))
  return [(a, b) for a, b in segs if _seg_len_m(a, b) > 0.3]


def _sample_ahead(
  lat: float, lon: float, bearing_deg: float,
  coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
  max_m: float, step_m: float = 12.0,
) -> list[tuple[float, float, float, float]]:
  """(along_m, lat, lon, heading_deg) from the closest point up to max_m."""
  segs = _ahead_segments(lat, lon, bearing_deg, coords)
  if not segs or max_m <= 0.0:
    return []
  out: list[tuple[float, float, float, float]] = []
  h0 = _bearing_deg(segs[0][0][0], segs[0][0][1], segs[0][1][0], segs[0][1][1])
  out.append((0.0, float(segs[0][0][0]), float(segs[0][0][1]), float(h0)))
  dist = 0.0
  next_d = step_m
  for a, b in segs:
    slen = _seg_len_m(a, b)
    if slen < 1e-6:
      continue
    hdg = _bearing_deg(a[0], a[1], b[0], b[1])
    while next_d <= dist + slen + 1e-6 and next_d <= max_m + 1e-6:
      t = (next_d - dist) / slen
      plat, plon = _interp_ll(a, b, min(1.0, max(0.0, t)))
      out.append((float(next_d), float(plat), float(plon), float(hdg)))
      next_d += step_m
    dist += slen
    if dist >= max_m:
      break
  return out


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  phi1, phi2 = math.radians(lat1), math.radians(lat2)
  dlon = math.radians(lon2 - lon1)
  x = math.sin(dlon) * math.cos(phi2)
  y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
  return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _offset_point(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
  theta = dist_m / EARTH_R
  brng = math.radians(bearing_deg)
  phi1 = math.radians(lat)
  lam1 = math.radians(lon)
  phi2 = math.asin(math.sin(phi1) * math.cos(theta) + math.cos(phi1) * math.sin(theta) * math.cos(brng))
  lam2 = lam1 + math.atan2(
    math.sin(brng) * math.sin(theta) * math.cos(phi1),
    math.cos(theta) - math.sin(phi1) * math.sin(phi2),
  )
  return math.degrees(phi2), (math.degrees(lam2) + 540.0) % 360.0 - 180.0


def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  y = math.radians(lat - lat0) * EARTH_R
  return x, y


def _point_to_polyline_m(lat: float, lon: float, coords: list[tuple[float, float]]) -> tuple[float, float | None]:
  """Return (min distance m, heading of closest segment or None)."""
  if len(coords) < 2:
    return 1e9, None
  best = 1e9
  best_heading = None
  px, py = 0.0, 0.0  # query is origin
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-6:
      t = 0.0
    else:
      t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dx, dy = ax + t * abx - px, ay + t * aby - py
    dist = math.hypot(dx, dy)
    if dist < best:
      best = dist
      best_heading = _bearing_deg(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
  return best, best_heading


def _seg_len_m(a: tuple[float, float], b: tuple[float, float]) -> float:
  x, y = _local_xy(b[0], b[1], a[0], a[1])
  return math.hypot(x, y)


def _remaining_ahead(
  lat: float, lon: float, bearing_deg: float, coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
) -> tuple[float, tuple[float, float], float] | None:
  """Meters along the way from the closest point to the ahead endpoint.

  Ahead is the digitization direction if GPS heading agrees (<90°), else reverse.
  Returns (remaining_m, end_latlon, end_heading_deg) or None.
  """
  if len(coords) < 2:
    return None
  best = 1e12
  best_i = 0
  best_t = 0.0
  best_hdg = 0.0
  px, py = 0.0, 0.0
  for i in range(len(coords) - 1):
    ax, ay = _local_xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _local_xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 < 1e-6 else max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    dist = math.hypot(ax + t * abx - px, ay + t * aby - py)
    if dist < best:
      best = dist
      best_i = i
      best_t = t
      best_hdg = _bearing_deg(coords[i][0], coords[i][1], coords[i + 1][0], coords[i + 1][1])
  s = 0.0
  for i in range(best_i):
    s += _seg_len_m(coords[i], coords[i + 1])
  s += best_t * _seg_len_m(coords[best_i], coords[best_i + 1])
  total = 0.0
  for i in range(len(coords) - 1):
    total += _seg_len_m(coords[i], coords[i + 1])
  with_digitization = _wrap_heading_delta(bearing_deg, best_hdg) <= 90.0
  if with_digitization:
    remaining = max(0.0, total - s)
    end = coords[-1]
    end_hdg = _bearing_deg(coords[-2][0], coords[-2][1], coords[-1][0], coords[-1][1])
  else:
    remaining = max(0.0, s)
    end = coords[0]
    end_hdg = (_bearing_deg(coords[0][0], coords[0][1], coords[1][0], coords[1][1]) + 180.0) % 360.0
  return remaining, (float(end[0]), float(end[1])), float(end_hdg)


class OsmSpeedLimitDB:
  def __init__(self, path: str):
    self.path = path
    self._con: sqlite3.Connection | None = None

  @property
  def loaded(self) -> bool:
    return self._con is not None

  def open(self) -> bool:
    if self._con is not None:
      return True
    if not self.path or not os.path.isfile(self.path):
      return False
    con = sqlite3.connect(self.path, check_same_thread=False)
    con.row_factory = sqlite3.Row
    try:
      con.execute("SELECT way_id FROM ways LIMIT 1")
    except sqlite3.Error:
      con.close()
      return False
    self._con = con
    return True

  def close(self) -> None:
    if self._con is not None:
      self._con.close()
      self._con = None

  @staticmethod
  def create(path: str) -> sqlite3.Connection:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if os.path.exists(path):
      os.remove(path)
    con = sqlite3.connect(path)
    con.executescript("""
      CREATE TABLE meta (
        key TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
      CREATE TABLE ways (
        way_id INTEGER PRIMARY KEY,
        name TEXT,
        highway TEXT,
        maxspeed_ms REAL NOT NULL,
        min_lat REAL NOT NULL,
        max_lat REAL NOT NULL,
        min_lon REAL NOT NULL,
        max_lon REAL NOT NULL,
        coords BLOB NOT NULL
      );
      CREATE VIRTUAL TABLE ways_rtree USING rtree(
        way_id, min_lat, max_lat, min_lon, max_lon
      );
    """)
    con.executemany(
      "INSERT INTO meta(key, value) VALUES (?, ?)",
      [
        ("attribution", "© OpenStreetMap contributors"),
        ("license", "ODbL"),
        ("license_url", "https://www.openstreetmap.org/copyright"),
        ("format", "nap-osm-speedlimit-v1"),
      ],
    )
    con.commit()
    return con

  @staticmethod
  def meta_get(path: str, key: str) -> str | None:
    if not path or not os.path.isfile(path):
      return None
    try:
      con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
      try:
        row = con.execute("SELECT value FROM meta WHERE key = ?", (key,)).fetchone()
        return str(row[0]) if row else None
      finally:
        con.close()
    except sqlite3.Error:
      return None

  @staticmethod
  def way_count(path: str) -> int | None:
    raw = OsmSpeedLimitDB.meta_get(path, "way_count")
    if raw is None:
      return None
    try:
      return int(raw)
    except ValueError:
      return None

  @staticmethod
  def insert_way(con: sqlite3.Connection, way_id: int, name: str, highway: str,
                 maxspeed_ms: float, coords: list[tuple[float, float]]) -> None:
    coords = simplify_coords(coords)
    # maxspeed_ms=0 is geometry-only (junction detect, never posted LIMIT).
    if len(coords) < 2 or maxspeed_ms < 0:
      return
    lats = [c[0] for c in coords]
    lons = [c[1] for c in coords]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    blob = _pack_coords(coords)
    con.execute(
      "INSERT OR REPLACE INTO ways(way_id, name, highway, maxspeed_ms, min_lat, max_lat, min_lon, max_lon, coords) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
      (int(way_id), name or "", highway or "", float(maxspeed_ms), min_lat, max_lat, min_lon, max_lon, blob),
    )
    con.execute(
      "INSERT OR REPLACE INTO ways_rtree(way_id, min_lat, max_lat, min_lon, max_lon) VALUES (?, ?, ?, ?, ?)",
      (int(way_id), min_lat, max_lat, min_lon, max_lon),
    )

  @staticmethod
  def delete_ways_intersecting_bbox(con: sqlite3.Connection, south: float, west: float, north: float, east: float) -> int:
    """Remove ways whose stored bbox intersects south,west,north,east. Returns deleted count."""
    rows = con.execute(
      "SELECT way_id FROM ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
      (south, north, west, east),
    ).fetchall()
    ids = [int(r[0]) for r in rows]
    if not ids:
      return 0
    for i in range(0, len(ids), 500):
      chunk = ids[i:i + 500]
      q = ",".join("?" * len(chunk))
      con.execute(f"DELETE FROM ways WHERE way_id IN ({q})", chunk)
      con.execute(f"DELETE FROM ways_rtree WHERE way_id IN ({q})", chunk)
    return len(ids)

  @staticmethod
  def recount_ways(con: sqlite3.Connection) -> int:
    n = int(con.execute("SELECT COUNT(*) FROM ways").fetchone()[0])
    con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", ("way_count", str(n)))
    return n

  def _candidates(self, lat: float, lon: float) -> list[sqlite3.Row]:
    if self._con is None:
      return []
    pad = SEARCH_PAD_DEG
    ids = [
      int(r[0]) for r in self._con.execute(
        "SELECT way_id FROM ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
        (lat - pad, lat + pad, lon - pad, lon + pad),
      )
    ]
    if not ids:
      return []
    q = f"SELECT * FROM ways WHERE way_id IN ({','.join('?' * len(ids))})"
    return list(self._con.execute(q, ids))

  def _ways_in_bbox(self, min_lat: float, max_lat: float, min_lon: float, max_lon: float) -> list[sqlite3.Row]:
    if self._con is None:
      return []
    ids = [
      int(r[0]) for r in self._con.execute(
        "SELECT way_id FROM ways_rtree WHERE max_lat >= ? AND min_lat <= ? AND max_lon >= ? AND min_lon <= ?",
        (min_lat, max_lat, min_lon, max_lon),
      )
    ]
    if not ids:
      return []
    q = f"SELECT * FROM ways WHERE way_id IN ({','.join('?' * len(ids))})"
    return list(self._con.execute(q, ids))

  def _best_match(self, lat: float, lon: float, bearing_deg: float | None) -> SpeedLimitMatch | None:
    best: SpeedLimitMatch | None = None
    best_score = 1e12
    for row in self._candidates(lat, lon):
      if float(row["maxspeed_ms"]) <= 0:
        continue
      coords = _unpack_coords(row["coords"])
      dist, seg_heading = _point_to_polyline_m(lat, lon, coords)
      if dist > MAX_MATCH_DISTANCE_M:
        continue
      heading_pen = 0.0
      if bearing_deg is not None and seg_heading is not None:
        delta = _wrap_heading_delta(bearing_deg, seg_heading)
        # Opposite-direction ways: treat as a large penalty so we pick the
        # carriageway we are actually on when a dual carriageway is nearby.
        if delta > 180.0 - HEADING_ALIGN_DEG:
          heading_pen = 40.0
        elif delta > HEADING_ALIGN_DEG:
          heading_pen = 15.0
      score = dist + heading_pen
      if score < best_score:
        best_score = score
        best = SpeedLimitMatch(
          speed_limit_ms=float(row["maxspeed_ms"]),
          way_id=int(row["way_id"]),
          road_name=row["name"] or "",
          highway=row["highway"] or "",
          distance_m=float(dist),
          coords=tuple((float(c[0]), float(c[1])) for c in coords),
        )
    return best

  def _geodesic_next(self, lat: float, lon: float, bearing_deg: float,
                     current_ms: float) -> tuple[float, float] | None:
    """Heading-ray probes. Can skip a short way if GPS heading is off the road."""
    prev_same_d = 0.0
    for d in LOOKAHEAD_M:
      alat, alon = _offset_point(lat, lon, bearing_deg, d)
      ahead = self._best_match(alat, alon, bearing_deg)
      if ahead is None:
        continue
      if abs(ahead.speed_limit_ms - current_ms) <= 0.3:
        prev_same_d = d
        continue
      next_dist = 0.5 * (prev_same_d + d) if prev_same_d > 0 else d
      return float(ahead.speed_limit_ms), float(next_dist)
    return None

  def _along_way_next(self, lat: float, lon: float, bearing_deg: float,
                      match: SpeedLimitMatch) -> tuple[float, float] | None:
    """Follow the matched way to its end, then the next way, within LOOKAHEAD_MAX_M.

    Catches a short intermediate limit (US 12 60→50 before 30) that geodesic
    40 m heading probes skip when the road curves or the 50 way is short.
    """
    if not match.coords:
      return None
    current_ms = float(match.speed_limit_ms)
    traveled = 0.0
    cur_lat, cur_lon = float(lat), float(lon)
    cur_brg = float(bearing_deg)
    cur_coords: list[tuple[float, float]] | tuple[tuple[float, float], ...] = match.coords
    seen = {int(match.way_id)}
    for _ in range(24):
      ahead_info = _remaining_ahead(cur_lat, cur_lon, cur_brg, cur_coords)
      if ahead_info is None:
        return None
      rem, end_ll, end_hdg = ahead_info
      if traveled + rem > LOOKAHEAD_MAX_M + 1.0:
        return None
      traveled += rem
      nxt = None
      # Step past the end; 12 m can still match the way we just left
      # (MAX_MATCH_DISTANCE_M=35).
      for step_m in (12.0, 25.0, 40.0):
        plat, plon = _offset_point(end_ll[0], end_ll[1], end_hdg, step_m)
        cand = self._best_match(plat, plon, end_hdg)
        if cand is not None and int(cand.way_id) not in seen:
          nxt = cand
          break
      if nxt is None:
        return None
      seen.add(int(nxt.way_id))
      if abs(nxt.speed_limit_ms - current_ms) > 0.3:
        return float(nxt.speed_limit_ms), float(traveled)
      if not nxt.coords:
        return None
      cur_coords = nxt.coords
      cur_lat, cur_lon = plat, plon
      cur_brg = end_hdg
    return None

  def lookup(self, lat: float, lon: float, bearing_deg: float | None = None,
             v_ego_ms: float = 0.0) -> SpeedLimitMatch | None:
    if self._con is None:
      return None
    gps_match = self._best_match(float(lat), float(lon), bearing_deg)
    qlat, qlon = float(lat), float(lon)
    lead_m = osm_sign_lead_m(v_ego_ms)
    lead_match = gps_match
    if lead_m > 0.0 and bearing_deg is not None:
      qlat, qlon = _offset_point(qlat, qlon, float(bearing_deg), lead_m)
      lead_match = self._best_match(qlat, qlon, bearing_deg)

    # GNSS lag: raise posted when the 1.5 s point is already in a higher zone.
    # Do not snap posted down — any decrease (10/15/20 mph, short zones) must
    # ease with kin+110 m, not jump when GPS or the offset crosses the boundary.
    raised = (
      lead_match is not None and gps_match is not None
      and float(lead_match.speed_limit_ms) > float(gps_match.speed_limit_ms) + 0.3
    )
    if raised or gps_match is None:
      match = lead_match
    else:
      match = gps_match
    if match is None:
      return None
    if bearing_deg is None:
      return match

    # Decrease lookahead is separate: remaining to the next lower from GPS,
    # minus v*1.5 s so ease starts at the lag-corrected zone (not a posted cliff).
    if raised:
      along = self._along_way_next(qlat, qlon, float(bearing_deg), match)
      geo = self._geodesic_next(qlat, qlon, float(bearing_deg), match.speed_limit_ms)
      picked = along if along is not None else geo
    else:
      along = self._along_way_next(float(lat), float(lon), float(bearing_deg), match)
      geo = self._geodesic_next(float(lat), float(lon), float(bearing_deg), match.speed_limit_ms)
      picked = along if along is not None else geo
      if picked is not None:
        nxt, dist = picked
        picked = (nxt, max(0.0, float(dist) - lead_m))
    # Prefer along-way (true road distance, does not skip short ways). If it
    # finds nothing, geodesic may still see a nearby different-speed way.
    if picked is None:
      return match
    next_limit, next_dist = picked
    if next_limit <= 0:
      return match
    return SpeedLimitMatch(
      speed_limit_ms=match.speed_limit_ms,
      way_id=match.way_id,
      road_name=match.road_name,
      highway=match.highway,
      distance_m=match.distance_m,
      next_speed_limit_ms=next_limit,
      next_distance_m=next_dist,
    )

  def _junction_at_sample(
    self, plat: float, plon: float, heading: float, seen: set[int], rows: list[sqlite3.Row],
  ) -> tuple[bool, bool, float, float]:
    """Left/right flags and dest speeds for turn-ways within JUNCTION_RADIUS_M of a sample."""
    has_left = has_right = False
    left_ms = right_ms = 0.0
    for row in rows:
      wid = int(row["way_id"])
      if wid in seen:
        continue
      coords = _unpack_coords(row["coords"])
      dist, seg_hdg = _point_to_polyline_m(plat, plon, coords)
      if dist > JUNCTION_RADIUS_M or seg_hdg is None:
        continue
      left, right = _turn_sides(heading, seg_hdg)
      dest = float(row["maxspeed_ms"])
      if left:
        has_left = True
        if dest > 0.0:
          left_ms = dest if left_ms <= 0.0 else min(left_ms, dest)
      if right:
        has_right = True
        if dest > 0.0:
          right_ms = dest if right_ms <= 0.0 else min(right_ms, dest)
    return has_left, has_right, left_ms, right_ms

  def lookup_intersection(self, lat: float, lon: float, bearing_deg: float | None = None,
                          v_ego_ms: float = 0.0) -> IntersectionAhead | None:
    """Nearest turn junction along the matched way, or None.

    A junction is another stored highway way within JUNCTION_RADIUS_M whose
    heading is 35–145° off the current road (both directions of that way), or a
    sharp bend on the current way. Cross streets with maxspeed_ms=0 still count
    (never as posted LIMIT). Distance is along-way from GPS, minus v*1.5 s.
    """
    if self._con is None or bearing_deg is None:
      return None
    match = self._best_match(float(lat), float(lon), bearing_deg)
    if match is None or not match.coords:
      return None

    horizon = float(INTERSECTION_LOOKAHEAD_M)
    lead_m = osm_sign_lead_m(v_ego_ms)
    traveled = 0.0
    cur_lat, cur_lon = float(lat), float(lon)
    cur_brg = float(bearing_deg)
    cur_coords: list[tuple[float, float]] | tuple[tuple[float, float], ...] = match.coords
    seen = {int(match.way_id)}
    current_ms = float(match.speed_limit_ms)

    for _ in range(12):
      remain = horizon - traveled
      if remain <= 1.0:
        break
      samples = _sample_ahead(cur_lat, cur_lon, cur_brg, cur_coords, remain, step_m=12.0)
      if samples:
        lats = [s[1] for s in samples]
        lons = [s[2] for s in samples]
        dlat = 30.0 / 111000.0
        dlon = 30.0 / (111000.0 * max(0.2, math.cos(math.radians(samples[0][1]))))
        rows = self._ways_in_bbox(min(lats) - dlat, max(lats) + dlat, min(lons) - dlon, max(lons) + dlon)
        # Same-way sharp bend: heading change 35–145° at a vertex.
        prev_hdg = samples[0][3]
        bend_at: tuple[float, bool, bool] | None = None
        for along, _slat, _slon, hdg in samples:
          # Same-way bend uses the outgoing heading only (not bidirectional).
          d = _signed_heading_delta(prev_hdg, hdg)
          ad = abs(d)
          if TURN_MIN_DEG <= ad <= TURN_MAX_DEG:
            bend_at = (along, d < 0.0, d > 0.0)
            break
          prev_hdg = hdg

        first: IntersectionAhead | None = None
        for along, slat, slon, hdg in samples:
          hl, hr, lms, rms = self._junction_at_sample(slat, slon, hdg, seen, rows)
          if bend_at is not None and abs(along - bend_at[0]) <= JUNCTION_CLUSTER_M:
            if bend_at[1]:
              hl = True
              lms = current_ms if lms <= 0.0 else min(lms, current_ms)
            if bend_at[2]:
              hr = True
              rms = current_ms if rms <= 0.0 else min(rms, current_ms)
          if hl or hr:
            first = IntersectionAhead(
              distance_m=traveled + along,
              has_left=hl,
              has_right=hr,
              left_speed_ms=lms,
              right_speed_ms=rms,
            )
            break
        if first is None and bend_at is not None:
          along, is_left, is_right = bend_at
          first = IntersectionAhead(
            distance_m=traveled + along,
            has_left=is_left,
            has_right=is_right,
            left_speed_ms=current_ms if is_left else 0.0,
            right_speed_ms=current_ms if is_right else 0.0,
          )
        if first is not None:
          dist = max(0.0, float(first.distance_m) - lead_m)
          return IntersectionAhead(
            distance_m=dist,
            has_left=first.has_left,
            has_right=first.has_right,
            left_speed_ms=first.left_speed_ms,
            right_speed_ms=first.right_speed_ms,
          )

      ahead_info = _remaining_ahead(cur_lat, cur_lon, cur_brg, cur_coords)
      if ahead_info is None:
        return None
      rem, end_ll, end_hdg = ahead_info
      if traveled + rem > horizon + 1.0:
        return None
      traveled += rem
      nxt = None
      plat, plon = float(end_ll[0]), float(end_ll[1])
      for step_m in (12.0, 25.0, 40.0):
        plat, plon = _offset_point(end_ll[0], end_ll[1], end_hdg, step_m)
        cand = self._best_match(plat, plon, end_hdg)
        if cand is not None and int(cand.way_id) not in seen:
          nxt = cand
          break
      if nxt is None:
        return None
      nxt_hdg = None
      if nxt.coords and len(nxt.coords) >= 2:
        _, nxt_hdg = _point_to_polyline_m(plat, plon, list(nxt.coords))
      if nxt_hdg is not None:
        d = _signed_heading_delta(end_hdg, nxt_hdg)
        ad = abs(d)
        if TURN_MIN_DEG <= ad <= TURN_MAX_DEG:
          dist = max(0.0, traveled - lead_m)
          left, right = d < 0.0, d > 0.0
          dest = float(nxt.speed_limit_ms)
          return IntersectionAhead(
            distance_m=dist,
            has_left=left,
            has_right=right,
            left_speed_ms=dest if left else 0.0,
            right_speed_ms=dest if right else 0.0,
          )
      seen.add(int(nxt.way_id))
      if not nxt.coords:
        return None
      cur_coords = nxt.coords
      cur_lat, cur_lon = plat, plon
      cur_brg = end_hdg
      current_ms = float(nxt.speed_limit_ms)
    return None
