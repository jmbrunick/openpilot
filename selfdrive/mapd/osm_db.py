"""Offline OSM speed-limit SQLite (R-tree) for comma 3X.

Schema is NAP-owned. Data is OpenStreetMap (ODbL). Query path is GPS → nearest
heading-aligned way with an explicit maxspeed tag.
"""
from __future__ import annotations

import math
import os
import sqlite3
import struct
from dataclasses import dataclass

from openpilot.selfdrive.mapd.constants import (
  HEADING_ALIGN_DEG,
  LOOKAHEAD_M,
  MAX_MATCH_DISTANCE_M,
  SEARCH_PAD_DEG,
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
    if len(coords) < 2 or maxspeed_ms <= 0:
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

  def _best_match(self, lat: float, lon: float, bearing_deg: float | None) -> SpeedLimitMatch | None:
    best: SpeedLimitMatch | None = None
    best_score = 1e12
    for row in self._candidates(lat, lon):
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
        )
    return best

  def lookup(self, lat: float, lon: float, bearing_deg: float | None = None) -> SpeedLimitMatch | None:
    if self._con is None:
      return None
    match = self._best_match(lat, lon, bearing_deg)
    if match is None:
      return match
    if bearing_deg is None:
      return match

    next_limit = 0.0
    next_dist = 0.0
    prev_same_d = 0.0
    for d in LOOKAHEAD_M:
      alat, alon = _offset_point(lat, lon, bearing_deg, d)
      ahead = self._best_match(alat, alon, bearing_deg)
      if ahead is None:
        continue
      if abs(ahead.speed_limit_ms - match.speed_limit_ms) <= 0.3:
        prev_same_d = d
        continue
      # Refine: first different probe, then midpoint toward last same-limit probe.
      next_limit = ahead.speed_limit_ms
      next_dist = 0.5 * (prev_same_d + d) if prev_same_d > 0 else d
      break
    if next_limit > 0:
      return SpeedLimitMatch(
        speed_limit_ms=match.speed_limit_ms,
        way_id=match.way_id,
        road_name=match.road_name,
        highway=match.highway,
        distance_m=match.distance_m,
        next_speed_limit_ms=next_limit,
        next_distance_m=next_dist,
      )
    return match
