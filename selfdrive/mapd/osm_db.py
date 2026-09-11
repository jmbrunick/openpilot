"""Offline OSM speed-limit SQLite (R-tree) for comma 3X.

Schema is NAP-owned. Data is OpenStreetMap (ODbL). Query path is GPS → nearest
heading-aligned way. nextSpeedLimit follows that matched way (bearing + class),
not a nearby off-route fill. A lower limit that only lasts MIN_ZONE_LENGTH_M
(~250 ft) along heading is ignored (cross-street bleed / intersection stub).
Tagged OSM maxspeed is authoritative; Minnesota packs may include statutory
estimates for unmarked highways (never uploaded to OSM).
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
  LOOKAHEAD_MAX_M,
  MAX_MATCH_DISTANCE_M,
  MIN_ZONE_LENGTH_M,
  SEARCH_PAD_DEG,
  osm_sign_lead_m,
)

# Functional class for route continuity. Residential fills next to a trunk
# must not count as the "next limit ahead" even when a heading ray hits them.
_HIGHWAY_RANK = {
  "motorway": 0, "motorway_link": 0,
  "trunk": 1, "trunk_link": 1,
  "primary": 2, "primary_link": 2,
  "secondary": 3, "secondary_link": 3,
  "tertiary": 4, "tertiary_link": 4,
  "unclassified": 5,
  "residential": 6, "living_street": 6, "alley": 7,
}

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


def _continues_route(
  bearing_deg: float | None, seg_heading: float | None,
  from_name: str, from_highway: str, to_name: str, to_highway: str,
) -> bool:
  """True if a candidate is the road ahead, not an off-route cross street / fill.

  Bearing must stay within HEADING_ALIGN_DEG. A named road may change class
  (US 12 trunk → primary in town). A major way must not jump onto an unnamed
  residential / living_street fill just because it is geometrically closer.
  """
  if bearing_deg is not None and seg_heading is not None:
    if _wrap_heading_delta(bearing_deg, seg_heading) > HEADING_ALIGN_DEG:
      return False
  fn = (from_name or "").strip().lower()
  tn = (to_name or "").strip().lower()
  if fn and tn and fn == tn:
    return True
  fr = _HIGHWAY_RANK.get((from_highway or "").strip().lower(), 5)
  tr = _HIGHWAY_RANK.get((to_highway or "").strip().lower(), 5)
  # trunk/primary/secondary → residential 30 fill (v3 MN statutory).
  if fr <= 3 and tr >= 6:
    return False
  return True


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

  def _best_match(self, lat: float, lon: float, bearing_deg: float | None,
                  along_route: SpeedLimitMatch | None = None) -> SpeedLimitMatch | None:
    best: SpeedLimitMatch | None = None
    best_score = 1e12
    for row in self._candidates(lat, lon):
      coords = _unpack_coords(row["coords"])
      dist, seg_heading = _point_to_polyline_m(lat, lon, coords)
      if dist > MAX_MATCH_DISTANCE_M:
        continue
      name = row["name"] or ""
      highway = row["highway"] or ""
      heading_pen = 0.0
      if along_route is not None:
        if not _continues_route(
          bearing_deg, seg_heading,
          along_route.road_name, along_route.highway, name, highway,
        ):
          continue
      elif bearing_deg is not None and seg_heading is not None:
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
          road_name=name,
          highway=highway,
          distance_m=float(dist),
          coords=tuple((float(c[0]), float(c[1])) for c in coords),
        )
    return best

  def _limit_persists_min_zone(
    self, lat: float, lon: float, bearing_deg: float, match: SpeedLimitMatch,
    along_route: SpeedLimitMatch | None = None,
  ) -> bool:
    """True if this limit is still present MIN_ZONE_LENGTH_M along heading.

    Cross-street bleed matches a long side road at the intersection, but
    MIN_ZONE_LENGTH_M along the highway heading is already back on the
    main road. A tiny stub way fails the same way. Remaining-along-way
    alone is not enough: a long N-S fill has lots of remaining.
    """
    plat, plon = _offset_point(lat, lon, bearing_deg, MIN_ZONE_LENGTH_M)
    ahead = self._best_match(plat, plon, bearing_deg, along_route=along_route)
    if ahead is not None:
      return abs(ahead.speed_limit_ms - match.speed_limit_ms) <= 0.3
    rem = _remaining_ahead(lat, lon, bearing_deg, match.coords or ())
    if rem is None or rem[0] < MIN_ZONE_LENGTH_M:
      return False
    _dist, seg_hdg = _point_to_polyline_m(lat, lon, list(match.coords or ()))
    if seg_hdg is None:
      return False
    return _wrap_heading_delta(bearing_deg, seg_hdg) <= HEADING_ALIGN_DEG

  def _stabilize_match(
    self, lat: float, lon: float, bearing_deg: float | None, match: SpeedLimitMatch | None,
  ) -> SpeedLimitMatch | None:
    """Drop a matched lower limit that only lasts a stub along heading.

    OSM/GPS at a crossroads can snap onto a side street for ~a car length.
    Prefer the road that is still there MIN_ZONE_LENGTH_M ahead. Never snap
    posted down to a lower ahead zone — that decrease eases via next.
    """
    if match is None or bearing_deg is None:
      return match
    if self._limit_persists_min_zone(lat, lon, bearing_deg, match):
      return match
    plat, plon = _offset_point(lat, lon, bearing_deg, MIN_ZONE_LENGTH_M)
    ahead = self._best_match(plat, plon, bearing_deg)
    if ahead is None or int(ahead.way_id) == int(match.way_id):
      return match
    if ahead.speed_limit_ms + 0.3 < match.speed_limit_ms:
      return match
    # Same named road changing speed is a real zone end, not cross-street
    # bleed. Do not raise posted early; the 1.5 s GNSS offset does that.
    match_name = (match.road_name or "").strip().lower()
    ahead_name = (ahead.road_name or "").strip().lower()
    if match_name and match_name == ahead_name and ahead.speed_limit_ms > match.speed_limit_ms + 0.3:
      return match
    if not self._limit_persists_min_zone(plat, plon, bearing_deg, ahead):
      return match
    stable = self._best_match(lat, lon, bearing_deg, along_route=ahead)
    if stable is not None and abs(stable.speed_limit_ms - ahead.speed_limit_ms) <= 0.3:
      return stable
    return ahead

  def _geodesic_next(self, lat: float, lon: float, bearing_deg: float,
                     match: SpeedLimitMatch) -> tuple[float, float] | None:
    """Heading-ray probes after the matched way ends (OSM gap). Route-bearing only."""
    prev_same_d = 0.0
    current_ms = float(match.speed_limit_ms)
    for d in LOOKAHEAD_M:
      alat, alon = _offset_point(lat, lon, bearing_deg, d)
      ahead = self._best_match(alat, alon, bearing_deg, along_route=match)
      if ahead is None:
        continue
      if abs(ahead.speed_limit_ms - current_ms) <= 0.3:
        prev_same_d = d
        continue
      if not self._limit_persists_min_zone(alat, alon, bearing_deg, ahead, along_route=match):
        continue
      next_dist = 0.5 * (prev_same_d + d) if prev_same_d > 0 else d
      return float(ahead.speed_limit_ms), float(next_dist)
    return None

  def _along_way_next(self, lat: float, lon: float, bearing_deg: float,
                      match: SpeedLimitMatch) -> tuple[tuple[float, float] | None, bool]:
    """Follow the matched way to its end, then the next on-route way.

    Returns (picked, resolved). resolved means the next LOOKAHEAD_MAX_M along
    this road is known (a different limit, or the same limit continuing) — do
    not fall back to a heading ray that can hit off-route fills.

    Catches a short intermediate limit (US 12 60→50 before 30) that geodesic
    40 m heading probes skip when the road curves or the 50 way is short.
    """
    if not match.coords:
      return None, False
    current_ms = float(match.speed_limit_ms)
    traveled = 0.0
    cur_lat, cur_lon = float(lat), float(lon)
    cur_brg = float(bearing_deg)
    cur_coords: list[tuple[float, float]] | tuple[tuple[float, float], ...] = match.coords
    seen = {int(match.way_id)}
    plat, plon = cur_lat, cur_lon
    for _ in range(24):
      ahead_info = _remaining_ahead(cur_lat, cur_lon, cur_brg, cur_coords)
      if ahead_info is None:
        return None, False
      rem, end_ll, end_hdg = ahead_info
      if traveled + rem > LOOKAHEAD_MAX_M + 1.0:
        return None, True
      nxt = None
      # Step past the end; 12 m can still match the way we just left
      # (MAX_MATCH_DISTANCE_M=35).
      for step_m in (12.0, 25.0, 40.0):
        plat, plon = _offset_point(end_ll[0], end_ll[1], end_hdg, step_m)
        cand = self._best_match(plat, plon, end_hdg, along_route=match)
        if cand is not None and int(cand.way_id) not in seen:
          nxt = cand
          break
      if nxt is None:
        return None, False
      seen.add(int(nxt.way_id))
      if abs(nxt.speed_limit_ms - current_ms) > 0.3:
        if self._limit_persists_min_zone(plat, plon, end_hdg, nxt, along_route=match):
          traveled += rem
          return (float(nxt.speed_limit_ms), float(traveled)), True
        # Stub / cross-street bleed: do not walk down that way. Stay on the
        # previous geometry; `seen` prevents re-picking the stub.
        continue
      traveled += rem
      if not nxt.coords:
        return None, False
      cur_coords = nxt.coords
      cur_lat, cur_lon = plat, plon
      cur_brg = end_hdg
    return None, True

  def _next_limit(self, lat: float, lon: float, bearing_deg: float,
                  match: SpeedLimitMatch) -> tuple[float, float] | None:
    along, resolved = self._along_way_next(lat, lon, bearing_deg, match)
    if along is not None:
      return along
    if resolved:
      return None
    return self._geodesic_next(lat, lon, bearing_deg, match)

  def lookup(self, lat: float, lon: float, bearing_deg: float | None = None,
             v_ego_ms: float = 0.0) -> SpeedLimitMatch | None:
    if self._con is None:
      return None
    gps_match = self._stabilize_match(
      float(lat), float(lon), bearing_deg,
      self._best_match(float(lat), float(lon), bearing_deg),
    )
    qlat, qlon = float(lat), float(lon)
    lead_m = osm_sign_lead_m(v_ego_ms)
    lead_match = gps_match
    if lead_m > 0.0 and bearing_deg is not None:
      qlat, qlon = _offset_point(qlat, qlon, float(bearing_deg), lead_m)
      lead_match = self._stabilize_match(
        qlat, qlon, bearing_deg,
        self._best_match(qlat, qlon, bearing_deg),
      )

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

    # Next limit follows the matched road (bearing + class/name). Do not use a
    # heading ray that can pick a nearby residential fill as "ahead".
    # Remaining is from GPS minus v*1.5 s so ease starts at the lag-corrected zone.
    if raised:
      picked = self._next_limit(qlat, qlon, float(bearing_deg), match)
    else:
      picked = self._next_limit(float(lat), float(lon), float(bearing_deg), match)
      if picked is not None:
        nxt, dist = picked
        picked = (nxt, max(0.0, float(dist) - lead_m))
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
