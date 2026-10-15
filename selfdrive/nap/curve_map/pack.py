"""NAP curve pack: road curvature precomputed from the OSM map (longitudinal only).

A separate, versioned sqlite per 0.25 deg tile, built from the speed-limit
map that is already on the device (`ways` table, Douglas-Peucker coords).
Nothing here touches the speed-limit DB: it is opened read-only.

Per drivable way: the polyline is resampled every STEP_M, curvature comes
from the turn angle at each vertex over its share of the two adjacent
segments, smoothed over SMOOTH_M. Every resampled point is stored (the
runtime walks the road, straights included) with its heading and signed
curvature.

Rebuild (any region, e.g. on a PC):
  python -m openpilot.selfdrive.nap.curve_map.pack --src speed_limits.sqlite --out DIR --bbox S W N E
"""
from __future__ import annotations

import argparse
import math
import os
import sqlite3
import struct

PACK_VERSION = 1
TILE_DEG = 0.25
STEP_M = 10.0
SMOOTH_M = 30.0
MARGIN_DEG = 0.02
EARTH_R = 6371000.0
DRIVABLE = frozenset({
  "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
  "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "road",
})
_HDR = struct.Struct("<I")
_F32 = struct.Struct("<ff")
_F64 = struct.Struct("<dd")


def tile_key(lat: float, lon: float) -> tuple[int, int]:
  return int(math.floor(lat / TILE_DEG)), int(math.floor(lon / TILE_DEG))


def tile_path(out_dir: str, key: tuple[int, int]) -> str:
  return os.path.join(out_dir, f"nap_curves_v{PACK_VERSION}_{key[0]}_{key[1]}.sqlite")


def unpack_coords(blob: bytes) -> list[tuple[float, float]]:
  n = _HDR.unpack_from(blob)[0]
  rest = len(blob) - _HDR.size
  fmt = _F32 if rest == n * _F32.size else (_F64 if rest == n * _F64.size else None)
  if fmt is None:
    return []
  return [fmt.unpack_from(blob, _HDR.size + i * fmt.size) for i in range(n)]


def _xy(lat, lon, lat0, lon0):
  return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)), math.radians(lat - lat0) * EARTH_R)


def way_profile(coords) -> list[tuple[float, float, float, float]]:
  """[(lat, lon, heading_deg, kappa)] every STEP_M along the way. kappa > 0 = left."""
  if len(coords) < 2:
    return []
  lat0, lon0 = coords[0]
  P = [_xy(a, b, lat0, lon0) for a, b in coords]
  seg = []
  for (x0, y0), (x1, y1) in zip(P[:-1], P[1:], strict=True):
    seg.append((math.hypot(x1 - x0, y1 - y0), math.atan2(x1 - x0, y1 - y0)))
  # vertex curvature over half of each adjacent segment
  s_v = [0.0]
  for L, _ in seg:
    s_v.append(s_v[-1] + L)
  k_v = [0.0] * len(P)
  for i in range(1, len(P) - 1):
    L0, h0 = seg[i - 1]
    L1, h1 = seg[i]
    if L0 < 0.5 or L1 < 0.5:
      continue
    turn = (h0 - h1 + math.pi) % (2 * math.pi) - math.pi  # left positive (heading is cw from north)
    k_v[i] = turn / (0.5 * (L0 + L1))
  total = s_v[-1]
  n = max(1, int(math.ceil(total / STEP_M)))
  out = []
  half = SMOOTH_M / 2.0
  for j in range(n + 1):
    s = min(total, j * STEP_M)  # the last point is the true way end (joins the next way)
    # position / heading
    i = max(0, min(len(seg) - 1, next((q for q in range(len(seg)) if s_v[q + 1] >= s), len(seg) - 1)))
    f = 0.0 if seg[i][0] <= 0 else (s - s_v[i]) / seg[i][0]
    x = P[i][0] + f * (P[i + 1][0] - P[i][0])
    y = P[i][1] + f * (P[i + 1][1] - P[i][1])
    # each vertex's turn spread over its own share, then a SMOOTH_M box
    acc = 0.0
    for v in range(1, len(P) - 1):
      if k_v[v] == 0.0:
        continue
      a = s_v[v] - 0.5 * seg[v - 1][0]
      b = s_v[v] + 0.5 * seg[v][0]
      ov = min(b, s + half) - max(a, s - half)
      if ov > 0:
        acc += k_v[v] * ov
    k = acc / SMOOTH_M
    lat = lat0 + math.degrees(y / EARTH_R)
    lon = lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0))))
    out.append((lat, lon, math.degrees(seg[i][1]) % 360.0, k))
  return out


def build(src: str, out_path: str, s: float, w: float, n: float, e: float) -> int:
  con = sqlite3.connect(f"file:{src}?mode=ro", uri=True)
  tmp = out_path + ".partial"
  if os.path.exists(tmp):
    os.remove(tmp)
  os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
  o = sqlite3.connect(tmp)
  o.executescript("""
    CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
    CREATE TABLE pts (id INTEGER PRIMARY KEY, way_id INTEGER NOT NULL, seq INTEGER NOT NULL,
                      lat REAL NOT NULL, lon REAL NOT NULL, hdg REAL NOT NULL, k REAL NOT NULL);
    CREATE INDEX pts_way ON pts(way_id, seq);
    CREATE VIRTUAL TABLE pts_rtree USING rtree(id, min_lat, max_lat, min_lon, max_lon);
  """)
  rows = con.execute(
    "SELECT ways.way_id, ways.highway, ways.coords FROM ways_rtree JOIN ways ON ways.way_id = ways_rtree.way_id " +
    "WHERE ways_rtree.max_lat >= ? AND ways_rtree.min_lat <= ? AND ways_rtree.max_lon >= ? AND ways_rtree.min_lon <= ?",
    (s, n, w, e))
  pid = 0
  for way_id, hw, blob in rows:
    if (hw or "").strip().lower() not in DRIVABLE:
      continue
    for seq, (la, lo, hd, k) in enumerate(way_profile(unpack_coords(blob))):
      pid += 1
      o.execute("INSERT INTO pts VALUES (?,?,?,?,?,?,?)", (pid, int(way_id), seq, la, lo, round(hd, 1), round(k, 5)))
      o.execute("INSERT INTO pts_rtree VALUES (?,?,?,?,?)", (pid, la, la, lo, lo))
  o.executemany("INSERT INTO meta VALUES (?,?)", [("version", str(PACK_VERSION)), ("bbox", f"{s},{w},{n},{e}"),
                                                  ("step_m", str(STEP_M)), ("smooth_m", str(SMOOTH_M))])
  o.commit()
  o.close()
  con.close()
  os.replace(tmp, out_path)
  return pid


def build_tile(src: str, out_dir: str, key: tuple[int, int]) -> str:
  s, w = key[0] * TILE_DEG - MARGIN_DEG, key[1] * TILE_DEG - MARGIN_DEG
  p = tile_path(out_dir, key)
  build(src, p, s, w, s + TILE_DEG + 2 * MARGIN_DEG, w + TILE_DEG + 2 * MARGIN_DEG)
  return p


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--src", required=True)
  ap.add_argument("--out", required=True, help="output directory (tiles)")
  ap.add_argument("--bbox", nargs=4, type=float, metavar=("S", "W", "N", "E"), required=True)
  a = ap.parse_args()
  s, w, n, e = a.bbox
  k0, k1 = tile_key(s, w), tile_key(n, e)
  for i in range(k0[0], k1[0] + 1):
    for j in range(k0[1], k1[1] + 1):
      print(build_tile(a.src, a.out, (i, j)))


if __name__ == "__main__":
  main()
