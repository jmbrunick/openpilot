"""NAP curve pack: mapped road geometry for the map curve preview (longitudinal only).

A separate, versioned sqlite per 0.25 deg tile. v2 is built from raw OSM
(every node, no Douglas-Peucker), resampled every STEP_M along each drivable
way. Only positions and headings are stored: curvature is computed at run
time over the walked road (geometry.py), so turns where two ways join come
out the same as turns inside a way.

Rebuild (any region, on a PC; needs `pip install osmium`):
  python -m openpilot.selfdrive.nap.curve_map.pack --pbf state-latest.osm.pbf --out DIR --bbox S W N E
"""
from __future__ import annotations

import argparse
import math
import os
import sqlite3

PACK_VERSION = 2
TILE_DEG = 0.25
STEP_M = 5.0
MARGIN_DEG = 0.02
EARTH_R = 6371000.0
DRIVABLE = frozenset({
  "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
  "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "road",
})


def tile_key(lat: float, lon: float) -> tuple[int, int]:
  return int(math.floor(lat / TILE_DEG)), int(math.floor(lon / TILE_DEG))


def tile_path(out_dir: str, key: tuple[int, int]) -> str:
  return os.path.join(out_dir, f"nap_curves_v{PACK_VERSION}_{key[0]}_{key[1]}.sqlite")


def tile_bbox(key: tuple[int, int]) -> tuple[float, float, float, float]:
  s, w = key[0] * TILE_DEG - MARGIN_DEG, key[1] * TILE_DEG - MARGIN_DEG
  return s, w, s + TILE_DEG + 2 * MARGIN_DEG, w + TILE_DEG + 2 * MARGIN_DEG


def resample(coords, step: float = STEP_M) -> list[tuple[float, float, float]]:
  """[(lat, lon, heading_deg)] every `step` m along the polyline; the last point is the true end."""
  if len(coords) < 2:
    return []
  lat0, lon0 = coords[0]
  c = math.cos(math.radians(lat0))
  P = [(math.radians(lo - lon0) * EARTH_R * c, math.radians(la - lat0) * EARTH_R) for la, lo in coords]
  seg = []
  for (x0, y0), (x1, y1) in zip(P[:-1], P[1:], strict=True):
    L = math.hypot(x1 - x0, y1 - y0)
    if L > 1e-3:
      seg.append((x0, y0, x1, y1, L, math.atan2(x1 - x0, y1 - y0)))
  if not seg:
    return []
  total = sum(s[4] for s in seg)
  n = max(1, int(math.ceil(total / step)))
  out = []
  i, s0 = 0, 0.0
  for j in range(n + 1):
    s = min(total, j * step)
    while i < len(seg) - 1 and s0 + seg[i][4] < s:
      s0 += seg[i][4]
      i += 1
    x0, y0, x1, y1, L, h = seg[i]
    f = min(1.0, max(0.0, (s - s0) / L))
    x, y = x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    out.append((lat0 + math.degrees(y / EARTH_R), lon0 + math.degrees(x / (EARTH_R * c)), math.degrees(h) % 360.0))
  return out


def _new_db(tmp: str) -> sqlite3.Connection:
  if os.path.exists(tmp):
    os.remove(tmp)
  os.makedirs(os.path.dirname(tmp) or ".", exist_ok=True)
  o = sqlite3.connect(tmp)
  o.executescript("""
    CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
    CREATE TABLE pts (id INTEGER PRIMARY KEY, way_id INTEGER NOT NULL, seq INTEGER NOT NULL,
                      lat REAL NOT NULL, lon REAL NOT NULL, hdg REAL NOT NULL);
    CREATE INDEX pts_way ON pts(way_id, seq);
    CREATE VIRTUAL TABLE pts_rtree USING rtree(id, min_lat, max_lat, min_lon, max_lon);
  """)
  return o


def write_tile(ways, out_path: str, bbox, source: str) -> int:
  """ways: iterable of (way_id, [(lat, lon), ...]) already filtered to drivable."""
  s, w, n, e = bbox
  tmp = out_path + ".partial"
  o = _new_db(tmp)
  pid = 0
  for way_id, coords in ways:
    if not any(s <= la <= n and w <= lo <= e for la, lo in coords):
      continue
    for seq, (la, lo, hd) in enumerate(resample(coords)):
      pid += 1
      o.execute("INSERT INTO pts VALUES (?,?,?,?,?,?)", (pid, int(way_id), seq, la, lo, round(hd, 1)))
      o.execute("INSERT INTO pts_rtree VALUES (?,?,?,?,?)", (pid, la, la, lo, lo))
  o.executemany("INSERT INTO meta VALUES (?,?)", [("version", str(PACK_VERSION)), ("bbox", f"{s},{w},{n},{e}"),
                                                  ("step_m", str(STEP_M)), ("source", source)])
  o.commit()
  o.close()
  os.replace(tmp, out_path)
  return pid


def read_pbf_ways(pbf: str, bbox):
  """Drivable ways touching bbox from a raw OSM extract (full node geometry)."""
  import osmium  # PC-side tool only; the device never builds packs

  s, w, n, e = bbox
  ways = []
  fp = (osmium.FileProcessor(pbf, osmium.osm.NODE | osmium.osm.WAY).with_locations()
        .with_filter(osmium.filter.EntityFilter(osmium.osm.WAY)).with_filter(osmium.filter.KeyFilter("highway")))
  for obj in fp:
    if obj.tags.get("highway", "") not in DRIVABLE:
      continue
    try:
      coords = [(nd.lat, nd.lon) for nd in obj.nodes]
    except osmium.InvalidLocationError:
      continue
    if any(s <= la <= n and w <= lo <= e for la, lo in coords):
      ways.append((obj.id, coords))
  return ways


def main() -> None:
  ap = argparse.ArgumentParser()
  ap.add_argument("--pbf", required=True, nargs="+", help="raw OSM extract(s)")
  ap.add_argument("--out", required=True, help="output directory (tiles)")
  ap.add_argument("--bbox", nargs=4, type=float, metavar=("S", "W", "N", "E"), required=True)
  a = ap.parse_args()
  s, w, n, e = a.bbox
  k0, k1 = tile_key(s, w), tile_key(n, e)
  keys = [(i, j) for i in range(k0[0], k1[0] + 1) for j in range(k0[1], k1[1] + 1)]
  outer = (k0[0] * TILE_DEG - MARGIN_DEG, k0[1] * TILE_DEG - MARGIN_DEG,
           (k1[0] + 1) * TILE_DEG + MARGIN_DEG, (k1[1] + 1) * TILE_DEG + MARGIN_DEG)
  ways = {}
  for p in a.pbf:
    for wid, coords in read_pbf_ways(p, outer):
      ways[wid] = coords
  for key in keys:
    path = tile_path(a.out, key)
    write_tile(ways.items(), path, tile_bbox(key), ",".join(os.path.basename(p) for p in a.pbf))
    print(path)


if __name__ == "__main__":
  main()
