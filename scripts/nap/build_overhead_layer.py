#!/usr/bin/env python3
"""Build the NAP overhead layer (pack schema 6 table ``overhead``) on the box.

Tags, on drivable roads only:
  bridge    a bridge way (bridge=* not no, any kind: road, rail, foot, cycle)
            at a higher layer that crosses a drivable road segment
  gantry    man_made=gantry node or way within 15 m of a drivable road
  signal    highway=traffic_signals node that is a vertex of a drivable road
  confirmed scripts/nap/data/overhead_confirmed.json (box-side clearance-gate
            confirmations, reviewed)
Not streetlights, not power lines.

One Geofabrik state at a time; the PBF is deleted after use unless --keep.
OpenStreetMap data is ODbL: (c) OpenStreetMap contributors.

  python scripts/nap/build_overhead_layer.py --states minnesota --into speed_limits_us.sqlite
  python scripts/nap/build_overhead_layer.py --states minnesota --out overhead_mn.sqlite
  python scripts/nap/build_overhead_layer.py --pbf local.osm.pbf --out test.sqlite --keep
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sqlite3
import urllib.request

from openpilot.selfdrive.nap.path_object import overhead_map as OM

DRIVABLE = {"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
            "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street"}
GEOFABRIK = "https://download.geofabrik.de/north-america/us/{state}-latest.osm.pbf"
GRID_DEG = 0.01
GANTRY_SNAP_M = 15.0
CONFIRMED = os.path.join(os.path.dirname(__file__), "data", "overhead_confirmed.json")
# id spaces so OSM ids from nodes, ways and confirmed points never collide
ID_SIGNAL, ID_GANTRY, ID_BRIDGE, ID_CONFIRMED = 1 << 40, 2 << 40, 3 << 40, 4 << 40


def _layer(tags) -> int:
  try:
    lay = int(float(tags.get("layer", "")))
  except ValueError:
    lay = None
  bridge = tags.get("bridge", "no") not in ("no", "")
  if lay is None:
    lay = 1 if bridge else 0
  return lay


def _bearing(a, b) -> float:
  la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
  y = math.sin(lo2 - lo1) * math.cos(la2)
  x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
  return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def _xy(p, lat0):
  k = math.radians(1) * OM.EARTH_R_M
  return ((p[1]) * k * math.cos(math.radians(lat0)), p[0] * k)


def _seg_x(a, b, c, d):
  """Intersection (t along ab) of 2D segments ab and cd, or None."""
  r = (b[0] - a[0], b[1] - a[1])
  s = (d[0] - c[0], d[1] - c[1])
  den = r[0] * s[1] - r[1] * s[0]
  if abs(den) < 1e-12:
    return None
  q = (c[0] - a[0], c[1] - a[1])
  t = (q[0] * s[1] - q[1] * s[0]) / den
  u = (q[0] * r[1] - q[1] * r[0]) / den
  return t if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0 else None


def _cells(a, b):
  for gx in range(int(math.floor(min(a[1], b[1]) / GRID_DEG)), int(math.floor(max(a[1], b[1]) / GRID_DEG)) + 1):
    for gy in range(int(math.floor(min(a[0], b[0]) / GRID_DEG)), int(math.floor(max(a[0], b[0]) / GRID_DEG)) + 1):
      yield gx, gy


def extract(pbf: str) -> list[tuple]:
  import osmium  # pyosmium

  signals: set[int] = set()
  gantry_pts: list[tuple[int, float, float]] = []
  roads: dict = {}        # cell -> [(way_id, layer, a, b, node_a, node_b)]
  bridges: list = []      # (way_id, layer, coords, node_ids)
  sig_hits: dict = {}     # node id -> (lat, lon, way_id, heading)

  class H(osmium.SimpleHandler):
    def node(self, n):
      t = n.tags
      if t.get("highway") == "traffic_signals":
        signals.add(n.id)
      if t.get("man_made") == "gantry" and n.location.valid():
        gantry_pts.append((n.id, n.location.lat, n.location.lon))

    def way(self, w):
      t = w.tags
      try:
        pts = [(nd.lat, nd.lon, nd.ref) for nd in w.nodes if nd.location.valid()]
      except osmium.InvalidLocationError:
        return
      if len(pts) < 2:
        return
      if t.get("man_made") == "gantry":
        mid = pts[len(pts) // 2]
        gantry_pts.append((w.id, mid[0], mid[1]))
      hw = t.get("highway", "")
      is_bridge = t.get("bridge", "no") not in ("no", "") or t.get("man_made") == "bridge"
      if is_bridge and t.get("man_made") != "bridge":
        bridges.append((w.id, _layer(t), [(p[0], p[1]) for p in pts], {p[2] for p in pts}))
      if hw in DRIVABLE and t.get("area") != "yes":
        lay = _layer(t)
        for i in range(len(pts) - 1):
          a, b = pts[i], pts[i + 1]
          for c in _cells(a, b):
            roads.setdefault(c, []).append((w.id, lay, (a[0], a[1]), (b[0], b[1]), a[2], b[2]))
          if a[2] in signals and a[2] not in sig_hits:
            sig_hits[a[2]] = (a[0], a[1], w.id, _bearing(a, b))
        last = pts[-1]
        if last[2] in signals and last[2] not in sig_hits:
          sig_hits[last[2]] = (last[0], last[1], w.id, _bearing(pts[-2], last))

  H().apply_file(pbf, locations=True, idx="flex_mem")
  out: list[tuple] = []
  for nid, (la, lo, wid, hd) in sig_hits.items():
    out.append((ID_SIGNAL | nid, la, lo, "signal", wid, hd, "osm"))
  # bridges over lower drivable roads
  seen = set()
  for bwid, blay, coords, bnodes in bridges:
    for i in range(len(coords) - 1):
      a, b = coords[i], coords[i + 1]
      cands = set()
      for c in _cells(a, b):
        cands.update(roads.get(c, ()))
      lat0 = a[0]
      A, B = _xy(a, lat0), _xy(b, lat0)
      for rwid, rlay, ra, rb, na, nb in cands:
        if rwid == bwid or rlay >= blay or na in bnodes or nb in bnodes:
          continue
        t = _seg_x(A, B, _xy(ra, lat0), _xy(rb, lat0))
        if t is None:
          continue
        key = (bwid, rwid)
        if key in seen:
          continue
        seen.add(key)
        la, lo = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        out.append((ID_BRIDGE | (len(seen) << 1), la, lo, "bridge", rwid, _bearing(ra, rb), f"osm:way/{bwid}"))
  # gantries snapped to the nearest drivable road
  for gid, la, lo in gantry_pts:
    best = None
    for c in _cells((la, lo), (la, lo)):
      for rwid, _rlay, ra, rb, _na, _nb in roads.get(c, ()):
        P, A, B = _xy((la, lo), la), _xy(ra, la), _xy(rb, la)
        dx, dy = B[0] - A[0], B[1] - A[1]
        L2 = dx * dx + dy * dy or 1e-9
        t = max(0.0, min(1.0, ((P[0] - A[0]) * dx + (P[1] - A[1]) * dy) / L2))
        dist = math.hypot(P[0] - (A[0] + t * dx), P[1] - (A[1] + t * dy))
        if dist <= GANTRY_SNAP_M and (best is None or dist < best[0]):
          best = (dist, rwid, _bearing(ra, rb))
    if best is not None:
      out.append((ID_GANTRY | gid, la, lo, "gantry", best[1], best[2], "osm"))
  return out


def confirmed() -> list[tuple]:
  data = json.load(open(CONFIRMED))
  return [(ID_CONFIRMED | int(p["id"]), float(p["lat"]), float(p["lon"]), "confirmed", 0,
           float(p["heading_deg"]) if p.get("heading_deg") is not None else None, "clearance") for p in data["points"]]


def write(con: sqlite3.Connection, rows: list[tuple], set_schema: bool) -> None:
  OM.ensure_table(con)
  con.executemany(f"INSERT OR REPLACE INTO {OM.TABLE} VALUES(?,?,?,?,?,?,?)", rows)
  try:
    con.execute("CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY, value TEXT NOT NULL)")
    if set_schema:
      con.execute("INSERT OR REPLACE INTO meta VALUES('pack_schema','6')")
    con.execute("INSERT OR REPLACE INTO meta VALUES('overhead_count',?)",
                (str(con.execute(f"SELECT COUNT(*) FROM {OM.TABLE}").fetchone()[0]),))
  except sqlite3.Error:
    pass
  con.commit()


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--states", default="", help="comma list of Geofabrik US state slugs, e.g. minnesota")
  ap.add_argument("--pbf", action="append", default=[], help="local .osm.pbf (repeatable)")
  ap.add_argument("--into", help="existing pack sqlite to add the table to (sets pack_schema 6)")
  ap.add_argument("--out", help="new standalone sqlite with only the overhead table")
  ap.add_argument("--keep", action="store_true", help="keep downloaded PBFs")
  ap.add_argument("--tmp", default="/tmp")
  a = ap.parse_args()
  if bool(a.into) == bool(a.out):
    ap.error("give exactly one of --into / --out")
  con = sqlite3.connect(a.into or a.out)
  counts: dict = {}
  sources = [(p, False) for p in a.pbf] + [(s.strip(), True) for s in a.states.split(",") if s.strip()]
  for src, download in sources:
    path = src
    if download:
      path = os.path.join(a.tmp, f"{src}-latest.osm.pbf")
      print(f"downloading {src} ...", flush=True)
      urllib.request.urlretrieve(GEOFABRIK.format(state=src), path)
    try:
      rows = extract(path)
    finally:
      if download and not a.keep:
        os.remove(path)
    write(con, rows, bool(a.into))
    for r in rows:
      counts[r[3]] = counts.get(r[3], 0) + 1
    print(src, dict(counts), flush=True)
  rows = confirmed()
  write(con, rows, bool(a.into))
  counts["confirmed"] = len(rows)
  print("overhead rows:", counts)
  con.close()
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
