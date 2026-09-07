#!/usr/bin/env python3
"""Build a NAP OSM speed-limit sqlite from Geofabrik PBF / Overpass / GeoJSON.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

This is a PC tool. Do not commit the sqlite to git. Attach the zstd file to
GitHub Release tag osm-us-speed-limits-v1 (asset speed_limits_us.sqlite.zst).

US tagged-maxspeed ways ~3.4M (Taginfo 2026-09). After polyline simplify,
expect roughly 0.8–1.5 GB sqlite and 200–500 MB zstd — one US-wide asset.

Examples:

  # After: osmium tags-filter us-latest.osm.pbf w/highway w/maxspeed -o us-ms.osm.pbf
  python scripts/nap/build_osm_speed_limits.py --pbf us-ms.osm.pbf --out speed_limits_us.sqlite
  zstd -19 speed_limits_us.sqlite -o speed_limits_us.sqlite.zst

  python scripts/nap/build_osm_speed_limits.py --from-json overpass.json --out speed_limits.sqlite
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from openpilot.selfdrive.mapd.maps_manifest import ATTRIBUTION, LICENSE, LICENSE_URL
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import ways_from_overpass
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed


def _ways_from_pyosmium(pbf: str) -> list[dict]:
  try:
    import osmium  # type: ignore
  except ImportError as e:
    raise RuntimeError(
      "Reading .osm.pbf needs pyosmium (pip install osmium) or convert with osmium-tool first."
    ) from e

  out: list[dict] = []

  class H(osmium.SimpleHandler):
    def way(self, w):
      if "highway" not in w.tags:
        return
      ms = parse_maxspeed(w.tags.get("maxspeed"))
      if ms is None:
        ms = parse_maxspeed(w.tags.get("maxspeed:forward"))
      if ms is None:
        return
      coords = []
      try:
        for n in w.nodes:
          if not n.location.valid():
            return
          coords.append((float(n.lat), float(n.lon)))
      except osmium.InvalidLocationError:
        return
      if len(coords) < 2:
        return
      out.append({
        "way_id": int(w.id),
        "name": w.tags.get("name") or w.tags.get("ref") or "",
        "highway": w.tags.get("highway") or "",
        "maxspeed_ms": ms,
        "coords": coords,
      })

  H().apply_file(pbf, locations=True)
  return out


def write_db(path: str, ways: list[dict], extra_meta: dict | None = None) -> int:
  con = OsmSpeedLimitDB.create(path)
  n = 0
  for w in ways:
    OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"])
    n += 1
    if n % 100000 == 0:
      print(f"  {n} ways…", file=sys.stderr, flush=True)
      con.commit()
  meta = {
    "attribution": ATTRIBUTION,
    "license": LICENSE,
    "license_url": LICENSE_URL,
    "way_count": str(n),
  }
  if extra_meta:
    meta.update({str(k): str(v) for k, v in extra_meta.items()})
  con.executemany("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", list(meta.items()))
  con.commit()
  con.close()
  return n


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description="Build NAP OSM speed-limit sqlite (ODbL)")
  p.add_argument("--pbf", help="OSM PBF (ideally pre-filtered to highway+maxspeed)")
  p.add_argument("--from-json", help="Overpass JSON")
  p.add_argument("--out", required=True, help="Output sqlite path")
  p.add_argument("--zst", action="store_true", help="Also write <out>.zst via Python zstandard")
  args = p.parse_args(argv)

  if args.from_json:
    with open(args.from_json, encoding="utf-8") as f:
      payload = json.load(f)
    ways = ways_from_overpass(payload)
    source = os.path.basename(args.from_json)
  elif args.pbf:
    print(f"Reading {args.pbf} (pyosmium)…", file=sys.stderr)
    ways = _ways_from_pyosmium(args.pbf)
    source = os.path.basename(args.pbf)
  else:
    p.error("provide --pbf or --from-json")

  print(f"Writing {len(ways)} ways → {args.out}", file=sys.stderr)
  n = write_db(args.out, ways, extra_meta={"source": source})
  sz = os.path.getsize(args.out) if os.path.isfile(args.out) else 0
  print(f"Wrote {n} ways, {sz / 1e6:.0f} MB sqlite (ODbL, {ATTRIBUTION})")
  if args.zst:
    import zstandard as zstd
    zst_path = args.out + ".zst"
    cctx = zstd.ZstdCompressor(level=19)
    with open(args.out, "rb") as inf, open(zst_path, "wb") as outf:
      cctx.copy_stream(inf, outf)
    zsz = os.path.getsize(zst_path)
    print(f"Wrote {zst_path} ({zsz / 1e6:.0f} MB). Attach to GitHub Release {os.path.basename(zst_path)}.")
  if n == 0:
    return 2
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
