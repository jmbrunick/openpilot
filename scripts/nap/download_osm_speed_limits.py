#!/usr/bin/env python3
"""Build a small-region OSM speed-limit SQLite DB for NAP mapd (Overpass).

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

For the full United States after flash, use the GitHub Release download instead:

  python -m scripts.nap.fetch_osm_maps

  # or Settings → NAP → Download US Maps

On the 3X, Settings → NAP → Map Speed Limit → Refresh maps overlays live OSM
within 100 miles of the vehicle onto that US sqlite (see
python -m scripts.nap.refresh_osm_maps). This script is the PC / bbox builder.

To *publish* a US Release from a PC (Geofabrik PBF), see scripts/nap/build_osm_speed_limits.py
and docs-nap/map-speed.md.

Small bbox / Overpass examples (run on a PC):

  # Bounding box (south,west,north,east)
  python scripts/nap/download_osm_speed_limits.py --bbox 37.6,-122.5,37.9,-122.2 \\
      --out /tmp/speed_limits.sqlite

  # Center + radius
  python scripts/nap/download_osm_speed_limits.py --lat 37.7749 --lon -122.4194 --radius-km 25

On the device the file belongs at:
  /data/media/0/osm/speed_limits.sqlite

scp example:
  scp speed_limits.sqlite comma@<dongle>:/data/media/0/osm/speed_limits.sqlite
"""
from __future__ import annotations

import argparse
import json
import sys

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import (
  OVERPASS_URL,
  bbox_from_center,
  fetch_overpass,
  ways_from_overpass,
)


def write_db(path: str, ways: list[dict], extra_meta: dict | None = None) -> int:
  con = OsmSpeedLimitDB.create(path)
  n = 0
  for w in ways:
    OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"])
    n += 1
  meta = {"way_count": str(n)}
  if extra_meta:
    meta.update({str(k): str(v) for k, v in extra_meta.items()})
  con.executemany("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", list(meta.items()))
  con.commit()
  con.close()
  return n


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description="Download OSM maxspeed ways into a NAP mapd sqlite DB")
  p.add_argument("--bbox", help="south,west,north,east")
  p.add_argument("--lat", type=float)
  p.add_argument("--lon", type=float)
  p.add_argument("--radius-km", type=float, default=20.0)
  p.add_argument("--from-json", help="Use a saved Overpass JSON instead of the network")
  p.add_argument("--out", default=default_db_path(), help="Output sqlite path")
  p.add_argument("--overpass-url", default=OVERPASS_URL)
  args = p.parse_args(argv)

  if args.from_json:
    with open(args.from_json, encoding="utf-8") as f:
      payload = json.load(f)
    bbox_s = "file"
  else:
    if args.bbox:
      parts = [float(x.strip()) for x in args.bbox.split(",")]
      if len(parts) != 4:
        p.error("--bbox must be south,west,north,east")
      bbox = (parts[0], parts[1], parts[2], parts[3])
    elif args.lat is not None and args.lon is not None:
      bbox = bbox_from_center(args.lat, args.lon, args.radius_km)
    else:
      p.error("provide --bbox or --lat/--lon (or --from-json)")
    print(f"Overpass bbox={bbox} …", file=sys.stderr)
    payload = fetch_overpass(bbox, url=args.overpass_url)
    bbox_s = ",".join(str(x) for x in bbox)

  ways = ways_from_overpass(payload)
  n = write_db(args.out, ways, extra_meta={"bbox": bbox_s})
  print(f"Wrote {n} ways → {args.out}")
  if n == 0:
    print("No maxspeed ways found. Try a larger bbox or a denser OSM region.", file=sys.stderr)
    return 2
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
