#!/usr/bin/env python3
"""Build an offline OSM speed-limit SQLite DB for NAP mapd.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Examples (run on a PC, then copy the sqlite file to the comma 3X):

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
import math
import sys
import urllib.parse
import urllib.request

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.overpass import ways_from_overpass

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
USER_AGENT = "NotAutopilot-mapd/1.0 (OSM ODbL; https://github.com/NotAutopilot/openpilot)"


def bbox_from_center(lat: float, lon: float, radius_km: float) -> tuple[float, float, float, float]:
  dlat = radius_km / 111.0
  dlon = radius_km / (111.0 * max(0.2, abs(math.cos(math.radians(lat)))))
  return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def overpass_query(south: float, west: float, north: float, east: float) -> str:
  return f"""
[out:json][timeout:180];
way["highway"]["maxspeed"]({south},{west},{north},{east});
out geom;
""".strip()


def fetch_overpass(bbox: tuple[float, float, float, float], url: str = OVERPASS_URL) -> dict:
  q = overpass_query(*bbox)
  data = urllib.parse.urlencode({"data": q}).encode()
  req = urllib.request.Request(url, data=data, headers={"User-Agent": USER_AGENT})
  with urllib.request.urlopen(req, timeout=240) as resp:
    return json.loads(resp.read().decode("utf-8"))


def write_db(path: str, ways: list[dict], extra_meta: dict | None = None) -> int:
  con = OsmSpeedLimitDB.create(path)
  n = 0
  for w in ways:
    OsmSpeedLimitDB.insert_way(con, w["way_id"], w["name"], w["highway"], w["maxspeed_ms"], w["coords"])
    n += 1
  if extra_meta:
    con.executemany("INSERT OR REPLACE INTO meta(key, value) VALUES (?, ?)", list(extra_meta.items()))
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
    payload = fetch_overpass(bbox)
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
