#!/usr/bin/env python3
"""Check the published US OSM speed-limit index and download if newer.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Run on the device (Wi-Fi):

  python -m scripts.nap.refresh_osm_maps

This is a version check + download of the GitHub maps-index.json (not Overpass).
If maps are not installed yet, it performs the same first install as
`python -m scripts.nap.fetch_osm_maps`.
"""
import sys

from openpilot.selfdrive.mapd.fetch_maps import main

if __name__ == "__main__":
  raise SystemExit(main(["--refresh", *sys.argv[1:]]))
