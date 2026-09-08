#!/usr/bin/env python3
"""Download the prebuilt US OSM speed-limit DB onto comma 3X.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Run on the device (Wi-Fi):

  python -m scripts.nap.fetch_osm_maps

To check for a published refresh without a full first-time download:

  python -m scripts.nap.refresh_osm_maps

Or from a PC, then scp the sqlite (see --out).
"""
from openpilot.selfdrive.mapd.fetch_maps import main

if __name__ == "__main__":
  raise SystemExit(main())
