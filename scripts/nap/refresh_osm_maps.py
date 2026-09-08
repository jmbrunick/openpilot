#!/usr/bin/env python3
"""Refresh installed US OSM maps with live Overpass data around the vehicle.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright

Run on the device (offroad, Wi-Fi):

  python -m scripts.nap.refresh_osm_maps

Queries OSM maxspeed ways within 100 miles (~160.9 km) of a current GNSS fix
or last stored GPS, then merges those way_ids into the already-installed
`/data/media/0/osm/speed_limits.sqlite`. The rest of the US pack is kept.

This is not a published-pack / maps-index.json download. For first install of
the full US sqlite use:

  python -m scripts.nap.fetch_osm_maps
"""
from openpilot.selfdrive.mapd.local_refresh import main

if __name__ == "__main__":
  raise SystemExit(main())
