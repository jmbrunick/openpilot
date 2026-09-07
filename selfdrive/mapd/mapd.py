#!/usr/bin/env python3
"""GPS → OSM speed-limit lookup for NAP.

Publishes liveMapDataNAP. Does not actuate; card.py / the long planner consume
the limit through the existing vCruise path.
"""
import math
import os
import time

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB

MAPD_HZ = 2.0
GPS_MAX_AGE_S = 2.5
GPS_MAX_ACC_M = 50.0
RELOAD_PERIOD_S = 15.0


def _gps_sample(sm) -> tuple[float, float, float | None, bool]:
  """Return (lat, lon, bearing_deg or None, ok). Prefer external GNSS."""
  now = time.monotonic()
  for sock in ("gpsLocationExternal", "gpsLocation"):
    if sm.recv_frame.get(sock, -1) <= 0:
      continue
    if (now - sm.recv_time[sock]) > GPS_MAX_AGE_S:
      continue
    g = sm[sock]
    if abs(g.latitude) < 1e-6 and abs(g.longitude) < 1e-6:
      continue
    if g.horizontalAccuracy > GPS_MAX_ACC_M and g.horizontalAccuracy > 0:
      continue
    bearing = float(g.bearingDeg) if (g.speed > 1.0 or g.bearingDeg) else None
    if bearing is not None and (math.isnan(bearing) or bearing < 0):
      bearing = None
    return float(g.latitude), float(g.longitude), bearing, True
  return 0.0, 0.0, None, False


def _db_path(params: Params) -> str:
  try:
    raw = params.get("NAPMapSpeedDbPath") or ""
  except Exception:
    raw = ""
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="ignore")
  raw = str(raw).strip()
  return raw or default_db_path()


def main():
  params = Params()
  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
  pm = messaging.PubMaster(["liveMapDataNAP"])
  rk = Ratekeeper(MAPD_HZ, print_delay_threshold=None)

  db = OsmSpeedLimitDB(_db_path(params))
  last_reload = 0.0
  last_path = db.path

  cloudlog.info("mapd starting, db=%s", db.path)
  if not os.path.isfile(db.path):
    cloudlog.warning("mapd: no OSM sqlite at %s — Settings → NAP → Download US Maps (ODbL)", db.path)

  while True:
    sm.update(0)
    now = time.monotonic()
    if now - last_reload > RELOAD_PERIOD_S:
      path = _db_path(params)
      if path != last_path:
        db.close()
        db.path = path
        last_path = path
      db.open()
      last_reload = now

    lat, lon, bearing, gps_ok = _gps_sample(sm)
    match = db.lookup(lat, lon, bearing) if gps_ok and db.loaded else None

    msg = messaging.new_message("liveMapDataNAP")
    msg.valid = gps_ok and db.loaded and match is not None
    d = msg.liveMapDataNAP
    d.latitude = lat
    d.longitude = lon
    d.bearingDeg = float(bearing or 0.0)
    d.dbLoaded = bool(db.loaded)
    d.source = "osm"
    if match is not None:
      d.speedLimit = float(match.speed_limit_ms)
      d.speedLimitValid = True
      d.nextSpeedLimit = float(match.next_speed_limit_ms)
      d.nextSpeedLimitDistance = float(match.next_distance_m)
      d.roadName = match.road_name
      d.highway = match.highway
      d.wayId = int(match.way_id)
      d.matchDistance = float(match.distance_m)
    pm.send("liveMapDataNAP", msg)
    rk.keep_time()


if __name__ == "__main__":
  main()
