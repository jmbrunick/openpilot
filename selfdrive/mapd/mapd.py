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
from openpilot.selfdrive.mapd.gps_fix import (
  LAST_GPS_WRITE_PERIOD_S,
  gps_sample_from_sm,
  persist_last_gps_position,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB

MAPD_HZ = 2.0
RELOAD_PERIOD_S = 15.0


def _db_path(params: Params) -> str:
  try:
    raw = params.get("NAPMapSpeedDbPath") or ""
  except Exception:
    raw = ""
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="ignore")
  raw = str(raw).strip()
  return raw or default_db_path()


def _v_ego_ms(sm) -> float:
  """Wheel vEgo when carState is live; else GNSS speed. OSM 1.5 s lag offset."""
  if sm.recv_frame.get("carState", -1) > 0:
    try:
      v = float(sm["carState"].vEgo)
      if math.isfinite(v) and v > 0.0:
        return v
    except Exception:
      pass
  for sock in ("gpsLocationExternal", "gpsLocation"):
    if sm.recv_frame.get(sock, -1) <= 0:
      continue
    try:
      v = float(sm[sock].speed)
    except Exception:
      continue
    if math.isfinite(v) and v > 0.0:
      return v
  return 0.0


def main():
  params = Params()
  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation", "carState"])
  pm = messaging.PubMaster(["liveMapDataNAP"])
  rk = Ratekeeper(MAPD_HZ, print_delay_threshold=None)

  db = OsmSpeedLimitDB(_db_path(params))
  last_reload = 0.0
  last_gps_write = 0.0
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

    lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=now)
    if gps_ok and (last_gps_write == 0.0 or (now - last_gps_write) >= LAST_GPS_WRITE_PERIOD_S):
      persist_last_gps_position(params, lat, lon)
      last_gps_write = now
    v_ego_ms = _v_ego_ms(sm) if gps_ok else 0.0
    match = db.lookup(lat, lon, bearing, v_ego_ms=v_ego_ms) if gps_ok and db.loaded else None
    ix = db.lookup_intersection(lat, lon, bearing, v_ego_ms=v_ego_ms) if gps_ok and db.loaded else None

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
    if ix is not None:
      d.intersectionDistance = float(ix.distance_m)
      d.intersectionHasLeft = bool(ix.has_left)
      d.intersectionHasRight = bool(ix.has_right)
      d.intersectionLeftSpeed = float(ix.left_speed_ms)
      d.intersectionRightSpeed = float(ix.right_speed_ms)
    pm.send("liveMapDataNAP", msg)
    rk.keep_time()


if __name__ == "__main__":
  main()
