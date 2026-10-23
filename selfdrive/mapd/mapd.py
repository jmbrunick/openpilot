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
from openpilot.selfdrive.mapd.constants import osm_sign_lead_m
from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.gps_fix import (
  LAST_GPS_WRITE_PERIOD_S,
  gps_sample_from_sm,
  persist_last_gps_position,
)
from openpilot.selfdrive.mapd.map_speed_policy import keep_rural_default_ms, map_source_for
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB
from openpilot.selfdrive.mapd.speed_overrides import PARAM as SIGN_CONFIRMED_PARAM, SpeedOverrides, along_way, effective_next
from openpilot.selfdrive.mapd.roundabout_map import PARAM_RING, RingCache, roundabout_comfort_speed_ms
from openpilot.selfdrive.mapd.roundabout_status import RingDataWatch, map_msg_valid
from openpilot.selfdrive.mapd.tesla_limit import (
  PARAM_ENABLE as TESLA_PARAM, PARAM_PRIMARY as TESLA_PRIMARY_PARAM, MapdTeslaHook, next_for_published, select_current,
)

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


def _param_bool(params: Params, key: str) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def main():
  params = Params()
  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation", "carState", "liveSpeedSignNAP"])
  pm = messaging.PubMaster(["liveMapDataNAP"])
  rk = Ratekeeper(MAPD_HZ, print_delay_threshold=None)

  db = OsmSpeedLimitDB(_db_path(params))
  last_reload = 0.0
  last_gps_write = 0.0
  rural_prev = None
  last_limit_log = None
  last_path = db.path

  overrides = SpeedOverrides()
  ring_cache = RingCache()
  ring_watch = RingDataWatch()
  # Tesla map limit CAN read. Runs while NAPTeslaLimitPrimary (default on:
  # Tesla sets the current limit) or NAPTeslaLimitEnable (log + review notes)
  # is on. The `can` socket only exists while one of them is on.
  tesla: MapdTeslaHook | None = None
  auto_confirm_seq = 0

  def _publish_ring(payload: dict) -> None:
    try:
      params.put(PARAM_RING, payload)
    except Exception:
      cloudlog.exception("mapd: NAPRoundaboutRing write failed")

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
      overrides.load()
      last_reload = now
      tesla_log_on = _param_bool(params, TESLA_PARAM)
      tesla_primary_on = _param_bool(params, TESLA_PRIMARY_PARAM)
      tesla_on = tesla_log_on or tesla_primary_on
      if tesla_on and tesla is None:
        try:
          tesla = MapdTeslaHook(messaging)
          cloudlog.info("mapd: Tesla map limit on")
        except Exception:
          cloudlog.exception("mapd: Tesla map limit init failed")
      elif not tesla_on and tesla is not None:
        tesla = None
        cloudlog.info("mapd: Tesla map limit off")
      if tesla is not None:
        tesla.flags_on = tesla_log_on
        tesla.primary_on = tesla_primary_on
      if db.loaded:
        note = ring_watch.check(db.path)
        if note is not None:
          getattr(cloudlog, note[0])(note[1])

    lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=now)
    if gps_ok and (last_gps_write == 0.0 or (now - last_gps_write) >= LAST_GPS_WRITE_PERIOD_S):
      persist_last_gps_position(params, lat, lon)
      last_gps_write = now
    v_ego_ms = _v_ego_ms(sm) if gps_ok else 0.0
    match = db.lookup(lat, lon, bearing, v_ego_ms=v_ego_ms) if gps_ok and db.loaded else None
    # Driver-confirmed sign (HUD YES): key it to this way + heading, once.
    try:
      raw_conf = params.get(SIGN_CONFIRMED_PARAM)
    except Exception:
      raw_conf = None
    if raw_conf is not None or overrides.pending is not None:
      try:
        added = overrides.consume(raw_conf, match, bearing, now, way_coords=db.way_coords, db=db)
        if raw_conf is not None:
          params.remove(SIGN_CONFIRMED_PARAM)
        if added is not None and str(added.get("source", "")).startswith("sign_auto"):
          auto_confirm_seq += 1
        if added is not None:
          cloudlog.info("mapd: sign override way=%s heading=%s mph=%s", added["way_id"], added["heading"], added["mph"])
      except Exception:
        cloudlog.exception("mapd: sign override failed")
    # Span rows need the way polyline; a match with a next limit can arrive
    # without coords, so fall back to the stored way.
    sign_ms = (overrides.limit_ms(int(match.way_id), bearing, lat, lon, match.coords or db.way_coords(int(match.way_id)))
               if match is not None else None)

    tesla_st = None
    if tesla is not None:
      try:
        tesla.update_can()
        tesla_st = tesla.state(now)
        tesla.note_sign(sm["liveSpeedSignNAP"] if sm.recv_frame.get("liveSpeedSignNAP", -1) > 0 else None, now)
      except Exception:
        cloudlog.exception("mapd: Tesla map limit update failed")

    msg = messaging.new_message("liveMapDataNAP")
    msg.valid = map_msg_valid(gps_ok=gps_ok, db_loaded=db.loaded, matched=match is not None, ring_hint=False)
    d = msg.liveMapDataNAP
    d.latitude = lat
    d.longitude = lon
    d.bearingDeg = float(bearing or 0.0)
    d.dbLoaded = bool(db.loaded)
    d.source = "osm"
    d.autoConfirmSeq = auto_confirm_seq
    osm_raw_mph = 0
    learn_way, learn_s, learned = None, None, None
    if tesla_st is not None:
      d.teslaBaseMph = int(tesla_st.base_mph)
      d.teslaMppMph = int(tesla_st.mpp_mph)
      d.teslaConf = int(tesla_st.conf)
      d.teslaValid = bool(tesla_st.valid)
    if match is not None:
      est = bool(getattr(match, "estimated", False))
      lim_ms = float(match.speed_limit_ms)
      if sign_ms is None:
        # Oct 13: untagged rural tertiary/secondary stays 55 along the same road.
        lim_ms = keep_rural_default_ms(lim_ms, estimated=est, highway=match.highway,
                                       road_name=match.road_name, prev=rural_prev)
      rural_prev = (match.road_name, match.highway, lim_ms)
      # Raw OSM (tagged or road-type default), never the sign override.
      osm_raw_mph = int(round(lim_ms * 2.23694)) if lim_ms > 0 else 0
      d.osmRawMph = osm_raw_mph
      # Current limit: confirmed sign > Tesla base > active Tesla-learned
      # section > OSM (Tesla parts: NAPTeslaLimitPrimary).
      tesla_mph, learned_ms = 0, None
      if tesla is not None:
        try:
          tesla_mph = tesla.primary_mph(now, (match.road_name, str(match.highway or "").replace("_link", "")))
          learned = tesla.learned_store()
          if learned is not None:
            aw = along_way(lat, lon, match.coords or db.way_coords(int(match.way_id)))
            learn_way, learn_s = int(match.way_id), (aw[0] if aw is not None else None)
            learned_ms = learned.limit_ms(learn_way, learn_s)
        except Exception:
          cloudlog.exception("mapd: Tesla primary limit failed")
      cur_ms, cur_src = select_current(sign_ms, tesla_mph, lim_ms, learned_ms)
      d.speedLimit = float(cur_ms)
      d.speedLimitValid = True
      d.source = cur_src or map_source_for(est)
      key = (int(match.way_id), d.source, round(float(d.speedLimit), 1))
      if key != last_limit_log:
        last_limit_log = key
        src = ("default" if est else "tagged") + (" sign" if sign_ms is not None else "")
        msg_txt = " ".join((
          f"mapd_limit way={int(match.way_id)}", f"name={match.road_name!r}", f"hw={match.highway}",
          f"mph={float(d.speedLimit) * 2.23694:.0f}", f"raw_mph={float(match.speed_limit_ms) * 2.23694:.0f}",
          f"src={src}", f"pub={d.source}", f"tesla={tesla_mph}",
        ))
        cloudlog.error(msg_txt)
      # Next limit follows the effective limit ahead: confirmed sections >
      # active Tesla-learned sections > OSM. Live Tesla has no lookahead, so
      # this is walked from the sign / learned / OSM value, never the live
      # Tesla one (a Tesla-vs-OSM mismatch here is not a change ahead); then a
      # next equal to the published value is dropped.
      osm_next = ((float(match.next_speed_limit_ms), float(match.next_distance_m))
                  if float(match.next_speed_limit_ms) > 0 else None)
      walk_ms = float(sign_ms if sign_ms is not None else lim_ms)
      try:
        nxt = effective_next(overrides.entries, db, int(match.way_id),
                             match.coords or db.way_coords(int(match.way_id)), lat, lon, bearing,
                             walk_ms, sign_ms is not None, osm_next, osm_sign_lead_m(v_ego_ms),
                             learned=learned.limit_ms if learned is not None and learned.has_active() else None)
      except Exception:
        cloudlog.exception("mapd: effective next limit failed")
        nxt = osm_next
      if cur_src in ("tesla", "tesla_learned"):
        nxt = next_for_published(nxt, float(d.speedLimit))
      d.nextSpeedLimit = float(nxt[0]) if nxt is not None else 0.0
      d.nextSpeedLimitDistance = float(nxt[1]) if nxt is not None else 0.0
      d.roadName = match.road_name
      d.highway = match.highway
      d.wayId = int(match.way_id)
      d.matchDistance = float(match.distance_m)
    # Same road as the published speed limit. Freeway / trunk matches drop
    # nearby residential loops (I-74 Maritime / Seaway false RB).
    current_hw = match.highway if match is not None else None
    rb = db.find_roundabout(lat, lon, bearing, current_highway=current_hw, v_ego_ms=v_ego_ms,
                            current_coords=(match.coords if match is not None else None)) if gps_ok and db.loaded else None
    if rb is not None and (rb.on_roundabout or rb.approaching):
      d.onRoundabout = bool(rb.on_roundabout)
      d.approachingRoundabout = bool(rb.approaching)
      d.roundaboutDistance = float(rb.distance_m)
      d.roundaboutWayId = int(rb.way_id)
      # Ring geometry (center / radius / sense) for controlsd via param, and a
      # ring speed that keeps the inner lane ≤ 2.5 m/s² (OSM maxspeed still caps).
      geom = ring_cache.update(db, int(rb.way_id), lat, lon, now, publish=_publish_ring)
      if geom is not None:
        d.roundaboutSpeedLimit = float(roundabout_comfort_speed_ms(
          geom.radius_m, geom.lanes, max(float(rb.speed_limit_ms), float(geom.maxspeed_ms))))
      else:
        d.roundaboutSpeedLimit = float(rb.speed_limit_ms)
      # A ring has no maxspeed to match. Keep the message valid for its hint.
      msg.valid = map_msg_valid(gps_ok=gps_ok, db_loaded=db.loaded, matched=match is not None, ring_hint=True)
    if tesla is not None:
      try:
        # Learn where the debounced Tesla base disagrees with raw OSM (way None = off the map: closes the section).
        tesla.learn_tick(now, time.time(), learn_way, learn_s, osm_raw_mph, v_ego_ms,  # noqa: TID251  wall time for pass gaps
                         lat, lon, bearing, match.road_name if match is not None else "")
      except Exception:
        cloudlog.exception("mapd: Tesla learned limits failed")
      try:
        flag = tesla.flag_tick(now, time.time(), lat, lon, bearing, _v_ego_ms(sm),  # noqa: TID251  wall time in the flag line
                               match.road_name if match is not None else "", osm_raw_mph, gps_ok)
        if flag is not None:
          d.reviewFlag = True
          d.reviewReason = flag["reason"]
          cloudlog.info("mapd: review flag %s %s", flag["kind"], flag["reason"])
      except Exception:
        cloudlog.exception("mapd: review flag failed")
    pm.send("liveMapDataNAP", msg)
    rk.keep_time()


if __name__ == "__main__":
  main()
