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
from openpilot.selfdrive.mapd.speed_overrides import PARAM as SIGN_CONFIRMED_PARAM, SpeedOverrides, effective_next
from openpilot.selfdrive.mapd.roundabout_map import PARAM_RING, RingCache, roundabout_comfort_speed_ms
from openpilot.selfdrive.mapd.roundabout_status import RingDataWatch, map_msg_valid
from openpilot.selfdrive.mapd.tesla_limit import PARAM_ENABLE as TESLA_PARAM, MapdTeslaHook
from openpilot.selfdrive.mapd import temp_limit

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
  sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation", "carState", "liveSpeedSignNAP", "coneLineNAP"])
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
  # NAPTeslaLimitEnable (default off): Tesla map limit + review flags. The
  # `can` socket only exists while it is on.
  tesla: MapdTeslaHook | None = None
  auto_confirm_seq = 0
  # Speed-sign Temp (NAPSpeedSignTemp): this drive only, in memory, never saved.
  temp = temp_limit.TempLimit()

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
      tesla_on = _param_bool(params, TESLA_PARAM)
      if tesla_on and tesla is None:
        try:
          tesla = MapdTeslaHook(messaging)
          cloudlog.info("mapd: Tesla map limit on")
        except Exception:
          cloudlog.exception("mapd: Tesla map limit init failed")
      elif not tesla_on and tesla is not None:
        tesla = None
        cloudlog.info("mapd: Tesla map limit off")
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
    end_seq0 = temp.end_seq
    try:
      temp.advance(now, _v_ego_ms(sm))
      sign_live = sm["liveSpeedSignNAP"] if sm.recv_frame.get("liveSpeedSignNAP", -1) > 0 else None
      raw_temp = params.get(temp_limit.PARAM)
      if raw_temp is not None:
        params.remove(temp_limit.PARAM)
        started = temp.consume(raw_temp, match, lat if gps_ok else None, lon if gps_ok else None,
                               bearing, db=db, sign_live=sign_live)
        if started is not None:
          cloudlog.info("mapd: temp limit %s mph start %s", started.mph, started.info)
          temp_limit.log_event({"event": "start", "mph": started.mph, "read_seq": started.read_seq, **started.info})
          temp_limit.write_state(params, temp, now)
      if sm.updated.get("liveSpeedSignNAP", False):
        temp.note_sign(sign_live)
      if sm.updated.get("coneLineNAP", False):
        temp.note_cones(sm["coneLineNAP"])
    except Exception:
      cloudlog.exception("mapd: temp limit input failed")
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
      d.speedLimit = float(sign_ms if sign_ms is not None else lim_ms)
      d.speedLimitValid = True
      d.source = map_source_for(est, sign=sign_ms is not None)
      key = (int(match.way_id), d.source, round(float(d.speedLimit), 1))
      if key != last_limit_log:
        last_limit_log = key
        src = ("default" if est else "tagged") + (" sign" if sign_ms is not None else "")
        msg_txt = " ".join((
          f"mapd_limit way={int(match.way_id)}", f"name={match.road_name!r}", f"hw={match.highway}",
          f"mph={float(d.speedLimit) * 2.23694:.0f}", f"raw_mph={float(match.speed_limit_ms) * 2.23694:.0f}",
          f"src={src}",
        ))
        cloudlog.error(msg_txt)
      # Next limit follows the effective limit ahead (confirmed sections, else OSM).
      osm_next = ((float(match.next_speed_limit_ms), float(match.next_distance_m))
                  if float(match.next_speed_limit_ms) > 0 else None)
      try:
        nxt = effective_next(overrides.entries, db, int(match.way_id),
                             match.coords or db.way_coords(int(match.way_id)), lat, lon, bearing,
                             float(d.speedLimit), sign_ms is not None, osm_next, osm_sign_lead_m(v_ego_ms))
      except Exception:
        cloudlog.exception("mapd: effective next limit failed")
        nxt = osm_next
      # Temp beats everything above (sign sections, Tesla, OSM) until it ends.
      try:
        temp_ms = temp.tick(match, match.coords or db.way_coords(int(match.way_id)), lat, lon, bearing,
                            _v_ego_ms(sm), float(d.speedLimit), osm_raw_mph,
                            int(tesla_st.base_mph) if tesla_st is not None else 0,
                            bool(tesla_st.valid) if tesla_st is not None else False)
        nxt = temp_limit.apply_to_msg(d, temp, temp_ms, nxt)
      except Exception:
        cloudlog.exception("mapd: temp limit failed")
      d.nextSpeedLimit = float(nxt[0]) if nxt is not None else 0.0
      d.nextSpeedLimitDistance = float(nxt[1]) if nxt is not None else 0.0
      d.roadName = match.road_name
      d.highway = match.highway
      d.wayId = int(match.way_id)
      d.matchDistance = float(match.distance_m)
    else:
      try:
        temp.tick(None, None, lat, lon, bearing, _v_ego_ms(sm), 0.0)
        temp_limit.apply_to_msg(d, temp, None, None)
      except Exception:
        cloudlog.exception("mapd: temp limit failed")
    if temp.end_seq != end_seq0 and temp.last_end is not None:
      cloudlog.info("mapd: temp limit ended %s", temp.last_end)
      temp_limit.log_event({"event": "end", **temp.last_end})
      temp_limit.write_state(params, temp, now)
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
