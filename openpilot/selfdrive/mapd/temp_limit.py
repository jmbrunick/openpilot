"""Temporary speed limit from the speed-sign prompt's Temp button.

A Temp tap (comma HUD or the MCU page) writes NAPSpeedSignTemp
({mph, seq, read_seq, way_id, heading, lat, lon, bearing}). mapd consumes it
once per seq and publishes the mph as the current limit (source "temp"), above
confirmed sign sections, Tesla and OSM, until the first of:

  sign   a different speed-limit sign is read (a new prompt, or a new sign
         placement, with a different mph). A new read of the SAME mph is a
         work-zone cue instead (repeat sign inside the zone).
  map    the map's limit changes: the limit mapd would publish without Temp
         (confirmed section / Tesla / OSM), raw OSM, or Tesla's base moves by
         MAP_CHANGE_MPH or more for MAP_CHANGE_TICKS ticks.
  road   the road changes or forks: the car leaves the same chain a confirmed
         sign would cover (speed_overrides.build_chain: same road from the
         sign forward in the read direction, stopping at a map-limit change, a
         road change or a fork; the length cap is raised to CHAIN_MAX_M so it
         never ends a Temp before the construction rule), or turns around on it. With no
         road at the read, the road mapd matches first (same name, or same
         class for unnamed roads) is the anchor. Leaving must last
         ROAD_OFF_TICKS ticks AND ROAD_OFF_M meters, so a 3 s map-match
         flicker onto a cross street (Oct 21 15:18:55 CDT, 304th Street) does
         not end it; map-limit changes are ignored on those off-road ticks.
  clear  no construction seen for NO_CUE_END_M of driving. Cues: a radar cone
         line (coneLineNAP active / left / right, not parked) or a same-mph
         repeat sign. Measured from the sign when no cue was ever seen.

Never persisted: nothing here writes map_speed_overrides.json, confirmed.jsonl,
auto-confirm or Tesla-learned rows. The param is cleared on manager start and
on the offroad transition, and mapd only runs onroad, so a Temp never outlives
the drive. Display + the existing Map Speed Limit path only; nothing here
brakes or steers by itself.

NO_CUE_END_M (Oct 23 2026, from the logged construction passes):
  Oct 10 17:30-17:33 CT (route 17c): ~4.7 km of cones with zero radar cone
    detections (pre-#302 detector), the longest cue-free stretch inside a zone.
  Oct 21 15:17-15:21 CDT: cones for ~25 s, then ~3.0 km of shifted lane with
    no cue before the next detection.
  Oct 7 / Oct 10 16:48 / Oct 11: continuous 5-20 s detections.
  5000 m covers the 4.7 km gap so a quiet stretch inside one zone does not end
  the Temp early, and still ends it ~3 min after the zone at 60 mph.
"""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field

from openpilot.selfdrive.mapd.speed_overrides import (
  _same_road,
  along_way,
  build_chain,
  chain_dist,
  heading_delta,
)

PARAM = "NAPSpeedSignTemp"
STATE_PARAM = "NAPSpeedSignTempState"   # mapd -> MCU page: {active, mph} / {active: false, end, end_t}
MPH_TO_MS = 0.44704
NO_CUE_END_M = 5000.0
MAP_CHANGE_MPH = 3
MAP_CHANGE_TICKS = 2       # 1 s at 2 Hz
ROAD_OFF_TICKS = 3         # 1.5 s at 2 Hz (GPS jitter at a junction)
ROAD_OFF_M = 100.0         # and this far driven off the road: a junction mismatch is not a road change
REVERSE_TICKS = 3
REVERSE_MIN_MS = 3.0
CHAIN_MAX_M = 50000.0      # far past NO_CUE_END_M: the length cap is never what ends a Temp
MAX_SIGN_BACK_M = 1000.0   # cap on "measured from the sign" when the read was far back
END_NOTE_S = 6.0           # UI shows why it ended this long

END_REASONS = {
  "sign": "new sign",
  "map": "limit changed",
  "road": "road changed",
  "clear": "no construction seen",
}


def end_text(reason: str) -> str:
  return END_REASONS.get(str(reason or ""), str(reason or ""))


def _mph(ms) -> int:
  try:
    v = float(ms)
  except (TypeError, ValueError):
    return 0
  return int(round(v / MPH_TO_MS)) if math.isfinite(v) and v > 0 else 0


def _dist_m(lat1, lon1, lat2, lon2) -> float:
  if None in (lat1, lon1, lat2, lon2):
    return 0.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = p2 - p1, math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * 6371000.0 * math.asin(min(1.0, math.sqrt(a)))


def parse_temp(raw) -> dict | None:
  if raw is None or raw == b"" or raw == "":
    return None
  try:
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
    mph = int(d["mph"])
    seq = float(d["seq"])
  except Exception:
    return None
  if not (5 <= mph <= 85) or mph % 5:
    return None
  out = {"mph": mph, "seq": seq}
  for k in ("read_seq", "way_id"):
    try:
      out[k] = int(d[k]) if d.get(k) is not None else 0
    except (TypeError, ValueError):
      out[k] = 0
  for k in ("heading", "lat", "lon", "bearing"):
    try:
      v = float(d[k]) if d.get(k) is not None else None
      out[k] = v if v is not None and math.isfinite(v) else None
    except (TypeError, ValueError):
      out[k] = None
  if out["lat"] is not None and out["lon"] is not None and out["lat"] == 0.0 and out["lon"] == 0.0:
    out["lat"] = out["lon"] = None
  return out


def request_temp(mph: int, rec: dict | None, read_seq=None, way_id=None, heading=None, params=None,
                 now: float | None = None) -> bool:
  """UI Temp tap: hand the mph (+ read-time way / heading / location) to mapd. Never saved."""
  payload: dict = {"mph": int(mph), "seq": float(time.monotonic() if now is None else now)}
  if read_seq is not None:
    payload["read_seq"] = int(read_seq)
  if way_id:
    payload["way_id"] = int(way_id)
  if heading is not None:
    payload["heading"] = float(heading)
  for k in ("lat", "lon", "bearing"):
    if (rec or {}).get(k) is not None:
      payload[k] = (rec or {})[k]
  if parse_temp(payload) is None:
    return False
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    params.put(PARAM, payload)
    return True
  except Exception:
    return False


@dataclass
class TempState:
  mph: int
  seq: float
  read_seq: int
  way_id: int
  heading: float | None
  chain: list | None = None           # build_chain spans, when the read-time way had geometry
  road: tuple | None = None           # (name, highway) anchor when there is no chain
  started_odo: float = 0.0
  last_cue_odo: float = 0.0
  base_under: int | None = None
  base_osm: int | None = None
  base_tesla: int | None = None
  map_ticks: int = 0
  off_ticks: int = 0
  off_odo: float | None = None        # odometer when the current off-road run began
  rev_ticks: int = 0
  cues: int = 0
  info: dict = field(default_factory=dict)


class TempLimit:
  def __init__(self, no_cue_m: float = NO_CUE_END_M) -> None:
    self.no_cue_m = float(no_cue_m)
    self.state: TempState | None = None
    self.last_seq: float | None = None
    self.odo = 0.0
    self._t: float | None = None
    self.end_reason = ""
    self.end_seq = 0
    self.last_end: dict | None = None
    self._prop_seq = -1
    self._sign_seq = -1

  @property
  def active(self) -> bool:
    return self.state is not None

  def limit_ms(self) -> float | None:
    return None if self.state is None else self.state.mph * MPH_TO_MS

  def no_cue_m_now(self) -> float:
    return 0.0 if self.state is None else max(0.0, self.odo - self.state.last_cue_odo)

  # --- inputs ---------------------------------------------------------------
  def advance(self, now: float, v_ego: float) -> None:
    dt = 0.0 if self._t is None else max(0.0, min(2.0, float(now) - self._t))
    self._t = float(now)
    v = float(v_ego) if v_ego is not None and math.isfinite(float(v_ego)) else 0.0
    self.odo += max(0.0, v) * dt

  def consume(self, raw, match, lat, lon, bearing, db=None, sign_live=None) -> TempState | None:
    """Start a Temp from the param, once per seq. Replaces any running Temp."""
    conf = parse_temp(raw)
    if conf is None or conf["seq"] == self.last_seq:
      return None
    self.last_seq = conf["seq"]
    way_id = int(conf.get("way_id") or 0)
    heading = conf.get("heading") if conf.get("heading") is not None else conf.get("bearing")
    if heading is None:
      heading = bearing
    st = TempState(mph=conf["mph"], seq=conf["seq"], read_seq=int(conf.get("read_seq") or 0),
                   way_id=way_id, heading=heading)
    slat = conf["lat"] if conf.get("lat") is not None else lat
    slon = conf["lon"] if conf.get("lon") is not None else lon
    coords = None
    if way_id > 0 and db is not None and hasattr(db, "way_coords"):
      try:
        coords = db.way_coords(way_id)
      except Exception:
        coords = None
    if not coords and match is not None and way_id > 0 and int(getattr(match, "way_id", 0)) == way_id:
      coords = getattr(match, "coords", None)
    if coords and slat is not None and heading is not None:
      pos = along_way(slat, slon, coords)
      if pos is not None:
        from_s, seg_h = pos
        d = 1 if heading_delta(heading, seg_h) <= 90.0 else -1
        try:
          st.chain = build_chain(way_id, from_s, d, coords, db, max_m=CHAIN_MAX_M)
        except Exception:
          st.chain = None
    if st.chain is None:
      if way_id > 0 and db is not None and hasattr(db, "way_info"):
        info = db.way_info(way_id)
        if info is not None:
          st.road = (info.get("name", ""), info.get("highway", ""))
      if st.road is None and match is not None:
        st.road = (getattr(match, "road_name", "") or "", getattr(match, "highway", "") or "")
    back = min(MAX_SIGN_BACK_M, _dist_m(slat, slon, lat, lon)) if slat is not None else 0.0
    st.started_odo = self.odo
    st.last_cue_odo = self.odo - back
    st.info = {"way_id": way_id, "chain_m": (max(max(sp[3], sp[4]) for sp in st.chain) if st.chain else None),
               "road": st.road, "back_m": round(back, 1)}
    self.state = st
    self._prop_seq = st.read_seq
    if sign_live is not None:
      try:
        self._sign_seq = int(sign_live.signSeq)
      except Exception:
        pass
    return st

  def note_sign(self, sign_live) -> None:
    """liveSpeedSignNAP: any new read with another mph ends the Temp; the same mph is a cue."""
    if sign_live is None:
      return
    try:
      pmph, pseq = int(sign_live.proposedMph), int(sign_live.proposedSeq)
      smph, sseq = int(sign_live.signMph), int(sign_live.signSeq)
    except Exception:
      return
    reads = []
    if pmph > 0 and pseq != self._prop_seq:
      self._prop_seq = pseq
      reads.append(pmph)
    if sseq > 0 and sseq != self._sign_seq:
      first = self._sign_seq < 0
      self._sign_seq = sseq
      if smph > 0 and not first:
        reads.append(smph)
    st = self.state
    if st is None:
      return
    for mph in reads:
      if mph != st.mph:
        self._end("sign", f"read {mph}")
        return
      self.note_cue()

  def note_cones(self, cone) -> None:
    if cone is None:
      return
    try:
      cue = ((bool(cone.active) and not bool(cone.barrier)) or bool(cone.leftActive) or bool(cone.rightActive)) \
        and not bool(cone.parked)
    except Exception:
      return
    if cue:
      self.note_cue()

  def note_cue(self) -> None:
    if self.state is not None:
      self.state.last_cue_odo = self.odo
      self.state.cues += 1

  # --- per tick -------------------------------------------------------------
  def tick(self, match, coords, lat, lon, bearing, v_ego, under_ms, osm_raw_mph=0,
           tesla_mph=0, tesla_valid=False) -> float | None:
    """Run the end checks. Returns the Temp limit (m/s) while it lasts, else None."""
    st = self.state
    if st is None:
      return None
    # road change / fork / turn-around (first, so a junction mismatch does not count as a map change)
    off = rev = False
    if match is not None:
      off, rev = self._road_check(st, match, coords, lat, lon, bearing, v_ego)
      st.off_ticks = st.off_ticks + 1 if off else 0
      st.off_odo = (st.off_odo if st.off_odo is not None else self.odo) if off else None
      st.rev_ticks = st.rev_ticks + 1 if rev else 0
      off_long = st.off_ticks >= ROAD_OFF_TICKS and self.odo - st.off_odo >= ROAD_OFF_M
      if off_long or st.rev_ticks >= REVERSE_TICKS:
        self._end("road", f"way={int(getattr(match, 'way_id', 0))} road={getattr(match, 'road_name', '')!r}"
                  + (" reversed" if rev else ""))
        return None
    # map's limit change (only while on the Temp's road)
    changed = False
    if match is not None and not off:
      under = _mph(under_ms)
      if under > 0:
        if st.base_under is None:
          st.base_under = under
        elif abs(under - st.base_under) >= MAP_CHANGE_MPH:
          changed = True
      osm = int(osm_raw_mph or 0)
      if osm > 0:
        if st.base_osm is None:
          st.base_osm = osm
        elif abs(osm - st.base_osm) >= MAP_CHANGE_MPH:
          changed = True
    if tesla_valid and int(tesla_mph or 0) > 0 and not off:
      if st.base_tesla is None:
        st.base_tesla = int(tesla_mph)
      elif abs(int(tesla_mph) - st.base_tesla) >= MAP_CHANGE_MPH:
        changed = True
    st.map_ticks = st.map_ticks + 1 if changed else 0
    if st.map_ticks >= MAP_CHANGE_TICKS:
      self._end("map", f"under={_mph(under_ms)} osm={int(osm_raw_mph or 0)} tesla={int(tesla_mph or 0)}")
      return None
    # construction no longer seen
    if self.odo - st.last_cue_odo >= self.no_cue_m:
      self._end("clear", f"{self.odo - st.last_cue_odo:.0f} m without a cue")
      return None
    return st.mph * MPH_TO_MS

  def _road_check(self, st: TempState, match, coords, lat, lon, bearing, v_ego) -> tuple[bool, bool]:
    way = int(getattr(match, "way_id", 0))
    if st.chain:
      spans = [sp for sp in st.chain if int(sp[0]) == way]
      if not spans or not coords:
        return True, False
      pos = along_way(lat, lon, coords)
      if pos is None:
        return False, False
      s, seg_h = pos
      if chain_dist({"chain": st.chain}, way, s) is None:
        return True, False
      sp = next((x for x in spans if float(x[1]) - 1.0 <= s <= float(x[2]) + 1.0), spans[0])
      span_dir = 1 if float(sp[4]) >= float(sp[3]) else -1
      rev = False
      if bearing is not None and float(v_ego or 0.0) >= REVERSE_MIN_MS:
        car_dir = 1 if heading_delta(bearing, seg_h) <= 90.0 else -1
        rev = car_dir != span_dir
      return False, rev
    if st.road is None:
      st.road = (getattr(match, "road_name", "") or "", getattr(match, "highway", "") or "")
      return False, False
    same = _same_road(st.road[0], st.road[1], {"name": getattr(match, "road_name", ""),
                                               "highway": getattr(match, "highway", "")})
    return (not same), False

  def _end(self, reason: str, detail: str = "") -> None:
    st = self.state
    if st is None:
      return
    self.state = None
    self.end_reason = reason
    self.end_seq += 1
    self.last_end = {"mph": st.mph, "reason": reason, "detail": detail,
                     "driven_m": round(self.odo - st.started_odo, 1),
                     "no_cue_m": round(self.odo - st.last_cue_odo, 1), "cues": st.cues}


def apply_to_msg(d, temp: TempLimit, temp_ms: float | None, nxt):
  """Publish the Temp on liveMapDataNAP. Returns the next limit to publish."""
  d.tempEndReason = temp.end_reason
  d.tempEndSeq = int(temp.end_seq)
  if temp_ms is None:
    return nxt
  d.tempActive = True
  d.tempMph = int(temp.state.mph)
  d.tempNoCueM = float(temp.no_cue_m_now())
  d.speedLimit = float(temp_ms)
  d.speedLimitValid = True
  d.source = "temp"
  if nxt is not None and abs(float(nxt[0]) - float(temp_ms)) < 0.5:
    return None    # the limit ahead equals the Temp: nothing to announce
  return nxt


def state_payload(temp: TempLimit, now: float) -> dict:
  if temp.state is not None:
    return {"active": True, "mph": int(temp.state.mph)}
  out: dict = {"active": False}
  if temp.end_reason:
    out.update(end=temp.end_reason, end_t=float(now))
  return out


def write_state(params, temp: TempLimit, now: float) -> None:
  """Start / end only (not every tick). Display on the MCU page."""
  try:
    params.put(STATE_PARAM, state_payload(temp, now))
  except Exception:
    pass


def read_state(params, now: float | None = None) -> dict | None:
  """MCU view: {"mph": n} while a Temp runs, {"ended": text} for END_NOTE_S after it ends, else None."""
  try:
    raw = params.get(STATE_PARAM)
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else (dict(raw) if raw else None)
  except Exception:
    return None
  if not isinstance(d, dict):
    return None
  if d.get("active"):
    try:
      return {"mph": int(d["mph"])}
    except (KeyError, TypeError, ValueError):
      return None
  if d.get("end"):
    now = time.monotonic() if now is None else float(now)
    if 0.0 <= now - float(d.get("end_t") or 0.0) <= END_NOTE_S:
      return {"ended": end_text(d["end"])}
  return None


def log_path() -> str:
  base = "/data/media/0/nap/speedsign_feedback" if os.path.isdir("/data/media/0") else os.path.join(
    os.path.expanduser("~"), ".comma", "media", "0", "nap", "speedsign_feedback")
  return os.path.join(base, "temp_limit.jsonl")


def log_event(row: dict, path: str | None = None) -> None:
  """Start / end rows for review. A log, never read back as a limit."""
  try:
    path = path or log_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a") as fh:
      fh.write(json.dumps(dict(row, t=round(time.time(), 3)), separators=(",", ":")) + "\n")  # noqa: TID251  wall time for the log
  except Exception:
    pass
