"""Local speed-limit overrides from driver-confirmed camera signs.

A HUD YES tap on the speed-sign popup writes NAPSpeedSignConfirmed
({mph, seq, lat, lon, bearing}). mapd consumes it once per seq, keys it to
the OSM way it is currently matched to (same db.lookup way / heading logic
that produces liveMapDataNAP.speedLimit) plus the travel heading, and stores
it here. From then on, whenever mapd matches that way in that direction it
publishes the override as speedLimit (source "sign"), so the existing
speed-limit sign and the Map Speed Limit cruise path (MAX = limit + offset)
use it on this and future drives. The opposite direction keeps OSM.
A newer YES on the same way + direction replaces the old value. Nothing is
uploaded (OSM edits are future work). NO taps never reach this file.

v2.3: the payload carries way_id + heading matched when the sign was READ
(liveSpeedSignNAP.proposedWayId / proposedHeading). Those are stored as-is;
the way the car is on at tap time is never used. A YES without a read-time
way stores nothing.

A YES whose mph already is the limit for that way + heading (the stored
override, or OSM when the car is matched to that way and there is none)
rewrites nothing; the sign itself is still recorded by two_stage.record_answer.

Both directions (Oct 2026): a YES now covers the read-time way from the
sign's position onward in the travel direction to the end of that way, and
that span applies to BOTH directions of travel. Rows carry
{"both": true, "from_s": m along the digitized way, "dir": +1|-1}; a row
without a sign position covers the whole way, both directions. Old
heading-only rows (no "both") still match within +-60 deg.

Oct 22 hygiene: rows written now carry "v": ENTRY_VERSION plus the read
lat/lon. A row with no "v" and no "both" was written by the old tap-time
keying (e.g. the Oct 14 US 12 eastbound 30 that covered 4.5 km of 60 road).
Those are LEGACY: kept in the file untouched for later verification, never
applied to speedLimit and never removed by a new Yes. mapd logs
"legacy override present: X mph vs OSM Y" when the car is on such a way
(selfdrive/mapd/list_legacy_overrides.py lists them all next to OSM).
A YES without a read-time way stores nothing (no tap-time matching).

NAPSpeedSignClearOverride (bool) removes the current rows that cover where
the car is now on the matched way. Legacy rows are never removed by it.
"""
from __future__ import annotations

import json
import math
import os
import time

PARAM = "NAPSpeedSignConfirmed"
CLEAR_PARAM = "NAPSpeedSignClearOverride"
ENTRY_VERSION = 2
FILENAME = "map_speed_overrides.json"
MPH_TO_MS = 0.44704
HEADING_MATCH_DEG = 60.0
PENDING_MAX_S = 20.0   # kept for imports; tap-time matching is gone
MAX_ENTRIES = 5000


def overrides_path() -> str:
  env = os.environ.get("NAP_MAP_SPEED_OVERRIDES")
  if env:
    return env
  base = "/data/media/0/nap" if os.path.isdir("/data/media/0") else os.path.join(
    os.path.expanduser("~"), ".comma", "media", "0", "nap")
  return os.path.join(base, FILENAME)


EARTH_R = 6371000.0


def _xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  return (math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R,
          math.radians(lat - lat0) * EARTH_R)


def _bearing(a, b) -> float:
  x, y = _xy(b[0], b[1], a[0], a[1])
  return math.degrees(math.atan2(x, y)) % 360.0


def along_way(lat: float, lon: float, coords) -> tuple[float, float] | None:
  """(meters from the way's first node to the closest point, that segment's bearing)."""
  if lat is None or lon is None or not coords or len(coords) < 2:
    return None
  best, best_s, best_h, s = 1e18, 0.0, 0.0, 0.0
  for i in range(len(coords) - 1):
    ax, ay = _xy(coords[i][0], coords[i][1], lat, lon)
    bx, by = _xy(coords[i + 1][0], coords[i + 1][1], lat, lon)
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    seg = math.sqrt(ab2)
    t = 0.0 if ab2 < 1e-6 else max(0.0, min(1.0, (-ax * abx - ay * aby) / ab2))
    d = math.hypot(ax + t * abx, ay + t * aby)
    if d < best:
      best, best_s, best_h = d, s + t * seg, _bearing(coords[i], coords[i + 1])
    s += seg
  return best_s, best_h


def heading_delta(a: float, b: float) -> float:
  d = abs(float(a) - float(b)) % 360.0
  return 360.0 - d if d > 180.0 else d


def parse_confirmed(raw) -> dict | None:
  if raw is None:
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
  try:
    if d.get("way_id") is not None and int(d["way_id"]) > 0:
      out["way_id"] = int(d["way_id"])
  except (TypeError, ValueError):
    pass
  for k in ("lat", "lon", "bearing", "heading"):
    try:
      if d.get(k) is not None:
        out[k] = float(d[k])
    except (TypeError, ValueError):
      pass
  return out


def _row_ok(e) -> bool:
  return isinstance(e, dict) and "way_id" in e and "mph" in e and "heading" in e


def is_legacy(e) -> bool:
  """Unversioned heading-only row from the tap-time keying. Never applied."""
  return _row_ok(e) and "v" not in e and not e.get("both")


def is_current(e) -> bool:
  return _row_ok(e) and not is_legacy(e)


class SpeedOverrides:
  def __init__(self, path: str | None = None) -> None:
    self.path = path or overrides_path()
    self.rows: list[dict] = []      # every row in the file, legacy included, file order
    self.entries: list[dict] = []   # rows that apply
    self.legacy: list[dict] = []    # unversioned tap-time rows: logged, never applied
    self._mtime = None
    self.last_seq: float | None = None
    self.pending: dict | None = None
    self.pending_t = 0.0
    self.load()

  def load(self) -> None:
    try:
      m = os.path.getmtime(self.path)
      if m == self._mtime:
        return
      with open(self.path) as f:
        data = json.load(f)
      self.rows = [e for e in data.get("overrides", []) if isinstance(e, dict)]
      self.entries = [e for e in self.rows if is_current(e)]
      self.legacy = [e for e in self.rows if is_legacy(e)]
      self._mtime = m
    except (OSError, ValueError):
      if not os.path.exists(self.path):
        self.rows, self.entries, self.legacy = [], [], []
        self._mtime = None

  def save(self) -> None:
    os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
    tmp = self.path + ".tmp"
    with open(tmp, "w") as f:
      # Legacy and unknown rows ride along untouched; only current rows are trimmed.
      keep = {id(e) for e in self.entries[-MAX_ENTRIES:]}
      cur = {id(e) for e in self.entries}
      rows = [e for e in self.rows if id(e) not in cur or id(e) in keep]
      json.dump({"version": ENTRY_VERSION, "overrides": rows}, f, separators=(",", ":"))
    os.replace(tmp, self.path)
    self._mtime = os.path.getmtime(self.path)

  def find(self, way_id: int, bearing: float | None, lat: float | None = None, lon: float | None = None,
           coords=None) -> dict | None:
    """Newest row covering this point of the way. Spans ignore the heading."""
    best = None
    pos = None
    for e in self.entries:
      if int(e["way_id"]) != int(way_id):
        continue
      if e.get("both"):
        if e.get("from_s") is None:
          best = e
          continue
        if pos is None:
          pos = along_way(lat, lon, coords) or (None, None)
        if pos[0] is not None and (pos[0] - float(e["from_s"])) * int(e.get("dir", 1)) >= -1.0:
          best = e
      elif bearing is not None and heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG:
        best = e  # newest wins (appended last)
    return best

  def find_legacy(self, way_id: int, bearing: float | None) -> dict | None:
    if bearing is None:
      return None
    best = None
    for e in self.legacy:
      try:
        if int(e["way_id"]) == int(way_id) and heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG:
          best = e
      except (KeyError, TypeError, ValueError):
        continue
    return best

  def _drop(self, gone) -> None:
    ids = {id(e) for e in gone}
    self.entries = [e for e in self.entries if id(e) not in ids]
    self.rows = [e for e in self.rows if id(e) not in ids]

  def clear(self, way_id: int, bearing: float | None, lat: float | None = None, lon: float | None = None,
            coords=None) -> list[dict]:
    """Remove every current row that applies here (repeat find). Legacy rows stay."""
    gone = []
    while True:
      e = self.find(way_id, bearing, lat, lon, coords)
      if e is None:
        break
      gone.append(e)
      self._drop([e])
    if gone:
      self.save()
    return gone

  def limit_ms(self, way_id: int, bearing: float | None, lat: float | None = None, lon: float | None = None,
               coords=None) -> float | None:
    e = self.find(way_id, bearing, lat, lon, coords)
    return None if e is None else int(e["mph"]) * MPH_TO_MS

  def span(self, heading: float, conf: dict, coords) -> tuple[float | None, int]:
    """(from_s, dir) for a sign at conf lat/lon read while heading `heading`."""
    pos = along_way(conf.get("lat"), conf.get("lon"), coords)
    if pos is None:
      return None, 1
    return round(pos[0], 1), (1 if heading_delta(heading, pos[1]) <= 90.0 else -1)

  def add(self, way_id: int, heading: float, conf: dict, road: str = "", now: float | None = None,
          coords=None) -> dict:
    now = time.time() if now is None else now  # noqa: TID251  wall clock in the stored row
    from_s, d = self.span(heading, conf, coords)

    def covered(e) -> bool:
      if int(e["way_id"]) != int(way_id):
        return False
      if not e.get("both"):
        return True        # the new span covers both directions of the old heading-only row
      if from_s is None:
        return True
      return e.get("from_s") is not None and int(e.get("dir", 1)) == d and (float(e["from_s"]) - from_s) * d >= -1.0
    self._drop([e for e in self.entries if covered(e)])
    e = {"v": ENTRY_VERSION, "way_id": int(way_id), "heading": round(float(heading), 1), "mph": int(conf["mph"]),
         "both": True, "from_s": from_s, "dir": d,
         "lat": conf.get("lat"), "lon": conf.get("lon"), "road": road, "t": round(now, 1), "source": "sign_yes"}
    self.entries.append(e)
    self.rows.append(e)
    self.save()
    return e

  def same_as_current(self, way_id: int, heading: float, mph: int, match=None, conf: dict | None = None,
                      coords=None) -> bool:
    """True when mph already is the limit mapd uses at the sign on this way."""
    conf = conf or {}
    e = self.find(way_id, heading, conf.get("lat"), conf.get("lon"), coords)
    if e is not None:
      return int(e["mph"]) == int(mph)
    if match is not None and int(getattr(match, "way_id", 0) or 0) == int(way_id):
      try:
        return round(float(match.speed_limit_ms) / MPH_TO_MS) == int(mph)
      except (AttributeError, TypeError, ValueError):
        return False
    return False

  def consume(self, raw_param, match, bearing: float | None, mono_now: float, way_coords=None) -> dict | None:
    """Turn a new YES (by seq) into an override on its read-time way.

    Returns the stored entry when one was written this call. `match` is only
    used for the read-time way's coords / OSM limit, never as the way.
    """
    conf = parse_confirmed(raw_param)
    if conf is not None and conf["seq"] != self.last_seq:
      self.last_seq = conf["seq"]
      if "way_id" in conf or "heading" in conf:
        # v2.3: read-time way + heading; never the current match.
        hdg = conf.get("heading", conf.get("bearing"))
        if conf.get("way_id") and hdg is not None:
          coords = way_coords(int(conf["way_id"])) if way_coords is not None else None
          if not coords and match is not None and int(getattr(match, "way_id", 0) or 0) == int(conf["way_id"]):
            coords = getattr(match, "coords", None)
          if self.same_as_current(int(conf["way_id"]), float(hdg), conf["mph"], match, conf, coords):
            return None
          return self.add(int(conf["way_id"]), float(hdg), conf, coords=coords)
        return None
    return None


def request_override(mph: int, rec: dict | None, seq: float, params=None,
                     way_id: int | None = None, heading: float | None = None) -> bool:
  """HUD YES: hand the confirmed mph (+ the way / heading it was READ on) to mapd."""
  payload: dict = {"mph": int(mph), "seq": float(seq)}
  if way_id is not None:
    payload["way_id"] = int(way_id)
    if heading is not None:
      payload["heading"] = float(heading)
  for k in ("lat", "lon", "bearing"):
    if (rec or {}).get(k) is not None:
      payload[k] = (rec or {})[k]
  if parse_confirmed(payload) is None:
    return False
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    params.put(PARAM, payload)
    return True
  except Exception:
    return False
