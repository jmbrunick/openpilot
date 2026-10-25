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

Sign sections (Oct 22 2026): OSM way boundaries are ignored (a way can be
100 m between driveways). With the map loaded, a YES stores a "chain": the
read-time way from the sign forward in the travel direction, then each
continuing way of the same road (same name; unnamed roads: same class),
while the map's limit equals the value the YES replaced (the map's limit on
the sign's way), stopping at the first way whose map limit differs,
CHAIN_MAX_M, a road change, or a fork with more than one same-road
continuation (and, at lookup, at the next confirmed sign). Never behind the sign. Each piece carries the distance from
the sign, and a point covered by several signs uses the NEAREST sign behind
it along its chain (ties: newest), so a newer sign ends the previous span at
its position, an upstream re-confirm never erases downstream signs, and the
section applies to both directions of travel. Chain rows win over legacy rows
and over OSM maxspeed. Every YES is written (no "already the limit" skip),
so a 50 that matches a short way's OSM tag still carries forward.

Oct 22 hygiene (ported Oct 24): rows written now carry "v": ENTRY_VERSION.
A row with no "v", no "both" and no "chain" was written by the old tap-time
keying (e.g. the Oct 14 US 12 eastbound 30 that covered 4.5 km of 60 road).
Those are LEGACY: kept in the file untouched for later verification, never
applied to speedLimit, never removed by a new Yes or by Clear. mapd logs
"legacy override present: X mph vs OSM Y" when the car is on such a way
(selfdrive/mapd/list_legacy_overrides.py lists them all next to OSM).
A YES without a read-time way stores nothing (no tap-time matching).
NAPSpeedSignClearOverride (bool) removes the current rows (sign chains
included) that cover where the car is now.
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
CHAIN_MAX_M = 10000.0  # cap on how far one confirmed sign reaches
JOIN_TOL_M = 5.0       # way end nodes closer than this are connected
SAME_SIGN_M = 40.0     # a YES this close (same direction) replaces that sign's row


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


def _poly_len(coords) -> float:
  return sum(math.hypot(*_xy(coords[i + 1][0], coords[i + 1][1], coords[i][0], coords[i][1]))
             for i in range(len(coords) - 1))


def _near(a, b, tol: float = JOIN_TOL_M) -> bool:
  return math.hypot(*_xy(a[0], a[1], b[0], b[1])) <= tol


def _same_road(name: str, highway: str, w: dict) -> bool:
  a, b = (name or "").strip().lower(), (w.get("name") or "").strip().lower()
  if a or b:
    return a == b
  return (highway or "").strip().lower() == (w.get("highway") or "").strip().lower()


def _mph(ms) -> int:
  try:
    return int(round(float(ms) / MPH_TO_MS))
  except (TypeError, ValueError):
    return 0


def build_chain(way_id: int, from_s: float, d: int, coords, db, max_m: float = CHAIN_MAX_M) -> list[list]:
  """[[way_id, s0, s1, dist_at_s0, dist_at_s1], ...] from the sign forward (d = +1 along the
  digitized way, -1 against it) through same-road continuations. Stops at max_m, a road change
  (no continuation) or a fork (more than one)."""
  L = _poly_len(coords)
  from_s = max(0.0, min(L, float(from_s)))
  if d > 0:
    spans = [[int(way_id), from_s, L, 0.0, L - from_s]]
    acc, end = L - from_s, coords[-1]
  else:
    spans = [[int(way_id), 0.0, from_s, from_s, 0.0]]
    acc, end = from_s, coords[0]
  if acc >= max_m:
    s0, s1, a0, a1 = spans[0][1:]
    spans[0] = ([way_id, s0, s0 + max_m, 0.0, max_m] if d > 0 else [way_id, s1 - max_m, s1, max_m, 0.0])
    return spans
  info = db.way_info(int(way_id)) if db is not None and hasattr(db, "way_info") else None
  if info is None or not hasattr(db, "ways_at"):
    return spans
  name, hw = info.get("name", ""), info.get("highway", "")
  replaced = _mph(info.get("maxspeed_ms"))   # the map's limit at the sign: the value this YES replaces
  seen = {int(way_id)}
  while acc < max_m:
    cands = [w for w in db.ways_at(end[0], end[1], JOIN_TOL_M)
             if int(w["way_id"]) not in seen and _same_road(name, hw, w)]
    if len(cands) != 1:
      break           # road change (none) or ambiguous fork (several)
    w = cands[0]
    if _mph(w.get("maxspeed_ms")) != replaced:
      break           # the map's own limit changes here: the next posted section starts
    c = w["coords"]
    wl = _poly_len(c)
    r = min(wl, max_m - acc)
    if _near(c[0], end):
      spans.append([int(w["way_id"]), 0.0, r, acc, acc + r])
      end = c[-1]
    else:
      spans.append([int(w["way_id"]), wl - r, wl, acc + r, acc])
      end = c[0]
    seen.add(int(w["way_id"]))
    acc += r
  return spans


def chain_dist(e: dict, way_id: int, s: float | None) -> float | None:
  """Distance from e's sign along its chain to point s of way_id; None if not covered."""
  for sp in e.get("chain") or ():
    if int(sp[0]) != int(way_id):
      continue
    s0, s1, a0, a1 = float(sp[1]), float(sp[2]), float(sp[3]), float(sp[4])
    if s is None or not (s0 - 1.0 <= s <= s1 + 1.0):
      continue
    f = 0.0 if s1 - s0 < 1e-6 else (min(max(s, s0), s1) - s0) / (s1 - s0)
    return a0 + f * (a1 - a0)
  return None


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
  if d.get("auto") in AUTO_SOURCES:
    out["auto"] = str(d["auto"])
  return out


# Auto-confirm sources (NAPSignAutoConfirm). The stored row says which map agreed.
AUTO_SOURCES = ("tesla", "osm")


def row_source(conf: dict) -> str:
  return f"sign_auto_{conf['auto']}" if conf.get("auto") in AUTO_SOURCES else "sign_yes"


def _row_ok(e) -> bool:
  return isinstance(e, dict) and "way_id" in e and "mph" in e and "heading" in e


def is_legacy(e) -> bool:
  """Unversioned heading-only row from the tap-time keying. Kept, never applied."""
  return _row_ok(e) and "v" not in e and not e.get("both") and not e.get("chain")


def is_current(e) -> bool:
  return _row_ok(e) and not is_legacy(e)


class SpeedOverrides:
  def __init__(self, path: str | None = None) -> None:
    self.path = path or overrides_path()
    self.entries: list[dict] = []   # applied rows (sign chains, spans)
    self.legacy: list[dict] = []    # kept, never applied
    self.rows: list[dict] = []      # every row in the file, legacy/unknown included, file order
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
      # Legacy and unknown rows ride along untouched; only applied rows are trimmed.
      keep = {id(e) for e in self.entries[-MAX_ENTRIES:]}
      cur = {id(e) for e in self.entries}
      rows = [e for e in self.rows if id(e) not in cur or id(e) in keep]
      json.dump({"version": ENTRY_VERSION, "overrides": rows}, f, separators=(",", ":"))
    os.replace(tmp, self.path)
    self._mtime = os.path.getmtime(self.path)

  def find(self, way_id: int, bearing: float | None, lat: float | None = None, lon: float | None = None,
           coords=None) -> dict | None:
    """Row that sets the limit at this point. Sign chains first: the nearest sign behind the point
    along its chain (ties: newest). Else the newest legacy row covering it. Spans ignore the heading."""
    pos = None
    if any(e.get("chain") for e in self.entries):
      pos = along_way(lat, lon, coords) or (None, None)
      best_c, best_d = None, 1e18
      for e in self.entries:
        if not e.get("chain"):
          continue
        dd = chain_dist(e, way_id, pos[0])
        if dd is not None and dd <= best_d:
          best_c, best_d = e, dd
      if best_c is not None:
        return best_c
    best = None
    for e in self.entries:
      if e.get("chain"):
        continue
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
    """Newest legacy row on this way within +-60 deg (log only; never applied)."""
    best = None
    for e in self.legacy:
      if int(e["way_id"]) == int(way_id) and (bearing is None or heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG):
        best = e
    return best

  def _append(self, e: dict) -> None:
    self.entries.append(e)
    self.rows.append(e)

  def _drop(self, gone) -> None:
    ids = {id(e) for e in gone}
    if ids:
      self.entries = [e for e in self.entries if id(e) not in ids]
      self.rows = [e for e in self.rows if id(e) not in ids]

  def clear(self, way_id: int, bearing: float | None, lat: float | None = None, lon: float | None = None,
            coords=None) -> list[dict]:
    """Remove the applied rows (sign chains included) covering this point. Legacy rows stay."""
    self.load()
    pos = along_way(lat, lon, coords) or (None, None)
    gone = []
    for e in self.entries:
      if e.get("chain"):
        if chain_dist(e, way_id, pos[0]) is not None:
          gone.append(e)
        continue
      if int(e["way_id"]) != int(way_id):
        continue
      if e.get("both"):
        if e.get("from_s") is None or pos[0] is None or \
           (pos[0] - float(e["from_s"])) * int(e.get("dir", 1)) >= -1.0:
          gone.append(e)
      elif bearing is not None and heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG:
        gone.append(e)
    if gone:
      self._drop(gone)
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
          coords=None, db=None) -> dict:
    now = time.time() if now is None else now  # noqa: TID251  wall clock in the stored row
    from_s, d = self.span(heading, conf, coords)
    if from_s is not None and coords:
      chain = build_chain(int(way_id), from_s, d, coords, db)
      ways = {int(sp[0]) for sp in chain}

      def replaced(e) -> bool:
        if e.get("chain"):     # the same sign confirmed again (same spot, same direction)
          return (e.get("lat") is not None and conf.get("lat") is not None
                  and _near((e["lat"], e["lon"]), (conf["lat"], conf["lon"]), SAME_SIGN_M)
                  and heading_delta(e.get("heading", 0.0), heading) <= HEADING_MATCH_DEG)
        return int(e["way_id"]) in ways   # older span rows on ways this section now covers
      self._drop([e for e in self.entries if replaced(e)])   # legacy rows are not in entries: kept
      e = {"v": ENTRY_VERSION, "way_id": int(way_id), "heading": round(float(heading), 1), "mph": int(conf["mph"]),
           "both": True, "from_s": from_s, "dir": d, "chain": [[sp[0]] + [round(float(x), 1) for x in sp[1:]] for sp in chain],
           "lat": conf.get("lat"), "lon": conf.get("lon"), "road": road, "t": round(now, 1), "source": row_source(conf)}
      self._append(e)
      self.save()
      return e

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
         "lat": conf.get("lat"), "lon": conf.get("lon"), "road": road, "t": round(now, 1), "source": row_source(conf)}
    self._append(e)
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

  def consume(self, raw_param, match, bearing: float | None, mono_now: float, way_coords=None, db=None) -> dict | None:
    """Turn a new YES (by seq) into an override on its read-time way (never the tap-time match).

    Returns the stored entry when one was written this call.
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
          if not (coords and conf.get("lat") is not None) and \
             self.same_as_current(int(conf["way_id"]), float(hdg), conf["mph"], match, conf, coords):
            return None   # legacy whole-way row only; a sign section is always written
          return self.add(int(conf["way_id"]), float(hdg), conf, coords=coords, db=db)
        return None
    return None   # no read-time way: store nothing (tap-time matching made the legacy rows)


def request_override(mph: int, rec: dict | None, seq: float, params=None,
                     way_id: int | None = None, heading: float | None = None,
                     auto: str | None = None) -> bool:
  """HUD YES: hand the confirmed mph (+ the way / heading it was READ on) to mapd.

  auto="tesla" / "osm" marks a speedsignd auto-confirm (NAPSignAutoConfirm):
  same override path, the row is stored with source sign_auto_<auto>.
  """
  payload: dict = {"mph": int(mph), "seq": float(seq)}
  if auto is not None:
    if auto not in AUTO_SOURCES:
      return False
    payload["auto"] = auto
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


# --- Effective next limit (Oct 22 2026) -------------------------------------
# nextSpeedLimit / nextSpeedLimitDistance follow the EFFECTIVE limit ahead
# (confirmed section where one covers the road, else OSM). Override events:
# a section starting ahead, a section ending (hands back to OSM) and one
# section handing to another. An OSM change inside a section is masked.
# A plain OSM -> OSM change keeps mapd's existing OSM next (its stub filters
# and GNSS-lag lead are unchanged).
NEXT_STEP_M = 5.0
NEXT_MAX_M = 600.0   # = constants.LOOKAHEAD_MAX_M


def _point_at(coords, s: float) -> tuple[float, float]:
  acc = 0.0
  for i in range(len(coords) - 1):
    a, b = coords[i], coords[i + 1]
    seg = math.hypot(*_xy(b[0], b[1], a[0], a[1]))
    if acc + seg >= s or i == len(coords) - 2:
      f = 0.0 if seg < 1e-6 else max(0.0, min(1.0, (s - acc) / seg))
      return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
    acc += seg
  return coords[-1]


def chain_at(entries, way_id: int, s: float) -> dict | None:
  best, best_d = None, 1e18
  for e in entries:
    if not e.get("chain"):
      continue
    dd = chain_dist(e, way_id, s)
    if dd is not None and dd <= best_d:
      best, best_d = e, dd
  return best


def route_ahead(db, way_id: int, coords, lat: float, lon: float, bearing: float,
                max_m: float = NEXT_MAX_M) -> list[tuple[dict, float, float, int, float]]:
  """Pieces ahead on the same road: [(way_info, s_from, s_to, dir, dist_at_s_from)].
  Stops at max_m, a road change or a fork (same rule as sign sections)."""
  info = db.way_info(int(way_id)) if db is not None and hasattr(db, "way_info") else None
  if info is None or not coords:
    return []
  pos = along_way(lat, lon, coords)
  if pos is None:
    return []
  s, seg_h = pos
  L = _poly_len(coords)
  d = 1 if heading_delta(bearing, seg_h) <= 90.0 else -1
  rem = (L - s) if d > 0 else s
  pieces = [(info, s, s + d * min(rem, max_m), d, 0.0)]
  acc, end = rem, (coords[-1] if d > 0 else coords[0])
  seen = {int(way_id)}
  name, hw = info.get("name", ""), info.get("highway", "")
  while acc < max_m and hasattr(db, "ways_at"):
    cands = [w for w in db.ways_at(end[0], end[1], JOIN_TOL_M)
             if int(w["way_id"]) not in seen and _same_road(name, hw, w)]
    if len(cands) != 1:
      break
    w = cands[0]
    c = w["coords"]
    wl = _poly_len(c)
    r = min(wl, max_m - acc)
    if _near(c[0], end):
      pieces.append((w, 0.0, r, 1, acc))
      end = c[-1]
    else:
      pieces.append((w, wl, wl - r, -1, acc))
      end = c[0]
    seen.add(int(w["way_id"]))
    acc += wl
  return pieces


def effective_next(entries, db, way_id: int, coords, lat: float, lon: float, bearing: float | None,
                   cur_ms: float, cur_is_sign: bool, osm_next: tuple[float, float] | None,
                   lead_m: float = 0.0, learned=None) -> tuple[float, float] | None:
  """(next effective limit m/s, distance m) ahead, or None when it does not change.

  osm_next is mapd's OSM next (limit, distance) or None. lead_m is the GNSS-lag
  lead mapd already subtracts from OSM next distances; override events get the same.

  learned (NAPTeslaLimitPrimary, Oct 23 2026) is an optional callable
  (way_id, s) -> m/s or None: an ACTIVE Tesla-learned section. Priority at each
  point ahead: confirmed sign section > learned section > OSM. A learned value
  is an event like a sign section (start / end / change); it never masks a
  confirmed section. cur_ms / cur_is_sign describe the sign or OSM value here,
  never the live Tesla value (Tesla has no lookahead)."""
  has_chain = any(e.get("chain") for e in entries)
  if bearing is None or (not has_chain and learned is None):
    return osm_next
  pieces = route_ahead(db, way_id, coords, lat, lon, float(bearing))
  if not pieces:
    return osm_next

  def at(w, s, osm_ms):
    e = chain_at(entries, int(w["way_id"]), s) if has_chain else None
    if e is not None:
      return int(e["mph"]) * MPH_TO_MS, True
    if learned is not None:
      lm = learned(int(w["way_id"]), s)
      if lm is not None:
        return float(lm), True
    return osm_ms, False

  prev_ms, prev_sign = float(cur_ms), bool(cur_is_sign)
  if not prev_sign:   # compare OSM to OSM (the published value may be a rural-default hold)
    w0, s0 = pieces[0][0], pieces[0][1]
    prev_ms = float(w0.get("maxspeed_ms") or prev_ms)
    lm = learned(int(w0["way_id"]), s0) if learned is not None else None
    if lm is not None:   # an active learned section here (no sign section here by cur_is_sign)
      prev_ms, prev_sign = float(lm), True
  walked = 0.0
  for w, s_from, s_to, d, dist0 in pieces:
    osm_ms = float(w.get("maxspeed_ms") or 0.0)
    n = int(abs(s_to - s_from) // NEXT_STEP_M)
    for k in range(n + 1):
      s = s_from + d * k * NEXT_STEP_M
      ms, masked = at(w, s, osm_ms)
      dist = dist0 + k * NEXT_STEP_M
      walked = dist
      if abs(ms - prev_ms) <= 0.3:
        prev_sign = masked
        continue
      if masked or prev_sign:
        return ms, max(0.0, dist - NEXT_STEP_M / 2 - lead_m)
      # plain OSM -> OSM change, not masked: keep mapd's OSM next
      return osm_next   # (None when mapd's stub filter dropped it)
  # No change on the known road. Beyond it no section applies; use OSM next only
  # past the walked part and only if it differs from the effective limit.
  if osm_next is not None and osm_next[1] + lead_m >= walked - 1.0 and abs(osm_next[0] - prev_ms) > 0.3:
    return osm_next
  return None
