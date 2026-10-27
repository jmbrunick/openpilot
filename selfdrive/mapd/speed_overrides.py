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

At the sign (Oct 23 2026, Clarkfield US 59 / DeGraff US 12): speed-change
signs stand where the map's own limit changes, so a section keyed to the car's
read-time way and stopped at the first map change could end a meter after the
sign. Now:
  * the section starts at the SIGN (reader's located sign_lat / sign_lon, else
    the forward range sign_range_m), not the car's position at the read; a sign
    at / past the end of the read-time way starts on the next way ahead.
  * same road = same name OR same route number (OSM ref when the map has it,
    or a route-style name: "US 59" == "US Highway 59").
  * a sign within CARRY_M of a map limit change carries onto the first next way
    even though its map limit differs (and, there only, across a name change
    when that way is the single straight-through continuation); after that it
    stops at the next map limit change as before.

Never behind the sign (Oct 26 2026, DeGraff Atlantic Ave, route 000001cd 08:51:55):
a section is the road from the sign forward in its read direction, and it applies to
traffic in both directions on that stretch -- but nothing on the far side of the sign.
A row read before the reader located signs starts at the read point, which is
behind the sign (the car reads it on approach): a westbound 60 read on the 30
street covered ~45 m of it, and eastbound MAX jumped to 59 there. Such rows are
moved up to the sign (sign_cut_m): to the first way boundary within
SIGN_AHEAD_MAX_M where the map turns toward the sign's value (no such change:
the read point stays). New rows record sign_at ("located" / "map_change" /
"read"); old rows are fixed in memory at load (fix_starts, logged once).
The 1 m end tolerance no longer reaches behind a sign or a span's from_s.
"""
from __future__ import annotations

import json
import math
import os
import re
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
CARRY_M = 150.0        # a sign this close to a map limit change carries past it (once)
SIGN_AHEAD_MAX_M = 150.0   # sanity cap on how far ahead of the read point the sign can be
STRAIGHT_DEG = 30.0    # straight-through continuation at a way end (name change at the sign)


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


_ROUTE_PATTERNS = (
  # (prefix key, regex) -- "US 59", "US Highway 59", "U.S. Hwy 59", "MN 7", "State Highway 7",
  # "County Road 31", "CR 31", "CSAH 31", "I 90", "Interstate 90".
  ("us", r"^u\.?\s?s\.?\s*(?:highway|hwy|route|rte)?\s*(\d+[a-z]?)$"),
  ("i", r"^(?:i|interstate)(?:\s*-\s*|\s+|-?)(?:highway\s*)?(\d+)$"),
  ("cr", r"^(?:county\s+(?:road|highway|rd|hwy|state\s+aid\s+highway)|cr|csah|co\.?\s*rd\.?|co\.?\s*hwy)\s*(\d+[a-z]?)$"),
  ("st", r"^(?:mn|wi|ia|nd|sd|il|in|ky|th|sh|sr|state\s+(?:highway|hwy|route|road|trunk\s+highway)"
         + r"|(?:minnesota|wisconsin|iowa)\s+(?:state\s+)?(?:highway|hwy|trunk\s+highway))\s*(\d+[a-z]?)$"),
)


def route_keys(*labels: str) -> set[str]:
  """Route numbers named by OSM name / ref strings ("US 59;MN 7" -> {"us59", "st7"})."""
  out: set[str] = set()
  for lab in labels:
    for part in str(lab or "").replace(",", ";").split(";"):
      t = " ".join(part.strip().lower().split())
      if not t:
        continue
      for key, pat in _ROUTE_PATTERNS:
        m = re.match(pat, t)
        if m:
          out.add(key + m.group(1))
          break
  return out


def _same_road(name: str, highway: str, w: dict, ref: str = "") -> bool:
  """Same name, or a shared route number (OSM ref, or a route-style name). Unnamed: same class."""
  a, b = (name or "").strip().lower(), (w.get("name") or "").strip().lower()
  if a and a == b:
    return True
  ka, kb = route_keys(name, ref), route_keys(w.get("name") or "", w.get("ref") or "")
  if ka and ka & kb:
    return True
  if a or b:
    return False
  return (highway or "").strip().lower() == (w.get("highway") or "").strip().lower()


def _end_heading(coords, at_end: bool) -> float:
  """Travel bearing leaving the way at its last node (at_end) or its first node (traveling against it)."""
  return _bearing(coords[-2], coords[-1]) if at_end else _bearing(coords[1], coords[0])


def _start_heading(w_coords, end) -> float:
  """Travel bearing entering way w at `end`."""
  return _bearing(w_coords[0], w_coords[1]) if _near(w_coords[0], end) else _bearing(w_coords[-1], w_coords[-2])


def _next_way(db, end, h_in: float, name: str, hw: str, ref: str, seen: set, straight_ok: bool):
  """The single continuation at a way end: same road (name / ref), else (straight_ok) the single
  straight-through way. None at a road change or a fork."""
  ways = [w for w in db.ways_at(end[0], end[1], JOIN_TOL_M) if int(w["way_id"]) not in seen]
  same = [w for w in ways if _same_road(name, hw, w, ref)]
  if len(same) == 1:
    return same[0]
  if same or not straight_ok:
    return None
  straight = [w for w in ways if len(w["coords"]) >= 2
              and heading_delta(_start_heading(w["coords"], end), h_in) <= STRAIGHT_DEG]
  return straight[0] if len(straight) == 1 else None


def _r6(x):
  return None if x is None else round(float(x), 6)


def _mph(ms) -> int:
  try:
    return int(round(float(ms) / MPH_TO_MS))
  except (TypeError, ValueError):
    return 0


def _walk(db, way_id: int, s: float, d: int, coords, skip_m: float, info: dict | None):
  """Move skip_m meters ahead of (way_id, s) in direction d along the road. Returns
  (way_id, s, d, coords, info, moved_m). Crosses way ends into the next way ahead (same road, or
  the single straight-through way); stops at the last way end it can reach."""
  moved = 0.0
  seen = {int(way_id)}
  while True:
    L = _poly_len(coords)
    rem = (L - s) if d > 0 else s
    if skip_m - moved <= rem + 0.05:
      s = s + d * max(0.0, skip_m - moved)
      return int(way_id), max(0.0, min(L, s)), d, coords, info, skip_m
    moved += rem
    end = coords[-1] if d > 0 else coords[0]
    if info is None or db is None or not hasattr(db, "ways_at"):
      return int(way_id), (L if d > 0 else 0.0), d, coords, info, moved
    w = _next_way(db, end, _end_heading(coords, d > 0), info.get("name", ""), info.get("highway", ""),
                  info.get("ref", ""), seen, straight_ok=True)
    if w is None:
      return int(way_id), (L if d > 0 else 0.0), d, coords, info, moved
    way_id, coords, info = int(w["way_id"]), w["coords"], w
    seen.add(way_id)
    if _near(coords[0], end):
      d, s = 1, 0.0
    else:
      d, s = -1, _poly_len(coords)


def build_chain(way_id: int, from_s: float, d: int, coords, db, max_m: float = CHAIN_MAX_M,
                carry_m: float = CARRY_M, mph: int | None = None) -> list[list]:
  """[[way_id, s0, s1, dist_at_s0, dist_at_s1], ...] from the sign forward (d = +1 along the
  digitized way, -1 against it) through same-road continuations (same name or route number).
  Stops at max_m, a road change (no continuation) or a fork (more than one), and at the first way
  whose map limit differs from the map's limit at the sign -- except that a map limit change
  within carry_m of the sign is carried past once (onto that one next way; a name change there is
  allowed when it is the single straight-through way), then the chain stops at the next change.
  With the sign's mph given, the carry needs the sign to be closer to the next way's map limit
  than to the one it replaces (the map's change is the change the sign marks): a 35 read 109 m
  before a map 40 -> 30 change is not carried onto the 30 (Montevideo MN 7, Oct 23)."""
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
  name, hw, ref = info.get("name", ""), info.get("highway", ""), info.get("ref", "")
  replaced = _mph(info.get("maxspeed_ms"))   # the map's limit at the sign: the value this YES replaces
  carried = False
  h_in = _end_heading(coords, d > 0)
  seen = {int(way_id)}
  while acc < max_m:
    near_sign = (not carried) and acc <= carry_m
    w = _next_way(db, end, h_in, name, hw, ref, seen, straight_ok=near_sign)
    if w is None:
      break           # road change (none) or ambiguous fork (several)
    same = _same_road(name, hw, w, ref)
    nxt = _mph(w.get("maxspeed_ms"))
    if nxt != replaced:
      if not near_sign or (mph is not None and abs(int(mph) - nxt) >= abs(int(mph) - replaced)):
        break         # the map's own limit changes here: the next posted section starts
      carried = True  # the sign stands at the map's change: carry onto this way, stop at its next change
      replaced = nxt
    elif not same:
      break           # a name change with no map limit change is a road change
    # the road is followed way to way ("US 59" -> "Main Street" ref US 59 -> "Main Street"); a
    # straight-through name change is only taken at the map's change at the sign (13th St -> US Hwy 59)
    name, hw, ref = w.get("name", ""), w.get("highway", ""), w.get("ref", "")
    c = w["coords"]
    wl = _poly_len(c)
    r = min(wl, max_m - acc)
    if _near(c[0], end):
      spans.append([int(w["way_id"]), 0.0, r, acc, acc + r])
      end = c[-1]
      h_in = _end_heading(c, True)
    else:
      spans.append([int(w["way_id"]), wl - r, wl, acc + r, acc])
      end = c[0]
      h_in = _end_heading(c, False)
    seen.add(int(w["way_id"]))
    acc += r
  return spans


def manual_stretch(way_id: int, s: float, d: int, coords, db):
  """Whole map-limit stretch around the car at (way_id, s), travelling d. Walks back (against d)
  to the previous map limit change (or road change / fork / CHAIN_MAX_M), then builds the chain
  forward from there past the car to the next map limit change. No carry at either end.
  Returns (way_id, from_s, d, coords, chain, back_m) for the stretch start, or None when the
  forward chain does not reach the car (e.g. a fork seen only in one direction)."""
  back = build_chain(int(way_id), s, -d, coords, db, carry_m=-1.0)
  if not back:
    return None
  far = max(back, key=lambda sp: max(float(sp[3]), float(sp[4])))
  w0 = int(far[0])
  back_m = max(float(far[3]), float(far[4]))
  if float(far[4]) >= float(far[3]):   # walked back along +s: the far end is s1, forward is -1
    s0, d0 = float(far[2]), -1
  else:
    s0, d0 = float(far[1]), 1
  c0 = coords if w0 == int(way_id) else (db.way_coords(w0) if db is not None and hasattr(db, "way_coords") else None)
  if not c0:
    return None
  chain = build_chain(w0, s0, d0, c0, db, max_m=back_m + CHAIN_MAX_M, carry_m=-1.0)
  e = {"chain": chain}
  if chain_dist(e, int(way_id), s) is None:
    return None
  return w0, round(s0, 1), d0, c0, chain, back_m


def chain_dist(e: dict, way_id: int, s: float | None) -> float | None:
  """Distance from e's sign along its chain to point s of way_id; None if not covered."""
  start = min((min(float(sp[3]), float(sp[4])) for sp in e.get("chain") or ()), default=0.0)
  for sp in e.get("chain") or ():
    if int(sp[0]) != int(way_id):
      continue
    s0, s1, a0, a1 = float(sp[1]), float(sp[2]), float(sp[3]), float(sp[4])
    if s is None or not (s0 - 1.0 <= s <= s1 + 1.0):
      continue
    # Never behind the sign (Oct 26 2026): the 1 m end tolerance only applies away from the sign.
    if (s < s0 and a0 <= start + 0.05) or (s > s1 and a1 <= start + 0.05):
      continue
    f = 0.0 if s1 - s0 < 1e-6 else (min(max(s, s0), s1) - s0) / (s1 - s0)
    return a0 + f * (a1 - a0)
  return None


def trim_chain(chain, cut_m: float) -> list[list]:
  """Drop the first cut_m meters of a chain (from its sign end); distances restart at 0 at the new start."""
  if cut_m <= 0.0:
    return [list(sp) for sp in chain]
  out = []
  for sp in chain:
    w, s0, s1, a0, a1 = int(sp[0]), float(sp[1]), float(sp[2]), float(sp[3]), float(sp[4])
    if max(a0, a1) <= cut_m:
      continue
    if min(a0, a1) < cut_m:
      f = (cut_m - a0) / (a1 - a0)
      sc = s0 + f * (s1 - s0)
      if a0 < a1:
        s0, a0 = sc, cut_m
      else:
        s1, a1 = sc, cut_m
    out.append([w, round(s0, 1), round(s1, 1), round(a0 - cut_m, 1), round(a1 - cut_m, 1)])
  return out


def sign_cut_m(chain, mph: int, db) -> tuple[float, str]:
  """Where along a chain built from the READ POINT the sign itself stands (Oct 26 2026, DeGraff
  Atlantic Ave: a westbound 60 read on the 30 street, its sign at the town edge where the map turns
  60; the section from the read point covered ~45 m behind the sign and eastbound MAX jumped).
  The car reads a sign before reaching it, so the sign is ahead of the read point:
    * map_change: the first way boundary within SIGN_AHEAD_MAX_M where the map's limit moves to
      (or toward) the sign's value while the first way's map limit is not the sign's -- signs stand
      where the posted limit changes;
    * read: no map evidence -- keep the read point (the best position known; the reader's
      located sign, stored on every row since Oct 23, is the real fix).
  Returns (meters to cut from the chain start, how it was found). Never cuts the whole chain."""
  pcs = sorted(chain, key=lambda sp: min(float(sp[3]), float(sp[4])))
  total = max((max(float(sp[3]), float(sp[4])) for sp in pcs), default=0.0)
  if not pcs or total <= 1.0:
    return 0.0, "read"
  def info(w):
    return db.way_info(int(w)) if db is not None and hasattr(db, "way_info") else None
  i0 = info(pcs[0][0])
  first = _mph(i0.get("maxspeed_ms")) if i0 else None
  if first is not None and first != int(mph):
    prev = first
    for sp in pcs[1:]:
      a = min(float(sp[3]), float(sp[4]))
      if a > SIGN_AHEAD_MAX_M:
        break
      iw = info(sp[0])
      m = _mph(iw.get("maxspeed_ms")) if iw else None
      if m is not None and m != prev and abs(int(mph) - m) < abs(int(mph) - prev):
        return min(a, total - 1.0), "map_change"
      prev = m if m is not None else prev
  return 0.0, "read"


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
  for k in ("lat", "lon", "bearing", "heading", "sign_lat", "sign_lon", "sign_range_m"):
    try:
      if d.get(k) is not None:
        out[k] = float(d[k])
    except (TypeError, ValueError):
      pass
  if d.get("auto") in AUTO_SOURCES:
    out["auto"] = str(d["auto"])
  if d.get("manual"):
    out["manual"] = True   # popup entry (limit_source.py): starts where the car is, no sign search
  return out


# Auto-confirm sources (NAPSignAutoConfirm). The stored row says which map agreed.
AUTO_SOURCES = ("tesla", "osm")
MANUAL_SOURCE = "manual"   # driver-entered limit from the LIMIT-plate long-press popup


def row_source(conf: dict) -> str:
  if conf.get("manual"):
    return MANUAL_SOURCE
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
        if pos[0] is not None and (pos[0] - float(e["from_s"])) * int(e.get("dir", 1)) >= 0.0:
          best = e
      elif bearing is not None and heading_delta(e["heading"], bearing) <= HEADING_MATCH_DEG:
        best = e  # newest wins (appended last)
    return best

  def fix_starts(self, db) -> list[dict]:
    """Rows written before the sign was located (no sign_at, no read_lat) start at the read point,
    behind the sign. Move each start up to its sign (sign_cut_m), in memory; the original chain is
    kept as chain_read. Returns the rows changed this call (mapd logs them once)."""
    done = []
    for e in self.entries:
      if not e.get("chain") or e.get("sign_at") or e.get("read_lat") is not None:
        continue
      cut, how = sign_cut_m(e["chain"], int(e["mph"]), db)
      e["sign_at"] = how
      if cut <= 0.0:
        continue
      e["chain_read"] = e["chain"]
      e["chain"] = trim_chain(e["chain"], cut)
      e["cut_m"] = round(cut, 1)
      done.append(e)
    return done

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
           (pos[0] - float(e["from_s"])) * int(e.get("dir", 1)) >= 0.0:
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

  @staticmethod
  def sign_ahead_m(heading: float, conf: dict) -> float:
    """How far ahead of the read point (conf lat/lon, the car at the photo) the sign stands, along
    the heading: the reader's located sign (sign_lat / sign_lon) projected on the heading, else its
    range (sign_range_m), else 0 (old payloads: the read point). Clamped to [0, SIGN_AHEAD_MAX_M]."""
    f = None
    try:
      if conf.get("sign_lat") is not None and conf.get("sign_lon") is not None and conf.get("lat") is not None:
        x, y = _xy(float(conf["sign_lat"]), float(conf["sign_lon"]), float(conf["lat"]), float(conf["lon"]))
        h = math.radians(float(heading))
        f = x * math.sin(h) + y * math.cos(h)
      elif conf.get("sign_range_m") is not None:
        f = float(conf["sign_range_m"])
    except (TypeError, ValueError):
      f = None
    if f is None or not math.isfinite(f):
      return 0.0
    return max(0.0, min(SIGN_AHEAD_MAX_M, f))

  def add(self, way_id: int, heading: float, conf: dict, road: str = "", now: float | None = None,
          coords=None, db=None) -> dict:
    now = time.time() if now is None else now  # noqa: TID251  wall clock in the stored row
    from_s, d = self.span(heading, conf, coords)
    read_pt = (conf.get("lat"), conf.get("lon"))
    sign_pt = read_pt
    if from_s is not None and coords:
      ahead = self.sign_ahead_m(heading, conf)
      if ahead > 0.0:
        # start at the sign, not the car: walk ahead along the road (onto the next way when the
        # sign stands at / past the end of the read-time way)
        info = db.way_info(int(way_id)) if db is not None and hasattr(db, "way_info") else None
        way_id, s_at, d, coords, _, _moved = _walk(db, int(way_id), from_s, d, coords, ahead, info)
        from_s = round(s_at, 1)
        sign_pt = _point_at(coords, from_s)
      back_m = None
      if conf.get("manual"):
        # Manual Save (Oct 27 2026): the driver fixes a limit that has already been wrong for a
        # while, so the row covers the whole current map-limit stretch -- from the previous map
        # limit change behind the car to the next one ahead (both directions of travel).
        st = manual_stretch(int(way_id), from_s, d, coords, db)
        if st is not None:
          way_id, from_s, d, coords, chain, back_m = st
          sign_pt = _point_at(coords, from_s)
        else:   # stretch walk failed (fork behind, no map): start at the car, as before
          chain = build_chain(int(way_id), from_s, d, coords, db, mph=int(conf["mph"]))
      else:
        chain = build_chain(int(way_id), from_s, d, coords, db, mph=int(conf["mph"]))
      sign_at, cut = "located", 0.0
      if conf.get("manual"):
        sign_at = "manual_stretch" if back_m is not None else "manual"   # never moved at load
      elif ahead <= 0.0:   # no located sign: the chain starts at the read point, behind the sign
        cut, sign_at = sign_cut_m(chain, int(conf["mph"]), db)
        chain = trim_chain(chain, cut)
        if chain and cut > 0.0:
          w0 = int(chain[0][0])
          from_s = float(chain[0][1] if chain[0][3] <= chain[0][4] else chain[0][2])
          if w0 != int(way_id):
            coords = db.way_coords(w0) if db is not None and hasattr(db, "way_coords") else coords
            way_id = w0
          sign_pt = _point_at(coords, from_s)
      ways = {int(sp[0]) for sp in chain}

      def replaced(e) -> bool:
        if e.get("chain"):     # the same sign confirmed again (same spot, same direction)
          return (e.get("lat") is not None and sign_pt[0] is not None
                  and _near((e["lat"], e["lon"]), sign_pt, SAME_SIGN_M)
                  and heading_delta(e.get("heading", 0.0), heading) <= HEADING_MATCH_DEG)
        return int(e["way_id"]) in ways   # older span rows on ways this section now covers
      self._drop([e for e in self.entries if replaced(e)])   # legacy rows are not in entries: kept
      e = {"v": ENTRY_VERSION, "way_id": int(way_id), "heading": round(float(heading), 1), "mph": int(conf["mph"]),
           "both": True, "from_s": from_s, "dir": d, "chain": [[sp[0]] + [round(float(x), 1) for x in sp[1:]] for sp in chain],
           "lat": _r6(sign_pt[0]), "lon": _r6(sign_pt[1]), "road": road, "t": round(now, 1), "source": row_source(conf)}
      e["sign_at"] = sign_at
      if back_m is not None:
        e["back_m"] = round(back_m, 1)
      if cut > 0.0:
        e["cut_m"] = round(cut, 1)
      if sign_pt != read_pt:
        e["read_lat"], e["read_lon"] = read_pt
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
  for k in ("lat", "lon", "bearing", "sign_lat", "sign_lon", "sign_range_m"):
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
                max_m: float = NEXT_MAX_M, chain_ways: set | None = None) -> list[tuple[dict, float, float, int, float]]:
  """Pieces ahead on the same road: [(way_info, s_from, s_to, dir, dist_at_s_from)].
  Stops at max_m, a road change or a fork (same rule as sign sections). A straight-through name
  change is followed only onto a way a sign section covers (chain_ways), as sections do at a sign."""
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
  name, hw, ref = info.get("name", ""), info.get("highway", ""), info.get("ref", "")
  h_in = _end_heading(coords, d > 0)
  while acc < max_m and hasattr(db, "ways_at"):
    w = _next_way(db, end, h_in, name, hw, ref, seen, straight_ok=bool(chain_ways))
    if w is None:
      break
    if not _same_road(name, hw, w, ref) and int(w["way_id"]) not in (chain_ways or ()):
      break
    name, hw, ref = w.get("name", ""), w.get("highway", ""), w.get("ref", "")
    c = w["coords"]
    wl = _poly_len(c)
    r = min(wl, max_m - acc)
    if _near(c[0], end):
      pieces.append((w, 0.0, r, 1, acc))
      end = c[-1]
      h_in = _end_heading(c, True)
    else:
      pieces.append((w, wl, wl - r, -1, acc))
      end = c[0]
      h_in = _end_heading(c, False)
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
  chain_ways = {int(sp[0]) for e in entries for sp in (e.get("chain") or ())}
  pieces = route_ahead(db, way_id, coords, lat, lon, float(bearing), chain_ways=chain_ways)
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
