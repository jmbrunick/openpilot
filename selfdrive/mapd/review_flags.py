"""Sign review flags: log notes where a speed sign probably stands but no read matched.

Purely a log for the training-data loop (Oct 23 2026): each line says "check
this spot for a speed limit sign; if there is one, add it to the sign
training set". No userBookmark (that would preserve segments and evict the
driver's own bookmarks), no alert, no voice note.

Event = a 3 s-debounced change of the Tesla base limit (0x238 mux 3) and / or
the raw OSM limit. Changes within MERGE_S of each other are one event. The
event is decided READ_AFTER_S after its last change:
  * turn (bearing change > TURN_DEG over +-TURN_WIN_S) or vehicle speed
    < MIN_SPEED_MPH                             -> dropped
  * a sign read with the new value from -READ_BEFORE_S..+READ_AFTER_S -> dropped
  * both maps changed to the same new value     -> review_flags.jsonl
  * otherwise (one map only / maps disagree)    -> review_single.jsonl
A spot (DEDUPE_M, same heading, same new value) is logged once per 7 days.

Tesla-learned sections (tesla_learned.py, NAPTeslaLimitPrimary): when a
section where the Tesla base map disagrees with OSM becomes ACTIVE (same
value on 2 passes), note_disagree() writes kind "disagree", reason
tesla_osm_disagree_tesla_<t>_osm_<o>, into review_flags.jsonl with the same
route / segment / offset fields and the same 7-day same-spot dedupe.

Each line carries the route / segment / offset and the monotonic time of the
change so tools/nap/review_flag_frames.py (box side) can cut camera frames.
"""
from __future__ import annotations

import collections
import json
import math
import os
import re

MERGE_S = 30.0
READ_BEFORE_S = 20.0
READ_AFTER_S = 45.0
TURN_DEG = 35.0
TURN_WIN_S = 12.0
MIN_SPEED_MPH = 20.0
HOLD_S = 3.0
GAP_RESET_S = 3.0
DEDUPE_M = 200.0
DEDUPE_HDG_DEG = 45.0
DEDUPE_S = 7 * 86400.0
HISTORY_S = 120.0
MS_TO_MPH = 2.23694

FLAGS_NAME = "review_flags.jsonl"
SINGLE_NAME = "review_single.jsonl"
DEFAULT_DIR = "/data/media/0/nap"
MAX_LOG_BYTES = 2_000_000


def hav_m(lat1, lon1, lat2, lon2) -> float:
  p = math.radians
  return 6371000.0 * math.hypot(p(lon2 - lon1) * math.cos(p((lat1 + lat2) / 2)), p(lat2 - lat1))


def hdg_delta(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


class _Debounce:
  """Known-to-known change after HOLD_S; zeros (unknown) are ignored."""

  def __init__(self, key: str) -> None:
    self.key = key
    self.cur = None
    self.cand = None   # (value, t)
    self.last_t = None

  def step(self, t: float, v: int):
    if self.last_t is not None and t - self.last_t > GAP_RESET_S:
      self.cur, self.cand = None, None
    self.last_t = t
    if not v:
      return None
    if self.cur is None:
      self.cur = v
      return None
    if v == self.cur:
      self.cand = None
      return None
    if self.cand is None or self.cand[0] != v:
      self.cand = (v, t)
    if t - self.cand[1] >= HOLD_S - 0.01:
      ev = {"key": self.key, "t": self.cand[1], "old": self.cur, "new": v}
      self.cur, self.cand = v, None
      return ev
    return None


_SEG_RE = re.compile(r"^([0-9a-f]{8}--[0-9a-f]{10})--(\d+)$")


class SegmentClock:
  """Which loggerd segment is being written, and since when (monotonic).

  Cheap: checks only for the NEXT segment dir each call; a full scan of the
  log root at most every RESCAN_S. The offset is +- one call period (~1 s).
  """
  RESCAN_S = 30.0

  def __init__(self, root: str | None = None) -> None:
    self.root = root
    self.route: str | None = None
    self.seg: int | None = None
    self.start_mono: float | None = None
    self._scan_t = -1e9

  def _root(self) -> str | None:
    if self.root is None:
      try:
        from openpilot.system.hardware.hw import Paths
        self.root = Paths.log_root()
      except Exception:
        return None
    return self.root

  def update(self, mono: float) -> None:
    root = self._root()
    if root is None:
      return
    if self.route is not None and os.path.isdir(os.path.join(root, f"{self.route}--{self.seg + 1}")):
      self.seg += 1
      self.start_mono = mono
      return
    if mono - self._scan_t < self.RESCAN_S:
      return
    self._scan_t = mono
    best, best_m = None, -1.0
    try:
      with os.scandir(root) as it:
        for e in it:
          m = _SEG_RE.match(e.name)
          if m and e.is_dir(follow_symlinks=False):
            mt = e.stat(follow_symlinks=False).st_mtime
            if mt > best_m:
              best, best_m = m, mt
    except OSError:
      return
    if best is None:
      return
    route, seg = best.group(1), int(best.group(2))
    if (route, seg) != (self.route, self.seg):
      self.route, self.seg = route, seg
      self.start_mono = mono   # first seen; refined box-side from the log

  def at(self, mono: float) -> dict:
    if self.route is None:
      return {"route": None, "segment": None, "seg_offset_s": None}
    off = mono - self.start_mono if self.start_mono is not None else None
    return {"route": self.route, "segment": self.seg,
            "seg_offset_s": round(off, 1) if off is not None else None}


class ReviewFlagger:
  def __init__(self, out_dir: str | None = None, seg_clock: SegmentClock | None = None) -> None:
    self.out_dir = out_dir or DEFAULT_DIR
    self.seg_clock = seg_clock or SegmentClock()
    self.hist: collections.deque = collections.deque()
    self.reads: collections.deque = collections.deque()   # (mono, mph)
    self.deb = {"tesla": _Debounce("tesla"), "osm": _Debounce("osm")}
    self.cluster: list[dict] = []
    self.recent: list[dict] | None = None   # dedupe memory, lazy-loaded
    self.written: list[dict] = []            # this process (tests / replay)

  # ---- inputs ----
  def add_read(self, mono: float, mph: int) -> None:
    if mph and 5 <= mph <= 85:
      self.reads.append((mono, int(mph)))

  def tick(self, mono: float, wall: float, lat: float, lon: float, bearing: float, speed_ms: float,
           road: str, tesla_mph: int, osm_mph: int, mpp_mph: int = 0, gps_ok: bool = True) -> dict | None:
    """Feed one ~1 Hz sample. Returns the flag dict when a line was written."""
    seg = self.seg_clock.at(mono)
    self.hist.append({"t": mono, "wall": wall, "lat": lat, "lon": lon, "brg": bearing, "v": speed_ms,
                      "road": road or "", "tesla": int(tesla_mph or 0), "osm": int(osm_mph or 0),
                      "mpp": int(mpp_mph or 0), "gps": gps_ok, **seg})
    while self.hist and mono - self.hist[0]["t"] > HISTORY_S:
      self.hist.popleft()
    while self.reads and mono - self.reads[0][0] > HISTORY_S:
      self.reads.popleft()
    if gps_ok:
      for key, v in (("tesla", tesla_mph), ("osm", osm_mph)):
        ev = self.deb[key].step(mono, int(v or 0))
        if ev is not None:
          self.cluster.append(ev)
          self.cluster.sort(key=lambda e: e["t"])
    out = None
    if self.cluster:
      # split off a finished cluster: last change + window passed, or a new change beyond MERGE_S
      first_new = next((i for i in range(1, len(self.cluster))
                        if self.cluster[i]["t"] - self.cluster[i - 1]["t"] >= MERGE_S), None)
      done = self.cluster[:first_new] if first_new is not None else self.cluster
      if mono - done[-1]["t"] >= READ_AFTER_S:
        self.cluster = self.cluster[first_new:] if first_new is not None else []
        out = self._decide(done, wall_now=wall)
    return out

  # ---- decision ----
  def _sample(self, t: float) -> dict | None:
    best = None
    for h in self.hist:
      if best is None or abs(h["t"] - t) < abs(best["t"] - t):
        best = h
    return best

  def _turn(self, t: float) -> bool:
    a, b = self._sample(t - TURN_WIN_S), self._sample(t + TURN_WIN_S)
    if a is None or b is None:
      return False
    # Bearing only: a road NAME change is not a turn (US 12 is Atlantic Ave in Benson).
    return hdg_delta(a["brg"], b["brg"]) > TURN_DEG

  def _reads_near(self, t: float) -> list[int]:
    return [m for (rt, m) in self.reads if t - READ_BEFORE_S <= rt <= t + READ_AFTER_S]

  def _decide(self, evs: list[dict], wall_now: float) -> dict | None:
    t0 = evs[0]["t"]
    g = self._sample(t0)
    if g is None:
      return None
    if self._turn(t0):
      return None
    speeds = [s["v"] for s in (self._sample(e["t"]) for e in evs) if s is not None]
    if not speeds or min(speeds) * MS_TO_MPH < MIN_SPEED_MPH:
      return None
    if any(e["new"] in self._reads_near(e["t"]) for e in evs):
      return None
    keys = {e["key"] for e in evs}
    both = keys == {"tesla", "osm"} and len({e["new"] for e in evs[-2:]}) == 1
    new = evs[-1]["new"]
    by = {k: next(({"old": e["old"], "new": e["new"], "t_mono": round(e["t"], 2)} for e in reversed(evs) if e["key"] == k), None)
          for k in ("tesla", "osm")}
    if both:
      reason = f"both_maps_{by['tesla']['old']}_to_{new}"
    elif len(keys) == 2:
      reason = f"maps_disagree_tesla_{by['tesla']['new']}_osm_{by['osm']['new']}"
    else:
      k = next(iter(keys))
      reason = f"{k}_only_{by[k]['old']}_to_{by[k]['new']}"
    flag = {
      "kind": "both" if both else "single",
      "reason": reason,
      "mph": new,
      "wall": round(g["wall"], 1),
      "t_mono": round(t0, 2),
      "frame_mono_ns": int(t0 * 1e9),
      "route": g["route"], "segment": g["segment"], "seg_offset_s": g["seg_offset_s"],
      "lat": round(g["lat"], 6), "lon": round(g["lon"], 6), "bearing": round(g["brg"], 1),
      "speed_mph": round(g["v"] * MS_TO_MPH, 1), "road": g["road"],
      "tesla": by["tesla"], "osm": by["osm"], "mpp_at_change": g["mpp"],
      "reads_nearby": self._reads_near(t0),
    }
    if self._dupe(flag):
      return None
    self._write(flag)
    return flag

  def note_disagree(self, mono: float, wall: float, lat: float, lon: float, bearing: float | None, speed_ms: float,
                    road: str, tesla_mph: int, osm_mph: int, way_id: int = 0) -> dict | None:
    """A Tesla-vs-OSM learned section just became active: check this spot for a sign."""
    seg = self.seg_clock.at(mono)
    t = round(float(mono), 2)
    flag = {
      "kind": "disagree",
      "reason": f"tesla_osm_disagree_tesla_{int(tesla_mph)}_osm_{int(osm_mph)}",
      "mph": int(tesla_mph),
      "wall": round(float(wall), 1),
      "t_mono": t,
      "frame_mono_ns": int(float(mono) * 1e9),
      "route": seg["route"], "segment": seg["segment"], "seg_offset_s": seg["seg_offset_s"],
      "lat": round(float(lat), 6), "lon": round(float(lon), 6), "bearing": round(float(bearing or 0.0), 1),
      "speed_mph": round(float(speed_ms) * MS_TO_MPH, 1), "road": road or "",
      "tesla": {"old": None, "new": int(tesla_mph), "t_mono": t},
      "osm": {"old": None, "new": int(osm_mph), "t_mono": t},
      "mpp_at_change": 0,
      "reads_nearby": [m for (rt, m) in self.reads if abs(mono - rt) <= HISTORY_S],
      "way_id": int(way_id),
    }
    if self._dupe(flag):
      return None
    self._write(flag)
    return flag

  # ---- persistence ----
  def _path(self, kind: str) -> str:
    # "both" and "disagree" (learned Tesla-vs-OSM) -> review_flags.jsonl; "single" -> review_single.jsonl
    return os.path.join(self.out_dir, SINGLE_NAME if kind == "single" else FLAGS_NAME)

  def _load_recent(self) -> list[dict]:
    if self.recent is None:
      self.recent = []
      for kind in ("both", "single"):
        try:
          with open(self._path(kind)) as f:
            for line in f:
              try:
                r = json.loads(line)
                self.recent.append({k: r.get(k) for k in ("kind", "lat", "lon", "bearing", "mph", "wall")})
              except Exception:
                continue
        except OSError:
          pass
    return self.recent

  def _dupe(self, flag: dict) -> bool:
    for r in self._load_recent():
      try:
        if (r["kind"] == flag["kind"] and r["mph"] == flag["mph"] and flag["wall"] - float(r["wall"]) < DEDUPE_S
            and hav_m(r["lat"], r["lon"], flag["lat"], flag["lon"]) < DEDUPE_M
            and hdg_delta(float(r["bearing"]), flag["bearing"]) < DEDUPE_HDG_DEG):
          return True
      except (KeyError, TypeError, ValueError):
        continue
    return False

  def _write(self, flag: dict) -> None:
    self.written.append(flag)
    self._load_recent().append({k: flag[k] for k in ("kind", "lat", "lon", "bearing", "mph", "wall")})
    path = self._path(flag["kind"])
    try:
      os.makedirs(self.out_dir, exist_ok=True)
      if os.path.exists(path) and os.path.getsize(path) > MAX_LOG_BYTES:
        os.replace(path, path + ".1")
      with open(path, "a") as f:
        f.write(json.dumps(flag, separators=(",", ":")) + "\n")
    except OSError:
      pass
