"""Tesla-learned limits: remember where the Tesla base map disagrees with OSM.

Gated by NAPTeslaLimitPrimary (default on). Tesla gives only the limit where
the car is now, so on its own it cannot drive the early slowdown. This file
records, per OSM way section, the debounced Tesla base limit (0x238 mux 3)
wherever it disagrees with raw OSM, so a later drive can look ahead with it.

Section = BIN_M of a way, keyed (way_id, int(s / BIN_M)) where s is meters
along the digitized way. Direction-agnostic (the Tesla base map has one value
per road section in this car's data); a pass the other way that sees a
different value simply resets the row, so a truly directional spot never
becomes active.

Per pass, every mapd tick inside one section counts the live debounced Tesla
value (never the held / MPP value). Leaving the section commits it when it saw
>= MIN_TICKS samples and one value has >= MAJORITY of them (so the ~40 m
where Tesla flips ahead of a sign does not split a section):
  * Tesla != OSM, no row          -> new row, 1 pass, inactive
  * Tesla != OSM, same value      -> +1 pass if another drive or > PASS_GAP_S
                                     since the row was last seen; ACTIVE at 2
  * Tesla != OSM, different value -> row replaced (1 pass, inactive)
  * Tesla == OSM                  -> row retired (deleted)
Active rows are used by mapd: current limit = confirmed sign > live Tesla >
active learned > OSM; lookahead (nextSpeedLimit) = confirmed sign > active
learned > OSM. They are never sign confirmations and never reach
auto-confirm (that compares a read with raw OSM and the live Tesla value).

File: /data/media/0/nap/tesla_learned_limits.json, separate from
map_speed_overrides.json. Compact rows, MAX_ROWS cap (oldest last-seen
pruned), ~63 B per row (10-digit way ids and unix times) so the cap is
~0.63 MB. Atomic write (tmp + rename),
at most every SAVE_PERIOD_S and only when something changed (a promotion
saves at once).
"""
from __future__ import annotations

import collections
import json
import os

FILENAME = "tesla_learned_limits.json"
BIN_M = 100.0
MAX_ROWS = 10000
PRUNE_TO = 9000
MIN_PASSES = 2
PASS_GAP_S = 1800.0      # same drive, > 30 min apart = another pass
MIN_TICKS = 2            # mapd 2 Hz: a section needs >= 1 s of live Tesla samples
MAJORITY = 0.7
MIN_SPEED_MS = 4.5       # ~10 mph; parked / crawling samples are skipped
SAVE_PERIOD_S = 60.0
MPH_TO_MS = 0.44704
VERSION = 1

# row: [way_id, bin, mph, osm_mph, passes, first_wall, last_wall, drive, active, drives]
W, B, MPH, OSM, N, FIRST, LAST, DRIVE, ACTIVE, DRIVES = range(10)


def learned_path() -> str:
  env = os.environ.get("NAP_TESLA_LEARNED")
  if env:
    return env
  base = "/data/media/0/nap" if os.path.isdir("/data/media/0") else os.path.join(
    os.path.expanduser("~"), ".comma", "media", "0", "nap")
  return os.path.join(base, FILENAME)


def bin_of(s: float) -> int:
  return max(0, int(float(s) // BIN_M))


class TeslaLearned:
  def __init__(self, path: str | None = None, drive: int | str = 0) -> None:
    self.path = path or learned_path()
    self.drive = drive   # mapd start unix time (int); any equality-comparable id works
    self.rows: dict[tuple[int, int], list] = {}
    self.dirty = False
    self._save_t: float | None = None
    self._cur: tuple[int, int] | None = None
    self._counts: collections.Counter = collections.Counter()
    self._osm = 0
    self._cur_wall = 0.0
    self.loaded = False
    self.n_active = 0

  # ---- persistence ----
  def load(self) -> None:
    self.loaded = True
    try:
      with open(self.path) as f:
        d = json.load(f)
      rows = d.get("rows", []) if isinstance(d, dict) else []
    except (OSError, ValueError):
      return
    out = {}
    for r in rows:
      try:
        r = [int(r[W]), int(r[B]), int(r[MPH]), int(r[OSM]), int(r[N]), int(r[FIRST]), int(r[LAST]),
             r[DRIVE], int(r[ACTIVE]), int(r[DRIVES])]
      except (TypeError, ValueError, IndexError):
        continue
      if 5 <= r[MPH] <= 85:
        out[(r[W], r[B])] = r
    self.rows = out
    self._recount()

  def save(self, mono: float, force: bool = False) -> bool:
    if not self.dirty:
      return False
    if not force and self._save_t is not None and mono - self._save_t < SAVE_PERIOD_S:
      return False
    self._save_t = mono
    tmp = self.path + ".tmp"
    try:
      os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
      with open(tmp, "w") as f:
        json.dump({"v": VERSION, "bin_m": BIN_M, "rows": list(self.rows.values())}, f, separators=(",", ":"))
      os.replace(tmp, self.path)
      self.dirty = False
      return True
    except OSError:
      return False

  # ---- lookups ----
  def limit_ms(self, way_id: int, s: float | None) -> float | None:
    """Active learned limit at s along way_id (m/s), else None."""
    if s is None or not self.rows:
      return None
    r = self.rows.get((int(way_id), bin_of(s)))
    if r is None or not r[ACTIVE]:
      return None
    return r[MPH] * MPH_TO_MS

  def has_active(self) -> bool:
    return self.n_active > 0

  def _recount(self) -> None:
    self.n_active = sum(1 for r in self.rows.values() if r[ACTIVE])

  # ---- learning ----
  def observe(self, way_id: int | None, s: float | None, tesla_mph: int, osm_mph: int, speed_ms: float,
              wall: float) -> list | None:
    """One mapd tick. tesla_mph = live debounced base (0 when not valid).
    Returns a row that just became ACTIVE (for a review note), else None."""
    key = (int(way_id), bin_of(s)) if way_id and s is not None else None
    promoted = None
    if key != self._cur:
      promoted = self._commit()
      self._cur, self._counts = key, collections.Counter()
    if key is not None and tesla_mph > 0 and osm_mph > 0 and speed_ms >= MIN_SPEED_MS:
      self._counts[int(tesla_mph)] += 1
      self._osm, self._cur_wall = int(osm_mph), float(wall)
    return promoted

  def flush(self) -> list | None:
    """Commit the open section (end of drive / tests)."""
    p = self._commit()
    self._cur, self._counts = None, collections.Counter()
    return p

  def _commit(self) -> list | None:
    if self._cur is None or not self._counts:
      return None
    total = sum(self._counts.values())
    v, c = self._counts.most_common(1)[0]
    if total < MIN_TICKS or c < MAJORITY * total:
      return None
    return self.commit_section(self._cur, v, self._osm, self._cur_wall)

  def commit_section(self, key: tuple[int, int], mph: int, osm_mph: int, wall: float) -> list | None:
    r = self.rows.get(key)
    if mph == osm_mph:
      if r is not None:
        del self.rows[key]        # Tesla agrees with OSM again: retire
        self.dirty = True
        self._recount()
      return None
    if r is None or r[MPH] != mph:
      self.rows[key] = [key[0], key[1], int(mph), int(osm_mph), 1, int(wall), int(wall), self.drive, 0, 1]
      self.dirty = True
      self._cap()
      self._recount()
      return None
    new_drive = r[DRIVE] != self.drive
    if new_drive or wall - r[LAST] > PASS_GAP_S:
      r[N] += 1
      if new_drive:
        r[DRIVES] += 1
    r[DRIVE], r[LAST], r[OSM] = self.drive, int(wall), int(osm_mph)
    self.dirty = True
    if not r[ACTIVE] and r[N] >= MIN_PASSES:
      r[ACTIVE] = 1
      self.n_active += 1
      self._save_t = None   # save soon: a promotion should survive a crash
      return r
    return None

  def _cap(self) -> None:
    if len(self.rows) <= MAX_ROWS:
      return
    keep = sorted(self.rows.items(), key=lambda kv: kv[1][LAST], reverse=True)[:PRUNE_TO]
    self.rows = dict(keep)
