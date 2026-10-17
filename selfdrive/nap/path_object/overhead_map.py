"""NAP overhead map layer: tagged overhead structures from the offline map pack.

Pack schema 6 adds table ``overhead`` (built on the box by
scripts/nap/build_overhead_layer.py, never learned on the car):
  bridge    a bridge way at a higher layer crossing a drivable road
  gantry    man_made=gantry over a drivable road
  signal    highway=traffic_signals node on a drivable road (mast arms are
            not mapped, so the match window is wider)
  confirmed box-side daytime clearance-gate confirmations (reviewed list)
Streetlights and power lines are deliberately not tagged.

mapd publishes the nearest tagged spot ahead on liveMapDataNAP
(overheadAheadM, overheadKind). ``overhead_match`` is the one rule the path
object response uses to drop a stationary object at a tagged spot:
  * the object's range is within the kind's window of overheadAheadM,
  * stationary (|vRel + vEgo| <= STATIONARY_MPS), obstacle class only,
  * the driving model sees no lead there, the camera sees no vehicle lights
    (clipped / red blobs) at road level at the radar point,
  * daylight: the camera sees nothing solid at road level and the clearance
    gate does not read the bottom low,
  * night: bridges, gantries and confirmed points only (never signals);
    beyond NIGHT_RECHECK_M (low-beam reach) as above, inside it every camera
    grab must see nothing solid at road level, or the ignore is off.
  Lights, solid-object and model-lead reads are sticky for a track's life.
Untagged spots are untouched. Braking stays log-only in this build.
"""
from __future__ import annotations

import math
import sqlite3
from dataclasses import dataclass

TABLE = "overhead"
EARTH_R_M = 6371000.0
KINDS = ("bridge", "gantry", "signal", "confirmed")
# Lag correction for the fix. 0 on this car's gpsLocation: on Oct 14 7:05 PM the
# signal-arm return sat ~15 m past the OSM signal node with no correction (a
# far-side mast arm); a 1.5 s correction would put it ~43 m past, and box-side
# confirmed points are placed from the same uncorrected fix.
GPS_LAG_S = 0.0
HORIZON_M = 250.0
LATERAL_M = 15.0
HEADING_TOL_DEG = 35.0
WINDOW_M = {"bridge": 25.0, "gantry": 25.0, "confirmed": 25.0, "signal": 35.0}
STATIONARY_MPS = 1.5
NIGHT_RECHECK_M = 60.0
CUTOFF_M = 1.83


@dataclass(frozen=True)
class Overhead:
  oid: int
  kind: str
  distance_m: float   # along the heading, GPS lag corrected
  lateral_m: float


def ensure_table(con: sqlite3.Connection) -> None:
  con.execute(
    f"CREATE TABLE IF NOT EXISTS {TABLE}(id INTEGER PRIMARY KEY, lat REAL NOT NULL, lon REAL NOT NULL," +
    " kind TEXT NOT NULL, under_way_id INTEGER NOT NULL DEFAULT 0, heading_deg REAL," +
    " source TEXT NOT NULL DEFAULT 'osm')")
  con.execute(f"CREATE INDEX IF NOT EXISTS {TABLE}_lat_lon ON {TABLE}(lat, lon)")


def has_table(con) -> bool:
  try:
    return con.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (TABLE,)).fetchone() is not None
  except sqlite3.Error:
    return False


def insert(con, oid: int, lat: float, lon: float, kind: str, under_way_id: int = 0,
           heading_deg: float | None = None, source: str = "osm") -> None:
  if kind not in KINDS:
    raise ValueError(kind)
  con.execute(f"INSERT OR REPLACE INTO {TABLE} VALUES(?,?,?,?,?,?,?)",
              (int(oid), float(lat), float(lon), kind, int(under_way_id), heading_deg, source))


def _acute(a: float, b: float) -> float:
  d = abs((a - b + 180.0) % 360.0 - 180.0)
  return min(d, 180.0 - d)


def overhead_ahead(con, lat: float, lon: float, bearing_deg: float, v_ego: float = 0.0,
                   horizon_m: float = HORIZON_M, lateral_m: float = LATERAL_M,
                   lag_s: float = GPS_LAG_S) -> Overhead | None:
  """Nearest tagged overhead spot ahead on our road, or None. Old packs return None."""
  if con is None or not all(math.isfinite(x) for x in (lat, lon, bearing_deg)) or not has_table(con):
    return None
  dlat = math.degrees(horizon_m / EARTH_R_M)
  dlon = dlat / max(math.cos(math.radians(lat)), 1e-6)
  rows = con.execute(f"SELECT id, lat, lon, kind, heading_deg FROM {TABLE} WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ?",
                     (lat - dlat, lat + dlat, lon - dlon, lon + dlon)).fetchall()
  h = math.radians(bearing_deg)
  hx, hy = math.sin(h), math.cos(h)
  k = math.radians(1) * EARTH_R_M
  lag = max(0.0, float(v_ego) if math.isfinite(v_ego) else 0.0) * lag_s
  best = None
  for oid, la, lo, kind, hd in rows:
    if hd is not None and kind != "signal" and math.isfinite(hd) and _acute(hd, bearing_deg) > HEADING_TOL_DEG:
      continue
    e = (lo - lon) * k * math.cos(math.radians(lat))
    n = (la - lat) * k
    along = e * hx + n * hy - lag
    lat_off = e * hy - n * hx
    if -WINDOW_M.get(kind, 25.0) <= along <= horizon_m and abs(lat_off) <= lateral_m:
      if best is None or along < best.distance_m:
        best = Overhead(int(oid), str(kind), along, lat_off)
  return best


def _f(v, d=float("nan")) -> float:
  try:
    out = float(v)
  except (TypeError, ValueError):
    return d
  return out if math.isfinite(out) else d


def overhead_match(obs, md, v_ego: float) -> bool:
  """True when this vision-confirmed object sits at a tagged overhead spot and every guard passes."""
  if obs is None or md is None:
    return False
  ahead = _f(getattr(md, "overheadAheadM", 0.0), 0.0)
  kind = str(getattr(md, "overheadKind", "") or "")
  if kind not in KINDS or ahead == 0.0:
    return False
  rng = _f(getattr(obs, "rangeM", None))
  v = _f(v_ego)
  v_rel = _f(getattr(obs, "vRel", None))
  if not (math.isfinite(rng) and math.isfinite(v) and math.isfinite(v_rel)):
    return False
  if abs(rng - ahead) > WINDOW_M[kind]:
    return False
  if abs(v_rel + v) > STATIONARY_MPS:
    return False
  if str(getattr(obs, "objectClass", "unknown")) != "obstacle":
    return False
  if bool(getattr(obs, "modelLead", True)) or bool(getattr(obs, "roadLights", True)):
    return False
  seen = bool(getattr(obs, "roadLevelSeen", True))
  if bool(getattr(obs, "daylight", False)):
    bottom = _f(getattr(obs, "bottomClearanceM", None))
    if seen or (math.isfinite(bottom) and bottom <= CUTOFF_M):
      return False
    return True
  # Night: signals never. Stopped cars wait at signals, and on Oct 14 a stopped
  # tanker at a tagged signal read no lights on most qcamera grabs.
  if kind == "signal":
    return False
  # Night: beyond low-beam reach the camera cannot judge; inside it, recheck every grab.
  if rng <= NIGHT_RECHECK_M and seen:
    return False
  return True


# Overlay file. The overhead layer ships as its own small versioned sqlite
# (selfdrive/nap/maps_update installs it atomically), so a tag update never
# rebuilds the big speed-limit pack. A v6 pack with the table also works.
OVERLAY_FILE = "overhead_us.sqlite"
REOPEN_CHECK_S = 15.0


class _Overlay:
  def __init__(self):
    self.path = ""
    self.con = None
    self.mtime = 0.0
    self.checked = -1e9

  def get(self, now: float):
    if now - self.checked < REOPEN_CHECK_S:
      return self.con
    self.checked = now
    try:
      from openpilot.selfdrive.mapd.db_paths import osm_dir
      import os
      path = os.path.join(osm_dir(), OVERLAY_FILE)
      mtime = os.path.getmtime(path) if os.path.isfile(path) else 0.0
    except Exception:
      return self.con
    if path == self.path and mtime == self.mtime:
      return self.con
    if self.con is not None:
      try:
        self.con.close()
      except Exception:
        pass
    self.con, self.path, self.mtime = None, path, mtime
    if mtime > 0.0:
      try:
        con = sqlite3.connect(f"file:{path}?mode=ro", uri=True, check_same_thread=False)
        self.con = con if has_table(con) else None
      except sqlite3.Error:
        self.con = None
    return self.con


_OVERLAY = _Overlay()


def overhead_ahead_live(lat: float, lon: float, bearing_deg: float, v_ego: float, pack_con=None, now: float | None = None):
  """mapd's one call: overlay file first, else a v6 pack's own table."""
  import time
  con = _OVERLAY.get(time.monotonic() if now is None else now)
  if con is not None:
    return overhead_ahead(con, lat, lon, bearing_deg, v_ego)
  return overhead_ahead(pack_con, lat, lon, bearing_deg, v_ego) if pack_con is not None else None
