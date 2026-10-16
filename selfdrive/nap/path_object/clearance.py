"""NAP overhead clearance gate for path objects (filter only, no braking).

Justin's rule: ignore any radar+vision path object whose bottom sits more
than 6 ft (1.83 m) above the road. Pre-AP radar has no elevation, so the
height comes from the road camera:

  For each camera grab of a track, the caller passes the Y plane plus the
  calibrated pixel rows of the radar point at road level (h = 0) and at
  h = 1 m. That gives pixels per metre at the radar range, so any height h
  maps to a row: v(h) = v_ground - h * ppm. We read horizontal-edge energy
  (vertical intensity gradient) in a car-width strip at each height on a
  grid. A real object edge at height h lands on the same h in every frame
  as the range shrinks; background farther away (tree lines, horizon)
  drifts across heights and averages out. The bottom is the lowest height
  whose averaged energy reaches half the peak.

Ignore needs, all of:
  * at least MIN_FRAMES grabs of the same track with a range spread of
    at least MIN_RANGE_SPREAD (so the edge actually tracked the radar),
  * bottom - error(d) > CUTOFF_M, error(d) = ERR_BASE_M + d * tan(ERR_PITCH_RAD),
  * inside NEAR_RANGE_M, bottom >= NEAR_MIN_M as well.
  * daylight: GPS sun elevation above SUN_MIN_DEG. At night the camera cannot see an unlit overhead edge, and
    taillights on a real vehicle can read as a high bottom (Oct 14 replay,
    track 977), so the gate never ignores at night. No GPS fix counts as night.
(The helper's lightingScore is not a day/night signal: it read 0.33 under
the Oct 15 overcast-noon bridge, same band as night.)
Once ignored, CLEAR_LOW_READS consecutive low reads (bottom <= cutoff) clear
it. One low read can be a bridge shadow on the road; three in a row at 5 Hz
(0.6 s) is a real low object.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

CUTOFF_M = 1.83              # 6 ft
NEAR_RANGE_M = 40.0
NEAR_MIN_M = 3.0
ERR_BASE_M = 0.10            # bottom-edge pixel error
ERR_PITCH_RAD = math.radians(0.4)  # residual calibration + body pitch
H_GRID = np.arange(0.0, 8.01, 0.25)
MIN_FRAMES = 3
MIN_RANGE_SPREAD = 0.15      # (max d - min d) / max d
PEAK_FRAC = 0.5
MIN_PPM = 4.0                # below ~4 px per metre the grid is too coarse
MAX_TRACKS = 16
CLEAR_LOW_READS = 3
SUN_MIN_DEG = 3.0
STALE_S = 2.0


def error_m(range_m: float) -> float:
  return ERR_BASE_M + max(0.0, float(range_m)) * math.tan(ERR_PITCH_RAD)


def edge_profile(plane, u: float, v_ground: float, ppm: float, half_w_px: float) -> np.ndarray | None:
  """Normalised horizontal-edge energy at each H_GRID height, or None."""
  if plane is None or not (math.isfinite(u) and math.isfinite(v_ground) and math.isfinite(ppm)) or ppm < MIN_PPM:
    return None
  h_img, w_img = plane.shape[:2]
  x0, x1 = int(max(0, u - half_w_px)), int(min(w_img, u + half_w_px))
  if x1 - x0 < 3:
    return None
  out = np.full(len(H_GRID), np.nan)
  half_band = max(1, int(round(ppm * 0.125)))
  for i, h in enumerate(H_GRID):
    v = int(round(v_ground - h * ppm))
    a, b = v - half_band, v + half_band + 1
    if a < 0 or b >= h_img:
      continue
    strip = plane[a:b + 1, x0:x1].astype(np.float32)
    out[i] = float(np.abs(np.diff(strip, axis=0)).mean())
  if not np.isfinite(out).any():
    return None
  peak = np.nanmax(out)
  return out / peak if peak > 1e-3 else np.zeros_like(out)


def bottom_from(acc: np.ndarray) -> float:
  """Lowest grid height reaching PEAK_FRAC of the peak."""
  if acc is None or not np.isfinite(acc).any():
    return float("nan")
  peak = np.nanmax(acc)
  if peak <= 1e-6:
    return float("nan")
  idx = np.where(np.nan_to_num(acc, nan=0.0) >= PEAK_FRAC * peak)[0]
  return float(H_GRID[idx[0]]) if len(idx) else float("nan")


def should_ignore(bottom_m: float, range_m: float, frames: int, spread: float) -> bool:
  if not (math.isfinite(bottom_m) and math.isfinite(range_m)):
    return False
  if frames < MIN_FRAMES or spread < MIN_RANGE_SPREAD:
    return False
  if bottom_m - error_m(range_m) <= CUTOFF_M:
    return False
  if range_m < NEAR_RANGE_M and bottom_m < NEAR_MIN_M:
    return False
  return True



def daylight_from_gps(gps) -> bool:
  """True when a GPS fix puts the sun above SUN_MIN_DEG. False on any doubt."""
  if gps is None:
    return False
  try:
    from openpilot.selfdrive.controls.lib.lat_low_visibility import solar_position
    lat, lon, ms = float(gps.latitude), float(gps.longitude), float(gps.unixTimestampMillis)
    if not (math.isfinite(lat) and math.isfinite(lon) and ms > 0) or (lat == 0.0 and lon == 0.0):
      return False
    elevation, _ = solar_position(lat, lon, ms / 1000.0)
    return math.isfinite(elevation) and elevation > SUN_MIN_DEG
  except Exception:
    return False


@dataclass
class _Track:
  acc: np.ndarray
  n: np.ndarray
  frames: int = 0
  d_min: float = math.inf
  d_max: float = 0.0
  last_t: float = 0.0
  ignored: bool = False
  low_run: int = 0


@dataclass
class ClearanceGate:
  tracks: dict = field(default_factory=dict)
  last: dict = field(default_factory=dict)   # track id -> (bottom_m, ignore)

  def update(self, track_id: int, range_m: float, profile, now: float, daylight: bool = False) -> tuple[float, bool]:
    """Fold one grab in. Returns (bottomClearanceM, overheadIgnore)."""
    tid = int(track_id)
    for k in [k for k, t in self.tracks.items() if now - t.last_t > STALE_S]:
      self.tracks.pop(k, None)
      self.last.pop(k, None)
    if profile is None or not math.isfinite(range_m):
      return self.last.get(tid, (float("nan"), False))
    t = self.tracks.get(tid)
    if t is None:
      if len(self.tracks) >= MAX_TRACKS:
        old = min(self.tracks, key=lambda k: self.tracks[k].last_t)
        self.tracks.pop(old)
        self.last.pop(old, None)
      t = self.tracks[tid] = _Track(np.zeros(len(H_GRID)), np.zeros(len(H_GRID)))
    ok = np.isfinite(profile)
    t.acc[ok] += profile[ok]
    t.n[ok] += 1
    t.frames += 1
    t.d_min, t.d_max = min(t.d_min, range_m), max(t.d_max, range_m)
    t.last_t = now
    mean = np.where(t.n > 0, t.acc / np.maximum(t.n, 1), np.nan)
    bottom = bottom_from(mean)
    bottom_now = bottom_from(profile)
    low_now = math.isfinite(bottom_now) and bottom_now <= CUTOFF_M
    t.low_run = t.low_run + 1 if low_now else 0
    spread = (t.d_max - t.d_min) / t.d_max if t.d_max > 0 else 0.0
    if not daylight or t.low_run >= CLEAR_LOW_READS:
      t.ignored = False
    elif t.ignored and range_m < NEAR_RANGE_M and not (math.isfinite(bottom) and bottom >= NEAR_MIN_M):
      t.ignored = False
    elif not t.ignored:
      t.ignored = should_ignore(bottom, range_m, t.frames, spread) and not low_now
    if t.low_run >= CLEAR_LOW_READS and math.isfinite(bottom_now):
      bottom = min(bottom, bottom_now) if math.isfinite(bottom) else bottom_now
    res = (bottom, t.ignored)
    self.last[tid] = res
    return res

  def get(self, track_id: int) -> tuple[float, bool]:
    return self.last.get(int(track_id), (float("nan"), False))
