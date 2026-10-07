"""Low-visibility lateral back-off.

When the driving model cannot see the lane, or the road camera exposure
collapses (sun low and ahead makes a milder drop count), ease lateral
toward the driver's current steering and raise one alert: "Low visibility".

Openpilot stays engaged. This module does not touch longitudinal control
and does not clear latActive — it only scales how far the lateral command
may sit from the wheel. Settings → NAP → Driving Mannerisms can turn it
Off (NAPLowVisBackoff, default On). Off is identity: authority stays 1
and the alert stays down.

Enter (model): either side's worse of lane line and road edge below
0.30 AND path lateral std at ~3 s above 2 m, held 0.40 s.
Exit: both sides' better of lane line and road edge at or above 0.55,
held 0.80 s. Path uncertainty alone cannot hold the latch once both
sides are clearly visible. A single frame cannot flip the latch.

Enter (camera): integration lines fall below 0.45× a recent bright
baseline (0.65× when the sun is within 12° of the horizon and 22° of
the heading), held 0.30 s. Exit when lines are back above 0.70× for 1 s.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

# --- model confidence -------------------------------------------------------
# laneLineProbs are already 0..1. Road-edge std (m) maps through
# exp(-std / EDGE_STD_SCALE): ~0.1 m → ~0.83, ~0.7 m → ~0.28, ~1.5 m → ~0.07.
LANE_CONF_ENTER = 0.30
LANE_CONF_EXIT = 0.55
EDGE_STD_SCALE_M = 0.55
PATH_T_S = 3.0
PATH_Y_STD_ENTER_M = 2.0
PATH_Y_STD_EXIT_M = 1.2
MODEL_ENTER_S = 0.40
MODEL_EXIT_S = 0.80

# --- camera exposure --------------------------------------------------------
EXPOSURE_WINDOW_S = 12.0
EXPOSURE_EXCLUDE_S = 1.0       # baseline ignores the newest second
EXPOSURE_FAST_S = 0.40
EXPOSURE_MIN_BASELINE = 280    # a camera that was never bright cannot "collapse"
EXPOSURE_COLLAPSE_RATIO = 0.45
EXPOSURE_SUN_RATIO = 0.65      # earlier heads-up when the sun is low and ahead
EXPOSURE_RECOVER_RATIO = 0.70
EXPOSURE_ENTER_S = 0.30
EXPOSURE_EXIT_S = 1.00

# --- sun (optional). Incident was ~0.3–1° up and ~7–8° right of ahead. -----
SUN_ELEV_MIN_DEG = -1.0
SUN_ELEV_MAX_DEG = 12.0
SUN_AHEAD_DEG = 22.0

# --- authority slew ---------------------------------------------------------
AUTHORITY_DOWN_PER_S = 1.0     # full fade in about 1 s
AUTHORITY_UP_PER_S = 0.80      # recovery a little slower than the drop

PARAM_LOW_VIS_BACKOFF = "NAPLowVisBackoff"


@dataclass(frozen=True)
class LowVisibilityOutput:
  authority: float
  active: bool
  alert: bool
  model_poor: bool
  camera_blind: bool


def edge_confidence(std_m: float) -> float:
  """Road-edge lateral std (m) → 0..1 confidence. Unknown std is not low."""
  try:
    std = float(std_m)
  except (TypeError, ValueError):
    return 1.0
  if not math.isfinite(std) or std < 0.0:
    return 1.0
  return math.exp(-std / EDGE_STD_SCALE_M)


def _finite(value, default: float = 1.0) -> float:
  try:
    v = float(value)
  except (TypeError, ValueError):
    return default
  return v if math.isfinite(v) else default


def side_confidences(lane_probs, edge_stds) -> tuple[float, float]:
  """(left, right) confidence. Missing signals count as seen (1), not blind.

  Lane lines: index 1 left, 2 right. Road edges: index 0 left, 1 right.
  A side is the worse of its lane probability and its road-edge confidence.
  """
  probs = list(lane_probs or [])
  edges = list(edge_stds or [])

  def lane(i: int) -> float:
    if i >= len(probs):
      return 1.0
    return _finite(probs[i], 1.0)

  def edge(i: int) -> float:
    if i >= len(edges):
      return 1.0
    return edge_confidence(edges[i])

  return min(lane(1), edge(0)), min(lane(2), edge(1))


def side_confidences_exit(lane_probs, edge_stds) -> tuple[float, float]:
  """(left, right) for the exit check only. Missing signals count as seen.

  Same indexes as side_confidences. A side is the better of its lane
  probability and its road-edge confidence, so a healthy lane line can
  leave low-visibility even when the road-edge std is still wide.
  """
  probs = list(lane_probs or [])
  edges = list(edge_stds or [])

  def lane(i: int) -> float:
    if i >= len(probs):
      return 1.0
    return _finite(probs[i], 1.0)

  def edge(i: int) -> float:
    if i >= len(edges):
      return 1.0
    return edge_confidence(edges[i])

  return max(lane(1), edge(0)), max(lane(2), edge(1))


def path_y_std_at(times, y_stds, t_query: float = PATH_T_S) -> float | None:
  """Interpolate lateral path std (m) at ``t_query`` seconds. None if unusable."""
  try:
    ts = [float(t) for t in (times or [])]
    ys = [float(y) for y in (y_stds or [])]
  except (TypeError, ValueError):
    return None
  n = min(len(ts), len(ys))
  pairs = [(t, y) for t, y in zip(ts[:n], ys[:n]) if math.isfinite(t) and math.isfinite(y)]
  if len(pairs) < 2:
    return None
  pairs.sort(key=lambda p: p[0])
  want = float(t_query)
  if want <= pairs[0][0]:
    return pairs[0][1]
  if want >= pairs[-1][0]:
    return pairs[-1][1]
  for (t0, y0), (t1, y1) in zip(pairs, pairs[1:]):
    if t0 <= want <= t1 and t1 > t0:
      w = (want - t0) / (t1 - t0)
      return y0 + w * (y1 - y0)
  return None


def model_is_poor(left: float, right: float, y_std: float | None) -> bool:
  if y_std is None or not math.isfinite(y_std):
    return False
  side_low = left < LANE_CONF_ENTER or right < LANE_CONF_ENTER
  return side_low and y_std > PATH_Y_STD_ENTER_M


def model_is_clear(left: float, right: float, y_std: float | None) -> bool:
  """Exit bar. ``left``/``right`` are exit confidences (better of lane and edge).

  Once both sides are clearly visible, path std cannot hold the latch.
  ``y_std`` stays in the signature so callers and older tests still pass it;
  it does not block a clear exit.
  """
  del y_std
  return left >= LANE_CONF_EXIT and right >= LANE_CONF_EXIT


def fade_curvature(model_k: float, measured_k: float, authority: float) -> float:
  """Blend the lateral target toward the wheel. Authority 1 is the model."""
  try:
    a = float(authority)
    mk = float(model_k)
    meas = float(measured_k)
  except (TypeError, ValueError):
    return float(measured_k) if measured_k is not None else 0.0
  if not math.isfinite(a):
    a = 1.0
  if not math.isfinite(mk):
    mk = meas if math.isfinite(meas) else 0.0
  if not math.isfinite(meas):
    meas = 0.0
  a = min(1.0, max(0.0, a))
  if a >= 1.0:
    return mk
  if a <= 0.0:
    return meas
  return a * mk + (1.0 - a) * meas


def angle_delta_deg(a_deg: float, b_deg: float) -> float:
  return abs((float(a_deg) - float(b_deg) + 180.0) % 360.0 - 180.0)


def sun_low_and_ahead(*, elevation_deg: float, heading_deg: float, azimuth_deg: float) -> bool:
  """Sun near the horizon and roughly in front of the camera heading."""
  try:
    elev = float(elevation_deg)
    heading = float(heading_deg)
    azimuth = float(azimuth_deg)
  except (TypeError, ValueError):
    return False
  if not all(math.isfinite(v) for v in (elev, heading, azimuth)):
    return False
  if not SUN_ELEV_MIN_DEG <= elev <= SUN_ELEV_MAX_DEG:
    return False
  return angle_delta_deg(heading, azimuth) <= SUN_AHEAD_DEG


def solar_position(lat_deg: float, lon_deg: float, unix_s: float) -> tuple[float, float]:
  """Approximate (elevation_deg, azimuth_deg). Azimuth is clockwise from north.

  About a degree, which is enough for "low and ahead" versus overhead or behind.
  """
  lat = float(lat_deg)
  lon = float(lon_deg)
  # Days since 2000-01-01 12:00 UTC (J2000-ish).
  days = (float(unix_s) - 946728000.0) / 86400.0
  mean_long = (280.460 + 0.9856474 * days) % 360.0
  mean_anom = math.radians((357.528 + 0.9856003 * days) % 360.0)
  lam = math.radians((mean_long + 1.915 * math.sin(mean_anom) + 0.020 * math.sin(2.0 * mean_anom)) % 360.0)
  eps = math.radians(23.439 - 0.0000004 * days)
  dec = math.asin(max(-1.0, min(1.0, math.sin(eps) * math.sin(lam))))
  ra = math.atan2(math.cos(eps) * math.sin(lam), math.cos(lam))
  gmst = (280.46061837 + 360.98564736629 * days) % 360.0
  lmst = math.radians((gmst + lon) % 360.0)
  ha = lmst - ra
  lat_r = math.radians(lat)
  sin_el = (math.sin(lat_r) * math.sin(dec)
            + math.cos(lat_r) * math.cos(dec) * math.cos(ha))
  elev = math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))
  # Azimuth clockwise from north. -sin(ha) puts morning sun in the east.
  az = math.degrees(math.atan2(
    -math.sin(ha),
    math.tan(dec) * math.cos(lat_r) - math.sin(lat_r) * math.cos(ha),
  ))
  az = (az + 360.0) % 360.0
  return elev, az


def sun_ahead_from_fix(*, latitude: float, longitude: float, unix_timestamp_millis: int,
                       bearing_deg: float, horizontal_accuracy_m: float = 0.0) -> bool | None:
  """True/False when the fix is usable. None when sun position should be ignored."""
  try:
    lat = float(latitude)
    lon = float(longitude)
    ts_ms = int(unix_timestamp_millis)
    bearing = float(bearing_deg)
    acc = float(horizontal_accuracy_m or 0.0)
  except (TypeError, ValueError):
    return None
  if not all(math.isfinite(v) for v in (lat, lon, bearing, acc)):
    return None
  if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
    return None
  if ts_ms < 1_000_000_000_000:  # need real unix milliseconds
    return None
  if acc > 50.0:
    return None
  elev, az = solar_position(lat, lon, ts_ms / 1000.0)
  return sun_low_and_ahead(elevation_deg=elev, heading_deg=bearing, azimuth_deg=az)


class _Exposure:
  def __init__(self):
    self._samples: deque[tuple[float, float]] = deque()
    self._t = 0.0
    self.blind = False
    self._enter_s = 0.0
    self._exit_s = 0.0

  def update(self, integ_lines, sun_ahead: bool, dt: float) -> bool:
    self._t += dt
    try:
      integ = float(integ_lines) if integ_lines is not None else None
    except (TypeError, ValueError):
      integ = None
    if integ is not None and math.isfinite(integ) and integ > 0.0:
      self._samples.append((self._t, integ))
    horizon = self._t - EXPOSURE_WINDOW_S
    while self._samples and self._samples[0][0] < horizon:
      self._samples.popleft()

    baseline = self._baseline()
    current = self._current()
    ratio = None
    if baseline is not None and current is not None and baseline >= EXPOSURE_MIN_BASELINE:
      ratio = current / baseline
    enter_ratio = EXPOSURE_SUN_RATIO if sun_ahead else EXPOSURE_COLLAPSE_RATIO
    raw = ratio is not None and ratio < enter_ratio
    clear = ratio is not None and ratio > EXPOSURE_RECOVER_RATIO
    if raw:
      self._enter_s += dt
      self._exit_s = 0.0
      if self._enter_s + 1e-12 >= EXPOSURE_ENTER_S:
        self.blind = True
    elif clear:
      self._exit_s += dt
      self._enter_s = 0.0
      if self._exit_s + 1e-12 >= EXPOSURE_EXIT_S:
        self.blind = False
    else:
      self._enter_s = 0.0
      self._exit_s = 0.0
    return self.blind

  def _baseline(self) -> float | None:
    old = [v for t, v in self._samples if self._t - t >= EXPOSURE_EXCLUDE_S]
    if len(old) < 3:
      return None
    return max(old)

  def _current(self) -> float | None:
    recent = [v for t, v in self._samples if self._t - t <= EXPOSURE_FAST_S]
    if not recent:
      return None
    return sum(recent) / len(recent)

  def clear_latch(self):
    self.blind = False
    self._enter_s = 0.0
    self._exit_s = 0.0


class LowVisibility:
  """Filter model + exposure into one latch and a slewed lateral authority."""

  def __init__(self):
    self.authority = 1.0
    self.active = False
    self._model = False
    self._poor_s = 0.0
    self._clear_s = 0.0
    self._exposure = _Exposure()

  def reset_latch(self):
    self.active = False
    self._model = False
    self._poor_s = 0.0
    self._clear_s = 0.0
    self.authority = 1.0
    self._exposure.clear_latch()

  def update(self, *, enabled: bool, lane_probs=None, edge_stds=None,
             path_t=None, path_y_std=None, integ_lines=None,
             sun_ahead: bool = False, dt: float = 0.01) -> LowVisibilityOutput:
    dt = float(dt) if dt and math.isfinite(float(dt)) else 0.01
    left, right = side_confidences(lane_probs, edge_stds)
    exit_left, exit_right = side_confidences_exit(lane_probs, edge_stds)
    y_std = path_y_std_at(path_t, path_y_std, PATH_T_S)
    poor = model_is_poor(left, right, y_std)
    clear = model_is_clear(exit_left, exit_right, y_std)
    # Entry still uses the worse of lane and edge, plus path std. Once
    # the latch is up, the better of lane and edge on each side can
    # leave even if the other signal (or path std) would still look poor.
    if self._model:
      if clear:
        self._clear_s += dt
        self._poor_s = 0.0
        if self._clear_s + 1e-12 >= MODEL_EXIT_S:
          self._model = False
      else:
        self._poor_s = 0.0
        self._clear_s = 0.0
    elif poor:
      self._poor_s += dt
      self._clear_s = 0.0
      if self._poor_s + 1e-12 >= MODEL_ENTER_S:
        self._model = True
    else:
      self._poor_s = 0.0
      self._clear_s = 0.0

    camera = self._exposure.update(integ_lines, bool(sun_ahead), dt)
    if not enabled:
      self._model = False
      self._poor_s = 0.0
      self._clear_s = 0.0
      self._exposure.clear_latch()
      self.active = False
      self.authority = 1.0
      return LowVisibilityOutput(1.0, False, False, False, False)

    self.active = bool(self._model or camera)
    target = 0.0 if self.active else 1.0
    if self.authority > target:
      self.authority = max(target, self.authority - AUTHORITY_DOWN_PER_S * dt)
    elif self.authority < target:
      self.authority = min(target, self.authority + AUTHORITY_UP_PER_S * dt)
    return LowVisibilityOutput(
      float(self.authority), bool(self.active), bool(self.active),
      bool(self._model), bool(camera))
