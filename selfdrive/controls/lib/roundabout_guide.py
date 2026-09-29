"""Roundabout Steering Assist (Pre-AP, NAPRoundaboutAssist, default Off).

Sep 28 PM (R126 16:52, R128 17:38 CT): through the Soco / Dellwood / Jonathan
Creek ring the model's curvature was a wide, slow S. It let go of the right
entry ~1 s early (outer lane, drifted toward the inner lane) or held it ~1 s
late and swung left at ~0.025 /m/s (inner lane, drifted out). Actuation, the
360° cap, the 250°/s rate limit and controlsd clipping were not involved.

Near / on a mapped OSM ring this blends the model's curvature toward a
circle follower for the lane (feed-forward 1/r + damped radial correction):

  * Pose: 1 Hz GNSS (≈0.66 s latency, compensated with odometry history) fused
    with wheel speed + yaw rate, in a local frame at the ring center.
  * Entry: farther than D_TURN_M (9 m) from the ring edge the blend can only
    hold / add right (never an early left). Inside D_TURN_M the target is the
    circle-follow curvature (1 s preview); target and output slew ≤ 0.035 /m/s.
    The circulation-side curl is capped at 2.2 m/s² (actual speed) and held
    while the car is > 3 mph over the ring speed.
  * Lane: target radius is R + 0.75·half-width until the car is tangent in
    the ring band, then the car's own radius (clamped to R ± half-width/2).
  * Exit: after ≥ 45° of circulation, the model asking for more right than
    the ring (it wants the exit) or the car leaving the band fades out.
  * Weight: W_MAX × map-match confidence (fix age, GNSS/odometry innovation,
    consistent fixes, distance to the mapped ring / approach roads, ring fit
    quality). The correction is capped at DK_MAX and is limited so the
    predicted path keeps a margin to the camera's lane lines / road edges.
  * Sep 29 11:57:48 (Scallywag): the left curl was ~2x the model, aimed at an
    inner-clamped line from a pose that read inside the lane (bearing accuracy
    30-50°, map-match bias 1.2→3.4 m). Now an unreliable pose (bearing accuracy
    > 30°, or bias > 2 m with bearing accuracy > 20°, or a bias still moving
    > 1 m) does not latch and the assist fades out (the model keeps the pass);
    a merely poor pose latches without the inner clamp and the assist is
    capped at 1.2x the model curvature for 3 s.
  * Driver torque overrides exactly as before: this only edits the model
    curvature upstream of the handoff / clip_curvature path.

Curvature sign is openpilot's (positive = right, NED yaw). Pure functions and
one small state class so it replays and unit-tests without cereal.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.mapd.roundabout import roundabout_target_ms
from openpilot.selfdrive.mapd.roundabout_map import RingGeometry, local_xy, roundabout_comfort_speed_ms

PARAM_ROUNDABOUT_ASSIST = "NAPRoundaboutAssist"

GPS_LATENCY_S = 0.66         # qcom gpsLocation vs logMonoTime, measured on the Sep 29 route (R126/R128 xcorr gave 0.9–1.0 s)
GPS_EXT_LATENCY_S = 0.2      # ublox gpsLocationExternal (not in the Sep 28 logs; unvalidated)
FIX_FRESH_S = 1.5
FIX_STALE_S = 3.0
HISTORY_S = 4.0
INNOV_GOOD_M = 3.0
INNOV_BAD_M = 8.0
INNOV_JUMP_M = 15.0
INNOV_POOR_M = 6.0
SPEED_TOL_MS = 2.5            # GNSS speed lags ~1 s like position; loose check
POS_GAIN = 0.5
HEADING_GAIN = 0.3
GOOD_FIXES = 3

V_MIN = 2.0                  # m/s
V_MAX = 13.5                 # m/s (~30 mph); the ring slow-down should be well under this
ENGAGE_EDGE_M = 25.0         # start blending (weight ramps) this far from the ring edge
FULL_EDGE_M = 12.0
D_TURN_M = 9.0               # hold the right entry until this far from the ring edge
LEAVE_EDGE_M = 60.0          # forget a finished ring once this far away
RING_TRIM_K = 0.02      # 1/m: max pure-pursuit trim around the ring curvature once circulating
A_LAT_MAX = 3.0            # m/s^2: cap on the guide target away from the circulation direction (right entry)
A_LAT_CIRC_MAX = 2.2       # m/s^2: cap on the circulation-side (left on a CCW ring) transition target before the latch, at actual speed
FOLLOW_OMEGA = 0.7            # rad/s, circle-follow bandwidth
FOLLOW_ZETA = 0.9
PRE_LATCH_HW = 0.75          # entry aims R + 0.75 hw (outer lane, away from the island)
OUT_SLEW = 0.035            # 1/m per s, max change of the blended output while active (poor / unverified pose)
OUT_SLEW_GOOD = 0.07         # ...with a good pose (R126/R128 fixtures need it to stay in the lane band)
HANDOFF_DK = 0.00005         # output must be this close to the model before handing back (well under one slew step)
PREVIEW_S = 1.0             # s, circle-follow preview (R126/R128 replay: 0.6-1.2 s trade inner vs outer drift)
TARGET_SLEW = 0.035          # 1/m per s, transition rate of the guide target (poor / unverified pose)
TARGET_SLEW_GOOD = 0.07
W_MAX = 0.8
W_RATE = 2.0                 # phase weight ramp, 1/s
DK_MAX = 0.045               # max |correction| (1/m)
TANGENT_DEG = 25.0
EXIT_MIN_TRAVEL_DEG = 45.0
EXIT_DK = 0.015
EXIT_HOLD_S = 0.3
EDGE_MARGIN_M = 0.3
HALF_CAR_M = 0.98
APPROACH_HALF_W_M = 4.0
MATCH_COS = 0.9              # approach segment (stored in travel direction) within ~25° of the heading
MATCH_MAX_M = 12.0
LANE_SNAP_M = 1.75           # one lane off the OSM centerline is not a GNSS error
BIAS_GAIN = 0.3
BIAS_MAX_M = 8.0
BEARING_ACC_UNRELIABLE_DEG = 30.0   # no latch / no inner clamp beyond this (Sep 29: 30-50°)
BIAS_UNRELIABLE_M = 2.0
BEARING_ACC_POOR_DEG = 20.0         # poor: latch allowed, assist capped at POOR_GPS_CAP x model
BIAS_DRIFT_M = 1.0                  # bias moved more than this over BIAS_WINDOW_S: poor pose
BIAS_LATCH_DRIFT_M = 1.0            # ...and more than this: not bias-consistent, do not latch yet
BIAS_WINDOW_S = 3.0
POOR_GPS_CAP = 1.2
POOR_GPS_FLOOR = 0.004              # 1/m
CAP_WINDOW_S = 3.0                  # the poor-GPS cap covers the left transition after the latch
SPEED_HOLD_MS = 3.0 * 0.44704       # hold the circulation-side curl until within 3 mph of the ring speed
LOG_PERIOD_S = 1.0


def _interp(x: float, xp: tuple[float, float], fp: tuple[float, float]) -> float:
  if x <= xp[0]:
    return fp[0]
  if x >= xp[1]:
    return fp[1]
  return fp[0] + (fp[1] - fp[0]) * (x - xp[0]) / (xp[1] - xp[0])


def _wrap(a: float) -> float:
  return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rot(x: float, y: float, d: float) -> tuple[float, float]:
  """Rotate a vector clockwise by d (bearing sense): heading b → b + d."""
  c, s = math.cos(d), math.sin(d)
  return x * c + y * s, -x * s + y * c


class PoseTracker:
  """GNSS + odometry pose in a local east/north frame (m) with bearing (rad, cw from north)."""

  def __init__(self) -> None:
    self.origin: tuple[float, float] | None = None
    self.reset()

  def reset(self) -> None:
    self.xo = self.yo = self.psio = 0.0
    self.t = None
    self.hist: deque = deque()
    self.initialized = False
    self.dpsi = 0.0
    self.tx = self.ty = 0.0
    self.innov = INNOV_BAD_M
    self.n_good = 0
    self.last_fix_t = -1e9
    self.bacc = 0.0     # last reported bearing accuracy (deg); 0 = unknown

  def set_origin(self, lat: float, lon: float) -> None:
    if self.origin is None or abs(self.origin[0] - lat) > 1e-7 or abs(self.origin[1] - lon) > 1e-7:
      self.origin = (float(lat), float(lon))
      keep_hist, keep_t = self.hist, self.t
      xo, yo, psio = self.xo, self.yo, self.psio
      self.reset()
      self.hist, self.t, self.xo, self.yo, self.psio = keep_hist, keep_t, xo, yo, psio

  def predict(self, t: float, v: float, yaw_rate: float) -> None:
    if not (math.isfinite(t) and math.isfinite(v) and math.isfinite(yaw_rate)):
      return   # a NaN would poison the odometry for the rest of the drive (seen in R126 qlog yaw)
    if self.t is not None:
      dt = min(max(t - self.t, 0.0), 0.2)
      self.psio += yaw_rate * dt
      self.xo += v * math.sin(self.psio) * dt
      self.yo += v * math.cos(self.psio) * dt
    self.t = t
    self.hist.append((t, self.xo, self.yo, self.psio, v))
    while self.hist and self.hist[0][0] < t - HISTORY_S:
      self.hist.popleft()

  def _odo_at(self, t: float):
    h = self.hist
    if h and h[-1][0] < t <= h[-1][0] + 0.05:
      t = h[-1][0]   # fix processed just before this cycle's predict
    if not h or t < h[0][0] or t > h[-1][0]:
      return None
    lo, hi = 0, len(h) - 1
    while hi - lo > 1:
      mid = (lo + hi) // 2
      if h[mid][0] <= t:
        lo = mid
      else:
        hi = mid
    a, b = h[lo], h[hi]
    f = 0.0 if b[0] <= a[0] else (t - a[0]) / (b[0] - a[0])
    return tuple(a[k] + f * (b[k] - a[k]) for k in range(1, 5))

  def _world(self, xo: float, yo: float) -> tuple[float, float]:
    x, y = _rot(xo, yo, self.dpsi)
    return x + self.tx, y + self.ty

  def fix(self, t_log: float, lat: float, lon: float, bearing_deg: float | None,
          gps_speed: float | None = None, bearing_acc_deg: float | None = None,
          latency_s: float = GPS_LATENCY_S) -> None:
    if self.origin is None or not all(math.isfinite(z) for z in (t_log, lat, lon)):
      return
    self.bacc = float(bearing_acc_deg) if (bearing_acc_deg is not None and math.isfinite(bearing_acc_deg)
                                           and bearing_acc_deg > 0.0) else 0.0
    odo = self._odo_at(t_log - latency_s)
    if odo is None:
      return
    xo, yo, psio, v_fix = odo
    fx, fy = local_xy(lat, lon, self.origin[0], self.origin[1])
    b_ok = (bearing_deg is not None and math.isfinite(bearing_deg) and v_fix > 3.0
            and (bearing_acc_deg is None or bearing_acc_deg <= 0.0 or bearing_acc_deg < 10.0))
    speed_ok = gps_speed is None or not math.isfinite(gps_speed) or abs(gps_speed - v_fix) < max(SPEED_TOL_MS, 0.25 * v_fix)
    if not self.initialized:
      if not b_ok:
        return
      self.dpsi = _wrap(math.radians(bearing_deg) - psio)
      wx, wy = _rot(xo, yo, self.dpsi)
      self.tx, self.ty = fx - wx, fy - wy
      self.initialized = True
      self.innov = INNOV_GOOD_M
      self.n_good = 1 if speed_ok else 0
      self.last_fix_t = t_log
      return
    px, py = self._world(xo, yo)
    ex, ey = fx - px, fy - py
    e = math.hypot(ex, ey)
    if e > INNOV_JUMP_M:
      # GNSS jump or a bad track: re-anchor, confidence starts over.
      self.initialized = False
      self.n_good = 0
      self.innov = INNOV_BAD_M
      self.fix(t_log, lat, lon, bearing_deg, gps_speed, bearing_acc_deg, latency_s)
      self.n_good = 0
      return
    self.innov = 0.7 * self.innov + 0.3 * e
    ax, ay = px + POS_GAIN * ex, py + POS_GAIN * ey
    if b_ok:
      self.dpsi += HEADING_GAIN * _wrap(math.radians(bearing_deg) - (psio + self.dpsi))
    wx, wy = _rot(xo, yo, self.dpsi)
    self.tx, self.ty = ax - wx, ay - wy
    # One poor fix costs two good ones (a hard reset made the assist drop out mid-ring on R126).
    self.n_good = self.n_good + 1 if (e < INNOV_POOR_M and speed_ok) else max(self.n_good - 2, 0)
    self.last_fix_t = t_log

  def pose(self) -> tuple[float, float, float] | None:
    if not self.initialized:
      return None
    x, y = self._world(self.xo, self.yo)
    return x, y, self.psio + self.dpsi

  def confidence(self, t: float) -> float:
    if not self.initialized:
      return 0.0
    c_fix = _interp(t - self.last_fix_t, (FIX_FRESH_S, FIX_STALE_S), (1.0, 0.0))
    c_innov = _interp(self.innov, (INNOV_GOOD_M, INNOV_BAD_M), (1.0, 0.0))
    c_count = min(1.0, self.n_good / GOOD_FIXES)
    return min(c_fix, c_innov, c_count)


def _arc_ahead(x: float, y: float, psi: float, v: float, yaw_rate: float, dt: float) -> tuple[float, float, float]:
  """Pose after dt s at constant speed and yaw rate (x east, y north, psi cw from north)."""
  n = 5
  h = dt / n
  for _ in range(n):
    psi_m = psi + 0.5 * yaw_rate * h
    x += v * math.sin(psi_m) * h
    y += v * math.cos(psi_m) * h
    psi += yaw_rate * h
  return x, y, psi


def circle_follow_curvature(x: float, y: float, psi: float, v: float, r_ref: float, ccw: bool) -> float:
  """Curvature (positive right) that follows the circle r_ref (center 0,0) in the circulation direction.

  Feed-forward 1/r_ref plus a critically-damped correction on radial offset
  e = r - r_ref and outward heading angle phi (radial speed = v sin phi):
    k_center = 1/r_ref + (2 ZETA OMEGA v sin(phi) + OMEGA^2 e) / v^2
  which makes e'' + 2 ZETA OMEGA e' + OMEGA^2 e ~= 0. Moving against the
  circulation counts as fully outward (turn toward the ring direction).
  """
  r = math.hypot(x, y)
  if r < 1e-3 or r_ref <= 0.0:
    return 0.0
  sense = 1.0 if ccw else -1.0
  hx, hy = math.sin(psi), math.cos(psi)            # heading unit vector (x east, y north)
  radial = (hx * x + hy * y) / r                    # sin(phi), outward positive
  tangential = sense * (-hx * y + hy * x) / r       # along the circulation
  sin_phi = radial if tangential > 0.0 else math.copysign(1.0, radial if radial != 0.0 else 1.0)
  vv = max(v, 3.0)
  k_center = 1.0 / r_ref + (2.0 * FOLLOW_ZETA * FOLLOW_OMEGA * vv * sin_phi + FOLLOW_OMEGA ** 2 * (r - r_ref)) / (vv * vv)
  return -sense * k_center


def _dist_to_polyline(x: float, y: float, pts: list[tuple[float, float]]) -> float:
  best = 1e9
  for (ax, ay), (bx, by) in zip(pts, pts[1:], strict=False):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    f = 0.0 if L2 <= 1e-9 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
    best = min(best, math.hypot(x - (ax + f * dx), y - (ay + f * dy)))
  return best


def map_match_confidence(ring: RingGeometry, x: float, y: float) -> float:
  """1 on the mapped ring band / an approach road, 0 once ≥ 4 m outside every mapped road."""
  r = math.hypot(x, y)
  excess = max(0.0, abs(r - ring.radius_m) - ring.half_width_m)
  on_approach_side = r > ring.radius_m + ring.half_width_m
  if on_approach_side:
    if ring.approaches:
      d_app = min(_dist_to_polyline(x, y, a) for a in ring.approaches if len(a) >= 2)
      excess = min(excess, max(0.0, d_app - APPROACH_HALF_W_M))
    else:
      return 0.7 * _interp(excess, (1.0, 45.0), (1.0, 0.0)) if excess < 45.0 else 0.0
  return _interp(excess, (1.0, 5.0), (1.0, 0.0))


def map_match_offset(ring: RingGeometry, x: float, y: float, psi: float) -> tuple[float, float, float] | None:
  """GNSS offset from the mapped road that no lane can explain: (nx, ny, excess_m) or None.

  On an approach road: signed perpendicular offset from the nearest roughly
  parallel segment, minus LANE_SNAP_M (a car can legitimately be one lane off
  the OSM centerline). On the ring: radial offset beyond the lane centers.
  """
  r = math.hypot(x, y)
  if r < 8.0:
    return None
  R, hw = ring.radius_m, ring.half_width_m
  if r <= R + hw + 2.0:
    e = r - R
    ex = math.copysign(max(abs(e) - 0.5 * hw, 0.0), e)
    return x / r, y / r, ex
  best = None
  hx, hy = math.sin(psi), math.cos(psi)
  for pts in ring.approaches:
    for (ax, ay), (bx, by) in zip(pts, pts[1:], strict=False):
      dx, dy = bx - ax, by - ay
      L = math.hypot(dx, dy)
      if L < 1e-3 or (dx * hx + dy * hy) / L < MATCH_COS:   # travel direction (see ring_geometry)
        continue
      f = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (L * L)))
      px, py = ax + f * dx, ay + f * dy
      d = math.hypot(x - px, y - py)
      if d <= MATCH_MAX_M and (best is None or d < best[0]):
        nx, ny = dy / L, -dx / L        # right-hand normal of the segment
        best = (d, nx, ny, (x - px) * nx + (y - py) * ny)
  if best is None:
    return None
  _, nx, ny, e = best
  return nx, ny, math.copysign(max(abs(e) - LANE_SNAP_M, 0.0), e)


def ring_confidence(ring: RingGeometry) -> float:
  c = min(_interp(ring.rms_m, (1.0, 3.0), (1.0, 0.0)), _interp(ring.coverage_deg, (200.0, 300.0), (0.0, 1.0)))
  return c if ring.tagged else 0.7 * c


@dataclass
class EdgeInfo:
  """Camera lateral bounds at lookahead x (m). y positive right (device frame)."""
  x: float
  path_y: float
  left_y: float | None = None
  right_y: float | None = None


def limit_correction_by_edges(dk: float, edges: EdgeInfo | None) -> float:
  """Scale the correction so the predicted path keeps a margin to lane lines / road edges."""
  if edges is None or abs(dk) < 1e-9:
    return dk
  x = max(edges.x, 3.0)
  dy = 0.5 * dk * x * x
  margin = HALF_CAR_M + EDGE_MARGIN_M
  if dk < 0.0 and edges.left_y is not None:
    allowed = edges.left_y + margin - edges.path_y   # most-negative lateral move allowed
    dy = max(dy, min(0.0, allowed))
  if dk > 0.0 and edges.right_y is not None:
    allowed = edges.right_y - margin - edges.path_y
    dy = min(dy, max(0.0, allowed))
  return 2.0 * dy / (x * x)


def lat_limited(k: float, v: float, sense: float, latched: bool = False) -> float:
  """Clip a target curvature: A_LAT_CIRC_MAX toward the circulation side (left on CCW) during the transition,
  A_LAT_MAX once latched (the ring itself needs ~2.3 m/s² at 15.8 mph) and away from the circulation side."""
  v2 = max(v * v, 1.0)
  k_circ, k_other = (A_LAT_MAX if latched else A_LAT_CIRC_MAX) / v2, A_LAT_MAX / v2
  lo, hi = (-k_circ, k_other) if sense > 0.0 else (-k_other, k_circ)
  return min(max(k, lo), hi)


def hold_curl(k_raw: float, model_k: float, sense: float) -> float:
  """Never add more circulation-side curvature than the model asks for (hold / add right only on CCW)."""
  return max(model_k, k_raw) if sense > 0.0 else min(model_k, k_raw)


def cap_poor_gps(dk: float, model_k: float, sense: float) -> float:
  """Poor GPS after the latch: the circulation-side correction stays within (POOR_GPS_CAP - 1) x |model| (+ a small
  floor so a near-zero model can still start the curl). Corrections away from the circulation side are untouched."""
  c = (POOR_GPS_CAP - 1.0) * abs(model_k) + POOR_GPS_FLOOR
  return max(dk, -c) if sense > 0.0 else min(dk, c)


def pose_quality(bearing_acc_deg: float, bias_m: float, bias_drift_m: float, innov_m: float) -> tuple[bool, bool]:
  """(unreliable, poor). Unreliable: never latch. Poor: latch without the inner clamp, capped assist.

  Bearing accuracy > 30° is unreliable. A map-match bias > 2 m is unreliable when the bearing is also degraded
  (> 20°: Sep 29 11:57:48 had 38° and 2.2-2.6 m); with a usable bearing (R126: 4.5 m, <= 19°) the bias estimator
  has already removed it, so the pose is only "poor" (no inner clamp, assist capped at 1.2x the model).
  """
  unreliable = bearing_acc_deg > BEARING_ACC_UNRELIABLE_DEG or \
    (bias_m > BIAS_UNRELIABLE_M and bearing_acc_deg > BEARING_ACC_POOR_DEG)
  poor = unreliable or bearing_acc_deg > BEARING_ACC_POOR_DEG or innov_m > INNOV_POOR_M
  return unreliable, poor


IDLE, ACTIVE, EXITING, DONE = "idle", "active", "exiting", "done"


class RoundaboutGuide:
  def __init__(self) -> None:
    self.pose = PoseTracker()
    self.ring: RingGeometry | None = None
    self.ring_v = 0.0
    self.reset_state()
    self.debug: dict = {}

  def reset_state(self) -> None:
    self.phase = IDLE
    self.phase_w = 0.0
    self.k_tgt = None
    self.latched = False
    self.r_ref = None
    self.theta_entry = 0.0
    self.travel = 0.0
    self.exit_t = 0.0
    self.last_t = None
    self.turning = False
    self.bias = (0.0, 0.0)
    self.dk_out = 0.0
    self.prev_out: float | None = None
    self.prev_model: float | None = None
    self.bias_hist: deque = deque()
    self.latch_t: float | None = None
    self.latch_poor = False
    self.events: list[str] = []
    self._blocked = ""
    self._holding = False
    self._capping = False

  def _ev(self, msg: str) -> None:
    if len(self.events) < 50:
      self.events.append(msg)

  def set_ring(self, ring: RingGeometry | None) -> None:
    if ring is None:
      return
    if self.ring is not None and set(ring.way_ids) == set(self.ring.way_ids):
      return
    self.ring = ring
    self.ring_v = float(roundabout_target_ms(roundabout_comfort_speed_ms(ring.radius_m, ring.lanes, ring.maxspeed_ms)))
    self.pose.set_origin(ring.lat, ring.lon)
    self.reset_state()

  def has_way(self, way_id: int) -> bool:
    return self.ring is not None and int(way_id) in self.ring.way_ids

  def gnss(self, t_log: float, lat: float, lon: float, bearing_deg: float | None,
           speed: float | None = None, bearing_acc_deg: float | None = None,
           latency_s: float = GPS_LATENCY_S) -> None:
    if self.pose.origin is None:
      self.pose.set_origin(lat, lon)
    self.pose.fix(t_log, lat, lon, bearing_deg, speed, bearing_acc_deg, latency_s)
    self._update_bias()

  def _update_bias(self) -> None:
    """Map-matched GNSS bias: the part of the offset from the mapped road no lane explains."""
    raw = self.pose.pose()
    if self.ring is None or raw is None or self.pose.n_good < 1:
      return
    m = map_match_offset(self.ring, raw[0], raw[1], raw[2])
    if m is None:
      return
    nx, ny, ex = m
    bx, by = self.bias
    bn = bx * nx + by * ny
    bn += BIAS_GAIN * (ex - bn)
    bx, by = bx + (bn - (bx * nx + by * ny)) * nx, by + (bn - (bx * nx + by * ny)) * ny
    mag = math.hypot(bx, by)
    if mag > BIAS_MAX_M:
      bx, by = bx * BIAS_MAX_M / mag, by * BIAS_MAX_M / mag
    self.bias = (bx, by)

  def corrected_pose(self) -> tuple[float, float, float] | None:
    raw = self.pose.pose()
    if raw is None:
      return None
    return raw[0] - self.bias[0], raw[1] - self.bias[1], raw[2]

  def _passthrough(self, model_k: float) -> float:
    self.dk_out = 0.0
    self.prev_out = self.prev_model = None
    return model_k

  def update(self, t: float, v: float, yaw_rate: float, model_k: float, *,
             enabled: bool, lat_active: bool, edges: EdgeInfo | None = None) -> float:
    """Return the (possibly blended) curvature. Exactly model_k whenever inactive."""
    dt = 0.0 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.2)
    self.last_t = t
    self.pose.predict(t, v, yaw_rate)
    self.debug = {"phase": self.phase, "w": 0.0}
    model_k = float(model_k)
    ring = self.ring
    pose = self.corrected_pose()
    if not enabled or ring is None or pose is None or not all(math.isfinite(z) for z in (model_k, v, yaw_rate)):
      self.phase_w, self.k_tgt, self.dk_out = 0.0, None, 0.0
      return self._passthrough(model_k)
    x, y, psi = pose
    R, hw = ring.radius_m, ring.half_width_m
    r = math.hypot(x, y)
    d_edge = r - (R + hw)
    if self.phase == DONE:
      if d_edge > LEAVE_EDGE_M:
        self.reset_state()
      return self._passthrough(model_k)
    theta = math.atan2(y, x)
    sense = 1.0 if ring.ccw else -1.0
    # Tangent (circulation) bearing vs heading.
    tx, ty = -sense * y / max(r, 1e-3), sense * x / max(r, 1e-3)
    tan_brg = math.atan2(tx, ty)
    align = abs(math.degrees(_wrap(psi - tan_brg)))
    radial_v = v * (math.sin(psi) * x + math.cos(psi) * y) / max(r, 1e-3)

    if self.phase == IDLE:
      if d_edge <= ENGAGE_EDGE_M and radial_v < -0.5 and V_MIN <= v <= V_MAX:
        self.phase = ACTIVE
      else:
        self.phase_w, self.k_tgt, self.dk_out = 0.0, None, 0.0
        return self._passthrough(model_k)

    # Pose quality: an unreliable pose (bearing accuracy > 30° / bias > 2 m) or a still-moving bias never latches.
    bias_m = math.hypot(*self.bias)
    self.bias_hist.append((t, self.bias[0], self.bias[1]))
    while self.bias_hist and self.bias_hist[0][0] < t - BIAS_WINDOW_S:
      self.bias_hist.popleft()
    drift = math.hypot(self.bias[0] - self.bias_hist[0][1], self.bias[1] - self.bias_hist[0][2])
    unreliable, poor = pose_quality(self.pose.bacc, bias_m, drift, self.pose.innov)
    can_latch = not unreliable and drift <= BIAS_LATCH_DRIFT_M
    if not self.latched and align < TANGENT_DEG and d_edge < 1.0 and not can_latch:
      kind = "unreliable" if unreliable else "bias_unsettled"
      if kind != self._blocked:
        self._blocked = kind
        self._ev(f"latch blocked {kind} bacc={self.pose.bacc:.0f} bias={bias_m:.1f} drift={drift:.1f} r={r:.1f} d_edge={d_edge:.1f}")
    if not self.latched and align < TANGENT_DEG and d_edge < 1.0 and can_latch:
      self.latched = True
      self.latch_t = t
      self.latch_poor = poor
      lo = R if (poor or bias_m > BIAS_UNRELIABLE_M) else R - 0.5 * hw      # doubtful pose: never the inner side
      self.r_ref = min(max(r, lo), R + 0.5 * hw)
      self.theta_entry = theta
      self.travel = 0.0
      self._ev(f"latched r={r:.1f} r_ref={self.r_ref:.1f} bias={bias_m:.1f} bacc={self.pose.bacc:.0f} poor={int(poor)} v={v:.1f}")
    elif self.latched:
      self.travel += math.degrees(sense * _wrap(theta - self.theta_entry))
      self.theta_entry = theta
    r_ref = self.r_ref if self.latched else R + PRE_LATCH_HW * hw   # outer part of the ring until tangent, then hold the lane
    # Evaluate the circle follower at the pose PREVIEW_S ahead (constant current yaw rate): leads the
    # target slew + actuator delay so the left transition starts before the car is already tangent.
    px, py, ppsi = _arc_ahead(x, y, psi, v, yaw_rate, PREVIEW_S)
    k_pp = circle_follow_curvature(px, py, ppsi, v, r_ref, ring.ccw)

    if d_edge > D_TURN_M and not self.turning:
      k_raw = max(model_k, k_pp)   # hold / add right only; never an early left
      phase_target = _interp(d_edge, (FULL_EDGE_M, ENGAGE_EDGE_M), (1.0, 0.0))
    else:
      self.turning = True
      k_raw = k_pp
      # Unreliable pose and not latched: fade the assist out, the model keeps this pass (Sep 29 11:57:48).
      phase_target = 0.0 if (unreliable and not self.latched) else 1.0
    if self.latched:
      # Circulating: stay near the lane's own circle curvature (pure pursuit only trims the radius).
      k_ring = -sense / max(r_ref, 1.0)
      k_raw = min(max(k_raw, k_ring - RING_TRIM_K), k_ring + RING_TRIM_K)
    k_raw = lat_limited(k_raw, v, sense, self.latched)
    holding = v > self.ring_v + SPEED_HOLD_MS
    if holding:
      k_raw = hold_curl(k_raw, model_k, sense)
    if holding != self._holding:
      self._holding = holding
      self._ev(f"curl {'held' if holding else 'released'} v={v:.1f} ring_v={self.ring_v:.1f}")

    if self.phase == ACTIVE:
      leaving = (self.latched and d_edge > 1.5 and radial_v > 0.5) or \
                (not self.latched and d_edge > 0.0 and radial_v > 1.0 and self.turning)
      wants_exit = self.latched and self.travel >= EXIT_MIN_TRAVEL_DEG and (model_k - k_raw) > EXIT_DK
      self.exit_t = self.exit_t + dt if wants_exit else 0.0
      if leaving or self.exit_t >= EXIT_HOLD_S or not (V_MIN <= v <= V_MAX + 2.0) or d_edge > ENGAGE_EDGE_M + 10.0:
        self.phase = EXITING
    if self.phase == EXITING:
      phase_target = 0.0

    step = W_RATE * dt
    self.phase_w = min(max(phase_target, self.phase_w - step), self.phase_w + step)

    if self.k_tgt is None or not lat_active:
      self.k_tgt = model_k
    else:
      tslew = TARGET_SLEW if poor else TARGET_SLEW_GOOD
      self.k_tgt += min(max(k_raw - self.k_tgt, -tslew * dt), tslew * dt)

    c_bias = _interp(bias_m, (3.0, BIAS_MAX_M), (1.0, 0.4))
    conf = min(self.pose.confidence(t), map_match_confidence(ring, x, y), ring_confidence(ring), c_bias)
    w = W_MAX * conf * self.phase_w
    self.debug = {"phase": self.phase, "w": w, "conf": conf, "bias": bias_m, "bacc": self.pose.bacc, "poor": poor,
                  "unreliable": unreliable, "hold": holding, "innov": self.pose.innov,
                  "n_good": self.pose.n_good, "r": r, "d_edge": d_edge, "k_pp": k_pp, "k_tgt": self.k_tgt, "r_ref": r_ref,
                  "align": align, "travel": self.travel, "latched": self.latched}
    if not lat_active:
      # Driver has the wheel (or lateral is off): no correction, nothing carried over.
      self.dk_out = 0.0
      if self.phase == EXITING and self.phase_w <= 0.0:
        self.phase, self.k_tgt = DONE, None
      return self._passthrough(model_k)
    dk = w * (self.k_tgt - model_k)
    dk = min(max(dk, -DK_MAX), DK_MAX)
    capping = self.latched and self.latch_poor and t - (self.latch_t or t) <= CAP_WINDOW_S
    if capping:
      dk = cap_poor_gps(dk, model_k, sense)
    if capping != self._capping:
      self._capping = capping
      self._ev(f"poor-GPS cap {'on' if capping else 'off'} bias={bias_m:.1f} bacc={self.pose.bacc:.0f}")
    # The correction itself moves at <= OUT_SLEW (model changes pass straight through).
    oslew = OUT_SLEW if poor else OUT_SLEW_GOOD
    self.dk_out += min(max(dk - self.dk_out, -oslew * dt), oslew * dt)
    # ...and the sum never moves faster than max(OUT_SLEW, the model's own rate).
    if self.prev_out is not None and self.prev_model is not None:
      allowed = max(oslew * dt, abs(model_k - self.prev_model))
      out = min(max(model_k + self.dk_out, self.prev_out - allowed), self.prev_out + allowed)
      self.dk_out = out - model_k
    self.prev_model = model_k
    self.prev_out = model_k + self.dk_out
    if self.phase == EXITING and self.phase_w <= 0.0 and abs(self.dk_out) < HANDOFF_DK:
      self.phase, self.k_tgt, self.dk_out = DONE, None, 0.0
      return self._passthrough(model_k)
    # Camera authority on lane lines / road edges applies to what is actually sent, immediately.
    return model_k + limit_correction_by_edges(self.dk_out, edges)


# --------------------------------------------------------------------------------------------
# controlsd glue (kept here so it is testable with a fake SubMaster / Params)

HINT_HOLD_S = 10.0
PARAM_READ_FRAMES = 100       # ~1 s at 100 Hz
LOOK_T_S = 1.0


def _interp_xy(xs, ys, x: float) -> float | None:
  try:
    n = min(len(xs), len(ys))
  except TypeError:
    return None
  if n < 2:
    return None
  xs = [float(xs[i]) for i in range(n)]
  ys = [float(ys[i]) for i in range(n)]
  if not (xs[0] <= x <= xs[-1]):
    return None
  for i in range(1, n):
    if xs[i] >= x:
      a, b = xs[i - 1], xs[i]
      f = 0.0 if b <= a else (x - a) / (b - a)
      return ys[i - 1] + f * (ys[i] - ys[i - 1])
  return None


def edges_from_model(model_v2, v_ego: float) -> EdgeInfo | None:
  """Nearest camera bound on each side of the model path at ~1 s (device frame, y right)."""
  try:
    x = min(max(float(v_ego) * LOOK_T_S, 5.0), 20.0)
    path_y = _interp_xy(model_v2.position.x, model_v2.position.y, x)
    if path_y is None:
      return None
    lefts, rights = [], []
    lines, probs = model_v2.laneLines, model_v2.laneLineProbs
    for i in (1, 2):
      if len(lines) > i and len(probs) > i and float(probs[i]) > 0.5:
        yy = _interp_xy(lines[i].x, lines[i].y, x)
        if yy is not None:
          (lefts if yy < path_y else rights).append(yy)
    edges, stds = model_v2.roadEdges, model_v2.roadEdgeStds
    for i in (0, 1):
      if len(edges) > i and len(stds) > i and float(stds[i]) < 1.0:
        yy = _interp_xy(edges[i].x, edges[i].y, x)
        if yy is not None:
          (lefts if yy < path_y else rights).append(yy)
    return EdgeInfo(x=x, path_y=path_y, left_y=max(lefts) if lefts else None,
                    right_y=min(rights) if rights else None)
  except Exception:
    return None


class RoundaboutAssist:
  """Pre-AP controlsd wrapper: toggle, GNSS feed, ring param, hint gating, edges."""

  def __init__(self, preap: bool, params=None) -> None:
    self.preap = bool(preap)
    self.params = params
    self.guide = RoundaboutGuide()
    self.enabled = False
    self._frame = -1
    self._last_hint_t = -1e9
    self._last_ring_read_t = -1e9
    self._gps_frames: dict[str, int] = {}
    self._last_log_t = -1e9
    self.active = False

  def _read_toggle(self) -> None:
    if self._frame < 0 or self._frame >= PARAM_READ_FRAMES:
      self._frame = 0
      on = False
      if self.params is not None:
        try:
          on = bool(self.params.get_bool(PARAM_ROUNDABOUT_ASSIST))
        except Exception:
          on = False
      self.enabled = on
    self._frame += 1

  def _load_ring(self, t: float, way_id: int) -> None:
    if self.params is None or self.guide.has_way(way_id) or t - self._last_ring_read_t < 1.0:
      return
    self._last_ring_read_t = t
    try:
      from openpilot.selfdrive.mapd.roundabout_map import PARAM_RING
      ring = RingGeometry.from_json(self.params.get(PARAM_RING))
    except Exception:
      ring = None
    if ring is not None and int(way_id) in ring.way_ids:
      self.guide.set_ring(ring)

  def _feed_gnss(self, sm) -> None:
    for sock, lat_s in (("gpsLocationExternal", GPS_EXT_LATENCY_S), ("gpsLocation", GPS_LATENCY_S)):
      try:
        frame = int(sm.recv_frame[sock])
      except Exception:
        continue
      if frame <= 0 or frame == self._gps_frames.get(sock):
        continue
      self._gps_frames[sock] = frame
      g = sm[sock]
      lat, lon = float(g.latitude), float(g.longitude)
      if not (math.isfinite(lat) and math.isfinite(lon)) or (abs(lat) < 1e-6 and abs(lon) < 1e-6):
        continue
      if hasattr(g, "hasFix") and not bool(g.hasFix):
        continue
      self.guide.gnss(float(sm.logMonoTime[sock]) * 1e-9, lat, lon, float(g.bearingDeg), float(g.speed),
                      float(getattr(g, "bearingAccuracyDeg", 0.0) or 0.0), lat_s)
      return   # one source per cycle; external preferred when present

  def update(self, sm, *, t: float, v_ego: float, yaw_rate: float, model_k: float, lat_active: bool,
             maneuver_active: bool, lane_change_active: bool, hint=None, model_v2=None) -> float:
    """Return the curvature to use in place of model_k (identical when Off / non-Pre-AP / idle)."""
    self._read_toggle()
    self.active = False
    if not self.preap:
      return float(model_k)
    if not self.enabled:
      self.guide.update(t, v_ego, yaw_rate, model_k, enabled=False, lat_active=False)
      return float(model_k)
    self._feed_gnss(sm)
    if hint is not None and getattr(hint, "way_id", 0):
      self._last_hint_t = t
      self._load_ring(t, int(hint.way_id))
    busy = self.guide.phase in (ACTIVE, EXITING)
    on = (t - self._last_hint_t) <= HINT_HOLD_S or busy
    if maneuver_active or lane_change_active:
      on = False
    edges = edges_from_model(model_v2, v_ego) if (on and model_v2 is not None) else None
    out = self.guide.update(t, v_ego, yaw_rate, model_k, enabled=on, lat_active=lat_active, edges=edges)
    self.active = self.guide.phase in (ACTIVE, EXITING)
    self._log(t, model_k, out)
    return out

  def _log(self, t: float, model_k: float, out: float) -> None:
    """Ring state through swaglog (no capnp): state changes, plus ~1 Hz while the assist is running."""
    g = self.guide
    ev, g.events = g.events, []
    for e in ev:
      cloudlog.warning(f"roundabout_assist {e}")
    if self.active and t - self._last_log_t >= LOG_PERIOD_S:
      self._last_log_t = t
      d = g.debug
      r_ref = d.get("r_ref")
      fields = [f"phase={g.phase}", f"latched={int(bool(d.get('latched')))}", f"w={d.get('w', 0.0):.2f}", f"conf={d.get('conf', 0.0):.2f}",
                f"bias={d.get('bias', 0.0):.1f}", f"bacc={d.get('bacc', 0.0):.0f}", f"poor={int(bool(d.get('poor')))}",
                f"hold={int(bool(d.get('hold')))}", f"r={d.get('r', 0.0):.1f}", f"r_ref={'-' if r_ref is None else f'{r_ref:.1f}'}",
                f"model={model_k:+.4f}", f"out={out:+.4f}"]
      cloudlog.info("roundabout_assist state " + " ".join(fields))
