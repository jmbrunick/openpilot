"""Radar stationary-object cone line. Log only — this module never steers.

A line is stationary returns (ground speed near zero after adding vEgo)
sitting beside the model path, spread along the road. Posts are tracked
in the device frame and carried forward with ego speed and yaw, then
matched to the next scan by position. Track ids are not used: the radar
reissues them every second or two.

One mailbox, a parked car (a short wide cluster), and a continuous
guardrail are not a cone line. A guardrail is flagged rather than
dropped when the spacing gives it away.

The scan is one pass over a capped point list, a nearest-neighbour
match, and a 3-coefficient fit. Call it at radar rate (~8 Hz).

Frames (this is the bug the Oct 7 posts exposed):
  radarState.yRel     +left
  model position.y    +right (device frame), same as PathObstacle y
  device y            -yRel, so +right
  path-relative lat   device y minus the path, +right. Negative means
                      the line is on the driver's left.
  side                +1 left of the path, -1 right. Opposite sign from
                      lat, because lat is +right.
  steeringTorque      +left. The hold compares torque to side, and path
                      y / curvature to lat. Those are different frames.
  wouldLimit          path shift, +right, that would open ~1 m of clearance
  wouldSteer          same shift, capped at 0.4 m. Logged only. Nothing
                      in controlsd applies it.

Thresholds:
  stationary ground speed     |vRel + vEgo| < 1.2 m/s
  beside the path             0.30–2.5 m to start a line. Once the line
                              is up, posts closer than that (the taper
                              inside the old 0.5 m band) stay on it.
  longitudinal window         6–70 m ahead of the camera
  cone line                   >= 4 tracked posts, span >= 10 m,
                              residual to a ground-frame quadratic <= 0.50 m,
                              median gap 1.5–18 m (discrete, not a rail)
  confirm / drop              1.0 s of fresh hits to become active.
                              A gap over 0.25 s resets that count.
                              An active line holds ~2 s after the hits stop,
                              and 1–3 posts on the fit are enough to refresh it.
  wouldSteer cap              0.4 m
  guardrail                   span >= 12 m, >= 8 posts, median gap < 1.25 m
  parked car                  short cluster (< 8 m) that is wide (>= 1.0 m)
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.radar_path_gate import RADAR_TO_CAMERA_M, path_y_at_x

STATIONARY_MPS = 1.2
LAT_MIN_M = 0.30
LAT_MAX_M = 2.50
X_MIN_M = 6.0
X_MAX_M = 70.0
X_KEEP_MIN_M = 4.0
MIN_COUNT = 4
MIN_SPAN_M = 10.0
MAX_LAT_STD_M = 0.50
MIN_GAP_M = 1.5
MAX_GAP_M = 18.0
CONFIRM_S = 1.0
DROP_S = 2.0
CONFIRM_GAP_S = 0.25
CLEARANCE_M = 1.0
WOULD_STEER_CAP_M = 0.40
LOOKAHEADS_M = (15.0, 30.0, 45.0)
BARRIER_MIN_COUNT = 8
BARRIER_MIN_SPAN_M = 12.0
BARRIER_MAX_GAP_M = 1.25
PARKED_MAX_SPAN_M = 8.0
PARKED_MIN_WIDTH_M = 1.0
ROAD_EDGE_MARGIN_M = 0.40
MIN_VEGO_CONFIRM_MPS = 3.0
POST_TTL_PRE_S = 1.2
POST_TTL_ACTIVE_S = 2.2
ASSOC_GATE_M = 2.2
FIT_TOL_M = 1.05
MAX_POINTS = 48
MAX_POSTS = 36
CONE_BUDGET_S = 0.002
MAX_QUADRATIC = 0.01


@dataclass(frozen=True)
class ConeLineSample:
  active: bool = False
  side: int = 0  # +1 left of the path, -1 right, 0 none
  confidence: float = 0.0
  count: int = 0
  lat_near: float = 0.0  # path-relative, +right
  lat_mid: float = 0.0
  lat_far: float = 0.0
  would_limit: float = 0.0  # path shift, +right, to keep ~1 m clearance
  would_steer: float = 0.0  # same shift, |v| <= 0.4 m. Not an actuator command.
  barrier: bool = False
  parked: bool = False
  span_m: float = 0.0


@dataclass
class _Post:
  x: float
  y: float  # device frame, +right
  age: float = 0.0


@dataclass
class _Group:
  side: int
  posts: list
  fit: tuple
  kind: str
  resid: float

  @property
  def count(self) -> int:
    return len(self.posts)

  @property
  def span(self) -> float:
    if len(self.posts) < 2:
      return 0.0
    xs = [p.x for p in self.posts]
    return max(xs) - min(xs)


def _num(value) -> float | None:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(out):
    return None
  return out


def _median(vals: list[float]) -> float:
  if not vals:
    return 0.0
  ordered = sorted(vals)
  n = len(ordered)
  mid = n // 2
  if n % 2:
    return ordered[mid]
  return 0.5 * (ordered[mid - 1] + ordered[mid])


def _std(vals: list[float]) -> float:
  n = len(vals)
  if n < 2:
    return 0.0
  mean = sum(vals) / n
  return math.sqrt(sum((v - mean) ** 2 for v in vals) / n)


def device_y_right(y_rel: float) -> float:
  """radarState.yRel is +left. Device / model / PathObstacle y is +right."""
  return -float(y_rel)


def model_yaw_rate_right(model) -> float:
  """Ego yaw rate in the device frame, rad/s, positive to the right.

  modelV2.orientationRate.z is around device z (down), so positive z is a
  right turn — the same sign as model position.y. Missing or absurd
  values are 0, which leaves speed-only compensation.
  """
  try:
    zs = model.orientationRate.z
    if len(zs) == 0:
      return 0.0
    z = float(zs[0])
  except Exception:
    return 0.0
  if not math.isfinite(z) or abs(z) > 1.5:
    return 0.0
  return z


def _solve3(a, b):
  """Solve a 3x3 system. None when the matrix is singular."""
  m = [a[i][:] + [b[i]] for i in range(3)]
  for col in range(3):
    pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
    if abs(m[pivot][col]) < 1e-8:
      return None
    m[col], m[pivot] = m[pivot], m[col]
    div = m[col][col]
    for j in range(col, 4):
      m[col][j] /= div
    for r in range(3):
      if r == col:
        continue
      factor = m[r][col]
      if factor == 0.0:
        continue
      for j in range(col, 4):
        m[r][j] -= factor * m[col][j]
  return (m[0][3], m[1][3], m[2][3])


def _fit_curve(xs: list[float], ys: list[float]) -> tuple | None:
  """Ground-frame quadratic y = a + b u + c u^2, u = x - mean(x).

  Four posts use a line. Five or more use a quadratic, and a wild
  curvature coefficient falls back to the line.
  """
  n = len(xs)
  if n == 0:
    return None
  x0 = sum(xs) / n
  us = [x - x0 for x in xs]
  sy = sum(ys)
  su = sum(us)
  su2 = sum(u * u for u in us)
  suy = sum(u * y for u, y in zip(us, ys))
  det = n * su2 - su * su
  if abs(det) < 1e-6:
    return (sy / n, 0.0, 0.0, x0)
  b = (n * suy - su * sy) / det
  a = (sy - b * su) / n
  linear = (a, b, 0.0, x0)
  if n < 5:
    return linear
  p = [0.0, 0.0, 0.0, 0.0, 0.0]
  rhs = [0.0, 0.0, 0.0]
  for u, y in zip(us, ys):
    up = 1.0
    for k in range(5):
      p[k] += up
      up *= u
    rhs[0] += y
    rhs[1] += u * y
    rhs[2] += u * u * y
  coef = _solve3(
    [[p[0], p[1], p[2]], [p[1], p[2], p[3]], [p[2], p[3], p[4]]],
    rhs,
  )
  if coef is None or abs(coef[2]) > MAX_QUADRATIC:
    return linear
  return (coef[0], coef[1], coef[2], x0)


def _eval(fit, x: float) -> float:
  a, b, c, x0 = fit
  u = x - x0
  return a + u * (b + u * c)


def _residual(posts, fit) -> float:
  if fit is None or len(posts) < 2:
    return 0.0
  return _std([p.y - _eval(fit, p.x) for p in posts])


def road_edges_xy(model) -> list[tuple[list[float], list[float]]]:
  """modelV2.roadEdges as (x, y) polylines. Empty when the model has none."""
  try:
    edges = model.roadEdges
  except (TypeError, AttributeError):
    return []
  out: list[tuple[list[float], list[float]]] = []
  try:
    iterator = list(edges)
  except TypeError:
    return []
  for edge in iterator:
    try:
      xs = [float(v) for v in edge.x]
      ys = [float(v) for v in edge.y]
    except (TypeError, ValueError, AttributeError):
      continue
    if len(xs) >= 2 and len(xs) == len(ys) and all(math.isfinite(v) for v in xs + ys):
      out.append((xs, ys))
  return out


def _outside_road(edges, x: float, y: float) -> bool:
  """True when both edges exist and the return sits past the road.

  Edge y is model / device y, +right, same as `y`.
  """
  if len(edges) < 2:
    return False
  ys = []
  for xs, ys_edge in edges[:2]:
    ey = path_y_at_x(xs, ys_edge, x)
    if ey is None or not math.isfinite(ey):
      return False
    ys.append(ey)
  lo, hi = min(ys), max(ys)
  if hi - lo < 1.0:
    return False
  return y > hi + ROAD_EDGE_MARGIN_M or y < lo - ROAD_EDGE_MARGIN_M


def _path_lat(y: float, x: float, path_x, path_y) -> float:
  py = path_y_at_x(path_x, path_y, x)
  if py is None or not math.isfinite(py):
    return y
  return y - py


def _side_of_lat(lat: float) -> int:
  """lat is +right. +1 is the driver's left, -1 the right."""
  if lat > 0.05:
    return -1
  if lat < -0.05:
    return 1
  return 0


def _would_shift(side: int, clearance: float) -> float:
  """Signed path shift, +right, that opens CLEARANCE_M beside the line."""
  if side == 0:
    return 0.0
  need = CLEARANCE_M - clearance
  if need <= 0.0:
    return 0.0
  # Left line (side +1): move the path right, which is positive y.
  return float(side) * need


def _cap_steer(shift: float) -> float:
  if not math.isfinite(shift) or shift == 0.0:
    return 0.0
  mag = min(abs(shift), WOULD_STEER_CAP_M)
  return math.copysign(mag, shift)


def _dedupe(posts: list[_Post]) -> list[_Post]:
  if len(posts) < 2:
    return posts
  posts = sorted(posts, key=lambda p: p.x)
  out = [posts[0]]
  for p in posts[1:]:
    prev = out[-1]
    if math.hypot(p.x - prev.x, p.y - prev.y) < 0.75:
      if p.age < prev.age:
        out[-1] = p
      continue
    out.append(p)
  return out


def _classify(side: int, posts: list[_Post], path_x, path_y) -> _Group | None:
  if len(posts) < 2:
    return None
  posts = sorted(posts, key=lambda p: p.x)
  xs = [p.x for p in posts]
  ys = [p.y for p in posts]
  span = xs[-1] - xs[0]
  gaps = [xs[i] - xs[i - 1] for i in range(1, len(xs)) if xs[i] - xs[i - 1] >= 0.3]
  med_gap = _median(gaps) if gaps else 0.0
  lats = [_path_lat(p.y, p.x, path_x, path_y) for p in posts]
  lat_range = max(lats) - min(lats)
  fit = _fit_curve(xs, ys)
  if fit is None:
    return None
  resid = _residual(posts, fit)
  group = _Group(side=side, posts=posts, fit=fit, kind="", resid=resid)
  if span >= BARRIER_MIN_SPAN_M and len(posts) >= BARRIER_MIN_COUNT and med_gap < BARRIER_MAX_GAP_M:
    group.kind = "barrier"
    return group
  wide_short = span < PARKED_MAX_SPAN_M and span > 0.4 and (
    lat_range >= PARKED_MIN_WIDTH_M or (len(posts) >= 4 and span < 6.0 and resid > MAX_LAT_STD_M)
  )
  if wide_short:
    group.kind = "parked"
    return group
  if (
    len(posts) >= MIN_COUNT
    and span >= MIN_SPAN_M
    and resid <= MAX_LAT_STD_M
    and med_gap >= MIN_GAP_M
    and med_gap <= MAX_GAP_M
  ):
    group.kind = "cones"
    return group
  return None


def _moved(posts: list[_Post], v: float, yaw: float, dt: float) -> list[_Post]:
  """Carry posts into the current device frame.

  y is +right and yaw is +right (positive = right turn). The vehicle
  moved forward by v*dt and rotated by yaw*dt.
  """
  ds = v * dt
  dpsi = yaw * dt
  if dpsi > 0.25:
    dpsi = 0.25
  elif dpsi < -0.25:
    dpsi = -0.25
  c = math.cos(dpsi)
  s = math.sin(dpsi)
  out = []
  for p in posts:
    xr = p.x - ds
    yr = p.y
    out.append(_Post(c * xr + s * yr, -s * xr + c * yr, p.age + dt))
  return out


def _aged(posts: list[_Post], dt: float) -> list[_Post]:
  return [_Post(p.x, p.y, p.age + dt) for p in posts]


def _match_cost(posts: list[_Post], meas: list[tuple[float, float]]) -> float:
  if not meas:
    return 0.0
  penalty = 4.0
  total = 0.0
  for mx, my in meas:
    best = penalty
    for p in posts:
      d = math.hypot(p.x - mx, p.y - my)
      if d < best:
        best = d
    total += best
  return total


def _choose_motion(posts, meas, v, yaw, dt) -> list[_Post]:
  """Speed-and-yaw compensation, unless a static repeat of the same scan fits better.

  Replayed tests hand the same vehicle-frame points every cycle. On the
  road the points step back by about v*dt, and the compensated posts are
  the ones that match.
  """
  if not posts or dt <= 0.0:
    return _aged(posts, dt)
  options = [_moved(posts, v, yaw, dt)]
  if abs(yaw) > 1e-4:
    options.append(_moved(posts, v, 0.0, dt))
  options.append(_aged(posts, dt))
  best = options[0]
  best_cost = _match_cost(best, meas)
  for cand in options[1:]:
    cost = _match_cost(cand, meas)
    if cost < best_cost - 1e-6:
      best = cand
      best_cost = cost
  return best


def _associate(posts: list[_Post], meas: list[tuple[float, float]]) -> list[_Post]:
  used = [False] * len(posts)
  for mx, my in meas:
    best_i = -1
    best_d = ASSOC_GATE_M
    for i, p in enumerate(posts):
      if used[i]:
        continue
      d = math.hypot(p.x - mx, p.y - my)
      if d < best_d:
        best_i = i
        best_d = d
    if best_i < 0:
      posts.append(_Post(mx, my, 0.0))
      used.append(True)
    else:
      used[best_i] = True
      posts[best_i].x = mx
      posts[best_i].y = my
      posts[best_i].age = 0.0
  return posts


def _lat_at(group: _Group, path_x, path_y, xq: float) -> float:
  xs = [p.x for p in group.posts]
  xmin, xmax = min(xs), max(xs)
  x = xmin if xq < xmin else xmax if xq > xmax else xq
  y = _eval(group.fit, x)
  return _path_lat(y, x, path_x, path_y)


def _sample_from_group(group: _Group | None, *, active: bool, confidence: float,
                       barrier: bool, parked: bool, path_x, path_y) -> ConeLineSample:
  if group is None or group.kind == "parked":
    return ConeLineSample(active=False, barrier=barrier, parked=parked, confidence=min(confidence, 0.49))
  lats = [_lat_at(group, path_x, path_y, x) for x in LOOKAHEADS_M]
  point_lats = [_path_lat(p.y, p.x, path_x, path_y) for p in group.posts]
  clearance = min(abs(v) for v in point_lats) if point_lats else min(abs(v) for v in lats)
  shift = _would_shift(group.side, clearance) if group.kind == "cones" else 0.0
  return ConeLineSample(
    active=active and group.kind == "cones",
    side=group.side,
    confidence=confidence,
    count=group.count,
    lat_near=lats[0],
    lat_mid=lats[1],
    lat_far=lats[2],
    would_limit=shift,
    would_steer=_cap_steer(shift) if group.kind == "cones" else 0.0,
    barrier=barrier or group.kind == "barrier",
    parked=parked,
    span_m=group.span,
  )


def _read_point(raw):
  if isinstance(raw, dict):
    d = _num(raw.get("dRel", raw.get("d_rel")))
    y = _num(raw.get("yRel", raw.get("y_rel")))
    v = _num(raw.get("vRel", raw.get("v_rel")))
    measured = raw.get("measured", True)
  else:
    d = _num(getattr(raw, "dRel", None))
    y = _num(getattr(raw, "yRel", None))
    v = _num(getattr(raw, "vRel", None))
    measured = getattr(raw, "measured", True)
  if d is None or y is None or v is None or not bool(measured):
    return None
  return d, y, v


class ConeLineDetector:
  """Temporal cone line. State is the post buffer plus the confirm timer."""

  def __init__(self) -> None:
    self._last_out = ConeLineSample()
    self.reset()

  def reset(self) -> None:
    self._posts: list[_Post] = []
    self._seen_s = 0.0
    self._miss_s = 0.0
    self._active = False
    self._side = 0
    self._fit = None
    self._last_group: _Group | None = None

  def update(self, points, v_ego: float, path_x, path_y, road_edges, dt: float,
             yaw_rate: float = 0.0) -> ConeLineSample:
    """yaw_rate is device-frame rad/s, positive to the right. Optional."""
    t0 = time.monotonic()
    try:
      sample = self._step(points, v_ego, path_x, path_y, road_edges, dt, yaw_rate, t0)
    except Exception:
      self.reset()
      sample = ConeLineSample()
    self._last_out = sample
    return sample

  def _step(self, points, v_ego, path_x, path_y, road_edges, dt, yaw_rate, t0) -> ConeLineSample:
    dt = 0.0 if dt is None else float(dt)
    if not math.isfinite(dt) or dt < 0.0:
      dt = 0.0
    dt = min(dt, 0.5)
    try:
      v_ego_f = float(v_ego)
    except (TypeError, ValueError):
      v_ego_f = 0.0
    if not math.isfinite(v_ego_f):
      v_ego_f = 0.0
    try:
      yaw = float(yaw_rate)
    except (TypeError, ValueError):
      yaw = 0.0
    if not math.isfinite(yaw):
      yaw = 0.0

    if time.monotonic() - t0 > CONE_BUDGET_S:
      return self._last_out

    meas = self._measurements(points, v_ego_f, path_x, path_y, road_edges or [], t0)
    self._posts = _choose_motion(self._posts, meas, v_ego_f, yaw, dt)
    self._posts = _associate(self._posts, meas)
    ttl = POST_TTL_ACTIVE_S if self._active else POST_TTL_PRE_S
    self._posts = [
      p for p in self._posts
      if p.age <= ttl and X_KEEP_MIN_M <= p.x <= X_MAX_M + 2.0
    ]
    self._posts = _dedupe(self._posts)
    if len(self._posts) > MAX_POSTS:
      self._posts = sorted(self._posts, key=lambda p: p.age)[:MAX_POSTS]

    if time.monotonic() - t0 > CONE_BUDGET_S:
      return self._last_out

    left, right = self._split(path_x, path_y)
    left_g = _classify(1, left, path_x, path_y)
    right_g = _classify(-1, right, path_x, path_y)
    parked = (left_g is not None and left_g.kind == "parked") or (right_g is not None and right_g.kind == "parked")
    barrier_group = None
    cones = None
    for group in (left_g, right_g):
      if group is None:
        continue
      if group.kind == "barrier" and (barrier_group is None or group.span > barrier_group.span):
        barrier_group = group
      elif group.kind == "cones" and (cones is None or group.count > cones.count):
        cones = group

    allow_confirm = v_ego_f >= MIN_VEGO_CONFIRM_MPS
    supported = False
    fresh_cones = cones is not None and any(p.age <= 1e-9 for p in cones.posts)
    same_side = cones is not None and (self._side == 0 or cones.side == self._side or not self._active)
    if fresh_cones and same_side and (allow_confirm or self._active):
      if self._side not in (0, cones.side) and not self._active:
        self._seen_s = 0.0
      self._side = cones.side
      self._fit = cones.fit
      self._last_group = cones
      self._seen_s = min(CONFIRM_S + 1.0, self._seen_s + dt)
      self._miss_s = 0.0
      supported = True
    elif self._active and self._fit is not None:
      held = left if self._side == 1 else right if self._side == -1 else []
      fresh_on = [p for p in held if p.age <= 1e-9 and abs(p.y - _eval(self._fit, p.x)) <= FIT_TOL_M]
      if fresh_on:
        self._miss_s = 0.0
        supported = True
        if len(held) >= 2:
          fit2 = _fit_curve([p.x for p in held], [p.y for p in held])
          if fit2 is not None and _residual(held, fit2) <= MAX_LAT_STD_M:
            self._fit = fit2
            self._last_group = _Group(self._side, held, fit2, "cones", _residual(held, fit2))

    if not supported:
      self._miss_s += dt
      if not self._active and self._miss_s > CONFIRM_GAP_S:
        self._seen_s = 0.0
        self._side = 0
        self._fit = None
        self._last_group = None
      if self._active and self._miss_s >= DROP_S:
        self._active = False
        self._seen_s = 0.0
        self._side = 0
        self._fit = None
        self._last_group = None

    if (
      not self._active
      and allow_confirm
      and self._last_group is not None
      and self._last_group.kind == "cones"
      and self._seen_s + 1e-9 >= CONFIRM_S
    ):
      self._active = True

    shown = self._last_group
    if shown is not None and shown.kind != "cones" and not self._active:
      shown = shown
    if self._active and (shown is None or shown.kind != "cones"):
      shown = self._last_group
    persist = 0.0 if CONFIRM_S <= 0 else min(1.0, self._seen_s / CONFIRM_S)
    resid = shown.resid if shown is not None else MAX_LAT_STD_M
    align = max(0.0, 1.0 - resid / MAX_LAT_STD_M)
    confidence = persist * (0.65 + 0.35 * align)
    if not self._active:
      confidence = min(confidence, 0.49)
    barrier = barrier_group is not None or (shown is not None and shown.kind == "barrier")
    if shown is None and barrier_group is not None:
      shown = barrier_group
      confidence = min(confidence, 0.49)
    if shown is not None and shown.kind == "parked":
      shown = None
    return _sample_from_group(
      shown,
      active=self._active and shown is not None and shown.kind == "cones",
      confidence=confidence,
      barrier=barrier,
      parked=parked,
      path_x=path_x,
      path_y=path_y,
    )

  def _measurements(self, points, v_ego, path_x, path_y, edges, t0):
    try:
      iterator = list(points) if points is not None else []
    except TypeError:
      return []
    meas = []
    for raw in iterator:
      if len(meas) >= MAX_POINTS or time.monotonic() - t0 > CONE_BUDGET_S:
        break
      fields = _read_point(raw)
      if fields is None:
        continue
      d_rel, y_rel, v_rel = fields
      if abs(v_rel + v_ego) >= STATIONARY_MPS:
        continue
      x = d_rel + RADAR_TO_CAMERA_M
      if x < X_MIN_M or x > X_MAX_M:
        continue
      y = device_y_right(y_rel)
      if _outside_road(edges, x, y):
        continue
      lat = _path_lat(y, x, path_x, path_y)
      if abs(lat) > LAT_MAX_M:
        continue
      if abs(lat) < LAT_MIN_M:
        if not (self._active and self._fit is not None and self._side in (-1, 1)
                and abs(y - _eval(self._fit, x)) <= FIT_TOL_M):
          continue
      meas.append((x, y))
    return meas

  def _split(self, path_x, path_y):
    left: list[_Post] = []
    right: list[_Post] = []
    for p in self._posts:
      if p.x < X_MIN_M or p.x > X_MAX_M:
        continue
      lat = _path_lat(p.y, p.x, path_x, path_y)
      if abs(lat) > LAT_MAX_M + 0.4:
        continue
      side = self._side if (
        self._active and self._fit is not None and abs(lat) < LAT_MIN_M
        and abs(p.y - _eval(self._fit, p.x)) <= FIT_TOL_M
      ) else _side_of_lat(lat)
      if side == 1:
        left.append(p)
      elif side == -1:
        right.append(p)
    return left, right


def publish_cone_line(pm, sample: ConeLineSample | None) -> None:
  """Publish one coneLineNAP. No-op when the socket or the sample is absent.

  wouldSteer is written for the log. It is not a curvature command.
  """
  if sample is None or pm is None or "coneLineNAP" not in getattr(pm, "sock", {}):
    return
  import cereal.messaging as messaging

  msg = messaging.new_message("coneLineNAP")
  msg.valid = True
  dest = msg.coneLineNAP
  dest.active = bool(sample.active)
  dest.side = int(sample.side)
  dest.confidence = float(sample.confidence)
  dest.count = int(min(255, max(0, sample.count)))
  dest.latNear = float(sample.lat_near)
  dest.latMid = float(sample.lat_mid)
  dest.latFar = float(sample.lat_far)
  dest.wouldLimit = float(sample.would_limit)
  dest.wouldSteer = float(sample.would_steer)
  dest.barrier = bool(sample.barrier)
  dest.parked = bool(sample.parked)
  dest.spanM = float(sample.span_m)
  pm.send("coneLineNAP", msg)
