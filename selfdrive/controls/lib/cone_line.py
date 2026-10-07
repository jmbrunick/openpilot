"""Radar stationary-object cone line. Log only — this module never steers.

A line is a handful of stationary returns (ground speed near zero after
adding vEgo) sitting 0.5–2.5 m to one side of the model path, spread along
the road, and present for about a second. One mailbox, a parked car (a
short wide cluster), and a continuous guardrail are not a cone line. A
guardrail is flagged rather than dropped when the spacing gives it away.

The scan is one pass over the points plus a sort of the few that fall in
the side bands. Call it at radar rate (~8 Hz), not at 100 Hz.

Thresholds (device frame, y +left, path-relative lateral):
  stationary ground speed     |vRel + vEgo| < 1.2 m/s
  alongside the path          0.5–2.5 m to one side
  longitudinal window         6–70 m ahead of the camera
  cone line                   >= 4 points, span >= 10 m, lateral std <= 0.50 m,
                              median gap 1.5–18 m (discrete, not a rail)
  confirm / drop              1.0 s to become active, 0.60 s of misses to clear
  clearance used for the log  1.0 m; wouldLimit is the path shift (+left)
                              that would open that gap, else 0
  guardrail                   span >= 12 m, >= 8 points, median gap < 1.25 m
  parked car                  short cluster (< 8 m) that is wide (>= 1.0 m)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.selfdrive.controls.lib.radar_path_gate import RADAR_TO_CAMERA_M, path_y_at_x

STATIONARY_MPS = 1.2
LAT_MIN_M = 0.50
LAT_MAX_M = 2.50
X_MIN_M = 6.0
X_MAX_M = 70.0
MIN_COUNT = 4
MIN_SPAN_M = 10.0
MAX_LAT_STD_M = 0.50
MIN_GAP_M = 1.5
MAX_GAP_M = 18.0
CONFIRM_S = 1.0
DROP_S = 0.60
CONFIRM_GAP_S = 0.25
CLEARANCE_M = 1.0
LOOKAHEADS_M = (15.0, 30.0, 45.0)
BARRIER_MIN_COUNT = 8
BARRIER_MIN_SPAN_M = 12.0
BARRIER_MAX_GAP_M = 1.25
PARKED_MAX_SPAN_M = 8.0
PARKED_MIN_WIDTH_M = 1.0
ROAD_EDGE_MARGIN_M = 0.40
MIN_VEGO_CONFIRM_MPS = 3.0


@dataclass(frozen=True)
class ConeLineSample:
  active: bool = False
  side: int = 0  # +1 left of the path, -1 right, 0 none
  confidence: float = 0.0
  count: int = 0
  lat_near: float = 0.0
  lat_mid: float = 0.0
  lat_far: float = 0.0
  would_limit: float = 0.0  # signed path shift, +left, to keep ~1 m clearance
  barrier: bool = False
  parked: bool = False
  span_m: float = 0.0


def _num(value) -> float | None:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(out):
    return None
  return out


def _point(p) -> tuple[float, float, float, bool] | None:
  if isinstance(p, dict):
    d = _num(p.get("dRel", p.get("d_rel")))
    y = _num(p.get("yRel", p.get("y_rel")))
    v = _num(p.get("vRel", p.get("v_rel")))
    measured = p.get("measured", True)
  else:
    d = _num(getattr(p, "dRel", None))
    y = _num(getattr(p, "yRel", None))
    v = _num(getattr(p, "vRel", None))
    measured = getattr(p, "measured", True)
  if d is None or y is None or v is None:
    return None
  return d, y, v, bool(measured)


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


def _lat_at(xs: list[float], lats: list[float], xq: float) -> float:
  if not xs:
    return 0.0
  if xq <= xs[0]:
    return lats[0]
  if xq >= xs[-1]:
    return lats[-1]
  for i in range(1, len(xs)):
    if xq <= xs[i]:
      dx = xs[i] - xs[i - 1]
      t = 0.0 if dx == 0.0 else (xq - xs[i - 1]) / dx
      return lats[i - 1] + t * (lats[i] - lats[i - 1])
  return lats[-1]


def _would_limit(side: int, lats: list[float]) -> float:
  if side == 0 or not lats:
    return 0.0
  clearance = min(abs(v) for v in lats)
  need = CLEARANCE_M - clearance
  if need <= 0.0:
    return 0.0
  # side -1 (line on the right) → shift the path left (+).
  return (-side) * need


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


def _outside_road(edges: list[tuple[list[float], list[float]]], x: float, y: float) -> bool:
  """True when both edges exist and the return sits past the road."""
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


@dataclass
class _Group:
  side: int
  xs: list[float]
  lats: list[float]
  kind: str  # "cones", "barrier", "parked"

  @property
  def count(self) -> int:
    return len(self.xs)

  @property
  def span(self) -> float:
    return self.xs[-1] - self.xs[0] if len(self.xs) >= 2 else 0.0


def _classify(side: int, pts: list[tuple[float, float]]) -> _Group | None:
  if len(pts) < 2:
    return None
  pts = sorted(pts, key=lambda it: it[0])
  xs = [p[0] for p in pts]
  lats = [p[1] for p in pts]
  span = xs[-1] - xs[0]
  gaps = [xs[i] - xs[i - 1] for i in range(1, len(xs))]
  med_gap = _median(gaps) if gaps else 0.0
  lat_std = _std(lats)
  lat_range = max(lats) - min(lats)
  group = _Group(side=side, xs=xs, lats=lats, kind="")
  if span >= BARRIER_MIN_SPAN_M and len(pts) >= BARRIER_MIN_COUNT and med_gap < BARRIER_MAX_GAP_M:
    group.kind = "barrier"
    return group
  wide_short = span < PARKED_MAX_SPAN_M and span > 0.4 and (
    lat_range >= PARKED_MIN_WIDTH_M or (len(pts) >= 4 and span < 6.0 and lat_std > MAX_LAT_STD_M)
  )
  if wide_short:
    group.kind = "parked"
    return group
  if (
    len(pts) >= MIN_COUNT
    and span >= MIN_SPAN_M
    and lat_std <= MAX_LAT_STD_M
    and med_gap >= MIN_GAP_M
    and med_gap <= MAX_GAP_M
  ):
    group.kind = "cones"
    return group
  return None


def _sample_from_group(group: _Group | None, *, active: bool, confidence: float,
                       barrier: bool, parked: bool) -> ConeLineSample:
  if group is None:
    return ConeLineSample(active=False, barrier=barrier, parked=parked, confidence=confidence)
  near, mid, far = (_lat_at(group.xs, group.lats, x) for x in LOOKAHEADS_M)
  return ConeLineSample(
    active=active and group.kind == "cones",
    side=group.side,
    confidence=confidence,
    count=group.count,
    lat_near=near,
    lat_mid=mid,
    lat_far=far,
    would_limit=_would_limit(group.side, group.lats),
    barrier=barrier or group.kind == "barrier",
    parked=parked,
    span_m=group.span,
  )


class ConeLineDetector:
  """Hysteresis around one frame of `_scan`. State is the confirm timer only."""

  def __init__(self) -> None:
    self._seen_s = 0.0
    self._miss_s = 0.0
    self._active = False
    self._side = 0
    self._last: _Group | None = None

  def reset(self) -> None:
    self._seen_s = 0.0
    self._miss_s = 0.0
    self._active = False
    self._side = 0
    self._last = None

  def update(self, points, v_ego: float, path_x, path_y, road_edges, dt: float) -> ConeLineSample:
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

    cones, barrier_group, parked = _scan(points, v_ego_f, path_x, path_y, road_edges)
    same_side = cones is not None and (self._side == 0 or cones.side == self._side or not self._active)
    # Below walking speed a parked row looks stationary. Don't start a new line.
    allow_confirm = v_ego_f >= MIN_VEGO_CONFIRM_MPS
    if cones is not None and same_side and (allow_confirm or self._active):
      if self._side not in (0, cones.side) and not self._active:
        self._seen_s = 0.0
      self._side = cones.side
      self._seen_s += dt
      self._miss_s = 0.0
      self._last = cones
    else:
      self._miss_s += dt
      if not self._active and self._miss_s > CONFIRM_GAP_S:
        self._seen_s = 0.0
        self._side = 0
        self._last = None
      if self._active and self._miss_s >= DROP_S:
        self._active = False
        self._seen_s = 0.0
        self._side = 0
        self._last = None

    if not self._active and self._last is not None and self._seen_s + 1e-9 >= CONFIRM_S:
      self._active = True

    shown = cones if cones is not None and (not self._active or cones.side == self._side) else self._last
    if self._active and shown is None:
      shown = self._last
    persist = 0.0 if CONFIRM_S <= 0 else min(1.0, self._seen_s / CONFIRM_S)
    if shown is not None and shown.lats:
      align = max(0.0, 1.0 - _std(shown.lats) / MAX_LAT_STD_M)
    else:
      align = 0.0
    confidence = persist * (0.65 + 0.35 * align)
    if not self._active:
      confidence = min(confidence, 0.49)
    barrier = barrier_group is not None or (shown is not None and shown.kind == "barrier")
    # A rail with no cone line still fills the log so the flag is visible.
    if shown is None and barrier_group is not None:
      shown = barrier_group
      confidence = min(confidence, 0.49)
    return _sample_from_group(
      shown if shown is not None and shown.kind != "parked" else None,
      active=self._active and shown is not None and shown.kind == "cones",
      confidence=confidence,
      barrier=barrier,
      parked=parked,
    )


def _scan(points, v_ego: float, path_x, path_y, road_edges):
  left: list[tuple[float, float]] = []
  right: list[tuple[float, float]] = []
  try:
    iterator = list(points) if points is not None else []
  except TypeError:
    iterator = []
  edges = road_edges or []
  for raw in iterator:
    fields = _point(raw)
    if fields is None:
      continue
    d_rel, y_rel, v_rel, measured = fields
    if not measured:
      continue
    if abs(v_rel + v_ego) >= STATIONARY_MPS:
      continue
    x = d_rel + RADAR_TO_CAMERA_M
    if x < X_MIN_M or x > X_MAX_M:
      continue
    y = -y_rel  # radar yRel is right-positive; device y is left-positive
    if _outside_road(edges, x, y):
      continue
    py = path_y_at_x(path_x, path_y, x)
    lat = y if py is None else y - py
    mag = abs(lat)
    if mag < LAT_MIN_M or mag > LAT_MAX_M:
      continue
    (left if lat > 0.0 else right).append((x, lat))

  left_g = _classify(1, left)
  right_g = _classify(-1, right)
  parked = (left_g is not None and left_g.kind == "parked") or (right_g is not None and right_g.kind == "parked")
  barrier = None
  cones = None
  for group in (left_g, right_g):
    if group is None:
      continue
    if group.kind == "barrier" and (barrier is None or group.span > barrier.span):
      barrier = group
    elif group.kind == "cones" and (cones is None or group.count > cones.count):
      cones = group
  return cones, barrier, parked


def publish_cone_line(pm, sample: ConeLineSample | None) -> None:
  """Publish one coneLineNAP. No-op when the socket or the sample is absent."""
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
  dest.barrier = bool(sample.barrier)
  dest.parked = bool(sample.parked)
  dest.spanM = float(sample.span_m)
  pm.send("coneLineNAP", msg)
