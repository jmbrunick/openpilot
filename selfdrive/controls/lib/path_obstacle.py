"""Radar-triggered obstacle log, plus an animal/person chime.

This module never brakes and never steers. A later stage can call
`future_brake_intent` (humans in or entering the path, and only when
radar and vision agree). Nothing in controlsd calls it.

Radar scans every track. Vision runs only after a candidate has
persisted, and only on that spot. The driving model has no animal or
pedestrian head, so the checker is a luma contrast/shape heuristic on
the projected patch, not a network.

Thresholds (device frame, y +left, path-relative lateral):
  range                         5–120 m ahead of the camera
  in the path                   |lateral| <= 1.70 m
  entering                      reaches the path within 3.2 s
                                (4.5 s for a person, 5 s for a slow
                                walker within 1 m of the lane edge)
  roadside                      past the lane edge out to 7.2 m
                                (about 8.9 m from the path center),
                                and inside a 52° radar half-angle
  slow along the road           |ground speed| <= 3 m/s
  person along the road         up to 9 m/s (a cyclist) if compact
                                and not the lead
  lateral motion                |v_lat| >= 0.45 m/s, or toward the path
  persist                       0.40 s before active; confidence
                                saturates near 0.80 s
  agree                         radar >= 0.55 and vision >= 0.45
  lighting weights              good light wR = 0.50, poor wR = 0.78,
                                linear in the lighting score
  chime                         animal or human, agree, zone in
                                {inPath, entering, roadside}, once per
                                track, 9 s global cooldown
  chime does not include        obstacle (tree, debris, post)
  lively                        0 still, 1 has moved in the last 2 s.
                                Ground speed, lateral speed, or
                                position wander past radar noise.
                                Dead-still cuts animal/person class
                                confidence to 0.45, not to zero.

Road-surface pop-in (first seen inside 14 m, stationary, not spanning)
and a thin wide curtain (span >= 9 m, depth < 2.5 m) are rejected.
Cone lines, guardrails and parked cars use coneLineNAP plus the same
stationary-cluster shapes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from openpilot.selfdrive.controls.lib.radar_path_gate import RADAR_TO_CAMERA_M, path_y_at_x

PARAM_OBSTACLE_LOG = "NAPObstacleLog"
PARAM_OBSTACLE_CHIME = "NAPObstacleChime"

# Classes that may chime. Add nothing else without a new product decision.
# Generic obstacles stay log-only.
CHIME_CLASSES = frozenset({"animal", "human"})
CHIME_ZONES = frozenset({"inPath", "entering", "roadside"})
# Future braking. Humans first, and never for a roadside-only detection.
BRAKE_CLASSES = frozenset({"human"})
BRAKE_ZONES = frozenset({"inPath", "entering"})

X_MIN_M = 5.0
X_MAX_M = 120.0
PATH_HALF_M = 1.70
ROADSIDE_EXTRA_M = 7.2
ROADSIDE_OUTER_M = PATH_HALF_M + ROADSIDE_EXTRA_M
FOV_HALF_DEG = 52.0
SLOW_ALONG_MPS = 3.0
HUMAN_ALONG_MAX_MPS = 9.0
LATERAL_MOVE_MPS = 0.45
ENTER_HORIZON_S = 3.2
HUMAN_ENTER_HORIZON_S = 4.5
WALKER_ENTER_HORIZON_S = 5.0
WALKER_EDGE_M = 2.70
PERSIST_S = 0.40
PERSIST_FULL_S = 0.80
RADAR_AGREE = 0.55
VISION_AGREE = 0.45
WR_GOOD = 0.50
WR_POOR = 0.78
CHIME_COOLDOWN_S = 9.0
CHIME_HOLD_S = 0.35
LIVELY_WINDOW_S = 2.0
# Below this, a Bosch return is noise. Above the full value, the cue is 1.
SPEED_NOISE_MPS = 0.40
SPEED_FULL_MPS = 1.40
VLAT_NOISE_MPS = 0.30
VLAT_FULL_MPS = 1.10
WANDER_NOISE_M = 0.40
WANDER_FULL_M = 1.15
# A frozen deer keeps this fraction of its animal/person class score.
LIVELY_FLOOR = 0.45
CLUSTER_DX_M = 5.5
CLUSTER_DY_M = 3.2
LANE_SPAN_M = 1.8
OVERHEAD_SPAN_M = 9.0
OVERHEAD_DEPTH_M = 2.5
POPIN_X_M = 14.0
LEAD_X_M = 4.5
LEAD_Y_M = 1.6
LEAD_PROB = 0.40

CLASS_ORDER = ("unknown", "human", "animal", "obstacle")
ZONE_ORDER = ("none", "inPath", "entering", "roadside")


def _clamp(value: float, lo: float, hi: float) -> float:
  return lo if value < lo else hi if value > hi else value


def _nan() -> float:
  return float("nan")


def _finite(value) -> float | None:
  try:
    out = float(value)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(out):
    return None
  return out


def _toward(lat: float, v_lat: float) -> float:
  """Speed toward the path center, m/s. Already-in-path uses |v_lat|."""
  if abs(lat) < 0.25:
    return abs(v_lat)
  return max(0.0, -math.copysign(1.0, lat) * v_lat)


@dataclass(frozen=True)
class VisionScore:
  """Patch verdict. conf is 'something solid above the road', 0..1."""

  conf: float = 0.0
  human: float = 0.0
  animal: float = 0.0
  obstacle: float = 0.0
  evaluated: bool = True


@dataclass(frozen=True)
class ConeHint:
  """The bits of coneLineNAP this scan needs. Tests build it directly."""

  active: bool = False
  side: int = 0
  lat_near: float = 0.0
  lat_mid: float = 0.0
  lat_far: float = 0.0
  barrier: bool = False
  parked: bool = False


@dataclass(frozen=True)
class BrakeIntent:
  """Not consumed by any controller. decel is always 0 in this stage."""

  allowed: bool
  reason: str
  decel: float = 0.0


@dataclass(frozen=True)
class ObstacleSample:
  active: bool = False
  track_id: int = 0
  range_m: float = 0.0
  lateral: float = 0.0
  v_rel: float = 0.0
  v_lat: float = 0.0
  radar_conf: float = 0.0
  vision_conf: float = _nan()
  vision_evaluated: bool = False
  lighting_score: float = 1.0
  w_radar: float = WR_GOOD
  w_vision: float = 1.0 - WR_GOOD
  fused_score: float = 0.0
  agree: bool = False
  reject_reason: str = "no_candidate"
  in_path: bool = False
  time_to_reach: float = _nan()
  object_class: str = "unknown"
  vision_conf_human: float = _nan()
  vision_conf_animal: float = _nan()
  vision_conf_obstacle: float = _nan()
  cluster_count: int = 0
  span_m: float = 0.0
  time_to_enter: float = _nan()
  entering: bool = False
  zone: str = "none"
  brake_gate: bool = False
  chimed: bool = False
  chime_reason: str = "no_candidate"
  lively_score: float = 0.0
  scan_us: float = 0.0
  vision_us: float = 0.0
  member_ids: tuple[int, ...] = ()

  def with_chime(self, chimed: bool, reason: str) -> ObstacleSample:
    return replace(self, chimed=bool(chimed), chime_reason=str(reason))


@dataclass(frozen=True)
class _Hit:
  """One radar candidate after the scan, before vision and the chime."""

  active: bool
  track_id: int
  member_ids: tuple[int, ...]
  x: float
  y: float
  lateral: float
  v_rel: float
  v_lat: float
  along: float
  radar_conf: float
  radar_class: str
  zone: str
  time_to_reach: float
  time_to_enter: float
  cluster_count: int
  span_m: float
  reject_reason: str
  lighting: float
  lively: float = 0.0


def project_road_point(x: float, y: float, z: float, *,
                       roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                       height: float = 1.22, focal: float = 1141.5,
                       cx: float = 672.0, cy: float = 380.0) -> tuple[float, float] | None:
  """Project a road-frame point (x forward, y left, z up) to pixels.

  Matches `get_view_frame_from_road_frame`: device y is right, z is down,
  then the view frame is x right, y down, z forward. Returns None when
  the point is behind the camera.
  """
  try:
    x, y, z = float(x), float(y), float(z)
    roll, pitch, yaw = float(roll), float(pitch), float(yaw)
    height, focal = float(height), float(focal)
    cx, cy = float(cx), float(cy)
  except (TypeError, ValueError):
    return None
  if not all(math.isfinite(v) for v in (x, y, z, roll, pitch, yaw, height, focal, cx, cy)):
    return None
  if focal <= 1.0:
    return None
  cr, sr = math.cos(roll), math.sin(roll)
  cp, sp = math.cos(pitch), math.sin(pitch)
  cyaw, syaw = math.cos(yaw), math.sin(yaw)
  # R = Rz @ Ry @ Rx, then diag(1, -1, -1) on the road point.
  rx, ry, rz = x, -y, -z
  # Rx
  ry, rz = cr * ry - sr * rz, sr * ry + cr * rz
  # Ry
  rx, rz = cp * rx + sp * rz, -sp * rx + cp * rz
  # Rz
  rx, ry = cyaw * rx - syaw * ry, syaw * rx + cyaw * ry
  # view = (y_right, z_down, x_forward) + (0, height, 0)
  vx, vy, vz = ry, rz + height, rx
  if vz <= 0.5:
    return None
  return cx + focal * (vx / vz), cy + focal * (vy / vz)


def projection_box(u: float, v_top: float, v_ground: float, focal: float, x: float,
                   half_m: float = 0.55) -> tuple[float, float, float, float]:
  """Pixel box (u0, v0, u1, v1) around the above-road segment."""
  half = focal * half_m / max(float(x), 1.0)
  half = _clamp(half, 3.0, 180.0)
  top, bot = (v_top, v_ground) if v_top <= v_ground else (v_ground, v_top)
  if bot - top < 4.0:
    bot = top + 4.0
  return u - half, top, u + half, bot


def fusion_weights(lighting: float) -> tuple[float, float]:
  """Good light shares the vote. Poor light leans on radar. Linear, no step."""
  score = _clamp(lighting if math.isfinite(lighting) else 1.0, 0.0, 1.0)
  w_radar = WR_POOR + (WR_GOOD - WR_POOR) * score
  return w_radar, 1.0 - w_radar


def fuse_scores(radar_conf: float, vision_conf: float | None, lighting: float) -> tuple[float, float, float, bool]:
  """Return fused, w_radar, w_vision, agree.

  Vision NaN does not vote. The fused score is then the radar score, and
  agree stays false because the camera has not confirmed the spot.
  """
  w_r, w_v = fusion_weights(lighting)
  radar = _clamp(radar_conf if math.isfinite(radar_conf) else 0.0, 0.0, 1.0)
  if vision_conf is None or not math.isfinite(vision_conf):
    return radar, w_r, w_v, False
  vision = _clamp(vision_conf, 0.0, 1.0)
  fused = _clamp(w_r * radar + w_v * vision, 0.0, 1.0)
  agree = radar >= RADAR_AGREE and vision >= VISION_AGREE
  return fused, w_r, w_v, agree


def lighting_score(*, lane_left: float = 1.0, lane_right: float = 1.0,
                   path_y_std: float | None = None, integ_lines: float | None = None,
                   gain: float | None = None, grey: float | None = None,
                   sun_ahead: bool = False, headlights: bool = False) -> float:
  """0 is a camera that cannot see, 1 is a bright clear frame. Continuous."""
  left = _finite(lane_left)
  right = _finite(lane_right)
  lane = _clamp(min(left if left is not None else 1.0, right if right is not None else 1.0), 0.0, 1.0)
  if path_y_std is None or not math.isfinite(path_y_std):
    path = 0.75
  else:
    path = _clamp(1.0 - (path_y_std - 0.4) / 2.6, 0.0, 1.0)
  exposure = 1.0
  if gain is not None and math.isfinite(gain) and gain > 0.0:
    exposure *= _clamp(1.2 - gain / 5.5, 0.1, 1.0)
  if integ_lines is not None and math.isfinite(integ_lines) and integ_lines > 0.0:
    exposure *= _clamp(1.35 - integ_lines / 2800.0, 0.15, 1.0)
  if grey is not None and math.isfinite(grey):
    if grey < 0.04 or grey > 0.55:
      exposure *= 0.50
    elif grey < 0.08 or grey > 0.40:
      exposure *= 0.75
  if sun_ahead:
    exposure *= 0.55
  if headlights:
    exposure *= 0.62
  model = 0.55 * lane + 0.45 * path
  return _clamp(0.42 * model + 0.58 * exposure, 0.0, 1.0)


def _mean_std(vals: list[float]) -> tuple[float, float]:
  n = len(vals)
  if n == 0:
    return 0.0, 0.0
  mean = sum(vals) / n
  var = sum((v - mean) ** 2 for v in vals) / n
  return mean, math.sqrt(var)


def metrics_from_rows(obj_rows, road_rows=None) -> dict:
  """Mean/std plus 3 column and 3 row means. Caps the sample count."""
  rows = [list(r) for r in (obj_rows or []) if r]
  if not rows:
    return {"mean": 0.0, "std": 0.0, "cols": [0.0, 0.0, 0.0], "bands": [0.0, 0.0, 0.0], "road": 0.0}
  height = len(rows)
  width = min(len(r) for r in rows)
  cells = height * width
  step = 1
  while cells / (step * step) > 2400 and step < 8:
    step += 1
  vals: list[float] = []
  cols = [[], [], []]
  bands = [[], [], []]
  for yi, row in enumerate(rows[::step]):
    y_full = yi * step
    band = 0 if y_full < height / 3 else 1 if y_full < 2 * height / 3 else 2
    for x_full, value in enumerate(row[:width:step]):
      try:
        pix = float(value)
      except (TypeError, ValueError):
        continue
      vals.append(pix)
      col = 0 if x_full < width / 3 else 1 if x_full < 2 * width / 3 else 2
      cols[col].append(pix)
      bands[band].append(pix)
  mean, std = _mean_std(vals)
  road_vals: list[float] = []
  for row in list(road_rows or [])[::step]:
    for value in list(row)[:width:step]:
      num = _finite(value)
      if num is not None:
        road_vals.append(num)
  road = _mean_std(road_vals)[0] if road_vals else (bands[2] and _mean_std(bands[2])[0]) or mean
  return {
    "mean": mean,
    "std": std,
    "cols": [_mean_std(c)[0] if c else mean for c in cols],
    "bands": [_mean_std(b)[0] if b else mean for b in bands],
    "road": road,
  }


def vision_from_regions(obj_mean: float, obj_std: float, road_mean: float,
                        cols: list[float], bands: list[float]) -> VisionScore:
  """Turn patch stats into a solid-above-road score and weak class scores.

  The class split is a shape prior (tall vs mid vs wide vs a thin post).
  It is not a trained detector. A raccoon-sized target often never clears
  the solid-object bar.
  """
  contrast = abs(obj_mean - road_mean)
  if contrast < 8.0 and obj_std < 12.0:
    solid = 0.08
  else:
    solid = _clamp(contrast / 42.0, 0.0, 1.0) * 0.75 + _clamp(obj_std / 28.0, 0.0, 1.0) * 0.25
  c0, c1, c2 = (cols + [obj_mean, obj_mean, obj_mean])[:3]
  b0, b1, b2 = (bands + [obj_mean, obj_mean, obj_mean])[:3]
  side = 0.5 * (c0 + c2)
  center_delta = abs(c1 - side)
  needle = center_delta > 14.0 and center_delta > abs(c0 - c2) + 8.0
  wide = abs(c0 - road_mean) > 12.0 and abs(c2 - road_mean) > 12.0 and abs(c1 - road_mean) > 12.0
  top_c, mid_c, bot_c = abs(b0 - road_mean), abs(b1 - road_mean), abs(b2 - road_mean)
  tall = top_c > 12.0 and mid_c > 10.0
  low_only = bot_c > 12.0 and top_c < 8.0 and mid_c < 10.0
  mid_body = mid_c > 12.0 and top_c < mid_c * 0.75
  human = solid * (0.85 if tall and not wide else 0.20 if tall else 0.05)
  animal = solid * (0.80 if mid_body and not needle and not wide else 0.22 if mid_body else 0.08)
  obstacle = solid * (0.85 if wide or low_only or needle else 0.35)
  human = _clamp(human, 0.0, solid if solid > 0 else 1.0)
  animal = _clamp(animal, 0.0, 1.0)
  obstacle = _clamp(obstacle, 0.0, 1.0)
  return VisionScore(conf=_clamp(solid, 0.0, 1.0), human=human, animal=animal, obstacle=obstacle, evaluated=True)


def score_row_patches(obj_rows, road_rows) -> VisionScore:
  stats = metrics_from_rows(obj_rows, road_rows)
  return vision_from_regions(stats["mean"], stats["std"], stats["road"], stats["cols"], stats["bands"])


def _class_name(raw) -> str:
  return str(raw).split(".")[-1]


def should_raise_chime(chimed, object_class, enabled: bool = True) -> bool:
  """selfdrived uses this. It does not look at longitudinal state."""
  if not enabled or not chimed:
    return False
  return _class_name(object_class) in CHIME_CLASSES


def chime_banner(object_class, zone) -> str:
  """Short HUD line. Same chime sound; the words say person or animal."""
  name = _class_name(object_class)
  zone_name = _class_name(zone)
  if zone_name.isdigit():
    idx = int(zone_name)
    zone_name = ZONE_ORDER[idx] if 0 <= idx < len(ZONE_ORDER) else "none"
  if name.isdigit():
    idx = int(name)
    name = CLASS_ORDER[idx] if 0 <= idx < len(CLASS_ORDER) else "unknown"
  roadside = zone_name == "roadside"
  if name == "human":
    return "Person near road" if roadside else "Person ahead"
  return "Animal near road" if roadside else "Animal ahead"


def future_brake_intent(sample: ObstacleSample) -> BrakeIntent | None:
  """Humans in or entering the path, and only when both sensors agree.

  Roadside never brakes. Animals and debris never brake in this stage.
  """
  if not sample.agree or sample.object_class not in BRAKE_CLASSES:
    return None
  if sample.zone not in BRAKE_ZONES:
    return None
  return BrakeIntent(True, "agree_human", 0.0)


class ChimeGate:
  """Once per track id, plus a global gap. Animals and people share it."""

  def __init__(self, cooldown_s: float = CHIME_COOLDOWN_S, hold_s: float = CHIME_HOLD_S):
    self.cooldown_s = float(cooldown_s)
    self.hold_s = float(hold_s)
    self._ids: set[int] = set()
    self._last = -1e9
    self._hold_until = -1e9
    self._hold_ids: set[int] = set()

  def reset(self) -> None:
    self._ids.clear()
    self._last = -1e9
    self._hold_until = -1e9
    self._hold_ids.clear()

  def consider(self, sample: ObstacleSample, enabled: bool, now: float) -> tuple[bool, str]:
    ids = set(sample.member_ids) or ({sample.track_id} if sample.track_id else set())
    if not enabled:
      return False, "disabled"
    if ids and ids & self._hold_ids and now <= self._hold_until:
      return True, "chimed"
    if not sample.active:
      if sample.reject_reason == "not_persistent":
        return False, "not_persistent"
      return False, sample.reject_reason or "no_candidate"
    if sample.object_class not in CHIME_CLASSES:
      return False, "class"
    if sample.zone not in CHIME_ZONES:
      return False, "zone"
    if not sample.agree:
      return False, "no_agree"
    if ids and ids & self._ids:
      return False, "already_tracked"
    if now - self._last < self.cooldown_s:
      return False, "cooldown"
    self._ids.update(ids)
    if len(self._ids) > 64:
      self._ids = set(list(self._ids)[-64:])
    self._last = now
    self._hold_until = now + self.hold_s
    self._hold_ids = set(ids)
    return True, "chimed"


def _point(raw) -> dict | None:
  if isinstance(raw, dict):
    src = raw
    get = src.get
  else:
    def get(name, default=None):
      return getattr(raw, name, default)
  d_rel = _finite(get("dRel", get("d_rel")))
  y_rel = _finite(get("yRel", get("y_rel")))
  v_rel = _finite(get("vRel", get("v_rel")))
  if d_rel is None or y_rel is None or v_rel is None:
    return None
  if not bool(get("measured", True)):
    return None
  track = get("trackId", get("track_id", 0))
  try:
    track_id = int(track)
  except (TypeError, ValueError):
    track_id = 0
  yv = _finite(get("yvRel", get("yv_rel")))
  rcs = _finite(get("rcs", get("rcsDb", get("amplitude"))))
  return {
    "id": track_id,
    "x": d_rel + RADAR_TO_CAMERA_M,
    "y": -y_rel,
    "v_rel": v_rel,
    "yv_rel": yv,
    "rcs": rcs,
  }


def _line_lat(cone: ConeHint, x: float) -> float | None:
  pts = ((15.0, cone.lat_near), (30.0, cone.lat_mid), (45.0, cone.lat_far))
  if x <= pts[0][0]:
    return pts[0][1]
  if x >= pts[-1][0]:
    return pts[-1][1]
  for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
    if x0 <= x <= x1 and x1 > x0:
      t = (x - x0) / (x1 - x0)
      return y0 + t * (y1 - y0)
  return pts[-1][1]


def _cone_reject(lat: float, x: float, cone: ConeHint | None) -> str | None:
  if cone is None:
    return None
  line = _line_lat(cone, x)
  if cone.barrier and line is not None and abs(lat - line) < 1.15:
    return "barrier"
  same_side = cone.side == 0 or (lat > 0) == (cone.side > 0) or abs(lat) < 0.4
  if cone.active and line is not None and abs(lat - line) < 0.80 and same_side:
    return "cone_line"
  if cone.parked and 1.5 < abs(lat) < 4.5:
    return "parked"
  return None


def _humanish(lat: float, v_lat: float, along: float, toward: float) -> bool:
  if 0.30 <= toward <= 3.4 and abs(along) < 4.0:
    return True
  if PATH_HALF_M < abs(lat) <= WALKER_EDGE_M and abs(along) < 2.2 and toward >= 0.15:
    return True
  if abs(lat) <= PATH_HALF_M + 0.3 and 2.0 < along <= HUMAN_ALONG_MAX_MPS and abs(v_lat) < 2.0:
    return True
  return False


def _zone_of(lat: float, v_lat: float, along: float) -> tuple[str, float]:
  if abs(lat) <= PATH_HALF_M:
    return "inPath", 0.0
  toward = _toward(lat, v_lat)
  dist = abs(lat) - PATH_HALF_M
  t_enter = dist / toward if toward >= 0.20 else float("inf")
  horizon = HUMAN_ENTER_HORIZON_S if _humanish(lat, v_lat, along, toward) else ENTER_HORIZON_S
  if _humanish(lat, v_lat, along, toward) and PATH_HALF_M < abs(lat) <= WALKER_EDGE_M and toward >= 0.15:
    horizon = max(horizon, WALKER_ENTER_HORIZON_S)
  if math.isfinite(t_enter) and t_enter <= horizon:
    return "entering", t_enter
  if PATH_HALF_M < abs(lat) <= ROADSIDE_OUTER_M:
    return "roadside", t_enter if math.isfinite(t_enter) else _nan()
  return "none", _nan()


def _fov_ok(x: float, lat: float) -> bool:
  return math.degrees(math.atan2(abs(lat), max(x, 0.5))) <= FOV_HALF_DEG


def _closing_time(x: float, v_rel: float) -> float:
  closing = -v_rel
  if closing <= 0.4:
    return _nan()
  return x / closing


def _ramp(value: float, noise: float, full: float) -> float:
  span = full - noise
  if span <= 1e-6:
    return 0.0
  return _clamp((abs(value) - noise) / span, 0.0, 1.0)


def lively_from_history(hist) -> float:
  """Max living-motion evidence in the last ~2 s. 0 is dead still.

  Ground speed, lateral speed, and position wander each count. One real
  cue in the window is enough. Jitter inside radar noise stays at 0.
  """
  if not hist:
    return 0.0
  now = float(hist[-1][0])
  window = [row for row in hist if now - float(row[0]) <= LIVELY_WINDOW_S]
  if not window:
    window = [hist[-1]]
  best = 0.0
  xs: list[float] = []
  ys: list[float] = []
  for _age, x, y, along, v_lat in window:
    xs.append(float(x))
    ys.append(float(y))
    speed = math.hypot(float(along), float(v_lat))
    best = max(best, _ramp(speed, SPEED_NOISE_MPS, SPEED_FULL_MPS), _ramp(v_lat, VLAT_NOISE_MPS, VLAT_FULL_MPS))
  span_t = float(window[-1][0]) - float(window[0][0])
  if span_t >= 1.0 and len(window) >= 4:
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    wander = max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys))
    best = max(best, _ramp(wander, WANDER_NOISE_M, WANDER_FULL_M))
  return best


def scale_living_vision(vision: VisionScore, lively: float) -> VisionScore:
  """Lively raises animal/person class scores. Still lowers them, not to 0."""
  scale = LIVELY_FLOOR + (1.0 - LIVELY_FLOOR) * _clamp(lively if math.isfinite(lively) else 0.0, 0.0, 1.0)
  return VisionScore(
    conf=vision.conf,
    human=_clamp(vision.human * scale, 0.0, 1.0),
    animal=_clamp(vision.animal * scale, 0.0, 1.0),
    obstacle=vision.obstacle,
    evaluated=vision.evaluated,
  )


def patch_signature(rows, grid: int = 8) -> tuple[float, ...]:
  """Coarse mean grid so a 5 Hz check can see the patch move."""
  data = [list(row) for row in (rows or []) if row]
  if not data:
    return ()
  height = len(data)
  width = min(len(row) for row in data)
  if width < 2 or height < 2:
    return ()
  cells = []
  for gy in range(grid):
    y0 = gy * height // grid
    y1 = max(y0 + 1, (gy + 1) * height // grid)
    for gx in range(grid):
      x0 = gx * width // grid
      x1 = max(x0 + 1, (gx + 1) * width // grid)
      acc = 0.0
      count = 0
      for row in data[y0:y1]:
        for value in row[x0:x1]:
          num = _finite(value)
          if num is not None:
            acc += num
            count += 1
      cells.append(acc / count if count else 0.0)
  return tuple(cells)


def patch_change_score(prev, rows) -> float:
  """0 when the patch matches, 1 when the gray levels really moved.

  About 12 counts of change is camera noise. Around 40 counts is motion.
  """
  old = prev if isinstance(prev, tuple) else patch_signature(prev)
  new = rows if isinstance(rows, tuple) else patch_signature(rows)
  if not old or not new or len(old) != len(new):
    return 0.0
  delta = sum(abs(a - b) for a, b in zip(old, new)) / len(old)
  return _clamp((delta - 12.0) / 28.0, 0.0, 1.0)


def _fuse_class(radar_class: str, span: float, count: int, vision: VisionScore | None) -> str:
  if count >= 2 and span >= LANE_SPAN_M:
    return "obstacle"
  if vision is None or not vision.evaluated or vision.conf < VISION_AGREE:
    return radar_class
  scores = (("human", vision.human), ("animal", vision.animal), ("obstacle", vision.obstacle))
  best, best_s = max(scores, key=lambda item: item[1])
  ordered = sorted((s for _, s in scores), reverse=True)
  margin = best_s - (ordered[1] if len(ordered) > 1 else 0.0)
  if radar_class in ("human", "animal") and best_s < 0.45:
    return radar_class
  if best_s >= 0.30 and margin >= 0.08:
    return best
  return radar_class


def _radar_class(members: list[dict], span: float, zone: str) -> str:
  count = len(members)
  lats = [m["lat"] for m in members]
  if count >= 2 and span >= LANE_SPAN_M and min(lats) < 0.6 and max(lats) > -0.6:
    return "obstacle"
  best = "unknown"
  for m in members:
    toward = _toward(m["lat"], m["v_lat"])
    along = m["along"]
    if _humanish(m["lat"], m["v_lat"], along, toward) and (count == 1 or span < 1.5):
      return "human"
    animalish = (
      (2.0 <= abs(m["v_lat"]) <= 12.0 and abs(along) < 5.0)
      or (toward >= 1.6 and abs(along) < 4.5 and toward > 3.2)
    )
    if animalish and (count == 1 or span < 1.6):
      best = "animal"
  if best == "animal":
    return best
  if zone == "inPath" and count >= 2 and span >= LANE_SPAN_M:
    return "obstacle"
  return "unknown"


def _cluster_reject(members: list[dict]) -> str | None:
  xs = [m["x"] for m in members]
  lats = [m["lat"] for m in members]
  count = len(members)
  if count < 2:
    return None
  x_span = max(xs) - min(xs)
  lat_span = max(lats) - min(lats)
  if lat_span >= OVERHEAD_SPAN_M and x_span < OVERHEAD_DEPTH_M and count >= 3:
    return "overhead"
  mean_lat = sum(lats) / count
  var = sum((v - mean_lat) ** 2 for v in lats) / count
  if x_span >= 10.0 and math.sqrt(var) <= 0.55 and count >= 4 and 0.5 <= abs(mean_lat) <= 3.0 and x_span > lat_span:
    return "cone_line"
  straddles = min(lats) < -0.2 and max(lats) > 0.2
  if x_span < 8.0 and lat_span >= 1.0 and abs(mean_lat) > 2.0 and not straddles and count >= 3:
    return "parked"
  return None


def _popin(members: list[dict]) -> bool:
  if any(m["first_x"] >= POPIN_X_M + 4.0 for m in members):
    return False
  if any(m["x"] > POPIN_X_M + 2.0 for m in members):
    return False
  if max(m["age"] for m in members) >= 0.55:
    return False
  lats = [m["lat"] for m in members]
  if len(members) >= 2 and max(lats) - min(lats) >= LANE_SPAN_M:
    return False
  return all(abs(m["v_lat"]) < 0.40 and abs(m["along"]) < SLOW_ALONG_MPS for m in members)


class PathObstacleDetector:
  """One candidate per radar frame. State is track age and the chime memory."""

  def __init__(self) -> None:
    self._tracks: dict[int, dict] = {}
    self._chime = ChimeGate()

  def reset(self) -> None:
    self._tracks.clear()
    self._chime.reset()

  def begin(self, points, v_ego: float, path_x, path_y, dt: float, *,
            lead_ids=(), cone: ConeHint | None = None, model_leads=(),
            lighting: float = 1.0) -> _Hit:
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
    light = lighting_score() if lighting is None else float(lighting)
    if not math.isfinite(light):
      light = 1.0

    parsed = []
    try:
      iterator = list(points) if points is not None else []
    except TypeError:
      iterator = []
    for raw in iterator:
      item = _point(raw)
      if item is not None:
        parsed.append(item)
    returns = self._advance(parsed, dt, v_ego_f, path_x, path_y)
    leads = {int(i) for i in lead_ids if i is not None}
    near_reason = "no_candidate"
    near_ret = None
    kept = []
    for ret in returns:
      reason = self._keep_reason(ret, leads, cone, model_leads)
      if reason is None:
        kept.append(ret)
      elif near_ret is None or ret["x"] < near_ret["x"]:
        near_ret = ret
        near_reason = reason
    clusters = _cluster(kept)
    best = None
    best_key = None
    rejected = near_reason
    rejected_members = [near_ret] if near_ret is not None else []
    for group in clusters:
      why = _cluster_reject(group)
      if why is None and _popin(group):
        why = "road_surface"
      if why is not None:
        if best is None:
          rejected = why
          rejected_members = group
        continue
      hit = self._score_group(group, light)
      if hit is None:
        continue
      key = (
        1 if hit.zone in ("inPath", "entering") else 0,
        1 if hit.radar_class in ("human", "animal") else 0,
        hit.radar_conf,
        -hit.x,
      )
      if best_key is None or key > best_key:
        best = hit
        best_key = key
    if best is not None:
      return best
    return self._miss(rejected_members, rejected, light)

  def commit(self, hit: _Hit, vision: VisionScore | None, *,
             chime_enabled: bool = True, now: float = 0.0,
             vision_motion: float = 0.0) -> ObstacleSample:
    try:
      motion = float(vision_motion)
    except (TypeError, ValueError):
      motion = 0.0
    if not math.isfinite(motion):
      motion = 0.0
    lively = _clamp(max(float(hit.lively), _clamp(motion, 0.0, 1.0)), 0.0, 1.0)
    use_vision = bool(hit.active and vision is not None and vision.evaluated)
    scored = scale_living_vision(vision, lively) if use_vision and vision is not None else None
    final_class = _fuse_class(hit.radar_class, hit.span_m, hit.cluster_count, scored)
    if not hit.cluster_count:
      final_class = "unknown"
    vision_conf = vision.conf if use_vision and vision is not None else None
    fused, w_r, w_v, agree = fuse_scores(hit.radar_conf, vision_conf, hit.lighting)
    agree = bool(agree and hit.active)
    zone = hit.zone if hit.reject_reason in ("none", "not_persistent") else "none"
    in_path = zone == "inPath"
    entering = zone == "entering"
    t_enter = 0.0 if in_path else hit.time_to_enter
    brake = bool(agree and final_class in BRAKE_CLASSES and zone in BRAKE_ZONES)
    if scored is not None:
      vh, va, vo = scored.human, scored.animal, scored.obstacle
      vconf = vision.conf if vision is not None else scored.conf
    else:
      vh = va = vo = vconf = _nan()
    sample = ObstacleSample(
      active=bool(hit.active),
      track_id=hit.track_id,
      range_m=hit.x,
      lateral=hit.lateral,
      v_rel=hit.v_rel,
      v_lat=hit.v_lat,
      radar_conf=hit.radar_conf if hit.cluster_count else 0.0,
      vision_conf=vconf,
      vision_evaluated=use_vision,
      lighting_score=hit.lighting,
      w_radar=w_r,
      w_vision=w_v,
      fused_score=fused if hit.cluster_count else 0.0,
      agree=agree,
      reject_reason="none" if hit.active else hit.reject_reason,
      in_path=in_path,
      time_to_reach=hit.time_to_reach,
      object_class=final_class,
      vision_conf_human=vh,
      vision_conf_animal=va,
      vision_conf_obstacle=vo,
      cluster_count=hit.cluster_count,
      span_m=hit.span_m,
      time_to_enter=t_enter,
      entering=entering,
      zone=zone,
      brake_gate=brake,
      lively_score=lively if hit.cluster_count else 0.0,
      member_ids=hit.member_ids,
    )
    chimed, reason = self._chime.consider(sample, chime_enabled, now)
    return sample.with_chime(chimed, reason)

  def update(self, points, v_ego: float, path_x, path_y, dt: float, *,
             lead_ids=(), cone: ConeHint | None = None, model_leads=(),
             vision: VisionScore | None = None, lighting: float = 1.0,
             chime_enabled: bool = True, now: float = 0.0,
             vision_motion: float = 0.0) -> ObstacleSample:
    hit = self.begin(
      points, v_ego, path_x, path_y, dt,
      lead_ids=lead_ids, cone=cone, model_leads=model_leads, lighting=lighting,
    )
    return self.commit(hit, vision, chime_enabled=chime_enabled, now=now, vision_motion=vision_motion)

  def _advance(self, parsed, dt, v_ego, path_x, path_y) -> list[dict]:
    seen = set()
    out = []
    for item in parsed:
      seen.add(item["id"])
      prev = self._tracks.get(item["id"])
      if prev is None or prev["gap"] > 0.35:
        state = {"age": dt, "first_x": item["x"], "gap": 0.0, "y": item["y"], "v_lat": 0.0, "hist": []}
      else:
        state = prev
        state["age"] = float(state["age"]) + dt
        state["gap"] = 0.0
        if item["yv_rel"] is not None:
          state["v_lat"] = -float(item["yv_rel"])
        elif dt > 1e-4:
          state["v_lat"] = (item["y"] - float(state["y"])) / dt
        state["y"] = item["y"]
      if item["yv_rel"] is not None:
        state["v_lat"] = -float(item["yv_rel"])
      state["x"] = item["x"]
      hist = state.setdefault("hist", [])
      hist.append((
        float(state["age"]), float(item["x"]), float(item["y"]),
        float(item["v_rel"]) + v_ego, float(state["v_lat"]),
      ))
      state["hist"] = [row for row in hist if float(state["age"]) - float(row[0]) <= LIVELY_WINDOW_S + 0.05][-24:]
      self._tracks[item["id"]] = state
      py = path_y_at_x(path_x, path_y, item["x"])
      lat = item["y"] if py is None else item["y"] - py
      out.append({
        "id": item["id"],
        "x": item["x"],
        "y": item["y"],
        "lat": lat,
        "v_rel": item["v_rel"],
        "v_lat": float(state["v_lat"]),
        "along": item["v_rel"] + v_ego,
        "rcs": item["rcs"],
        "age": float(state["age"]),
        "first_x": float(state["first_x"]),
        "lively": lively_from_history(state.get("hist") or ()),
      })
    for track_id in list(self._tracks):
      if track_id in seen:
        continue
      self._tracks[track_id]["gap"] = float(self._tracks[track_id]["gap"]) + dt
      if self._tracks[track_id]["gap"] > 0.50:
        del self._tracks[track_id]
    return out

  def _keep_reason(self, ret, leads, cone, model_leads) -> str | None:
    x, lat = ret["x"], ret["lat"]
    if x < X_MIN_M or x > X_MAX_M:
      return "out_of_range"
    if not _fov_ok(x, ret["y"]):
      return "fov"
    if ret["id"] in leads:
      return "lead"
    for lead in model_leads or ():
      try:
        lx, ly, prob = lead
      except (TypeError, ValueError):
        continue
      if _finite(prob) is None or float(prob) < LEAD_PROB:
        continue
      if abs(x - float(lx)) < LEAD_X_M and abs(ret["y"] - float(ly)) < LEAD_Y_M:
        return "lead"
    cone_why = _cone_reject(lat, x, cone)
    if cone_why is not None:
      return cone_why
    along, v_lat = ret["along"], ret["v_lat"]
    toward = _toward(lat, v_lat)
    human = _humanish(lat, v_lat, along, toward)
    moving = abs(v_lat) >= LATERAL_MOVE_MPS
    slow = abs(along) <= SLOW_ALONG_MPS
    cyclist = abs(lat) <= PATH_HALF_M + 0.4 and 2.0 < along <= HUMAN_ALONG_MAX_MPS and abs(v_lat) < 2.0
    if along < -2.5 and abs(v_lat) < 1.2 and not human:
      return "vehicle"
    if not (slow or moving or human or cyclist) or abs(along) > HUMAN_ALONG_MAX_MPS + 0.5:
      return "vehicle"
    zone, _t = _zone_of(lat, v_lat, along)
    if zone == "none":
      return "off_path"
    return None

  def _score_group(self, members: list[dict], lighting: float) -> _Hit | None:
    members = sorted(members, key=lambda m: m["x"])
    rep = members[0]
    lats = [m["lat"] for m in members]
    span = max(lats) - min(lats) if len(lats) > 1 else 0.0
    straddles = min(lats) < -0.2 and max(lats) > 0.2 and span >= 1.2
    zones = []
    for m in members:
      zone, t_enter = _zone_of(m["lat"], m["v_lat"], m["along"])
      zones.append((zone, t_enter, m))
    if straddles or any(z == "inPath" for z, _, _ in zones):
      zone, t_enter = "inPath", 0.0
    elif any(z == "entering" for z, _, _ in zones):
      entering = [(t, m) for z, t, m in zones if z == "entering"]
      t_enter, _m = min(entering, key=lambda item: item[0])
      zone = "entering"
    elif any(z == "roadside" for z, _, _ in zones):
      zone = "roadside"
      finite = [t for z, t, _m in zones if z == "roadside" and math.isfinite(t)]
      t_enter = min(finite) if finite else _nan()
    else:
      return None
    radar_class = _radar_class(members, span, zone)
    age = max(m["age"] for m in members)
    persist = _clamp(age / PERSIST_FULL_S, 0.0, 1.0)
    active = age + 1e-9 >= PERSIST_S
    if zone == "inPath":
      place = 1.0
    elif zone == "entering":
      horizon = HUMAN_ENTER_HORIZON_S if radar_class == "human" else ENTER_HORIZON_S
      t_use = t_enter if math.isfinite(t_enter) else horizon
      place = _clamp(1.0 - t_use / horizon, 0.35, 0.95)
      t_reach = _closing_time(rep["x"], rep["v_rel"])
      if radar_class == "human" and math.isfinite(t_reach) and math.isfinite(t_enter) and t_enter < t_reach:
        place = min(1.0, place + 0.12)
    else:
      place = 0.72
    penalty = 0.0
    if zone == "roadside" and all(abs(m["v_lat"]) < 0.35 and abs(m["along"]) < 1.0 for m in members):
      penalty += 0.22
    rcs_vals = [m["rcs"] for m in members if m["rcs"] is not None]
    if rcs_vals and max(rcs_vals) < -10.0:
      penalty += 0.30
    if rcs_vals and max(rcs_vals) < -15.0:
      penalty += 0.25
    clutter = _clamp(1.0 - penalty, 0.20, 1.0)
    conf = _clamp(0.18 + 0.78 * persist * place * clutter, 0.0, 1.0)
    if len(members) >= 2 and span >= LANE_SPAN_M and zone == "inPath":
      conf = min(1.0, conf + 0.06)
    if not active:
      conf = min(conf, 0.49)
    v_lat = max(members, key=lambda m: _toward(m["lat"], m["v_lat"]))["v_lat"]
    return _Hit(
      active=active,
      track_id=rep["id"],
      member_ids=tuple(m["id"] for m in members),
      x=rep["x"],
      y=rep["y"],
      lateral=rep["lat"],
      v_rel=rep["v_rel"],
      v_lat=v_lat,
      along=rep["along"],
      radar_conf=conf,
      radar_class=radar_class,
      zone=zone,
      time_to_reach=_closing_time(rep["x"], rep["v_rel"]),
      time_to_enter=0.0 if zone == "inPath" else t_enter,
      cluster_count=len(members),
      span_m=span,
      reject_reason="none" if active else "not_persistent",
      lighting=lighting,
      lively=max(float(m.get("lively", 0.0)) for m in members),
    )

  def _miss(self, members, reason: str, lighting: float) -> _Hit:
    if not members or members[0] is None:
      return _Hit(
        False, 0, (), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "unknown", "none",
        _nan(), _nan(), 0, 0.0, reason, lighting, 0.0,
      )
    group = [m for m in members if m is not None]
    rep = min(group, key=lambda m: m["x"])
    lats = [m["lat"] for m in group]
    return _Hit(
      False, rep["id"], tuple(m["id"] for m in group), rep["x"], rep["y"], rep["lat"],
      rep["v_rel"], rep["v_lat"], rep["along"], 0.0, "unknown", "none",
      _closing_time(rep["x"], rep["v_rel"]), _nan(), len(group),
      (max(lats) - min(lats)) if len(lats) > 1 else 0.0, reason, lighting,
      max(float(m.get("lively", 0.0)) for m in group),
    )


def _cluster(members: list[dict]) -> list[list[dict]]:
  n = len(members)
  if n == 0:
    return []
  parent = list(range(n))

  def find(i: int) -> int:
    while parent[i] != i:
      parent[i] = parent[parent[i]]
      i = parent[i]
    return i

  for i in range(n):
    for j in range(i):
      if abs(members[i]["x"] - members[j]["x"]) <= CLUSTER_DX_M and abs(members[i]["lat"] - members[j]["lat"]) <= CLUSTER_DY_M:
        ri, rj = find(i), find(j)
        if ri != rj:
          parent[rj] = ri
  groups: dict[int, list[dict]] = {}
  for i, member in enumerate(members):
    groups.setdefault(find(i), []).append(member)
  return list(groups.values())


def save_ppm(path: str, rows) -> bool:
  """Write a grayscale P5 thumbnail. False on any I/O problem."""
  try:
    data = [list(r) for r in rows if r]
    if not data:
      return False
    height = len(data)
    width = min(len(r) for r in data)
    if width < 2 or height < 2:
      return False
    raw = bytearray()
    for row in data:
      for value in row[:width]:
        try:
          pix = int(value)
        except (TypeError, ValueError):
          pix = 0
        raw.append(max(0, min(255, pix)))
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(path, "wb") as handle:
      handle.write(header)
      handle.write(raw)
    return True
  except OSError:
    return False


def prune_thumbs(directory: str, max_files: int = 24, max_bytes: int = 2_000_000) -> None:
  """Drop the oldest thumbnails until count and bytes are inside the cap."""
  import os
  try:
    names = [n for n in os.listdir(directory) if n.endswith(".ppm")]
  except OSError:
    return
  files = []
  for name in names:
    full = os.path.join(directory, name)
    try:
      files.append((os.path.getmtime(full), os.path.getsize(full), full))
    except OSError:
      continue
  files.sort()
  total = sum(size for _, size, _ in files)
  while files and (len(files) > max_files or total > max_bytes):
    _mtime, size, full = files.pop(0)
    total -= size
    try:
      os.remove(full)
    except OSError:
      pass
