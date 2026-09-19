"""Travel-path and oncoming gates for radar follow-lead selection.

Intersection max-regen on Scallywag (Fri 2026-09-18 ~20:49–20:52 CT)
came from Bosch tracks of left roadside signs and opposing-lane traffic
becoming leadOne. Rain-hold (#201, NAPWiperSpeed==3) then kept those
phantoms through vision mismatch.

Every radar follow candidate — vision association, rain incumbent, rain
in-lane pick, and the lost-track cache — must still sit on the model
travel path and must not be closing from ahead in the opposing sense.
Stationary in-path objects (stopped cars) stay valid: vLead ≈ 0, not
oncoming. Same-direction leads stay valid.
"""
from __future__ import annotations

import math
from typing import Any, Sequence

# Device-frame x: radar dRel is measured from the radar, ~1.52 m ahead
# of the camera / model origin. Keep in sync with radard.RADAR_TO_CAMERA.
RADAR_TO_CAMERA_M = 1.52

# Half a US lane (~3.7 m). In-path cars stay inside; adjacent / shoulder
# objects (signs, opposing lane) sit ~3–4 m off. Incumbent is a little
# wider so a real lead at the lane edge does not chatter off.
PATH_HALF_WIDTH_M = 1.8
PATH_INCUMBENT_HALF_WIDTH_M = 2.2

# Absolute speed toward ego. Stopped: vLead ≈ 0. Oncoming: vLead < 0
# (vRel ≈ −(v_ego + v_them)). Ignore at a crawl — vRel noise.
ONCOMING_VLEAD_MS = -1.5
ONCOMING_MIN_VEGO_MS = 2.0

# Radar-frame glitch floor (same as rain hold).
MIN_DREL_M = 0.5


def _num(obj: Any, *names: str) -> float | None:
  for name in names:
    if isinstance(obj, dict):
      val = obj.get(name)
    else:
      val = getattr(obj, name, None)
    if val is None:
      continue
    try:
      f = float(val)
    except (TypeError, ValueError):
      continue
    if math.isfinite(f):
      return f
  return None


def path_y_at_x(path_x: Sequence[float] | None, path_y: Sequence[float] | None,
                x: float) -> float | None:
  """Interpolate model travel-path y (device frame, +left) at x."""
  if path_x is None or path_y is None:
    return None
  n = min(len(path_x), len(path_y))
  if n < 2:
    return None
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  if not math.isfinite(x):
    return None
  xs = [float(v) for v in path_x[:n]]
  ys = [float(v) for v in path_y[:n]]
  if x <= xs[0]:
    return ys[0]
  if x >= xs[-1]:
    return ys[-1]
  for i in range(1, n):
    if x <= xs[i]:
      dx = xs[i] - xs[i - 1]
      t = 0.0 if dx == 0.0 else (x - xs[i - 1]) / dx
      return ys[i - 1] + t * (ys[i] - ys[i - 1])
  return ys[-1]


def model_path_xy(model: Any) -> tuple[list[float] | None, list[float] | None]:
  try:
    pos = model.position
    xs = [float(v) for v in pos.x]
    ys = [float(v) for v in pos.y]
  except (TypeError, ValueError, AttributeError):
    return None, None
  if len(xs) < 2 or len(xs) != len(ys):
    return None, None
  if not all(math.isfinite(v) for v in xs + ys):
    return None, None
  return xs, ys


def track_device_xy(track: Any) -> tuple[float, float] | None:
  """Radar track in the camera / model device frame (x forward, y left)."""
  d_rel = _num(track, "dRel")
  y_rel = _num(track, "yRel")
  if d_rel is None or y_rel is None:
    return None
  return d_rel + RADAR_TO_CAMERA_M, -y_rel


def path_lateral_m(track: Any, path_x: Sequence[float] | None = None,
                   path_y: Sequence[float] | None = None) -> float | None:
  """Signed lateral from the travel path (device +left). No path → −yRel."""
  xy = track_device_xy(track)
  if xy is None:
    return None
  x, y = xy
  py = path_y_at_x(path_x, path_y, x)
  return y if py is None else y - py


def track_v_lead(track: Any, v_ego: float) -> float | None:
  v_lead = _num(track, "vLead", "vLeadK")
  if v_lead is not None:
    return v_lead
  v_rel = _num(track, "vRel")
  if v_rel is None:
    return None
  return v_rel + float(v_ego)


def track_is_oncoming(track: Any, v_ego: float) -> bool:
  """True when the track is moving toward us in the opposing sense.

  Stationary / stopped-in-path: vLead ≈ 0 → False.
  Same-direction slower lead: vLead > 0 → False.
  Head-on / opposing: vLead ≲ −1.5 m/s → True.
  """
  if float(v_ego) < ONCOMING_MIN_VEGO_MS:
    return False
  v_lead = track_v_lead(track, v_ego)
  if v_lead is None:
    return False
  return v_lead < ONCOMING_VLEAD_MS


def track_is_in_path(track: Any, path_x: Sequence[float] | None = None,
                     path_y: Sequence[float] | None = None,
                     max_lat: float = PATH_HALF_WIDTH_M) -> bool:
  lat = path_lateral_m(track, path_x, path_y)
  if lat is None:
    return False
  return abs(lat) <= float(max_lat)


def radar_follow_ok(track: Any, v_ego: float = 0.0,
                    path_x: Sequence[float] | None = None,
                    path_y: Sequence[float] | None = None,
                    max_lat: float = PATH_HALF_WIDTH_M) -> bool:
  """True when a radar track may be a longitudinal follow lead."""
  d_rel = _num(track, "dRel")
  if d_rel is None or d_rel <= MIN_DREL_M:
    return False
  if track_is_oncoming(track, v_ego):
    return False
  return track_is_in_path(track, path_x, path_y, max_lat=max_lat)


def vision_lead_follow_ok(lead: Any, v_ego: float = 0.0,
                          path_x: Sequence[float] | None = None,
                          path_y: Sequence[float] | None = None,
                          max_lat: float = PATH_INCUMBENT_HALF_WIDTH_M) -> bool:
  """Vision-only lead: reject oncoming / far-off-path. Looser than radar.

  Do not use the tight radar half-width here — a cut-in the model still
  owns as leadOne can sit ~2 m off path for a few frames.
  """
  try:
    v = float(lead.v[0])
  except (TypeError, ValueError, AttributeError, IndexError):
    v = None
  if (v is not None and float(v_ego) >= ONCOMING_MIN_VEGO_MS and
      v < ONCOMING_VLEAD_MS):
    return False
  try:
    x = float(lead.x[0])
    y = float(lead.y[0])
  except (TypeError, ValueError, AttributeError, IndexError):
    return True
  if not math.isfinite(x) or not math.isfinite(y):
    return True
  py = path_y_at_x(path_x, path_y, x)
  lat = y if py is None else y - py
  return abs(lat) <= float(max_lat)
