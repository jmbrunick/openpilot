"""Travel-path and oncoming gates for radar follow-lead selection.

e1 qlog `1c95345a3286a5db|000000e1--b993674371` (tip `638f5f7d4`,
NAPWiperSpeed==3) confirmed #201 rain-hold latched off-path STAT /
oncoming tracks as leadOne (modelProb ≪ 0.15 → aTarget=−3.5).
Episodes: 20:46:45, 20:49:10, 20:51:52, 20:53:45, 20:54:53 CT.
#199 look-ahead exonerated.

The driving path is `modelV2.position` — the same plan path the UI
draws and vision uses for lead-in-path (`leadsV3` is the in-path lead
head). Radar used for long, including rain-hold, must sit on that path.
Rain-hold latches path-valid *associations* only; it does not prefer
an unassociated Bosch track in a wide FOV.

Stationary in-path objects (stopped cars) stay valid: vLead ≈ 0, not
oncoming. Same-direction leads stay valid.
"""
from __future__ import annotations

import math
from typing import Any, Sequence

# Device-frame x: radar dRel is measured from the radar, ~1.52 m ahead
# of the camera / model origin. Keep in sync with radard.RADAR_TO_CAMERA.
RADAR_TO_CAMERA_M = 1.52

# e1 qlog (1c95345a3286a5db|000000e1--b993674371): #201's 2.5 / 4.0 yRel
# windows held LEFT STAT signs at +2.2…+3.9 m. Acquire 1.5 m / incumbent
# 2.0 m (dig recommendation). Path-relative when modelV2.position exists.
PATH_HALF_WIDTH_M = 1.5
PATH_INCUMBENT_HALF_WIDTH_M = 2.0

# Absolute speed toward ego. Stopped: vLead ≈ 0. Oncoming: vLead < 0
# (vRel ≈ −(v_ego + v_them)). e1 track 101 was vLead ≈ −5…−6. Ignore at crawl.
ONCOMING_VLEAD_MS = -1.0
ONCOMING_MIN_VEGO_MS = 2.0

# Radar-frame glitch floor (same as rain hold).
MIN_DREL_M = 0.5

# Synthesized Fri 2026-09-18 ~20:54 CT right-exit from a roundabout
# (device +left). Dig will pin the live segment; these numbers are the
# path-vs-off-path geometry: exit ~4 m right at 30 m, entrant at yRel≈0.
ROUNDABOUT_EXIT_PATH_X = (0.0, 10.0, 20.0, 30.0, 50.0)
ROUNDABOUT_EXIT_PATH_Y = (0.0, -0.8, -2.2, -4.0, -6.0)
ROUNDABOUT_EXIT_X_M = 30.0

# Synthesized Fri 2026-09-18 ~20:55 CT left-hand curve. Path at 45 m is
# +4 m left. A roadside sign on the *outside* of the turn (~two 3.7 m
# lanes) sits ~7.4 m to the right of that path (device y ≈ −3.4).
CURVE_OUTSIDE_PATH_X = (0.0, 15.0, 30.0, 45.0, 70.0)
CURVE_OUTSIDE_PATH_Y = (0.0, 1.2, 2.8, 4.0, 5.5)
CURVE_OUTSIDE_X_M = 45.0
CURVE_OUTSIDE_TWO_LANES_M = 7.4


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
