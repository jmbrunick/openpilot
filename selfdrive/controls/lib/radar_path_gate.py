"""Travel-path and oncoming gates for radar follow-lead selection.

e1 qlog `1c95345a3286a5db|000000e1--b993674371` (tip `638f5f7d4`)
confirmed #201 rain-hold latched off-path STAT / oncoming tracks as
leadOne (modelProb ≪ 0.15 → aTarget=−3.5). Episodes: 20:46:45,
20:49:10, 20:51:52, 20:53:45, 20:54:53 CT. #199 look-ahead exonerated.

The driving path is `modelV2.position` — the same plan path the UI
draws and vision uses for lead-in-path (`leadsV3` is the in-path lead
head). Radar used for long — default path-gated prefer, not rain-Auto
only — must sit on that path. Prefer latches path-valid *associations*
only; it does not pick an unassociated Bosch track in a wide FOV.

Stationary in-path objects (stopped cars) stay valid: vLead ≈ 0, not
oncoming. Same-direction leads stay valid. Off-path / oncoming are
always rejected. Unhealthy radar / no path association falls back to
stock fusion.
"""
from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

# Device-frame x: radar dRel is measured from the radar, ~1.52 m ahead
# of the camera / model origin. Keep in sync with radard.RADAR_TO_CAMERA.
RADAR_TO_CAMERA_M = 1.52

# EP_2059: #201's 2.5 / 4.0 yRel windows latched 771/802 at
# +2.23…+2.48 (0.02–0.27 m inside 2.5) → aTarget=−3.5. Justin:
# RAIN_INLANE 2.5 → 2.0 m (~6.6 ft, ~1.6 ft tighter) + vLead < 0.
# Clean 21:02/21:03 never acquired (|yRel| > 2.5). Path-relative
# when modelV2 exists. Incumbent 2.0 so a latch cannot walk to 3.98.
PATH_HALF_WIDTH_M = 1.5
PATH_INCUMBENT_HALF_WIDTH_M = 2.0

# Absolute speed toward ego. Stopped: vLead ≈ 0. Oncoming: vLead < 0
# (vRel ≈ −(v_ego + v_them)). e1 track 101 vLead ≈ −5…−6; EP_2059
# 771/802 ≈ −22 / −19 m/s. Floor −1 m/s so stationary noise is not
# oncoming.
#
# This is a follow-lead publication gate only: do not make opposing
# traffic leadOne. It does not invent regen/hesitation. 20:58 / 21:02 /
# 21:03 mid-lane passers never associated and must stay ignored — same
# as stock. Off-path / high-|yRel| / unassociated is the actual bug.
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

# Synthesized Fri 2026-09-18 ~21:06–21:07 CT left turn. Path is already
# +4 m left at 30 m. Ego-forward radar (yRel≈0) is furniture / an
# oncoming semi that is NOT on the travel path. yRel-only is not enough.
LEFT_TURN_PATH_X = (0.0, 15.0, 30.0, 50.0)
LEFT_TURN_PATH_Y = (0.0, 1.8, 4.0, 6.5)
LEFT_TURN_X_M = 30.0

# 21:08 Atlantic left: path has already left by ~3 m at 10 m, so
# ego-forward gas-station furniture (yRel≈0) is off path even as dRel
# closes and the car would otherwise stop in the road.
ATLANTIC_LEFT_PATH_X = (0.0, 5.0, 10.0, 20.0, 40.0)
ATLANTIC_LEFT_PATH_Y = (0.0, 1.5, 3.2, 5.0, 7.0)

# Construction path collapse (Scallywag 08:57:58). Adjacent lane-line
# probs gone and |path y| at 15–30 m is not a lane. Raise the bar for
# a *new* association at |yRel| ≳ 2 (the right-side blip was ~−2.9).
# An already-owned lead is the planner's departing-release problem.
PATH_COLLAPSE_LANE_PROB = 0.15
PATH_COLLAPSE_X_LO_M = 15.0
PATH_COLLAPSE_X_HI_M = 30.0
PATH_COLLAPSE_ABS_Y_M = 20.0
PATH_COLLAPSE_NEW_YREL_M = 2.0


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
  """True when a radar track may be a longitudinal follow lead.

  Off-path / high-|yRel| / unassociated tracks fail the path gate.
  Oncoming fails only as a follow candidate (stay ignored) — it does
  not command a new brake. Straight opposing that never associates
  stays leadOne=None, same as the 20:58 CT positive control.
  """
  d_rel = _num(track, "dRel")
  if d_rel is None or d_rel <= MIN_DREL_M:
    return False
  if track_is_oncoming(track, v_ego):
    return False
  return track_is_in_path(track, path_x, path_y, max_lat=max_lat)


def lane_lines_collapsed(probs) -> bool:
  """True when both adjacent lane lines have collapsed probability.

  Index 1 is the left line, index 2 the right (same as LDW). A short
  or missing list is not collapse — do not invent a gate.
  """
  if probs is None:
    return False
  try:
    vals = [float(p) for p in list(probs)]
  except (TypeError, ValueError):
    return False
  if len(vals) < 3:
    return False
  if not all(math.isfinite(v) for v in vals[1:3]):
    return False
  return (
    vals[1] < PATH_COLLAPSE_LANE_PROB and vals[2] < PATH_COLLAPSE_LANE_PROB
  )


def path_y_absurd(path_x: Sequence[float] | None,
                  path_y: Sequence[float] | None) -> bool:
  """True when travel-path y at 15–30 m is not a drivable lane offset."""
  if path_x is None or path_y is None:
    return False
  try:
    n = min(len(path_x), len(path_y))
  except TypeError:
    return False
  if n < 2:
    return False
  xs = [PATH_COLLAPSE_X_LO_M, 22.0, PATH_COLLAPSE_X_HI_M]
  for i in range(n):
    try:
      x = float(path_x[i])
    except (TypeError, ValueError):
      continue
    if PATH_COLLAPSE_X_LO_M <= x <= PATH_COLLAPSE_X_HI_M and x not in xs:
      xs.append(x)
  peak = 0.0
  saw = False
  for x in xs:
    y = path_y_at_x(path_x, path_y, x)
    if y is None:
      continue
    saw = True
    peak = max(peak, abs(float(y)))
  return saw and peak >= PATH_COLLAPSE_ABS_Y_M


def path_model_collapsed(lane_probs, path_x: Sequence[float] | None = None,
                         path_y: Sequence[float] | None = None) -> bool:
  """Lane lines down and the near path is absurd. Both are required."""
  return lane_lines_collapsed(lane_probs) and path_y_absurd(path_x, path_y)


def collapse_blocks_new_lead(y_rel, collapsed, incumbent=False) -> bool:
  """Block a new lead at |yRel| ≳ 2 while the path model has collapsed.

  Right-side (negative yRel) is the construction blip; either side
  past the bar is the same new-association gate. An incumbent track
  stays so a real cut-in can walk off without a hard drop.
  """
  if not collapsed or incumbent or y_rel is None:
    return False
  try:
    y = float(y_rel)
  except (TypeError, ValueError):
    return False
  if not math.isfinite(y):
    return False
  return abs(y) >= PATH_COLLAPSE_NEW_YREL_M


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
