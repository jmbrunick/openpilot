"""Roundabout detect, speed ease, and outer in-lane path bias.

First slice (Justin): map `junction=roundabout` (or a closed circulating
way already in the OSM pack) — not steer. Funnel ~80–100 m. Soft comfort
decel toward 15–20 mph (OSM maxspeed on the ring when present). ~0.45 m
outer bias while circulating (right in RHT / US).

Do not treat a sharp town corner or signalized intersection as an RB.
Yield-before-merge, continue-circulate desire, and UI chip are later tips.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL
from openpilot.selfdrive.mapd.map_speed_policy import anticipatory_limit_ms

# Detect by this distance — do not wait for a big steer. Comfort ease may
# start as soon as the walk sees the ring; tests pin the 80–100 m funnel.
RB_FUNNEL_M = 100.0
RB_FUNNEL_MIN_M = 80.0

# Circulating target. OSM maxspeed on the RB way wins when present (often 20).
RB_V_MIN_MS = 15.0 * CV.MPH_TO_MS
RB_V_MAX_MS = 20.0 * CV.MPH_TO_MS
RB_V_DEFAULT_MS = 20.0 * CV.MPH_TO_MS

# In-lane outer bias while circulating. openpilot +y is left.
RB_OUTER_OFFSET_M = 0.45
RB_OUTER_OFFSET_MIN_M = 0.30
RB_OUTER_OFFSET_MAX_M = 0.60
# Preview length for a parallel-path curvature offset (κ ≈ 2 y / L²).
RB_PATH_LOOKAHEAD_M = 18.0

# Closed-loop geometry for published packs that do not store `junction`.
RB_RADIUS_MIN_M = 6.0
RB_RADIUS_MAX_M = 55.0
RB_CLOSED_GAP_M = 8.0
RB_ON_WAY_M = 25.0
# Service / path loops (cul-de-sac bulbs) are not circulating RBs unless tagged.
_UNTAGGED_SKIP_HIGHWAY = frozenset({
  "service", "footway", "path", "cycleway", "pedestrian", "steps", "track",
})
_RB_JUNCTIONS = frozenset({"roundabout", "circular"})

EARTH_R = 6371000.0


@dataclass(frozen=True)
class RoundaboutHint:
  on_roundabout: bool = False
  approaching: bool = False
  distance_m: float = 0.0
  speed_limit_ms: float = 0.0
  way_id: int = 0


def junction_is_roundabout(junction: str | None) -> bool:
  return (junction or "").strip().lower() in _RB_JUNCTIONS


def _local_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  x = math.radians(lon - lon0) * math.cos(math.radians(lat0)) * EARTH_R
  y = math.radians(lat - lat0) * EARTH_R
  return x, y


def way_radius_m(coords: list[tuple[float, float]] | tuple[tuple[float, float], ...]) -> float | None:
  if len(coords) < 3:
    return None
  lat0 = sum(c[0] for c in coords) / len(coords)
  lon0 = sum(c[1] for c in coords) / len(coords)
  rs = [math.hypot(*_local_xy(c[0], c[1], lat0, lon0)) for c in coords]
  r = sum(rs) / len(rs)
  return r if r > 0.5 else None


def way_is_closed_loop(coords: list[tuple[float, float]] | tuple[tuple[float, float], ...]) -> bool:
  if len(coords) < 4:
    return False
  gap = math.hypot(*_local_xy(coords[-1][0], coords[-1][1], coords[0][0], coords[0][1]))
  return gap <= RB_CLOSED_GAP_M


def way_is_roundabout(
  coords: list[tuple[float, float]] | tuple[tuple[float, float], ...],
  junction: str = "",
  highway: str = "",
) -> bool:
  """True for OSM circulating rings — not a sharp corner or cross street.

  Tagged `junction=roundabout|circular` is authoritative (multi-way rings).
  Published speed-limit packs may omit that column; a compact closed loop
  of typical RB radius is the fallback. Signalized intersections are
  crossing polylines, not a closed circulating way.
  """
  tagged = junction_is_roundabout(junction)
  r = way_radius_m(coords)
  if tagged:
    if r is None:
      return True
    return r <= (RB_RADIUS_MAX_M * 2.5)
  hw = (highway or "").strip().lower()
  if hw in _UNTAGGED_SKIP_HIGHWAY:
    return False
  if not way_is_closed_loop(coords) or r is None:
    return False
  return RB_RADIUS_MIN_M <= r <= RB_RADIUS_MAX_M


def roundabout_target_ms(speed_limit_ms: float = 0.0) -> float:
  """Soft circulating v. OSM ring maxspeed when present, else 20 mph."""
  if speed_limit_ms is not None and float(speed_limit_ms) > 0.5:
    return max(RB_V_MIN_MS, min(RB_V_MAX_MS, float(speed_limit_ms)))
  return RB_V_DEFAULT_MS


def roundabout_ease_v_ms(
  hint: RoundaboutHint | None,
  v_ego_ms: float,
  current_ms: float = 0.0,
  lookahead: int = LOOKAHEAD_NORMAL,
) -> float | None:
  """Decrease-only soft v toward the ring, or None if no RB in the funnel."""
  if hint is None or not (hint.on_roundabout or hint.approaching):
    return None
  target = roundabout_target_ms(hint.speed_limit_ms)
  if hint.on_roundabout or float(hint.distance_m) <= 0.0:
    return target
  v0 = max(float(v_ego_ms), float(current_ms), 0.0)
  if v0 <= target + 0.3:
    return target
  eased = anticipatory_limit_ms(
    max(float(current_ms), v0, target),
    target,
    float(hint.distance_m),
    v0,
    lookahead,
  )
  if eased is None:
    # Inside the funnel we still ease — do not wait for a posted next-limit.
    return target if float(hint.distance_m) <= RB_FUNNEL_M else None
  return max(target, min(v0, float(eased)))


def live_map_roundabout_hint(md) -> RoundaboutHint | None:
  """Read LiveMapDataNAP RB fields. Missing / old messages → None."""
  if md is None:
    return None
  try:
    on_rb = bool(getattr(md, "onRoundabout", False))
    approaching = bool(getattr(md, "approachingRoundabout", False))
    if not on_rb and not approaching:
      return None
    return RoundaboutHint(
      on_roundabout=on_rb,
      approaching=approaching,
      distance_m=float(getattr(md, "roundaboutDistance", 0.0) or 0.0),
      speed_limit_ms=float(getattr(md, "roundaboutSpeedLimit", 0.0) or 0.0),
      way_id=int(getattr(md, "roundaboutWayId", 0) or 0),
    )
  except Exception:
    return None


def roundabout_outer_path_offset_m(*, on_roundabout: bool, is_rhd: bool = False) -> float:
  """In-lane outer bias in openpilot y (left +). 0 off the ring.

  RHT / US: outer is right → negative y. RHD: outer is left → positive y.
  Magnitude is mid of the 0.3–0.6 m band.
  """
  if not on_roundabout:
    return 0.0
  mag = max(RB_OUTER_OFFSET_MIN_M, min(RB_OUTER_OFFSET_MAX_M, RB_OUTER_OFFSET_M))
  return mag if is_rhd else -mag


def roundabout_outer_curvature_bias(offset_m: float, lookahead_m: float = RB_PATH_LOOKAHEAD_M) -> float:
  """Curvature add-on that tracks a parallel path offset_m to the left.

  Positive offset (left) → positive curvature. 0.45 m right ≈ −0.0028 /m
  at the 18 m preview — a small in-lane nudge, not a lane change.
  """
  if abs(float(offset_m)) < 1e-6:
    return 0.0
  L = max(8.0, float(lookahead_m))
  return 2.0 * float(offset_m) / (L * L)
