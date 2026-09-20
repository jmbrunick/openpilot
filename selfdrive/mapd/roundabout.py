"""Roundabout detect, speed ease, and outer path bias.

Map `junction=roundabout` (or a closed circulating way already in the OSM
pack) — not steer. Funnel starts ~200 m so 40–45 mph can hit 15–20 by the
ring. Kinematic a (not Lookahead comfort −0.55) toward OSM maxspeed when
present. Clamp aTarget ≤ 0 while approaching / on the ring. ~3.2 m outer
path bias on entry + circulating (right in RHT / US) so a ~3 m inside cut
does not own lat.

Yield-before-merge, continue-circulate desire, and UI chip are later tips.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.constants import LOOKAHEAD_NORMAL

# Detect / start ease by this distance. Must still have fired by MIN.
# 45→20 at −1.0 needs ~180 m; 100 m was too late (Willmar 10:11 / 10:14).
RB_FUNNEL_M = 200.0
RB_FUNNEL_MIN_M = 80.0
# ~0.0035° ≈ 280 m EW at 45°N so a 200 m westbound approach is in the rtree.
RB_SEARCH_PAD_DEG = 0.0035

# Circulating target. OSM maxspeed on the RB way wins when present (often 20).
RB_V_MIN_MS = 15.0 * CV.MPH_TO_MS
RB_V_MAX_MS = 20.0 * CV.MPH_TO_MS
RB_V_DEFAULT_MS = 20.0 * CV.MPH_TO_MS

# Kinematic ease. Lookahead Early comfort is only 0.55 — that is why tip
# aTarget sat at −0.55 at 37–45 mph. Plan to ring speed from remaining d.
RB_A_MIN_MS2 = -2.0
RB_A_FLOOR_MS2 = -1.0
RB_A_FLOOR_V_MS = 40.0 * CV.MPH_TO_MS
RB_NEAR_SPEED_MS = 2.5 * CV.MPH_TO_MS
RB_ON_RING_DECEL_M = 28.0
RB_DECEL_D_MIN_M = 12.0

# Outer path bias. openpilot +y is left. EP1: OP lat R=14.9 vs OSM 18.2
# (−3.3 m inside); 0.45 m was far too small. 3.2 m counters that cut and
# sits near the outer half of the circulating lane / ring, not the island.
RB_OUTER_OFFSET_M = 3.2
RB_OUTER_OFFSET_MIN_M = 2.8
RB_OUTER_OFFSET_MAX_M = 3.6
# Preview length for a parallel-path curvature offset (κ ≈ 2 y / L²).
RB_PATH_LOOKAHEAD_M = 18.0
# Apply the same outer bias on the last stretch of approach (not 200 m out).
RB_ENTRY_BIAS_M = 50.0

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


def roundabout_hint_active(hint: RoundaboutHint | None) -> bool:
  return hint is not None and (hint.on_roundabout or hint.approaching)


def roundabout_ease_v_ms(
  hint: RoundaboutHint | None,
  v_ego_ms: float,
  current_ms: float = 0.0,
  lookahead: int = LOOKAHEAD_NORMAL,
) -> float | None:
  """Ring target as soon as the funnel is live. None if no RB.

  Do not interpolative-ease MAX over another 100 m — that left aTarget at
  Lookahead comfort (−0.55) after long enabled inside the funnel. `_` args
  kept so card / planner call sites stay unchanged.
  """
  _ = v_ego_ms, current_ms, lookahead
  if not roundabout_hint_active(hint):
    return None
  return roundabout_target_ms(hint.speed_limit_ms)


def roundabout_decel_ms2(hint: RoundaboutHint | None, v_ego_ms: float) -> float | None:
  """Kinematic −a to hit ring speed by remaining d, or 0 to clamp +a.

  None if no RB hint. While approaching / on-ring, never returns +a.
  From ≥40 mph, |a| is at least 1.0 until near ring speed (Willmar 10:11
  / 10:14: −0.55 at 37–45 mph was about half of 37→20 over 80–95 m).
  """
  if not roundabout_hint_active(hint):
    return None
  target = roundabout_target_ms(hint.speed_limit_ms)
  v0 = max(float(v_ego_ms), 0.0)
  if hint.on_roundabout or float(hint.distance_m) <= 0.0:
    d = RB_ON_RING_DECEL_M
  else:
    d = max(float(hint.distance_m), RB_DECEL_D_MIN_M)
  if v0 <= target + RB_NEAR_SPEED_MS:
    return 0.0
  a_kin = (target * target - v0 * v0) / (2.0 * d)
  a = max(RB_A_MIN_MS2, min(0.0, a_kin))
  if v0 >= RB_A_FLOOR_V_MS:
    a = min(a, RB_A_FLOOR_MS2)
  return a


def apply_roundabout_plan(
  v_ego_ms: float,
  v_cruise_ms: float,
  v_hud_ms: float,
  output_a_target: float,
  hint: RoundaboutHint | None,
  lookahead: int = LOOKAHEAD_NORMAL,
) -> tuple[float, float, float, float | None]:
  """Cap cruise / HUD to ring speed and min kinematic −a. No-op if no hint.

  Returns (v_cruise, v_hud, a_target, rb_v or None). a_target is ≤ 0 while
  the funnel is live (no +a rebound after a lead clears). Long-enable in
  the funnel is the same call — full ease on the first frame.
  """
  rb_v = roundabout_ease_v_ms(hint, v_ego_ms, v_hud_ms, lookahead)
  if rb_v is None:
    return float(v_cruise_ms), float(v_hud_ms), float(output_a_target), None
  v_cruise_ms = min(float(v_cruise_ms), float(rb_v))
  v_hud_ms = min(float(v_hud_ms), float(rb_v))
  a_rb = roundabout_decel_ms2(hint, v_ego_ms)
  if a_rb is not None:
    output_a_target = min(float(output_a_target), a_rb)
  output_a_target = min(float(output_a_target), 0.0)
  return v_cruise_ms, v_hud_ms, float(output_a_target), float(rb_v)


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


def _outer_bias_active(*, on_roundabout: bool, approaching: bool, distance_m: float) -> bool:
  if on_roundabout:
    return True
  return bool(approaching) and 0.0 < float(distance_m) <= RB_ENTRY_BIAS_M


def roundabout_outer_path_offset_m(
  *,
  on_roundabout: bool,
  approaching: bool = False,
  distance_m: float = 0.0,
  is_rhd: bool = False,
) -> float:
  """Outer-lane path offset in openpilot y (left +). 0 off the ring / far approach.

  RHT / US: outer is right → negative y. RHD: outer is left → positive y.
  Magnitude counters an EP1-class ~3 m inside cut and holds the outer half
  of the circulating lane. Applied on-ring and in the last 50 m of approach
  (not 200 m out on a straight).
  """
  if not _outer_bias_active(
    on_roundabout=on_roundabout, approaching=approaching, distance_m=distance_m,
  ):
    return 0.0
  mag = max(RB_OUTER_OFFSET_MIN_M, min(RB_OUTER_OFFSET_MAX_M, RB_OUTER_OFFSET_M))
  return mag if is_rhd else -mag


def roundabout_outer_curvature_bias(offset_m: float, lookahead_m: float = RB_PATH_LOOKAHEAD_M) -> float:
  """Curvature add-on that tracks a parallel path offset_m to the left.

  Positive offset (left) → positive curvature. 3.2 m right ≈ −0.020 /m at
  the 18 m preview — enough to walk a 3 m inside model path back to the
  OSM ring / outer half of the lane.
  """
  if abs(float(offset_m)) < 1e-6:
    return 0.0
  L = max(8.0, float(lookahead_m))
  return 2.0 * float(offset_m) / (L * L)
