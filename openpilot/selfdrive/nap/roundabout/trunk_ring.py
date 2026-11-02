"""Roundabouts on trunk roads (NAP, longitudinal only).

Oct 11 (#311) dropped every roundabout hint while the matched road is
motorway or trunk, to stop loops beside I-74 from slowing the car. That
also hid real rings ON a trunk road: US 12 (way 985352994) and the ring at
way 858192519 were missed on Oct 14, 7:01 / 7:02 PM CT (route 0000019d).

Rule:
- motorway / motorway_link: still suppressed (no at-grade rings there).
- trunk / trunk_link: a ring counts when it is itself a major road
  (trunk ... tertiary: the trunk runs through it), or when it touches the
  road we are on (some ring point within RING_TOUCH_M of the matched way).
  The I-74 false rings were residential / service loops beside the
  carriageway: neither.
- anything else: unchanged.

Pure functions, no state, cheap: run from mapd only.
"""
from __future__ import annotations

import math

MOTORWAY = frozenset({"motorway", "motorway_link"})
TRUNK = frozenset({"trunk", "trunk_link"})
RING_TOUCH_M = 20.0
MAJOR_RING = frozenset({"trunk", "trunk_link", "primary", "primary_link", "secondary",
                        "secondary_link", "tertiary", "tertiary_link"})
_EARTH_R = 6371000.0

GATE_OPEN = 0
GATE_TOUCH = 1
GATE_BLOCK = 2


def trunk_ring_gate(current_highway: str | None) -> int:
  hw = (current_highway or "").strip().lower()
  if hw in MOTORWAY:
    return GATE_BLOCK
  if hw in TRUNK:
    return GATE_TOUCH
  return GATE_OPEN


def _xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
  return (math.radians(lon - lon0) * _EARTH_R * math.cos(math.radians(lat0)),
          math.radians(lat - lat0) * _EARTH_R)


def _seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
  dx, dy = bx - ax, by - ay
  L2 = dx * dx + dy * dy
  t = 0.0 if L2 <= 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
  return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def ring_touches_way(ring_coords, way_coords, tol_m: float = RING_TOUCH_M) -> bool:
  """True when any ring vertex is within tol_m of the way polyline."""
  if not ring_coords or not way_coords:
    return False
  lat0, lon0 = float(ring_coords[0][0]), float(ring_coords[0][1])
  way = [_xy(float(a), float(b), lat0, lon0) for a, b in way_coords]
  for a, b in ring_coords:
    px, py = _xy(float(a), float(b), lat0, lon0)
    if len(way) == 1:
      if math.hypot(px - way[0][0], py - way[0][1]) <= tol_m:
        return True
      continue
    for (ax, ay), (bx, by) in zip(way[:-1], way[1:], strict=True):
      if _seg_dist(px, py, ax, ay, bx, by) <= tol_m:
        return True
  return False


def trunk_ring_ok(ring_highway: str | None, ring_coords, way_coords) -> bool:
  """GATE_TOUCH acceptance for one ring."""
  if (ring_highway or "").strip().lower() in MAJOR_RING:
    return True
  return ring_touches_way(ring_coords, way_coords)
