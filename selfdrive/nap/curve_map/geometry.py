"""Robust road curvature from a walked map polyline (NAP map curve preview).

Raw OSM nodes are noisy at the metre level: a three-node turn angle over
5-10 m reads a kink as a hairpin. Curvature here is the heading change
between two CHORD_M chords meeting at each point (a three-point circle on
a CHORD_M base), then a running median (kills single-node kinks), then a
SMOOTH_M box mean (the speed a car can hold through a bend depends on the
bend, not on one node).
"""
from __future__ import annotations

import math

CHORD_M = 20.0
MEDIAN_M = 25.0
SMOOTH_M = 30.0


def _wrap(a: float) -> float:
  return (a + math.pi) % (2.0 * math.pi) - math.pi


def curvature(xy: list[tuple[float, float]], step: float) -> list[float]:
  """Signed curvature (1/m, left > 0) at each point of an evenly spaced polyline."""
  n = len(xy)
  if n < 3:
    return [0.0] * n
  h = max(1, int(round(CHORD_M / step)))
  k = [0.0] * n
  for i in range(n):
    a, b = max(0, i - h), min(n - 1, i + h)
    if i - a < 1 or b - i < 1:
      continue
    (x0, y0), (x1, y1), (x2, y2) = xy[a], xy[i], xy[b]
    l0, l1 = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1)
    if l0 < 1.0 or l1 < 1.0:
      continue
    turn = _wrap(math.atan2(x1 - x0, y1 - y0) - math.atan2(x2 - x1, y2 - y1))
    k[i] = turn / (0.5 * (l0 + l1))
  m = max(1, int(round(MEDIAN_M / step)) // 2)
  med = [sorted(k[max(0, i - m):i + m + 1])[(min(n, i + m + 1) - max(0, i - m)) // 2] for i in range(n)]
  b = max(1, int(round(SMOOTH_M / step)) // 2)
  out = []
  for i in range(n):
    w = med[max(0, i - b):i + b + 1]
    out.append(sum(w) / len(w))
  return out
