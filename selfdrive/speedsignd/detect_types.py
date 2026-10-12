"""Shared MUTCD mph set and detection record. No I/O."""
from __future__ import annotations

from dataclasses import dataclass

# US R2-1 posted speeds. School-zone 15/25 and freeway 70/75/80 included.
MUTCD_MPH = frozenset(list(range(5, 90, 5)) + [100])


@dataclass(frozen=True)
class SpeedSign:
  mph: int
  conf: float
  bbox: tuple[int, int, int, int]  # x, y, w, h in the input frame
