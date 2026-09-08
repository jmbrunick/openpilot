"""Parse OpenStreetMap maxspeed tags into m/s.

OSM wiki: a unitless number is km/h. Explicit units (mph, knots) are honored.
Non-numeric values (signals, national, none) return None.
"""
from __future__ import annotations

import re

from openpilot.common.constants import CV

_NUMERIC = re.compile(
  r"^\s*(?P<val>\d+(?:\.\d+)?)\s*(?P<unit>mph|knots|km/h|kmh|kph)?\s*$",
  re.IGNORECASE,
)


def parse_maxspeed(tag: str | None) -> float | None:
  """Return speed limit in m/s, or None if the tag is missing/unusable."""
  if not tag:
    return None
  text = str(tag).strip()
  if not text or ":" in text:
    # Implicit country codes like "US:urban" / "sign" / "national" are not numeric limits.
    return None
  lowered = text.lower()
  if lowered in ("none", "signals", "variable", "walk", "unposted"):
    return None

  m = _NUMERIC.match(text)
  if m is None:
    return None
  val = float(m.group("val"))
  if val <= 0 or val > 200:
    return None
  unit = (m.group("unit") or "km/h").lower()
  if unit == "mph":
    return val * CV.MPH_TO_MS
  if unit == "knots":
    return val * CV.KNOTS_TO_MS
  return val * CV.KPH_TO_MS
