"""Overpass JSON → NAP OSM speed-limit ways."""
from __future__ import annotations

from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed


def ways_from_overpass(payload: dict) -> list[dict]:
  out = []
  for el in payload.get("elements", []):
    if el.get("type") != "way":
      continue
    tags = el.get("tags") or {}
    geom = el.get("geometry") or []
    coords = [(float(p["lat"]), float(p["lon"])) for p in geom if "lat" in p and "lon" in p]
    if len(coords) < 2:
      continue
    ms = parse_maxspeed(tags.get("maxspeed"))
    if ms is None:
      ms = parse_maxspeed(tags.get("maxspeed:forward"))
    if ms is None:
      continue
    out.append({
      "way_id": int(el["id"]),
      "name": tags.get("name") or tags.get("ref") or "",
      "highway": tags.get("highway") or "",
      "maxspeed_ms": ms,
      "coords": coords,
    })
  return out
