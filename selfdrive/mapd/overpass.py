"""Overpass JSON → NAP OSM speed-limit ways.

OpenStreetMap data is ODbL: © OpenStreetMap contributors.
https://www.openstreetmap.org/copyright
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
import urllib.request

from openpilot.selfdrive.mapd.maps_manifest import USER_AGENT
from openpilot.selfdrive.mapd.mn_statutory import (
  FILL_SOURCE,
  FILLABLE_HIGHWAY_REGEX,
  PlaceIndex,
  is_urban_way,
  places_from_overpass,
  statutory_maxspeed_ms,
)
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
_KEEP = "Previous maps were left unchanged. You can retry Refresh maps."


def bbox_from_center(lat: float, lon: float, radius_km: float) -> tuple[float, float, float, float]:
  """south, west, north, east box that contains a radius_km circle around lat/lon."""
  dlat = radius_km / 111.0
  dlon = radius_km / (111.0 * max(0.2, abs(math.cos(math.radians(lat)))))
  return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def overpass_query(
  south: float, west: float, north: float, east: float,
  *,
  timeout_s: int = 180,
  include_unmarked: bool = False,
) -> str:
  """Bbox query. Tagged maxspeed only, or all fillable highways + place polygons (MN fill)."""
  bbox = f"({south},{west},{north},{east})"
  if include_unmarked:
    return f"""
[out:json][timeout:{int(timeout_s)}];
(
  way["highway"~"{FILLABLE_HIGHWAY_REGEX}"]{bbox};
  way["place"~"^(city|town|village)$"]{bbox};
  rel["place"~"^(city|town|village)$"]{bbox};
  way["boundary"="administrative"]["admin_level"="8"]{bbox};
  rel["boundary"="administrative"]["admin_level"="8"]{bbox};
);
out geom;
""".strip()
  return f"""
[out:json][timeout:{int(timeout_s)}];
way["highway"]["maxspeed"]{bbox};
out geom;
""".strip()


def fetch_overpass(
  bbox: tuple[float, float, float, float],
  url: str = OVERPASS_URL,
  *,
  timeout_s: float = 240,
  query_timeout: int = 180,
  include_unmarked: bool = False,
) -> dict:
  """POST a bbox highway query. Raises RuntimeError with a retryable message on timeout/HTTP failure."""
  q = overpass_query(*bbox, timeout_s=query_timeout, include_unmarked=include_unmarked)
  data = urllib.parse.urlencode({"data": q}).encode()
  req = urllib.request.Request(url, data=data, headers={"User-Agent": USER_AGENT})
  try:
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
      raw = resp.read().decode("utf-8")
  except urllib.error.HTTPError as e:
    raise RuntimeError(f"OSM Overpass failed HTTP {e.code}. {_KEEP}") from e
  except urllib.error.URLError as e:
    raise RuntimeError(f"OSM Overpass failed: {e}. Need Wi-Fi to overpass-api.de. {_KEEP}") from e
  except TimeoutError as e:
    raise RuntimeError(f"OSM Overpass timed out. {_KEEP}") from e
  try:
    payload = json.loads(raw)
  except json.JSONDecodeError as e:
    raise RuntimeError(f"OSM Overpass returned invalid JSON: {e}. {_KEEP}") from e
  if not isinstance(payload, dict):
    raise RuntimeError(f"OSM Overpass returned a non-object JSON payload. {_KEEP}")
  remark = str(payload.get("remark") or "")
  if "timed out" in remark.lower() or payload.get("timeout"):
    raise RuntimeError(f"OSM Overpass timed out ({remark or 'query timeout'}). {_KEEP}")
  return payload


def tagged_maxspeed_ms(tags: dict) -> float | None:
  """Numeric OSM maxspeed / maxspeed:forward. Non-numeric tags are not usable."""
  ms = parse_maxspeed(tags.get("maxspeed"))
  if ms is None:
    ms = parse_maxspeed(tags.get("maxspeed:forward"))
  return ms


def resolve_way_speed(
  tags: dict,
  coords: list[tuple[float, float]],
  *,
  fill_unmarked: bool = False,
  places: PlaceIndex | None = None,
) -> tuple[float | None, str]:
  """Return (maxspeed_ms, source). source is 'osm' or MN_169.14. Tagged numeric wins."""
  ms = tagged_maxspeed_ms(tags)
  if ms is not None:
    return ms, "osm"
  if not fill_unmarked:
    return None, ""
  highway = str(tags.get("highway") or "")
  urban = is_urban_way(tags, coords, places)
  filled = statutory_maxspeed_ms(highway, tags, urban=urban)
  if filled is None:
    return None, ""
  return filled, FILL_SOURCE


def ways_from_overpass(payload: dict, *, fill_unmarked: bool = False) -> list[dict]:
  places = places_from_overpass(payload) if fill_unmarked else PlaceIndex()
  out = []
  for el in payload.get("elements", []):
    if el.get("type") != "way":
      continue
    tags = el.get("tags") or {}
    if "highway" not in tags:
      continue
    geom = el.get("geometry") or []
    coords = [(float(p["lat"]), float(p["lon"])) for p in geom if "lat" in p and "lon" in p]
    if len(coords) < 2:
      continue
    ms, source = resolve_way_speed(tags, coords, fill_unmarked=fill_unmarked, places=places)
    if ms is None:
      continue
    out.append({
      "way_id": int(el["id"]),
      "name": tags.get("name") or tags.get("ref") or "",
      "highway": tags.get("highway") or "",
      "maxspeed_ms": ms,
      "coords": coords,
      "source": source,
    })
  return out
