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
from openpilot.selfdrive.mapd.speed_limit import parse_maxspeed

OVERPASS_URL = "https://overpass-api.de/api/interpreter"


def bbox_from_center(lat: float, lon: float, radius_km: float) -> tuple[float, float, float, float]:
  """south, west, north, east box that contains a radius_km circle around lat/lon."""
  dlat = radius_km / 111.0
  dlon = radius_km / (111.0 * max(0.2, abs(math.cos(math.radians(lat)))))
  return lat - dlat, lon - dlon, lat + dlat, lon + dlon


def overpass_query(south: float, west: float, north: float, east: float, *, timeout_s: int = 180) -> str:
  return f"""
[out:json][timeout:{int(timeout_s)}];
way["highway"]["maxspeed"]({south},{west},{north},{east});
out geom;
""".strip()


def fetch_overpass(
  bbox: tuple[float, float, float, float],
  url: str = OVERPASS_URL,
  *,
  timeout_s: float = 240,
  query_timeout: int = 180,
) -> dict:
  """POST a bbox maxspeed query. Raises RuntimeError with a retryable message on timeout/HTTP failure."""
  q = overpass_query(*bbox, timeout_s=query_timeout)
  data = urllib.parse.urlencode({"data": q}).encode()
  req = urllib.request.Request(url, data=data, headers={"User-Agent": USER_AGENT})
  try:
    with urllib.request.urlopen(req, timeout=timeout_s) as resp:
      raw = resp.read().decode("utf-8")
  except urllib.error.HTTPError as e:
    raise RuntimeError(
      f"OSM Overpass failed HTTP {e.code}. Previous maps were left unchanged. You can retry Refresh maps."
    ) from e
  except urllib.error.URLError as e:
    raise RuntimeError(
      f"OSM Overpass failed: {e}. Need Wi-Fi to overpass-api.de. Previous maps were left unchanged. You can retry Refresh maps."
    ) from e
  except TimeoutError as e:
    raise RuntimeError(
      "OSM Overpass timed out. Previous maps were left unchanged. You can retry Refresh maps."
    ) from e
  try:
    payload = json.loads(raw)
  except json.JSONDecodeError as e:
    raise RuntimeError(
      f"OSM Overpass returned invalid JSON: {e}. Previous maps were left unchanged. You can retry Refresh maps."
    ) from e
  if not isinstance(payload, dict):
    raise RuntimeError("OSM Overpass returned a non-object JSON payload. Previous maps were left unchanged.")
  remark = str(payload.get("remark") or "")
  if "timed out" in remark.lower() or payload.get("timeout"):
    raise RuntimeError(
      f"OSM Overpass timed out ({remark or 'query timeout'}). Previous maps were left unchanged. You can retry Refresh maps."
    )
  return payload


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
