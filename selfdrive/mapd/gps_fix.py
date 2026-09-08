"""Vehicle location for mapd and Refresh maps.

Prefer a current GNSS sample (ublox / qcom). Fall back to comma's persistent
LastGPSPosition JSON. Never invent a city or default coordinate.
"""
from __future__ import annotations

import json
import math
import time
from typing import Any

GPS_MAX_AGE_S = 2.5
GPS_MAX_ACC_M = 50.0
LAST_GPS_WRITE_PERIOD_S = 60.0


def is_plausible_lat_lon(lat: float, lon: float) -> bool:
  if not math.isfinite(lat) or not math.isfinite(lon):
    return False
  if abs(lat) < 1e-6 and abs(lon) < 1e-6:
    return False
  return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


def parse_last_gps_position(raw: Any) -> tuple[float, float] | None:
  """Parse comma LastGPSPosition JSON: {"latitude": …, "longitude": …, "altitude": …}."""
  if raw is None:
    return None
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="replace")
  if isinstance(raw, dict):
    data = raw
  else:
    text = str(raw).strip()
    if not text:
      return None
    try:
      data = json.loads(text)
    except (json.JSONDecodeError, TypeError, ValueError):
      return None
  if not isinstance(data, dict):
    return None
  try:
    lat = float(data.get("latitude", data.get("lat")))
    lon = float(data.get("longitude", data.get("lon")))
  except (TypeError, ValueError):
    return None
  if not is_plausible_lat_lon(lat, lon):
    return None
  return lat, lon


def format_last_gps_position(lat: float, lon: float, altitude: float = 0.0) -> str:
  return json.dumps({"latitude": float(lat), "longitude": float(lon), "altitude": float(altitude)})


def persist_last_gps_position(params: Any, lat: float, lon: float, altitude: float = 0.0) -> None:
  if not is_plausible_lat_lon(lat, lon):
    return
  try:
    params.put("LastGPSPosition", format_last_gps_position(lat, lon, altitude))
  except Exception:
    pass


def load_last_gps_position(params: Any = None, raw: Any = None) -> tuple[float, float] | None:
  if raw is None and params is not None:
    try:
      raw = params.get("LastGPSPosition")
    except Exception:
      raw = None
  return parse_last_gps_position(raw)


def gps_sample_from_sm(sm: Any, *, now: float | None = None, max_age_s: float = GPS_MAX_AGE_S) -> tuple[float, float, float | None, bool]:
  """Return (lat, lon, bearing_deg or None, ok). Prefer external GNSS."""
  now = time.monotonic() if now is None else now
  for sock in ("gpsLocationExternal", "gpsLocation"):
    if sm.recv_frame.get(sock, -1) <= 0:
      continue
    if (now - sm.recv_time[sock]) > max_age_s:
      continue
    g = sm[sock]
    lat = float(g.latitude)
    lon = float(g.longitude)
    if not is_plausible_lat_lon(lat, lon):
      continue
    acc = float(getattr(g, "horizontalAccuracy", 0.0) or 0.0)
    if acc > GPS_MAX_ACC_M and acc > 0:
      continue
    speed = float(getattr(g, "speed", 0.0) or 0.0)
    bearing = float(getattr(g, "bearingDeg", 0.0) or 0.0) if (speed > 1.0 or getattr(g, "bearingDeg", None)) else None
    if bearing is not None and (math.isnan(bearing) or bearing < 0):
      bearing = None
    return lat, lon, bearing, True
  return 0.0, 0.0, None, False


def read_live_gnss(*, timeout_s: float = 8.0, sm: Any = None) -> tuple[float, float] | None:
  """Wait briefly for a valid GNSS fix. Returns None if cereal/GPS is unavailable."""
  owns_sm = sm is None
  if sm is None:
    try:
      import cereal.messaging as messaging
      sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
    except Exception:
      return None
  deadline = time.monotonic() + max(0.0, timeout_s)
  try:
    while True:
      sm.update(200)
      lat, lon, _bearing, ok = gps_sample_from_sm(sm)
      if ok:
        return lat, lon
      if time.monotonic() >= deadline:
        return None
  finally:
    if owns_sm:
      try:
        sm.stop()
      except Exception:
        pass
