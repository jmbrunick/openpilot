"""Vehicle location for mapd and Refresh maps.

Prefer a current GNSS sample (ublox / qcom). Fall back to comma's persistent
LastGPSPosition JSON. Never invent a city or default coordinate.
"""
from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Callable
from typing import Any

GPS_MAX_AGE_S = 2.5
GPS_MAX_ACC_M = 50.0
LAST_GPS_WRITE_PERIOD_S = 60.0

# Offroad Refresh maps: ublox may not be publishing yet, and yard/trees are
# coarser than mapd's 50 m match. Accept last received plausible lat/lon.
REFRESH_GNSS_WAIT_S = 15.0
REFRESH_GPS_MAX_ACC_M = 200.0

_PARAM_LAST_GPS_PATHS = (
  "/data/params/d/LastGPSPosition",
  os.path.expanduser("~/.comma/params/d/LastGPSPosition"),
)


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


def persist_last_gps_if_possible(lat: float, lon: float, altitude: float = 0.0) -> None:
  try:
    from openpilot.common.params import Params
    persist_last_gps_position(Params(), lat, lon, altitude)
  except Exception:
    pass


def load_last_gps_position(params: Any = None, raw: Any = None) -> tuple[float, float] | None:
  if raw is None and params is not None:
    try:
      raw = params.get("LastGPSPosition")
    except Exception:
      raw = None
  return parse_last_gps_position(raw)


def _last_gps_from_param_files() -> tuple[float, float] | None:
  for path in _PARAM_LAST_GPS_PATHS:
    try:
      with open(path, encoding="utf-8") as f:
        pos = parse_last_gps_position(f.read())
    except OSError:
      continue
    if pos is not None:
      return pos
  return None


def last_gps_from_params() -> tuple[float, float] | None:
  try:
    from openpilot.common.params import Params
    pos = load_last_gps_position(params=Params())
    if pos is not None:
      return pos
  except Exception:
    pass
  return _last_gps_from_param_files()


def gps_sample_from_sm(
  sm: Any,
  *,
  now: float | None = None,
  max_age_s: float | None = GPS_MAX_AGE_S,
  max_acc_m: float = GPS_MAX_ACC_M,
) -> tuple[float, float, float | None, bool]:
  """Return (lat, lon, bearing_deg or None, ok). Prefer external GNSS.

  max_age_s=None accepts the last received sample (Refresh maps). mapd keeps
  the 2.5 s / 50 m defaults so HUD MAX still tracks a live fix.
  """
  now = time.monotonic() if now is None else now
  for sock in ("gpsLocationExternal", "gpsLocation"):
    if sm.recv_frame.get(sock, -1) <= 0:
      continue
    if max_age_s is not None and (now - sm.recv_time[sock]) > max_age_s:
      continue
    g = sm[sock]
    lat, lon = float(g.latitude), float(g.longitude)
    if not is_plausible_lat_lon(lat, lon):
      continue
    acc = float(g.horizontalAccuracy)
    if acc > max_acc_m and acc > 0:
      continue
    bearing = float(g.bearingDeg) if (g.speed > 1.0 or g.bearingDeg) else None
    if bearing is not None and (math.isnan(bearing) or bearing < 0):
      bearing = None
    return lat, lon, bearing, True
  return 0.0, 0.0, None, False


def read_live_gnss(
  *,
  timeout_s: float = 8.0,
  sm: Any = None,
  max_age_s: float | None = GPS_MAX_AGE_S,
  max_acc_m: float = GPS_MAX_ACC_M,
  progress: Callable[[str], None] | None = None,
) -> tuple[float, float] | None:
  """Wait for a valid GNSS fix. Returns None if cereal/GPS is unavailable."""
  if sm is None:
    try:
      import cereal.messaging as messaging
      sm = messaging.SubMaster(["gpsLocationExternal", "gpsLocation"])
    except Exception:
      return None
  deadline = time.monotonic() + max(0.0, timeout_s)
  last_progress = 0.0
  while True:
    sm.update(200)
    lat, lon, _bearing, ok = gps_sample_from_sm(sm, max_age_s=max_age_s, max_acc_m=max_acc_m)
    if ok:
      return lat, lon
    now = time.monotonic()
    if progress is not None and (now - last_progress) >= 5.0:
      left = max(0.0, deadline - now)
      progress(f"Waiting for a satellite fix ({left:.0f}s left)...")
      last_progress = now
    if now >= deadline:
      return None
