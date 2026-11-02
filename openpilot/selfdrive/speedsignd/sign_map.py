"""Where a read speed sign stands: GPS at the photo + camera geometry.

For each burst decision (two_stage.burst_decision) speedsignd places the sign:
  forward range  d = FOCAL_PX * H / h            (H = 0.76 m speed sign)
  lateral offset x = d * tan(atan((u - cx) / f) - yaw_calib)   (+right)
  position       = GPS fix at the photo time + (d forward, x right) along
                   the GPS bearing (flat-earth, metres -> degrees).
The record goes to liveSpeedSignNAP (sign* fields) and is appended to
SIGN_MAP_PATH. Nothing here writes a map or uploads anything.

Uncertainty (1 sigma, metres): GPS horizontal accuracy (default 5 m) plus
range error from box height (2 px quantisation + 15% sign-size spread) and
the lateral error a 1 deg heading error gives at that range, root-sum-square.
"""
from __future__ import annotations

import json
import math
import os

from openpilot.selfdrive.speedsignd import two_stage

SIGN_MAP_PATH = "/data/media/0/nap/sign_map.jsonl"
SIGN_MAP_MAX_BYTES = 8 * 1024 * 1024
GPS_SIGMA_DEFAULT_M = 5.0
SIZE_SPREAD = 0.15
BOX_PX_SIGMA = 2.0
HEADING_SIGMA_DEG = 1.0
EARTH_R_M = 6378137.0


def locate(lat: float, lon: float, bearing_deg: float | None, box, *, rpy=None,
           gps_sigma_m: float | None = None, sign_h_m: float = two_stage.SIGN_H_M) -> dict | None:
  """Sign position from the GPS fix at the photo and the read box (frame px)."""
  if bearing_deg is None or not (math.isfinite(lat) and math.isfinite(lon)):
    return None
  x, _y, w, h = (float(v) for v in box)
  if h <= 1.0:
    return None
  d = two_stage.FOCAL_PX * float(sign_h_m) / h
  yaw = float(rpy[2]) if rpy is not None and len(rpy) >= 3 else 0.0
  ang = math.atan(((x + w / 2.0) - two_stage.CAM_CX) / two_stage.FOCAL_PX) - yaw
  fwd = d * math.cos(ang)
  right = d * math.sin(ang)
  b = math.radians(float(bearing_deg))
  north = fwd * math.cos(b) - right * math.sin(b)
  east = fwd * math.sin(b) + right * math.cos(b)
  slat = lat + math.degrees(north / EARTH_R_M)
  slon = lon + math.degrees(east / (EARTH_R_M * max(1e-6, math.cos(math.radians(lat)))))
  gps_s = float(gps_sigma_m) if gps_sigma_m and gps_sigma_m > 0 else GPS_SIGMA_DEFAULT_M
  range_s = d * math.hypot(BOX_PX_SIGMA / h, SIZE_SPREAD)
  lat_s = d * math.radians(HEADING_SIGMA_DEG)
  sigma = math.sqrt(gps_s ** 2 + range_s ** 2 + lat_s ** 2)
  return dict(lat=round(slat, 7), lon=round(slon, 7), heading=round(float(bearing_deg) % 360.0, 1),
              range_m=round(d, 1), offset_m=round(right, 1), sigma_m=round(sigma, 1))


def record(mph: int, conf: float, px: float, loc: dict, t_wall: float, reads: int) -> dict:
  return dict(t=round(float(t_wall), 3), mph=int(mph), conf=round(float(conf), 3), px=round(float(px), 1),
              reads=int(reads), **loc)


def append(rec: dict, path: str = SIGN_MAP_PATH, max_bytes: int = SIGN_MAP_MAX_BYTES) -> bool:
  """Append one JSON line. Never raises; stops at max_bytes."""
  try:
    d = os.path.dirname(path)
    if d:
      os.makedirs(d, exist_ok=True)
    if os.path.exists(path) and os.path.getsize(path) >= max_bytes:
      return False
    with open(path, "a") as f:
      f.write(json.dumps(rec, separators=(",", ":")) + "\n")
    return True
  except Exception:
    return False
