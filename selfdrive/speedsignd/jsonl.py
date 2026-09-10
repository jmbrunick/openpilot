"""Append-only JSONL writer for speed-sign observations."""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Any

RECORD_KEYS = ("t", "lat", "lon", "bearing", "mph", "conf")

# Skip a repeat of the same mph near the last write (even 1 Hz would spam).
DEDUP_COOLDOWN_S = 8.0
DEDUP_RADIUS_M = 40.0


def make_record(t: float, lat: float, lon: float, bearing: float | None, mph: int, conf: float) -> dict[str, Any]:
  return {
    "t": float(t),
    "lat": float(lat),
    "lon": float(lon),
    "bearing": None if bearing is None else float(bearing),
    "mph": int(mph),
    "conf": float(conf),
  }


def record_line(record: dict[str, Any]) -> str:
  payload = {k: record[k] for k in RECORD_KEYS}
  return json.dumps(payload, separators=(",", ":"), ensure_ascii=True) + "\n"


def append_jsonl(path: str, record: dict[str, Any]) -> None:
  parent = os.path.dirname(path)
  if parent:
    os.makedirs(parent, exist_ok=True)
  with open(path, "a", encoding="utf-8") as f:
    f.write(record_line(record))
    f.flush()


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, math.sqrt(a)))


@dataclass
class JsonlLogger:
  path: str
  cooldown_s: float = DEDUP_COOLDOWN_S
  radius_m: float = DEDUP_RADIUS_M
  _last: tuple[float, float, float, int] | None = None  # t, lat, lon, mph

  def accept(self, record: dict[str, Any]) -> bool:
    t = float(record["t"])
    lat = float(record["lat"])
    lon = float(record["lon"])
    mph = int(record["mph"])
    if self._last is not None:
      lt, llat, llon, lmph = self._last
      if mph == lmph and (t - lt) < self.cooldown_s and _haversine_m(lat, lon, llat, llon) < self.radius_m:
        return False
    return True

  def write(self, record: dict[str, Any]) -> bool:
    if not self.accept(record):
      return False
    append_jsonl(self.path, record)
    self._last = (float(record["t"]), float(record["lat"]), float(record["lon"]), int(record["mph"]))
    return True
