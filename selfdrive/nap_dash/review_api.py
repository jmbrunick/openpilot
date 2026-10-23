"""Read-only list of the sign review flags (selfdrive/mapd/review_flags.py).

Each line is a log note: "check this spot for a speed limit sign". The web UI
lists them; nothing here writes, bookmarks or alerts.
"""
from __future__ import annotations

import json
import os

from openpilot.selfdrive.mapd.review_flags import DEFAULT_DIR, FLAGS_NAME, SINGLE_NAME

MAX_ROWS = 100
_KEYS = ("kind", "reason", "mph", "wall", "route", "segment", "seg_offset_s", "lat", "lon", "bearing",
         "speed_mph", "road", "tesla", "osm", "reads_nearby")


def _tail(path: str, n: int) -> list[dict]:
  try:
    with open(path, "rb") as f:
      f.seek(0, os.SEEK_END)
      size = f.tell()
      f.seek(max(0, size - 256 * 1024))
      lines = f.read().decode("utf-8", errors="ignore").splitlines()
  except OSError:
    return []
  out = []
  for line in lines:
    try:
      r = json.loads(line)
    except ValueError:
      continue
    if isinstance(r, dict):
      out.append({k: r.get(k) for k in _KEYS})
  return out[-n:]


def read_flags(base: str | None = None, kind: str = "both", limit: int = MAX_ROWS) -> dict:
  base = base or DEFAULT_DIR
  limit = max(1, min(int(limit), MAX_ROWS))
  name = SINGLE_NAME if kind == "single" else FLAGS_NAME
  rows = _tail(os.path.join(base, name), limit)
  rows.reverse()   # newest first
  return {"kind": "single" if kind == "single" else "both", "flags": rows, "file": name}
