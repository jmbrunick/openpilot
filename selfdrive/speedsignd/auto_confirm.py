"""Auto-confirm a located sign read when a map independently agrees (NAPSignAutoConfirm).

Default off. While a Yes/No prompt is open, each tick checks liveMapDataNAP:
  * the read is located: way_id > 0 and a GPS fix at the read
  * read conf >= MIN_CONF
  * car heading within HEADING_DEG of the read heading (same direction of travel)
  * the car is AFTER_READ_M .. MAX_AFTER_READ_M past the read point, so the
    maps describe the zone the sign starts (Tesla base flips ~40 m before the
    sign, raw OSM ~25 m after; a misread equal to the OLD limit must not match)
  * read mph == debounced Tesla base (teslaValid) -> source "tesla"
    or read mph == raw OSM (osmRawMph, never the published / override value)
    -> source "osm". MPP never confirms.
On a match: the same request_override the HUD Yes uses (auto=<source>), and
the prompt closes the way a timeout does, so the comma HUD and nap_dash MCU
page both clear. A row goes to speedsign_feedback/auto_confirm.jsonl (the
driver Yes/No training feedback in feedback.jsonl is left to real answers).
"""
from __future__ import annotations

import json
import math
import os
import time

PARAM = "NAPSignAutoConfirm"
MIN_CONF = 0.85
HEADING_DEG = 60.0
AFTER_READ_M = 60.0
MAX_AFTER_READ_M = 400.0
PARAM_PERIOD_S = 5.0
LOG_PATH = "/data/media/0/nap/speedsign_feedback/auto_confirm.jsonl"


def _hav_m(lat1, lon1, lat2, lon2) -> float:
  p = math.radians
  return 6371000.0 * math.hypot(p(lon2 - lon1) * math.cos(p((lat1 + lat2) / 2)), p(lat2 - lat1))


def _hdg_delta(a: float, b: float) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


def match_source(p, md, lat: float | None, lon: float | None, bearing: float | None) -> str | None:
  """Pure check. p = confirm.Proposal, md = liveMapDataNAP (or any object with its fields)."""
  if p is None or md is None:
    return None
  if int(p.way_id or 0) <= 0 or not (p.lat or p.lon) or float(p.conf) < MIN_CONF:
    return None
  if lat is None or lon is None or bearing is None:
    return None
  if _hdg_delta(float(bearing), float(p.heading)) > HEADING_DEG:
    return None
  d = _hav_m(p.lat, p.lon, lat, lon)
  if not (AFTER_READ_M <= d <= MAX_AFTER_READ_M):
    return None
  mph = int(p.mph)
  if bool(getattr(md, "teslaValid", False)) and int(getattr(md, "teslaBaseMph", 0)) == mph:
    return "tesla"
  if int(getattr(md, "osmRawMph", 0)) == mph:
    return "osm"
  return None


class AutoConfirm:
  def __init__(self, log_path: str | None = None) -> None:
    self.enabled = False
    self._param_t = -1e9
    self.log_path = log_path or LOG_PATH
    self.count = 0

  def refresh(self, params, now: float) -> None:
    if now - self._param_t < PARAM_PERIOD_S:
      return
    self._param_t = now
    try:
      self.enabled = bool(params.get_bool(PARAM))
    except Exception:
      self.enabled = False

  def tick(self, plate, md, gps, now: float, request=None) -> str | None:
    """Returns the source when the open prompt was auto-confirmed this tick."""
    if not self.enabled or plate is None or plate.proposal is None or md is None:
      return None
    lat, lon, bearing = gps or (None, None, None)
    p = plate.proposal
    src = match_source(p, md, lat, lon, bearing)
    if src is None:
      return None
    if request is None:
      from openpilot.selfdrive.mapd.speed_overrides import request_override as request
    rec = {"lat": p.lat, "lon": p.lon, "bearing": p.heading}
    sign_fields = getattr(p, "sign_fields", None)
    if callable(sign_fields):
      rec.update(sign_fields())   # start the section at the sign (same as a Yes)
    ok = request(int(p.mph), rec, now, way_id=int(p.way_id), heading=float(p.heading), auto=src)
    # Same as a Yes: confirmed mph set, prompt + read plate closed (both screens clear).
    plate.confirmed_mph = p.mph
    plate._close(p)
    self.count += 1
    self._log({"t": round(time.time(), 1), "mph": int(p.mph), "conf": round(float(p.conf), 3),  # noqa: TID251  wall clock row
               "source": src, "seq": int(p.seq), "way_id": int(p.way_id), "heading": round(float(p.heading), 1),
               "lat": p.lat, "lon": p.lon, "applied": bool(ok),
               "tesla_base": int(getattr(md, "teslaBaseMph", 0)), "osm_raw": int(getattr(md, "osmRawMph", 0)),
               "secs_after_prompt": round(now - float(p.t), 1)})
    return src

  def _log(self, row: dict) -> None:
    try:
      os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
      with open(self.log_path, "a") as f:
        f.write(json.dumps(row, separators=(",", ":")) + "\n")
    except OSError:
      pass
