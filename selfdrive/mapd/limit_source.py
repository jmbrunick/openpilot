"""Speed limit source selection (PR #391 design).

A) NAPSpeedLimitPreferOsm: global source toggle. Off (default) keeps the
   existing order: confirmed sign > Tesla-learned > live Tesla > OSM.
   On: confirmed sign > OSM (Tesla is still read and shown, never used).
   Per road: the driver's manual override from the LIMIT popup (B / C).
B/C) NAPSpeedLimitManual {"mph", "seq"} from the LIMIT-plate long-press popup
   (arrows / tap on the Tesla or OSM value / Save). mapd keys it to the way and
   heading the car is on now and stores it as a confirmed override row starting
   at the car (both directions from there on, never behind; #387). Clear uses
   NAPSpeedSignClearOverride.
Optional rule: NAPSpeedLimitOsmFallback (default On). When the Tesla value
   that would win (learned or live) and the tagged OSM limit differ by more
   than FALLBACK_DIFF_MPH, OSM is used and the spot goes on the sign review
   list. Estimated (road-type default) OSM never triggers it.
"""
from __future__ import annotations

import json

from openpilot.selfdrive.mapd.tesla_limit import MPH_TO_MS, SOURCE_LEARNED, SOURCE_TESLA, select_current

PARAM_PREFER_OSM = "NAPSpeedLimitPreferOsm"
PARAM_OSM_FALLBACK = "NAPSpeedLimitOsmFallback"
PARAM_MANUAL = "NAPSpeedLimitManual"
FALLBACK_DIFF_MPH = 20
SOURCE_OSM_FALLBACK = "osm_fallback"


def choose_current(sign_ms: float | None, tesla_mph: int, osm_ms: float, learned_ms: float | None = None, *,
                   prefer_osm: bool = False, fallback_on: bool = True, osm_tagged_mph: int = 0
                   ) -> tuple[float, str | None, bool]:
  """(limit m/s, source, disagreement fallback fired).

  source as select_current ("sign" / "tesla_learned" / "tesla" / None for OSM),
  plus SOURCE_OSM_FALLBACK when the > 20 mph rule picked OSM."""
  if sign_ms is not None:
    return float(sign_ms), "sign", False
  if prefer_osm:
    return float(osm_ms), None, False
  cur_ms, src = select_current(None, tesla_mph, osm_ms, learned_ms)
  if fallback_on and src in (SOURCE_TESLA, SOURCE_LEARNED) and int(osm_tagged_mph) > 0:
    cur_mph = int(round(cur_ms / MPH_TO_MS))
    if abs(cur_mph - int(osm_tagged_mph)) > FALLBACK_DIFF_MPH:
      return float(osm_ms), SOURCE_OSM_FALLBACK, True
  return cur_ms, src, False


def parse_manual(raw) -> dict | None:
  """{"mph": 5..85 step 5, "seq": float} or None."""
  if raw is None:
    return None
  try:
    d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
    mph, seq = int(d["mph"]), float(d["seq"])
  except Exception:
    return None
  if not (5 <= mph <= 85) or mph % 5:
    return None
  return {"mph": mph, "seq": seq}


def manual_confirmed(raw, way_id: int | None, lat: float | None, lon: float | None,
                     bearing: float | None) -> dict | None:
  """NAPSpeedSignConfirmed-style payload for SpeedOverrides.consume: the way and
  heading the car is on now, starting at the car (sign point = car point)."""
  m = parse_manual(raw)
  if m is None or not way_id or lat is None or lon is None or bearing is None:
    return None
  return {"mph": m["mph"], "seq": m["seq"], "way_id": int(way_id), "heading": float(bearing),
          "lat": float(lat), "lon": float(lon), "sign_lat": float(lat), "sign_lon": float(lon), "manual": True}


def step_mph(mph: int, up: bool) -> int:
  """Popup arrows: 5 mph steps (an off-grid value snaps to the next 5), kept in 5..85."""
  mph = int(mph)
  if mph % 5 == 0:
    nxt = mph + (5 if up else -5)
  else:
    nxt = (mph // 5 + 1) * 5 if up else (mph // 5) * 5
  return max(5, min(85, nxt))


def request_manual(mph: int, seq: float, params=None) -> bool:
  if parse_manual({"mph": mph, "seq": seq}) is None:
    return False
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    params.put(PARAM_MANUAL, {"mph": int(mph), "seq": float(seq)})
    return True
  except Exception:
    return False
