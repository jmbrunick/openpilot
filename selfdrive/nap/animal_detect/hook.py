"""The one pathObstacle hook: copy the animal detector's class onto the
pathObstacleVisionNAP log for the same object. Log only; the chime, brakeGate
and every decision in path_obstacle stay exactly as they were."""
from __future__ import annotations

import math

STALE_S = 0.6
LATERAL_M = 1.5
RANGE_FRAC = 0.25


def animal_match(body, received_t: float, now: float, track_id: int, range_m: float, lateral: float) -> tuple[str, float]:
  """(class, conf) of the best animalDetectNAP detection on this radar track, else ("", nan)."""
  if body is None or now - received_t > STALE_S:
    return "", float("nan")
  try:
    best, best_conf = "", float("nan")
    for d in body.detections:
      same = bool(d.radarMatched) and int(d.radarTrackId) == int(track_id) and track_id != 0
      if not same:
        r, lat = float(d.range), float(d.lateral)
        same = (math.isfinite(r) and math.isfinite(range_m) and range_m > 0 and abs(r - range_m) <= RANGE_FRAC * range_m
                and math.isfinite(lat) and math.isfinite(lateral) and abs(lat - lateral) <= LATERAL_M)
      if same and not (float(d.conf) <= best_conf):
        best, best_conf = str(d.cls), float(d.conf)
    return best, best_conf
  except Exception:
    return "", float("nan")
