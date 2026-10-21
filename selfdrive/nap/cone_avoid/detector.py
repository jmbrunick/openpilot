"""Per-cone detections from radar posts and a CPU camera check.

Every cone is kept individually. Nothing is line-fitted for clearance, so one
cone knocked into the lane is never smoothed away by its row. A lone cone (one
detection, camera or radar) counts. Radar-only singles with no camera support
are kept but flagged low confidence (a ghost guard): logged, not steered on.
Coordinates: x forward (m), y path-relative +right (m).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

RADAR, CAMERA = "radar", "camera"
FUSE_M = 1.0            # radar and camera detections this close are the same cone
ROW_CONF = 0.9          # radar post that belongs to a tracked row (>= 2 posts)
GHOST_CONF = 0.3        # radar single, no camera: low confidence
CAMERA_CONF = 0.7       # camera alone (can create a line on its own)
FUSED_CONF = 0.95
CAR_HALF_M = 0.95


@dataclass
class ConeObs:
  x: float
  y: float
  source: str
  conf: float = 1.0
  age_s: float = 0.0
  in_row: bool = False


@dataclass
class Cone:
  x: float
  y: float
  conf: float
  sources: set = field(default_factory=set)
  low_conf: bool = False


def _finite(*v) -> bool:
  return all(isinstance(a, (int, float)) and math.isfinite(a) for a in v)


def fuse(obs: list[ConeObs], stale_s: float = 0.3) -> list[Cone]:
  """Fresh detections -> individual cones with a confidence each."""
  fresh = [o for o in obs if _finite(o.x, o.y, o.age_s) and o.age_s <= stale_s]
  cams = [o for o in fresh if o.source == CAMERA]
  rads = [o for o in fresh if o.source == RADAR]
  cones: list[Cone] = []
  used = set()
  for r in rads:
    match = None
    for i, c in enumerate(cams):
      if i not in used and math.hypot(c.x - r.x, c.y - r.y) <= FUSE_M:
        match = i
        break
    if match is not None:
      used.add(match)
      cones.append(Cone(r.x, r.y, FUSED_CONF, {RADAR, CAMERA}))
    elif r.in_row:
      cones.append(Cone(r.x, r.y, min(ROW_CONF, max(r.conf, 0.5)), {RADAR}))
    else:
      cones.append(Cone(r.x, r.y, GHOST_CONF, {RADAR}, low_conf=True))
  for i, c in enumerate(cams):
    if i not in used:
      cones.append(Cone(c.x, c.y, CAMERA_CONF, {CAMERA}))
  return cones


def camera_rows(cones: list[Cone], min_count: int = 2) -> dict[int, float]:
  """Median lateral of each side's camera cones: a line from the camera alone.

  Informational (logging, row return target). Clearance never uses this.
  """
  rows = {}
  for side in (-1, 1):
    ys = [c.y for c in cones if CAMERA in c.sources and (c.y < 0) == (side < 0)]
    if len(ys) >= min_count:
      rows[side] = float(np.median(ys))
  return rows


def obs_from_messages(cone_msg, vision_msg, cone_age_s: float, vision_age_s: float) -> list[ConeObs]:
  """Adapter from today's NAP messages.

  coneLineNAP: tracked radar posts (nearest post per side from leftClear /
  rightClear edge gaps, plus the row samples at 15/30/45 m). A one-post track
  still counts, it is just not a row.
  pathObstacleVisionNAP: a single vision-scored object (camera check on a radar
  trigger) with its own geometry. Agree -> camera detection at that spot.
  """
  out: list[ConeObs] = []
  if cone_msg is not None and (getattr(cone_msg, "active", False) or getattr(cone_msg, "count", 0) >= 1):
    n = int(getattr(cone_msg, "count", 0) or 0)
    row = n >= 2
    conf = float(getattr(cone_msg, "confidence", 0.0) or 0.0)
    for active_f, clear_f, sgn in (("leftActive", "leftClear", -1), ("rightActive", "rightClear", 1)):
      clr = float(getattr(cone_msg, clear_f, 0.0) or 0.0)
      if getattr(cone_msg, active_f, False) and clr > 0.0:
        out.append(ConeObs(8.0, sgn * (clr + CAR_HALF_M), RADAR, conf, cone_age_s, row))
    side = int(getattr(cone_msg, "side", 0) or 0)
    if side != 0:
      for x, f in ((15.0, "latNear"), (30.0, "latMid"), (45.0, "latFar")):
        y = float(getattr(cone_msg, f, 0.0) or 0.0)
        if y != 0.0 and math.isfinite(y):
          out.append(ConeObs(x, y, RADAR, conf, cone_age_s, row))
  if vision_msg is not None and getattr(vision_msg, "visionEvaluated", False) and getattr(vision_msg, "agree", False):
    rng = float(getattr(vision_msg, "rangeM", float("nan")))
    lat = float(getattr(vision_msg, "lateral", float("nan")))
    cls = str(getattr(vision_msg, "objectClass", "unknown"))
    if _finite(rng, lat) and 0.0 < rng <= 70.0 and cls not in ("human", "animal", "vehicle"):
      out.append(ConeObs(rng, lat, CAMERA, float(getattr(vision_msg, "visionConf", 0.7) or 0.7), vision_age_s))
  return out


# ---- CPU camera cone check (no GPU). ~2-5 ms on a 2x-downscaled road ROI. ----

def cone_mask(rgb: np.ndarray) -> np.ndarray:
  """Orange / fluorescent red-orange pixels (cones, drums, tubular posts)."""
  img = rgb.astype(np.int16)
  r, g, b = img[..., 0], img[..., 1], img[..., 2]
  return (r > 150) & (r - g > 50) & (g - b > 10) & (g > 40)


def camera_cone_candidates(rgb: np.ndarray, *, fx: float, cx: float, cy: float, cam_h: float = 1.22,
                           min_px: int = 12, max_out: int = 8) -> list[ConeObs]:
  """Find orange blobs and project their base onto a flat ground plane.

  Proposes cones on its own (not confirm-only). Column-band scan, so a row of
  posts and a lone cone are both returned as individual detections.
  """
  m = cone_mask(rgb)
  h, w = m.shape
  cols = np.flatnonzero(m.sum(axis=0) >= 3)
  out: list[ConeObs] = []
  if cols.size == 0:
    return out
  splits = np.flatnonzero(np.diff(cols) > 2) + 1
  for band in np.split(cols, splits):
    sub = m[:, band[0]:band[-1] + 1]
    if int(sub.sum()) < min_px:
      continue
    rows = np.flatnonzero(sub.any(axis=1))
    v_base = float(rows[-1])
    if v_base <= cy + 1:
      continue  # above the horizon, not on the ground
    x = fx * cam_h / (v_base - cy)
    u = 0.5 * (band[0] + band[-1])
    y = (u - cx) * x / fx
    out.append(ConeObs(float(x), float(y), CAMERA, CAMERA_CONF, 0.0))
    if len(out) >= max_out:
      break
  return out
