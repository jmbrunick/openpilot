"""Camera tracks and radar matching. Camera first: radar only refines range/vRel.

A detection is matched to a radar point when its bearing is within ±1.5 deg
and its camera ground-plane range is within ±25 % of the radar range. With no
radar match the camera track counts as confirmed after 3 hits in a row (one
missed cycle allowed), since animals are weak, flickery radar targets.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from openpilot.selfdrive.nap.animal_detect.config import (
  ANIMAL_CLASSES, CAMERA_CONFIRM_FRAMES, MATCH_BEARING_DEG, MATCH_RANGE_FRAC, RADAR_TO_CAMERA,
  TRACK_MATCH_IOU, TRACK_MAX_MISSES,
)
from openpilot.selfdrive.nap.animal_detect.postprocess import iou


@dataclass(frozen=True)
class RadarPoint:
  track_id: int
  d_rel: float   # m from the front bumper
  y_rel: float   # m, + left (openpilot radar convention)
  v_rel: float   # m/s

  @property
  def range_cam(self) -> float:
    return self.d_rel + RADAR_TO_CAMERA

  @property
  def bearing_deg(self) -> float:
    """+ right, same sign as the camera bearing."""
    return math.degrees(math.atan2(-self.y_rel, self.range_cam))


@dataclass
class Detection:
  cls: str
  conf: float
  camera: str
  box: tuple[float, float, float, float]   # source pixels of its camera
  bearing_deg: float
  range_cam: float                          # flat-road range, NaN above the horizon
  track_id: int = 0
  frames: int = 1
  confirmed: bool = False
  radar_track_id: int = 0
  radar_matched: bool = False
  range_m: float = float("nan")
  v_rel: float = float("nan")
  lateral_m: float = float("nan")

  @property
  def is_animal(self) -> bool:
    return self.cls in ANIMAL_CLASSES


def match_radar(det: Detection, points: list[RadarPoint]) -> RadarPoint | None:
  """Best radar point inside both gates, or None. NaN camera range never matches."""
  if not math.isfinite(det.range_cam) or det.range_cam <= 0.0:
    return None
  best, best_cost = None, float("inf")
  for p in points:
    if not (math.isfinite(p.d_rel) and math.isfinite(p.y_rel)):
      continue
    db = abs(p.bearing_deg - det.bearing_deg)
    dr = abs(p.range_cam - det.range_cam) / det.range_cam
    if db > MATCH_BEARING_DEG or dr > MATCH_RANGE_FRAC:
      continue
    cost = db / MATCH_BEARING_DEG + dr / MATCH_RANGE_FRAC
    if cost < best_cost:
      best, best_cost = p, cost
  return best


@dataclass
class _Track:
  tid: int
  camera: str
  cls: str
  box: tuple[float, float, float, float]
  hits: int = 1
  misses: int = 0


@dataclass
class Tracker:
  """Greedy IoU association per camera. Person and animal tracks are kept apart."""
  tracks: list[_Track] = field(default_factory=list)
  next_id: int = 1

  def reset(self) -> None:
    self.tracks.clear()

  def update(self, dets: list[Detection]) -> None:
    used: set[int] = set()
    for d in sorted(dets, key=lambda d: -d.conf):
      best, best_iou = None, TRACK_MATCH_IOU
      for t in self.tracks:
        if t.tid in used or t.camera != d.camera or (t.cls == "person") != (d.cls == "person"):
          continue
        o = iou(t.box, d.box)
        if o >= best_iou:
          best, best_iou = t, o
      if best is None:
        best = _Track(self.next_id, d.camera, d.cls, d.box, hits=0)
        self.next_id = self.next_id % 0xFFFFFFFF + 1
        self.tracks.append(best)
      best.hits += 1
      best.misses = 0
      best.box, best.cls = d.box, d.cls
      used.add(best.tid)
      d.track_id, d.frames = best.tid, best.hits
    for t in self.tracks:
      if t.tid not in used:
        t.misses += 1
    self.tracks = [t for t in self.tracks if t.misses <= TRACK_MAX_MISSES]


def fuse(dets: list[Detection], points: list[RadarPoint], tracker: Tracker) -> list[Detection]:
  tracker.update(dets)
  for d in dets:
    p = match_radar(d, points)
    if p is not None:
      d.radar_matched, d.radar_track_id = True, int(p.track_id)
      d.range_m, d.v_rel = p.range_cam, p.v_rel
    else:
      d.range_m = d.range_cam
    if math.isfinite(d.range_m):
      d.lateral_m = d.range_m * math.tan(math.radians(d.bearing_deg))
    d.confirmed = d.radar_matched or d.frames >= CAMERA_CONFIRM_FRAMES
  return dets
