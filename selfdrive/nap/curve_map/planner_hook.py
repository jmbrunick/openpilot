"""plannerd glue for the map curve preview (NAP, longitudinal only).

Reads the position mapd already publishes (liveMapDataNAP: no new
subscriber), feeds the background worker, and keeps the MapFeed that curve
follow merges. Without a curve pack on the device it does nothing.
"""
from __future__ import annotations

from openpilot.selfdrive.nap.curve_map.blend import MapFeed
from openpilot.selfdrive.nap.curve_map.worker import CurveMapWorker


class CurveMapSource:
  def __init__(self, worker: CurveMapWorker | None = None) -> None:
    self.worker = worker if worker is not None else CurveMapWorker()
    self.feed = MapFeed()
    self.mapd_way = 0
    self.mapd_match_m = 0.0

  def update(self, sm, v_ego: float):
    """Returns (feed or None, mapd way id, mapd match distance)."""
    if not self.worker.available:
      return None, 0, 0.0
    md = sm['liveMapDataNAP']
    if sm.valid.get('liveMapDataNAP', False) and md.dbLoaded and (md.latitude != 0.0 or md.longitude != 0.0):
      self.worker.submit(md.latitude, md.longitude, md.bearingDeg, v_ego)
      self.mapd_way, self.mapd_match_m = int(md.wayId), float(md.matchDistance)
    else:
      self.mapd_way, self.mapd_match_m = 0, 1e9  # fails the gate
    r = self.worker.poll()
    if r is not None:
      self.feed.set(r[0])
    return self.feed, self.mapd_way, self.mapd_match_m
