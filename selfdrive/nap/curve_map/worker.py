"""Background map curve preview for plannerd (NAP, longitudinal only).

The planner hands in the latest matched position (liveMapDataNAP) every
cycle; a daemon thread runs the sqlite walk at most once per PERIOD_S and
hands back the result. The control loop never waits on sqlite: submit()
and poll() only swap references.
"""
from __future__ import annotations

import os
import threading
import time

from openpilot.selfdrive.nap.curve_map.preview import MapCurvePreview

TILE_DIR = "/data/media/0/osm/nap_curves"
PERIOD_S = 1.0
HORIZON_S = 14.0
HORIZON_MIN_M = 150.0
HORIZON_PAD_M = 50.0


def horizon_m(v: float) -> float:
  return max(HORIZON_MIN_M, HORIZON_S * max(5.0, float(v)) + HORIZON_PAD_M)


class CurveMapWorker:
  def __init__(self, tile_dir: str = TILE_DIR, start: bool = True) -> None:
    self.available = os.path.isdir(tile_dir)
    self._preview = MapCurvePreview(tile_dir)
    self._req = None
    self._res = None
    self._lock = threading.Lock()
    self._wake = threading.Event()
    self.faults = 0
    if start and self.available:
      threading.Thread(target=self._run, name="nap_curve_map", daemon=True).start()

  def submit(self, lat: float, lon: float, bearing_deg: float, v: float) -> None:
    with self._lock:
      self._req = (float(lat), float(lon), float(bearing_deg), float(v))
    self._wake.set()

  def poll(self):
    """(preview or None,) once per finished query, else None."""
    with self._lock:
      r, self._res = self._res, None
    return r

  def run_once(self, req):
    lat, lon, brg, v = req
    try:
      return (self._preview.query(lat, lon, brg, horizon_m(v)),)
    except Exception:
      self.faults += 1
      return (None,)

  def _run(self) -> None:
    try:
      os.nice(10)
    except OSError:
      pass
    while True:
      self._wake.wait()
      t0 = time.monotonic()
      with self._lock:
        req, self._req = self._req, None
        self._wake.clear()
      if req is not None:
        res = self.run_once(req)
        with self._lock:
          self._res = res
      time.sleep(max(0.0, PERIOD_S - (time.monotonic() - t0)))
