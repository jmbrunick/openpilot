"""NAP phantom-object bookmark. Log only: never suppresses anything.

A wouldRegenAlert (radar+vision object in the brake zone) opens an episode
for that track. The episode ends as a *phantom* when all of these hold:
  * the track passed (last range <= PASS_RANGE_M) or dropped (no message
    for DROP_S),
  * the car drove past the spot (distance since the alert >= first range +
    PASS_MARGIN_M),
  * the camera checked it at close range (>= 1 grab within CLOSE_RANGE_M)
    and never saw anything solid or lit at road level,
  * the object was not a person or animal.
Then it logs cloudlog event ``phantomObjectNAP`` (GPS lat/lon/heading from
liveMapDataNAP, wall time, range history, overhead kind tag, day/night) and
requests a regular bookmark the same way the MCU / dashboard Bookmark does
(NAPBookmarkRequest = CLOCK_MONOTONIC seconds; bookmarkd publishes
userBookmark, so the segment is flagged for upload). Anything else (camera saw
it, car stopped short, timeout) closes the episode silently.
No nightly processing and no overlay update here.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

CLOSE_RANGE_M = 60.0
PASS_RANGE_M = 15.0
PASS_MARGIN_M = 5.0
DROP_S = 2.0
TIMEOUT_S = 30.0
MAX_HISTORY = 60
PROTECTED = ("human", "animal")
BOOKMARK_PARAM = "NAPBookmarkRequest"


def _f(v, d=float("nan")) -> float:
  try:
    out = float(v)
  except (TypeError, ValueError):
    return d
  return out if math.isfinite(out) else d


@dataclass
class _Episode:
  track_id: int
  t0: float
  wall0: float
  first_range: float
  object_class: str
  lat: float
  lon: float
  heading: float
  kind: str
  daylight: bool
  last_seen_t: float
  last_range: float
  travelled: float = 0.0
  close_grabs: int = 0
  solid_close: bool = False
  ranges: list = field(default_factory=list)


class PhantomTracker:
  def __init__(self):
    self.ep: _Episode | None = None
    self.last_t: float | None = None
    self.last_vision_t: float | None = None
    self.events: list[dict] = []   # emitted, for tests

  def update(self, now: float, v_ego: float, alert_edge: bool, obs, obs_fresh: bool, md, emit=None, bookmark=None) -> dict | None:
    """Call every planner cycle. Returns the phantom event dict when one fires."""
    dt = 0.0 if self.last_t is None else max(0.0, min(1.0, now - self.last_t))
    self.last_t = now
    v = max(0.0, _f(v_ego, 0.0))
    if alert_edge and obs is not None and self.ep is None:
      self._open(now, obs, md)
    ep = self.ep
    if ep is None:
      return None
    ep.travelled += v * dt
    tid = int(getattr(obs, "trackId", -1)) if obs is not None else -1
    if obs is not None and obs_fresh and tid == ep.track_id:
      rng = _f(getattr(obs, "rangeM", None))
      if math.isfinite(rng):
        ep.last_seen_t, ep.last_range = now, rng
        if len(ep.ranges) < MAX_HISTORY:
          ep.ranges.append((round(now - ep.t0, 2), round(rng, 1)))
        if bool(getattr(obs, "visionEvaluated", False)) and rng <= CLOSE_RANGE_M:
          ep.close_grabs += 1
          if bool(getattr(obs, "roadLevelSeen", True)) or bool(getattr(obs, "roadLights", True)):
            ep.solid_close = True
        if str(getattr(obs, "objectClass", "unknown")) in PROTECTED:
          ep.object_class = str(obs.objectClass)
    gone = (now - ep.last_seen_t) >= DROP_S or ep.last_range <= PASS_RANGE_M
    passed = ep.travelled >= ep.first_range + PASS_MARGIN_M
    if gone and passed:
      self.ep = None
      if ep.solid_close or ep.close_grabs == 0 or ep.object_class in PROTECTED:
        return None
      return self._fire(ep, emit, bookmark)
    if now - ep.t0 > TIMEOUT_S or (v < 0.5 and ep.travelled < ep.first_range):
      self.ep = None   # stopped short or took too long: no verdict
    return None

  def _open(self, now: float, obs, md) -> None:
    rng = _f(getattr(obs, "rangeM", None))
    if not math.isfinite(rng):
      return
    kind = ""
    if md is not None:
      ahead = _f(getattr(md, "overheadAheadM", 0.0), 0.0)
      k = str(getattr(md, "overheadKind", "") or "")
      if k and ahead != 0.0 and abs(ahead - rng) <= 35.0:
        kind = k
    self.ep = _Episode(
      track_id=int(getattr(obs, "trackId", -1)), t0=now, wall0=time.time(),  # noqa: TID251
      first_range=rng, object_class=str(getattr(obs, "objectClass", "unknown")),
      lat=_f(getattr(md, "latitude", None)) if md is not None else float("nan"),
      lon=_f(getattr(md, "longitude", None)) if md is not None else float("nan"),
      heading=_f(getattr(md, "bearingDeg", None)) if md is not None else float("nan"),
      kind=kind, daylight=bool(getattr(obs, "daylight", False)), last_seen_t=now, last_range=rng,
      ranges=[(0.0, round(rng, 1))])

  def _fire(self, ep: _Episode, emit, bookmark) -> dict:
    ev = dict(lat=ep.lat, lon=ep.lon, heading_deg=ep.heading, unix_time=round(ep.wall0, 1),
              first_range_m=round(ep.first_range, 1), ranges=ep.ranges, close_grabs=ep.close_grabs,
              overhead_kind=ep.kind, daylight=ep.daylight, object_class=ep.object_class, track_id=ep.track_id)
    self.events.append(ev)
    try:
      if emit is not None:
        emit("phantomObjectNAP", ev)
      if bookmark is not None:
        bookmark()
    except Exception:
      pass
    return ev


def cloudlog_emit(name: str, ev: dict) -> None:
  from openpilot.common.swaglog import cloudlog
  cloudlog.event(name, **ev)


def request_bookmark(params) -> None:
  """Same request the MCU / dashboard Bookmark makes; bookmarkd publishes userBookmark."""
  params.put(BOOKMARK_PARAM, int(time.monotonic()))
