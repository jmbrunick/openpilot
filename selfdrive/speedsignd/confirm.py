"""Plate / prompt state. Display only; nothing here brakes or limits.

The SIGN plate shows the newest read: an agreed read replaces the plate mph
at once and raises the Yes/No prompt for it, with the mapd way and heading
matched when the sign was READ. Only the LIMIT/MAX map override waits for
Yes (mapd stores it for that way, see mapd/speed_overrides.py). No and a
timeout change nothing in the map; the plate keeps the read. A read equal
to the current limit still prompts so Yes records the sign; mapd skips
rewriting an override that already matches. A failed read
leaves the plate as it was. The UI answers through NAPSpeedSignAnswer.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass

ANSWER_PARAM = "NAPSpeedSignAnswer"
ANSWER_POLL_S = 0.25
# Oct 22: the 15:47 30 prompt stayed up for the rest of the drive. An
# unanswered prompt now expires (No-equivalent: no map change) after this
# long, on a mapd way change, or once the car is this far from the read.
PROPOSAL_MAX_S = 60.0
PROPOSAL_MAX_M = 1000.0
# OSM ways split at many town intersections; give the driver this long to tap
# before a way change alone expires the prompt.
WAY_CHANGE_MIN_S = 15.0


def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  k = 111320.0
  dy = (lat2 - lat1) * k
  dx = (lon2 - lon1) * k * math.cos(math.radians((lat1 + lat2) / 2.0))
  return math.hypot(dx, dy)


@dataclass
class Proposal:
  mph: int
  conf: float
  seq: int
  way_id: int = 0
  heading: float = 0.0
  lat: float = 0.0
  lon: float = 0.0
  read_id: object = None   # the one read this prompt (mph, way, heading, location) came from
  t: float = 0.0           # monotonic time the prompt opened


class PlateState:
  def __init__(self) -> None:
    self.plate_mph = 0
    self.plate_conf = 0.0
    self.confirmed_mph = 0      # last Yes (map override); the plate may differ
    self.proposal: Proposal | None = None
    self.seq = 0
    self.answered_read_id = None
    self.expired_why = ""

  def propose(self, mph: int, conf: float, *, way_id: int = 0, heading: float | None = None,
              lat: float | None = None, lon: float | None = None, read_id=None,
              now: float | None = None) -> Proposal | None:
    """Open the prompt for one read. mph, way, heading and location always come
    from the same read: a newer read replaces the prompt and its location
    together (new seq), so an answer to the old seq is rejected. Only a repeat
    of the same read_id keeps the open prompt."""
    mph = int(mph)
    if mph <= 0:
      return None
    self.plate_mph, self.plate_conf = mph, float(conf)   # the plate follows the read
    if read_id is not None:
      if self.proposal is not None and self.proposal.read_id == read_id:
        return self.proposal
      if read_id == self.answered_read_id:
        return None       # this read was already answered; do not ask again
    self.seq += 1
    self.proposal = Proposal(mph, float(conf), self.seq, int(way_id or 0), float(heading or 0.0),
                             float(lat or 0.0), float(lon or 0.0), read_id,
                             time.monotonic() if now is None else float(now))
    return self.proposal

  def expire(self, now: float, way_id: int = 0, lat: float | None = None, lon: float | None = None) -> bool:
    """Drop an unanswered prompt: too old, on another way, or > 1 km from the read. True if dropped.

    Like a No: the map is untouched, the plate keeps the read, and the same
    read is not asked again."""
    p = self.proposal
    if p is None:
      return False
    why = ""
    if now - p.t > PROPOSAL_MAX_S:
      why = "age"
    elif p.way_id and way_id and int(way_id) != p.way_id and now - p.t >= WAY_CHANGE_MIN_S:
      why = "way"
    elif (p.lat or p.lon) and lat is not None and lon is not None and (lat or lon) and \
        _dist_m(p.lat, p.lon, float(lat), float(lon)) > PROPOSAL_MAX_M:
      why = "dist"
    if not why:
      return False
    self.answered_read_id = p.read_id
    self.proposal = None
    self.expired_why = why
    return True

  def answer(self, raw) -> bool:
    """Apply a UI answer {seq, mph, yes}. True when it matched the open prompt."""
    try:
      d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
      seq, mph, yes = int(d["seq"]), int(d["mph"]), bool(d["yes"])
    except Exception:
      return False
    p = self.proposal
    if p is None or seq != p.seq or mph != p.mph:
      return False
    if yes:
      self.confirmed_mph = p.mph
    self.answered_read_id = p.read_id
    self.proposal = None
    return True


def map_way_from_sm(sm) -> tuple[int, float | None]:
  """(wayId, bearing) mapd is matched to now; (0, None) if unknown."""
  try:
    if not sm.valid.get("liveMapDataNAP", False):
      return 0, None
    m = sm["liveMapDataNAP"]
    return int(m.wayId), float(m.bearingDeg)
  except Exception:
    return 0, None


def fill_proposal(dst, p: Proposal | None) -> None:
  if p is None:
    dst.proposedMph = 0
    return
  dst.proposedMph = int(p.mph)
  dst.proposedConf = float(p.conf)
  dst.proposedSeq = int(p.seq)
  dst.proposedWayId = int(p.way_id)
  dst.proposedHeading = float(p.heading)
  dst.proposedLat = float(p.lat)
  dst.proposedLon = float(p.lon)


def write_answer(seq: int, mph: int, yes: bool, params=None) -> bool:
  """UI side: hand a Yes/No on prompt `seq` to speedsignd."""
  try:
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    # JSON-typed param: Params.put needs the dict. A json.dumps() string raised
    # TypeError here, so speedsignd never saw any Yes/No (Oct 22 stuck 30).
    params.put(ANSWER_PARAM, {"seq": int(seq), "mph": int(mph), "yes": bool(yes)})
    return True
  except Exception:
    return False
