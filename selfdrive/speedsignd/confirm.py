"""v2.3 plate / prompt state. Display only; nothing here brakes or limits.

The plate shows the last speed the driver confirmed with Yes. An agreed
read never changes it: it raises the Yes/No prompt with the proposed mph,
plus the mapd way and heading matched when the sign was READ. Yes moves the
proposal to the plate (and mapd stores it for that way, see
mapd/speed_overrides.py). No, a failed read or a wrong read leave the old
plate up. The UI answers through the NAPSpeedSignAnswer param.
"""
from __future__ import annotations

import json
from dataclasses import dataclass

ANSWER_PARAM = "NAPSpeedSignAnswer"
ANSWER_POLL_S = 0.25


@dataclass
class Proposal:
  mph: int
  conf: float
  seq: int
  way_id: int = 0
  heading: float = 0.0
  lat: float = 0.0
  lon: float = 0.0


class PlateState:
  def __init__(self) -> None:
    self.plate_mph = 0
    self.plate_conf = 0.0
    self.proposal: Proposal | None = None
    self.seq = 0

  def propose(self, mph: int, conf: float, *, way_id: int = 0, heading: float | None = None,
              lat: float | None = None, lon: float | None = None) -> Proposal | None:
    mph = int(mph)
    if mph <= 0 or mph == self.plate_mph:
      return None
    if self.proposal is not None and self.proposal.mph == mph:
      return self.proposal
    self.seq += 1
    self.proposal = Proposal(mph, float(conf), self.seq, int(way_id or 0), float(heading or 0.0),
                             float(lat or 0.0), float(lon or 0.0))
    return self.proposal

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
      self.plate_mph, self.plate_conf = p.mph, p.conf
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
    params.put(ANSWER_PARAM, json.dumps({"seq": int(seq), "mph": int(mph), "yes": bool(yes)}))
    return True
  except Exception:
    return False
