"""Plate / prompt state. Display only; nothing here brakes or limits.

The SIGN plate shows a read only while its Yes/No prompt is open: an agreed
read puts its mph on the plate at once with the prompt (mapd way, heading
and location matched when the sign was READ). Any answer (comma or MCU) or
the PROMPT_TIMEOUT_S timeout clears both the prompt and the read plate.
Yes also hands the mph to mapd (NAPSpeedSignConfirmed), which makes it the
LIMIT for that way (both directions, from the sign on). Temp hands the mph to
mapd as a this-drive-only limit (NAPSpeedSignTemp, mapd/temp_limit.py) and is
never saved. No and timeouts change nothing in the map. The UI answers through
NAPSpeedSignAnswer.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass

ANSWER_PARAM = "NAPSpeedSignAnswer"
ANSWER_POLL_S = 0.25
PROMPT_TIMEOUT_S = 20.0   # unanswered prompt + read plate clear after this


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
    self.temp_mph = 0           # last Temp (this drive only); never a map override
    self.proposal: Proposal | None = None
    self.seq = 0
    self.answered_read_id = None

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
    if read_id is not None:
      if self.proposal is not None and self.proposal.read_id == read_id:
        return self.proposal
      if read_id == self.answered_read_id:
        return None       # this read was already answered; do not ask again
    self.plate_mph, self.plate_conf = mph, float(conf)   # the plate shows the read with its prompt
    self.seq += 1
    self.proposal = Proposal(mph, float(conf), self.seq, int(way_id or 0), float(heading or 0.0),
                             float(lat or 0.0), float(lon or 0.0), read_id,
                             time.monotonic() if now is None else float(now))
    return self.proposal

  def answer(self, raw) -> bool:
    """Apply a UI answer {seq, mph, yes[, answer]}. True when it matched the open prompt."""
    try:
      d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else dict(raw)
      seq, mph, yes = int(d["seq"]), int(d["mph"]), bool(d["yes"])
      kind = answer_kind(d.get("answer", yes))
    except Exception:
      return False
    p = self.proposal
    if p is None or seq != p.seq or mph != p.mph:
      return False
    if kind == "yes":
      self.confirmed_mph = p.mph
    elif kind == "temp":
      self.temp_mph = p.mph
    self._close(p)
    return True

  def expire(self, now: float | None = None) -> bool:
    """Timeout: clear an unanswered prompt and its read plate. True when one closed."""
    p = self.proposal
    now = time.monotonic() if now is None else float(now)
    if p is None or now - p.t < PROMPT_TIMEOUT_S:
      return False
    self._close(p)
    return True

  def _close(self, p: Proposal) -> None:
    self.answered_read_id = p.read_id
    self.proposal = None
    self.plate_mph, self.plate_conf = 0, 0.0


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


ANSWERS = ("yes", "no", "temp")


def answer_kind(v) -> str:
  """'yes' / 'no' / 'temp' from a bool or a label. Anything else raises ValueError."""
  if isinstance(v, bool):
    return "yes" if v else "no"
  s = str(v).strip().lower()
  if s in ANSWERS:
    return s
  raise ValueError(f"unknown answer {v!r}")


def write_answer(seq: int, mph: int, yes, params=None) -> bool:
  """UI side: hand a Yes/No/Temp on prompt `seq` to speedsignd. `yes` is a bool or 'temp'."""
  try:
    kind = answer_kind(yes)
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    params.put(ANSWER_PARAM, json.dumps({"seq": int(seq), "mph": int(mph), "yes": kind == "yes", "answer": kind}))
    return True
  except Exception:
    return False
