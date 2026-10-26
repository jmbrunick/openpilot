"""Short speed-limit segments are written out (Justin, Oct 26 2026).

A posted-limit segment shorter than SHORT_SEG_M (100 yd, ~91 m) whose jump from the
surrounding limit is MORE than SHORT_JUMP_MPH (15 mph), up or down, is not published: the
surrounding limit holds across it (Justin, Oct 26: "an increase or decrease that is under
100 yards of over 15 mph is what is ignored"). Example: a 60 read for ~50 m on a 30 street
(DeGraff, Clarkfield). Jumps of 15 mph or less always apply normally.

Applied to the final resolved values mapd publishes (sign sections > Tesla-learned >
Tesla > OSM; temp limits still beat it), both the current limit and the lookahead.

Causality. A segment's length is only known ahead of time from map geometry:
  * geometric sources (sign sections, learned sections, OSM -- anything the lookahead
    profile can see): at the change point the profile gives the distance to the next
    change, so a short segment is recognized the moment it starts and held through, rises
    and drops alike (geometry-backed, both ends known);
  * live Tesla (no lookahead): a rise of more than 15 mph is held until it has lasted SHORT_SEG_M
    of travel (delaying a rise is safe). A DROP is applied at once -- never delayed on a
    guess -- and if the previous limit comes back within SHORT_SEG_M the drop is ended at
    once (no rise hold): the sliver is cut short, not prevented.
"""
from __future__ import annotations

from openpilot.selfdrive.mapd import speed_overrides as so

SHORT_SEG_M = 91.44            # 100 yd
SHORT_JUMP_MPH = 15.0
SHORT_JUMP_MS = SHORT_JUMP_MPH * so.MPH_TO_MS + 0.05
EPS_MS = 0.3
PROFILE_MAX_M = 400.0
NEXT_MATCH_M = 40.0            # nxt (lead-adjusted) vs profile change distance
GEOMETRIC = (None, "sign", "tesla_learned", "osm", "osm_default", "osm_fallback")


def limit_profile(entries, db, way_id: int, coords, lat: float, lon: float, bearing: float | None,
                  learned=None, max_m: float = PROFILE_MAX_M) -> tuple[list[tuple[float, float]], float]:
  """([(start_m, ms), ...] change points from here, first at 0), walked meters. Same priority as the
  lookahead: confirmed section > learned section > OSM way maxspeed. ([], 0) when unknown."""
  if bearing is None or db is None or not coords:
    return [], 0.0
  try:
    chain_ways = {int(sp[0]) for e in entries for sp in (e.get("chain") or ())}
    pieces = so.route_ahead(db, int(way_id), coords, lat, lon, float(bearing), max_m=max_m, chain_ways=chain_ways)
  except Exception:
    return [], 0.0
  has_chain = bool(chain_ways)
  out: list[tuple[float, float]] = []
  walked = 0.0
  for w, s_from, s_to, d, dist0 in pieces:
    osm_ms = float(w.get("maxspeed_ms") or 0.0)
    n = int(abs(s_to - s_from) // so.NEXT_STEP_M)
    for k in range(n + 1):
      s = s_from + d * k * so.NEXT_STEP_M
      e = so.chain_at(entries, int(w["way_id"]), s) if has_chain else None
      if e is not None:
        ms = int(e["mph"]) * so.MPH_TO_MS
      else:
        lm = learned(int(w["way_id"]), s) if learned is not None else None
        ms = float(lm) if lm is not None else osm_ms
      dist = dist0 + k * so.NEXT_STEP_M
      if ms <= 0.0:
        return out, walked    # unknown limit ahead: stop the profile
      walked = dist
      if not out or abs(ms - out[-1][1]) > EPS_MS:
        out.append((dist, ms))
  return out, walked


def _filtered_changes(held_ms: float, prof, walked: float) -> list[tuple[float, float]]:
  """Change points after short > 15 mph segments are folded into the held limit around them."""
  out, held = [], float(held_ms)
  for i in range(1, len(prof)):
    start, ms = prof[i]
    end = prof[i + 1][0] if i + 1 < len(prof) else None   # None: runs past the walk (long)
    if end is not None and end - start < SHORT_SEG_M and abs(ms - held) > SHORT_JUMP_MS:
      continue
    if abs(ms - held) > EPS_MS:
      out.append((start, ms))
      held = ms
  return out


def filter_next(held_ms: float, nxt, prof, walked: float):
  """Lookahead with short segments written out. Unchanged unless the profile sees nxt's change."""
  if nxt is None or len(prof) < 2:
    return nxt
  c1 = prof[1]
  if abs(c1[1] - float(nxt[0])) > EPS_MS or abs(c1[0] - float(nxt[1])) > NEXT_MATCH_M + so.NEXT_STEP_M:
    return nxt                 # the lookahead is not the profile's first change: no geometry, leave it
  off = float(nxt[1]) - c1[0]  # keep the caller's GNSS lead
  ch = _filtered_changes(held_ms, prof, walked)
  if not ch:
    return None
  return ch[0][1], max(0.0, ch[0][0] + off)


class ShortSegmentHold:
  """Causal current-limit filter (see module doc)."""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.pub: float | None = None
    self.raw_prev: float | None = None
    self.odo_change = 0.0
    self.drop_from: float | None = None   # limit a live (non-geometric) drop came from
    self.odo_drop = 0.0
    self.held = False

  def update(self, raw_ms: float, src: str | None, odo_m: float, prof=None, walked: float = 0.0) -> float:
    raw = float(raw_ms)
    if self.raw_prev is None or abs(raw - self.raw_prev) > EPS_MS:
      self.odo_change = odo_m
    self.raw_prev = raw
    self.held = False
    if self.pub is not None and abs(raw - self.pub) <= EPS_MS:
      return self.pub
    if self.pub is None or abs(raw - self.pub) <= SHORT_JUMP_MS:
      self._adopt(raw, odo_m)
      return raw
    run = odo_m - self.odo_change
    geo = src in GEOMETRIC and prof and abs(prof[0][1] - raw) <= EPS_MS
    if geo:
      rem = prof[1][0] if len(prof) > 1 else None   # None: no change within the walk -> long
      if rem is not None and run + rem < SHORT_SEG_M:
        self.held = True
        return self.pub
      self._adopt(raw, odo_m)
      return raw
    # no geometry (live Tesla)
    if raw < self.pub:
      self._adopt(raw, odo_m, drop=True)
      return raw
    if self.drop_from is not None and abs(raw - self.drop_from) <= EPS_MS and odo_m - self.odo_drop < SHORT_SEG_M:
      self._adopt(raw, odo_m)        # the drop was a sliver: end it now
      return raw
    if run < SHORT_SEG_M:
      self.held = True
      return self.pub                # a rise must last 100 yd
    self._adopt(raw, odo_m)
    return raw

  def _adopt(self, raw: float, odo_m: float, drop: bool = False) -> None:
    if drop and self.pub is not None:
      self.drop_from, self.odo_drop = self.pub, odo_m
    elif not drop:
      self.drop_from = None
    self.pub = raw
