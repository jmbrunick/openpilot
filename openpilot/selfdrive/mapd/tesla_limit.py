"""Tesla Pre-AP map speed limit from the chassis CAN (NAPTeslaLimitEnable).

Two IC messages on bus 0 (opendbc tesla_preap.dbc):
  0x238 UI_driverAssistRoadSign, mux UI_roadSign == 3: UI_baseMapSpeedLimitMPS
        (0.25 m/s steps, 0 = unknown) + UI_splineLocConfidence. ~2.5 Hz.
        Changes ~40 m BEFORE the posted sign. This is the only value that
        can confirm a sign read.
  0x2F8 UI_gpsVehicleSpeed: UI_mppSpeedLimit (5 mph steps), ~1 Hz. Lags the
        sign by ~40 m and was wrong more often (Oct 19-22 study). UI_mapSpeedLimitUnits
        was always 0 on this US car (mph); 1 is treated as kph. Display /
        unknown fallback only; it NEVER confirms.

NAPTeslaLimitEnable alone is log-only (liveMapDataNAP tesla* fields + the
both-maps review notes). NAPTeslaLimitPrimary (default on) runs the same CAN
read by itself and changes what mapd publishes:
  current limit (speedLimit, MAX = limit + offset):
      confirmed sign > active Tesla-learned section (tesla_learned.py) >
      Tesla base (debounced, held through short dropouts) > OSM.
      No cap: wherever Tesla has a value it beats OSM, higher or lower.
      A learned section (Tesla seen disagreeing with OSM on 2 separate
      passes) beats the live value, so it covers the spot from the first
      meter and through Tesla dropouts; a pass where live Tesla reads a
      different value replaces the row (inactive until 2 passes again).
  lookahead (nextSpeedLimit, early slowdown):
      confirmed sign > active Tesla-learned section > OSM. The live Tesla value
      has no lookahead, so the walk starts from the sign / learned / OSM value
      and a next equal to the published limit is dropped (next_for_published):
      a Tesla-vs-OSM mismatch is never an "upcoming change".
  liveMapDataNAP.source: "sign" | "tesla" | "tesla_learned" | "osm" | "osm_default".
Auto-confirm still compares a read with raw OSM (osmRawMph) and the plain
debounced Tesla value (teslaBaseMph / teslaValid), never the published value,
and learned sections never count as confirmations. MPP is never used for a limit.
The learned layer and its review notes ride on NAPTeslaLimitPrimary (no
separate param): learning only makes sense while Tesla is the primary source.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

ADDR_ROAD_SIGN = 0x238
ADDR_GPS_SPEED = 0x2F8
ROAD_SIGN_BASE_MUX = 3
CAN_BUS = 0
MS_TO_MPH = 2.23694
KPH_TO_MPH = 0.621371

HOLD_S = 3.0          # base value must stay the same this long before it is published
MIN_CONF = 90         # UI_splineLocConfidence floor (99 is normal)
BASE_STALE_S = 2.0    # mux 3 arrives at 2.5 Hz; older than this = unknown
MPP_STALE_S = 3.0     # 1 Hz


def round5(mph: float) -> int:
  return int(5 * round(mph / 5.0))


def base_mph(base_mps: float) -> int:
  """UI_baseMapSpeedLimitMPS -> posted mph (nearest 5). 0 = unknown."""
  if not math.isfinite(base_mps) or base_mps <= 0.0:
    return 0
  mph = round5(base_mps * MS_TO_MPH)
  return mph if 5 <= mph <= 85 else 0


def mpp_mph(raw: float, units_kph: bool = False) -> int:
  if not math.isfinite(raw) or raw <= 0.0:
    return 0
  mph = round5(raw * KPH_TO_MPH) if units_kph else int(raw)
  return mph if 5 <= mph <= 85 else 0


@dataclass
class TeslaLimitState:
  base_mph: int = 0      # debounced base (confirm-grade)
  mpp_mph: int = 0       # latest fresh MPP (display only)
  conf: int = 0          # latest spline loc confidence
  valid: bool = False    # base_mph is debounced + fresh


class TeslaLimitFusion:
  """Debounce the base limit; keep MPP as a display-only side value.

  Rules: 0 = unknown (never published as a limit), a new value must hold
  HOLD_S with every sample at conf >= MIN_CONF, stale input -> invalid.
  """

  def __init__(self, hold_s: float = HOLD_S, min_conf: int = MIN_CONF) -> None:
    self.hold_s = hold_s
    self.min_conf = min_conf
    self.cand: int | None = None
    self.cand_t = 0.0
    self.stable = 0
    self.last_base_t: float | None = None
    self.last_mpp_t: float | None = None
    self.mpp = 0
    self.conf = 0

  def add_base(self, t: float, mph: int, conf: int) -> None:
    self.last_base_t = t
    self.conf = int(conf)
    if mph <= 0 or conf < self.min_conf:
      # unknown or low confidence breaks the hold and drops the published value
      self.cand, self.stable = None, 0
      return
    if mph != self.cand:
      self.cand, self.cand_t = mph, t
    if t - self.cand_t >= self.hold_s:
      self.stable = mph

  def add_mpp(self, t: float, mph: int) -> None:
    self.last_mpp_t = t
    self.mpp = mph

  def state(self, now: float) -> TeslaLimitState:
    base_fresh = self.last_base_t is not None and now - self.last_base_t <= BASE_STALE_S
    mpp_fresh = self.last_mpp_t is not None and now - self.last_mpp_t <= MPP_STALE_S
    valid = base_fresh and self.stable > 0
    return TeslaLimitState(
      base_mph=self.stable if valid else 0,
      mpp_mph=self.mpp if mpp_fresh else 0,
      conf=self.conf if base_fresh else 0,
      valid=valid,
    )


class TeslaCanReader:
  """CANParser for the two IC messages; feeds a TeslaLimitFusion.

  update() takes raw `can` events (capnp, as from messaging.drain_sock or a
  log), keeps only bus-0 frames for the two addresses (cheap prefilter),
  then lets CANParser checksum/counter-check and decode them. mux is not
  applied by CANParser, so every 0x238 frame is checked for UI_roadSign == 3.
  """

  def __init__(self, fusion: TeslaLimitFusion | None = None, bus: int = CAN_BUS) -> None:
    from opendbc.can.parser import CANParser
    self.bus = bus
    self.fusion = fusion or TeslaLimitFusion()
    nan = float("nan")   # no frequency check: the IC may stop sending in park
    self.cp = CANParser("tesla_preap", [("UI_driverAssistRoadSign", nan), ("UI_gpsVehicleSpeed", nan)], bus)
    self._addrs = (ADDR_ROAD_SIGN, ADDR_GPS_SPEED)

  def update(self, can_events) -> None:
    bus, addrs = self.bus, self._addrs
    batch = []
    for ev in can_events:
      frames = [(c.address, c.dat, c.src) for c in ev.can if c.src == bus and c.address in addrs]
      if frames:
        batch.append((ev.logMonoTime, frames))
    for item in batch:
      self.cp.update([item])
      t = item[0] * 1e-9
      rs = self.cp.vl_all["UI_driverAssistRoadSign"]
      for mux, mps, conf in zip(rs["UI_roadSign"], rs["UI_baseMapSpeedLimitMPS"], rs["UI_splineLocConfidence"], strict=False):
        if int(mux) == ROAD_SIGN_BASE_MUX:
          self.fusion.add_base(t, base_mph(float(mps)), int(conf))
      gs = self.cp.vl_all["UI_gpsVehicleSpeed"]
      for raw, units in zip(gs["UI_mppSpeedLimit"], gs["UI_mapSpeedLimitUnits"], strict=False):
        self.fusion.add_mpp(t, mpp_mph(float(raw), units_kph=bool(units)))


PARAM_ENABLE = "NAPTeslaLimitEnable"
FLAG_PERIOD_S = 1.0

# NAPTeslaLimitPrimary (default on): the Tesla base limit is the CURRENT limit
# mapd publishes when no confirmed sign covers the spot. It needs the CAN
# read, so mapd runs the reader while either param is on; review notes stay
# with NAPTeslaLimitEnable only.
PARAM_PRIMARY = "NAPTeslaLimitPrimary"
SOURCE_TESLA = "tesla"
SOURCE_LEARNED = "tesla_learned"   # active Tesla-learned section (tesla_learned.py); beats live Tesla
MPH_TO_MS = 0.44704
# Keep the last debounced base value through a short dropout (0 / low conf /
# a missed frame) on the same road, so a gap does not flip the limit to OSM
# and back. A sub-HOLD_S 0->60->0 flash never debounces, so it never shows.
# Real car (rlogs Oct 9-22, Oct 23 route 000001c2 qlogs): mux 3 drops to 0 for
# one frame (~0.4-1 s) about every 15 s while driving, which the 3 s debounce
# turned into a 3-4 s hole every ~15-20 s; a few gaps run 2-6 s. 6 s covers
# them; the same value coming back is bridged through its 3 s re-debounce.
PRIMARY_HOLD_S = 6.0
# A dropout that lands on a way-name change (US 12 -> Atlantic Ave in town) only
# gets a short grace: enough for the usual 1-frame gap, not a carried limit.
ROAD_CHANGE_HOLD_S = 3.0
SEEN_GRACE_S = 1.0    # a Tesla value seen this recently still counts through one 0 frame


class TeslaPrimary:
  """Current limit from the Tesla base map for NAPTeslaLimitPrimary.

  Priority (mapd): confirmed sign override > active learned section > this
  value > raw OSM.
  Returns mph (0 = no usable Tesla value, use OSM). Never reads MPP.
    * debounced base (TeslaLimitFusion.valid): use it, remember it + the road
    * base gone (0, conf < 90, stale): keep the last value up to hold_s, same road only
    * the same value coming back inside the hold: keep it through its 3 s
      re-debounce (no OSM flip while the fusion re-holds it), also across a
      map-match road-name flicker
    * a different value debouncing: keep the old one until the new one is
      published (up to hold_s + HOLD_S), so old -> new with no OSM blip
    * road changed (name / class) and Tesla has nothing: ROAD_CHANGE_HOLD_S
      grace, then 0 -> OSM; same road: hold_s, then 0 -> OSM
  teslaBaseMph / teslaValid in liveMapDataNAP stay the plain debounced
  values; auto-confirm and review notes never see the held value.
  """

  def __init__(self, hold_s: float = PRIMARY_HOLD_S) -> None:
    self.hold_s = hold_s
    self.last_mph = 0
    self.last_t: float | None = None
    self.last_road = None
    self.held = False
    self.seen_mph, self.seen_t = 0, None

  def reset(self) -> None:
    self.last_mph, self.last_t, self.last_road, self.held = 0, None, None, False
    self.seen_mph, self.seen_t = 0, None

  def update(self, fusion: TeslaLimitFusion, now: float, road=None) -> int:
    st = fusion.state(now)
    self.held = False
    if st.valid:
      self.last_mph, self.last_t, self.last_road = st.base_mph, now, road
      self.seen_mph, self.seen_t = st.base_mph, now
      return st.base_mph
    if self.last_t is None or self.last_mph <= 0:
      return 0
    fresh = fusion.last_base_t is not None and now - fusion.last_base_t <= BASE_STALE_S
    cand = int(fusion.cand or 0) if fresh else 0
    if cand > 0:
      self.seen_mph, self.seen_t = cand, now
    # a 1-frame 0 inside a new candidate is not "Tesla has nothing"
    pending = cand if cand > 0 else (self.seen_mph if self.seen_t is not None and now - self.seen_t <= SEEN_GRACE_S else 0)
    since = now - self.last_t
    if cand == self.last_mph and since <= self.hold_s:
      # same value back: bridge its re-debounce, also across a map-match
      # road-name flicker (Tesla itself still says this value here)
      self.last_t, self.last_road = now, road
    elif pending > 0 and pending != self.last_mph and since <= self.hold_s + HOLD_S:
      # a new value is debouncing: keep the old one until it is published,
      # so the limit goes old -> new, never old -> OSM -> new
      pass
    elif road != self.last_road and since > ROAD_CHANGE_HOLD_S:
      self.reset()     # new road and Tesla has nothing: OSM after a 1-frame grace
      return 0
    elif since > self.hold_s:
      self.reset()
      return 0
    self.held = True
    return self.last_mph


# Oct 27 (1d5 St Joseph / Cold Spring): a Yes holds past its way section,
# and a live Tesla value far above a low tagged OSM limit is not trusted.
SIGN_HOLD_MAX_M = 2000.0       # a Yes holds at most ~2 km past where it was given
SIGN_HOLD_MAX_S = 600.0        # and never longer than this (stopped / lost GPS)
TOWN_OSM_MAX_MPH = 35          # tagged OSM at or below this = in town
TOWN_TESLA_MAX_OVER_MPH = 10   # in town, live Tesla may exceed tagged OSM by at most this

_DIR_WORDS = {"n", "s", "e", "w", "north", "south", "east", "west", "ne", "nw", "se", "sw",
              "northeast", "northwest", "southeast", "southwest"}


def road_key(name) -> str:
  """Road name for hold matching: case-insensitive, without N/S/E/W words
  (West / East Minnesota Street is one road). '' = unnamed (no decision)."""
  words = [w for w in str(name or "").lower().replace(".", " ").split() if w not in _DIR_WORDS]
  return " ".join(words)


def _dist_m(lat1, lon1, lat2, lon2) -> float:
  rl = math.radians((lat1 + lat2) / 2.0)
  dy = (lat2 - lat1) * 111_320.0
  dx = (lon2 - lon1) * 111_320.0 * math.cos(rl)
  return math.hypot(dx, dy)


class SignHold:
  """A confirmed sign (Yes, or the existing #378 auto-confirm) keeps the limit
  past the end of its way section until the first of:
    * a new sign read with a different value (SIGN plate; same value refreshes)
    * a road-name change (a turn onto another road)
    * SIGN_HOLD_MAX_M driven since the Yes (or SIGN_HOLD_MAX_S)
  Tesla / Tesla-learned / OSM do not override it while it holds. A confirmed
  section (speed_overrides) on the current way still wins over the hold.
  Nothing is saved; it lives for this mapd run only."""

  def __init__(self, max_m: float = SIGN_HOLD_MAX_M, max_s: float = SIGN_HOLD_MAX_S) -> None:
    self.max_m, self.max_s = max_m, max_s
    self.reset()

  def reset(self) -> None:
    self.mph = 0
    self.road = ""
    self.t0 = 0.0
    self.dist_m = 0.0
    self.last_ll = None
    self.sign_seq = None
    self.end_reason = None
    self._now = 0.0
    self._pending_end = getattr(self, "_pending_end", None)

  @property
  def active(self) -> bool:
    return self.mph > 0

  def start(self, mph: int, road, now: float, lat=None, lon=None, sign_seq=None) -> None:
    self.reset()
    self.mph = int(mph)
    self.road = road_key(road)
    self.t0 = self._now = now
    self.last_ll = (lat, lon) if lat is not None and lon is not None else None
    self.sign_seq = sign_seq

  def _end(self, reason: str) -> None:
    mph = self.mph
    self.reset()
    self._pending_end = (reason, mph)

  def note_sign(self, seq, mph) -> None:
    """A SIGN plate read (liveSpeedSignNAP signSeq / signMph)."""
    if not self.active or seq is None:
      return
    seq, mph = int(seq), int(mph or 0)
    if self.sign_seq is None:
      self.sign_seq = seq
      return
    if seq == self.sign_seq:
      return
    self.sign_seq = seq
    if mph <= 0:
      return
    if mph == self.mph:
      self.dist_m, self.t0 = 0.0, self._now  # same value again: the hold restarts here
      return
    self._end("sign")

  def update(self, now: float, road, lat=None, lon=None) -> float | None:
    """Held limit (m/s) or None."""
    self.end_reason, self._pending_end = self._pending_end, None
    self._now = now
    if not self.active:
      return None
    if lat is not None and lon is not None:
      if self.last_ll is not None:
        step = _dist_m(self.last_ll[0], self.last_ll[1], lat, lon)
        if step < 500.0:        # ignore a GPS jump
          self.dist_m += step
      self.last_ll = (lat, lon)
    key = road_key(road)
    if key and not self.road:
      self.road = key
    if key and self.road and key != self.road:
      self._end("road")
    elif self.dist_m > self.max_m:
      self._end("distance")
    elif now - self.t0 > self.max_s:
      self._end("time")
    if self._pending_end is not None:
      self.end_reason, self._pending_end = self._pending_end, None
    return self.mph * MPH_TO_MS if self.active else None


def cap_town_tesla(tesla_mph: int, osm_ms: float, osm_estimated: bool) -> int:
  """Live Tesla limit in town: tagged OSM <= 35 mph and Tesla more than 10 mph
  above it -> 0 (use OSM). St Joseph 1d5: Tesla 55 on a tagged 25/30/35
  residential street; Cold Spring: Tesla 50 on a tagged 35."""
  t = int(tesla_mph or 0)
  if t <= 0 or osm_estimated or osm_ms <= 0:
    return t
  osm_mph = int(round(float(osm_ms) / MPH_TO_MS))
  if osm_mph <= TOWN_OSM_MAX_MPH and t > osm_mph + TOWN_TESLA_MAX_OVER_MPH:
    return 0
  return t


def select_current(sign_ms: float | None, tesla_mph: int, osm_ms: float,
                   learned_ms: float | None = None, hold_ms: float | None = None,
                   osm_estimated: bool = True) -> tuple[float, str | None]:
  """(published current limit m/s, source) for NAPTeslaLimitPrimary.

  Priority (Justin, Oct 23 + Oct 27): confirmed sign section > held Yes
  (SignHold) > active learned section > live (or held) Tesla > OSM. Tesla beats
  OSM higher or lower, except in town (tagged OSM <= 35 mph) where a Tesla value
  more than 10 mph above OSM is dropped (cap_town_tesla). source is "sign" /
  "tesla_learned" / "tesla", or None when OSM applies (caller keeps its
  osm / osm_default source). osm_estimated defaults to True (no town cap) for
  older callers."""
  if sign_ms is not None:
    return float(sign_ms), "sign"
  if hold_ms is not None and hold_ms > 0:
    return float(hold_ms), "sign"
  if learned_ms is not None and learned_ms > 0:
    return float(learned_ms), SOURCE_LEARNED
  tesla_mph = cap_town_tesla(tesla_mph, osm_ms, osm_estimated)
  if int(tesla_mph) > 0:
    return int(tesla_mph) * MPH_TO_MS, SOURCE_TESLA
  return float(osm_ms), None


def next_for_published(nxt: tuple[float, float] | None, published_ms: float) -> tuple[float, float] | None:
  """Drop a 'next' limit that equals what is already published.

  The lookahead (nextSpeedLimit) is computed from OSM + confirmed sections
  only, because Tesla has no lookahead. When Tesla leads OSM (Tesla flips ~40 m
  before the sign, OSM ~25 m after) OSM's next change is often the value
  Tesla already shows; that is not an upcoming change. A next that differs
  is kept: a lower one still drives the early slowdown from the published
  value (card eases from max(vEgo, current) down to it), a higher one is
  ignored by the decrease-only policy anyway."""
  if nxt is None:
    return None
  if abs(float(nxt[0]) - float(published_ms)) <= 0.3:
    return None
  return nxt


class MapdTeslaHook:
  """mapd side of NAPTeslaLimitEnable / NAPTeslaLimitPrimary: drains `can`, fuses,
  picks the Tesla current limit (primary) and runs review flags (enable only).

  Only exists while either param is on, so the `can` subscription (100 Hz) costs
  nothing when both are off. messaging is passed in so tests can run without msgq.
  """

  def __init__(self, messaging, flagger=None, reader: TeslaCanReader | None = None, sock=None) -> None:
    from openpilot.selfdrive.mapd.review_flags import ReviewFlagger
    self.messaging = messaging
    self.reader = reader or TeslaCanReader()
    self.sock = sock if sock is not None else messaging.sub_sock("can", conflate=False)
    self.flagger = flagger or ReviewFlagger()
    self.primary = TeslaPrimary()
    self.learned = None       # TeslaLearned, created on first use while primary_on
    self._learn_drive = int(time.time())  # noqa: TID251  one mapd start = one drive (only_onroad)
    # Set by mapd from the params each reload.
    self.flags_on = True      # NAPTeslaLimitEnable: review notes
    self.primary_on = False   # NAPTeslaLimitPrimary: Tesla sets the current limit
    self._flag_t = -1e9
    self._prop_seq = None
    self._sign_seq = None

  def update_can(self) -> None:
    self.reader.update(self.messaging.drain_sock(self.sock))

  def state(self, now: float) -> TeslaLimitState:
    # CAN logMonoTime is CLOCK_MONOTONIC ns, the same clock as time.monotonic()
    return self.reader.fusion.state(now)

  def note_sign(self, ss, now: float) -> None:
    """New sign reads from liveSpeedSignNAP (prompt opened / sign placed)."""
    if ss is None:
      return
    try:
      pseq, sseq = int(ss.proposedSeq), int(ss.signSeq)
      if pseq != self._prop_seq:
        if self._prop_seq is not None and int(ss.proposedMph) > 0:
          self.flagger.add_read(now, int(ss.proposedMph))
        self._prop_seq = pseq
      if sseq != self._sign_seq:
        if self._sign_seq is not None and int(ss.signMph) > 0:
          self.flagger.add_read(now, int(ss.signMph))
        self._sign_seq = sseq
    except Exception:
      pass

  def primary_mph(self, now: float, road=None) -> int:
    """Tesla mph for the published limit (0 = use OSM); 0 while NAPTeslaLimitPrimary is off."""
    if not self.primary_on:
      self.primary.reset()
      return 0
    return self.primary.update(self.reader.fusion, now, road)

  def learned_store(self):
    """The Tesla-learned table (loaded once), or None while NAPTeslaLimitPrimary is off."""
    if not self.primary_on:
      return None
    if self.learned is None:
      from openpilot.selfdrive.mapd.tesla_learned import TeslaLearned
      self.learned = TeslaLearned(drive=self._learn_drive)
      self.learned.load()
    return self.learned

  def learn_tick(self, now: float, wall: float, way_id, s, osm_raw_mph: int, speed_ms: float,
                 lat, lon, bearing, road: str) -> list | None:
    """Record live debounced Tesla vs raw OSM for the current way section.

    Only the live debounced base (never the held value, never MPP). A section
    that just became active gets a review note (kind "disagree", 7-day dedupe)."""
    store = self.learned_store()
    if store is None:
      return None
    st = self.state(now)
    row = store.observe(way_id, s, st.base_mph if st.valid else 0, int(osm_raw_mph or 0), speed_ms, wall)
    if row is not None:
      try:
        self.flagger.seg_clock.update(now)
        self.flagger.note_disagree(now, wall, float(lat or 0.0), float(lon or 0.0), bearing, speed_ms, road,
                                   row[2], row[3], way_id=row[0])
      except Exception:
        pass
    store.save(now)
    return row

  def flag_tick(self, now: float, wall: float, lat, lon, bearing, speed_ms: float, road: str,
                osm_raw_mph: int, gps_ok: bool) -> dict | None:
    if not self.flags_on or now - self._flag_t < FLAG_PERIOD_S:
      return None
    self._flag_t = now
    self.flagger.seg_clock.update(now)
    st = self.state(now)
    raw = self.reader.fusion
    base_raw = raw.cand if (raw.cand and raw.last_base_t is not None and now - raw.last_base_t <= BASE_STALE_S) else 0
    return self.flagger.tick(now, wall, float(lat or 0.0), float(lon or 0.0), float(bearing or 0.0), speed_ms,
                             road, int(base_raw or 0), int(osm_raw_mph or 0), st.mpp_mph, gps_ok=gps_ok)
