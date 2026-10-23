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

Nothing here actuates: mapd only copies the values into liveMapDataNAP.
"""
from __future__ import annotations

import math
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


class MapdTeslaHook:
  """mapd side of NAPTeslaLimitEnable: drains `can`, fuses, runs review flags.

  Only exists while the param is on, so the `can` subscription (100 Hz) costs
  nothing when off. messaging is passed in so tests can run without msgq.
  """

  def __init__(self, messaging, flagger=None, reader: TeslaCanReader | None = None, sock=None) -> None:
    from openpilot.selfdrive.mapd.review_flags import ReviewFlagger
    self.messaging = messaging
    self.reader = reader or TeslaCanReader()
    self.sock = sock if sock is not None else messaging.sub_sock("can", conflate=False)
    self.flagger = flagger or ReviewFlagger()
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

  def flag_tick(self, now: float, wall: float, lat, lon, bearing, speed_ms: float, road: str,
                osm_raw_mph: int, gps_ok: bool) -> dict | None:
    if now - self._flag_t < FLAG_PERIOD_S:
      return None
    self._flag_t = now
    self.flagger.seg_clock.update(now)
    st = self.state(now)
    raw = self.reader.fusion
    base_raw = raw.cand if (raw.cand and raw.last_base_t is not None and now - raw.last_base_t <= BASE_STALE_S) else 0
    return self.flagger.tick(now, wall, float(lat or 0.0), float(lon or 0.0), float(bearing or 0.0), speed_ms,
                             road, int(base_raw or 0), int(osm_raw_mph or 0), st.mpp_mph, gps_ok=gps_ok)
