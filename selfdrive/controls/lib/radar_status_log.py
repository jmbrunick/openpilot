"""2 Hz `radarstat` line (cloudlog -> qlog) for Pre-AP Bosch radar digs.

The Oct 4 2026 SensorDirty soft-disable could not be fully explained from qlog:
no liveTracks, CAN snapshots only every ~20 s, no 0x501 alert matrix near the
event. This line keeps the radar's own status bits and a small track summary in
qlog (no capnp change) so the next one can be settled from qlog alone:

  dirty/hw/sgu   TeslaRadarSguInfo (0x301, bus 1) RADC_SensorDirty / HWFail / SGUFail
  al             TeslaRadarAlertMatrix (0x501, bus 1) as 16 hex digits (bit 0 = a001)
  aln            names of the alert bits that are set (a007 sensorBlinded, a052 radomeHtrInop, ...)
  sa / aa        age (s) of the last 0x301 / 0x501 frame; None = never seen
  n / nm         radar tracks / measured tracks
  ns             stationary tracks (|vRel + vEgo| < STATIONARY_VREL_MS)
  nl             stationary tracks near the road edge: |yRel| < NEAR_LATERAL_Y_M and dRel < NEAR_LATERAL_D_M
  ysm            smallest |yRel| of a stationary track (None if none)
  d0             nearest track dRel
Never touches control; every failure is swallowed.
"""
from __future__ import annotations

import json
import time
from collections.abc import Iterable
from typing import Any

RADAR_BUS = 1
ADDR_SGU = 0x301
ADDR_ALERT = 0x501

# DBC TeslaRadarSguInfo, little-endian @1+. Kept equal to selfdrive/ui/radar/bosch_status.py (pinned by a test).
SGU_DIRTY_BIT = 44
SGU_HW_FAIL_BIT = 45
SGU_FAIL_BIT = 46

STATIONARY_VREL_MS = 1.5
NEAR_LATERAL_Y_M = 3.0
NEAR_LATERAL_D_M = 60.0
LOG_PERIOD_S = 0.5

# DBC TeslaRadarAlertMatrix bit n = a(n+1).
ALERT_NAMES: tuple[str, ...] = (
  "ecuInternalPerf", "flashPerformance", "vBatHigh", "adjustmentNotDone", "adjustmentReq", "adjustmentNotOk",
  "sensorBlinded", "plantModeActive", "configMismatch", "canBusOff", "bdyMIA", "espMIA", "gtwMIA", "sccmMIA",
  "adasMIA", "bdyInvalidCount", "adasInvalidCount", "espInvalidCount", "sccmInvalidCount", "bdyInvalidChkSm",
  "espInvalidChkSm", "sccmInvalidChkSm", "sccmInvalidChkSm2", "absValidity", "ambTValidity", "brakeValidity",
  "CntryCdValidity", "espValidity", "longAccOffValidity", "longAccValidity", "odoValidity", "gearValidity",
  "steerAngValidity", "steerAngSpdValidity", "indctrValidity", "vehStandStillValidity", "vinValidity",
  "whlRotValidity", "whlSpdValidity", "whlStandStillValidity", "wiperValidity", "xwdValidity", "yawOffValidity",
  "yawValidity", "bsdSanity", "rctaSanity", "lcwSanity", "steerAngOffSanity", "tireSizeSanity", "velocitySanity",
  "yawSanity", "radomeHtrInop", "espmodValidity", "gtwmodValidity", "stwmodValidity", "bcmodValidity",
  "dimodValidity", "opmodValidity", "drmiInvalidChkSm", "drmiInvalidCount", "radPositionMismatch",
  "strRackMismatch",
)


def radar_bus(src: int) -> bool:
  return (int(src) & 0x7F) == RADAR_BUS


def _bit(value: int, index: int) -> bool:
  return bool((value >> index) & 1)


def decode_sgu(dat: bytes) -> dict[str, int]:
  v = int.from_bytes(bytes(dat)[:8], "little")
  return {"dirty": int(_bit(v, SGU_DIRTY_BIT)), "hw": int(_bit(v, SGU_HW_FAIL_BIT)), "sgu": int(_bit(v, SGU_FAIL_BIT))}


def alert_names(value: int) -> list[str]:
  return [ALERT_NAMES[i] for i in range(len(ALERT_NAMES)) if _bit(value, i)]


def track_summary(points: Iterable[Any], v_ego: float) -> dict[str, Any]:
  n = nm = ns = nl = 0
  ysm: float | None = None
  d0: float | None = None
  for p in points:
    n += 1
    d, y, vr = float(p.dRel), float(p.yRel), float(p.vRel)
    if bool(p.measured):
      nm += 1
    if d0 is None or d < d0:
      d0 = d
    if abs(vr + float(v_ego)) < STATIONARY_VREL_MS:
      ns += 1
      if ysm is None or abs(y) < ysm:
        ysm = abs(y)
      if abs(y) < NEAR_LATERAL_Y_M and d < NEAR_LATERAL_D_M:
        nl += 1
  return {"n": n, "nm": nm, "ns": ns, "nl": nl,
          "ysm": None if ysm is None else round(ysm, 1), "d0": None if d0 is None else round(d0, 1)}


class RadarStatusLogger:
  """Remembers the latest SguInfo / AlertMatrix frames, emits one line per LOG_PERIOD_S."""

  def __init__(self, period_s: float = LOG_PERIOD_S) -> None:
    self.period_s = period_s
    self._sgu: bytes | None = None
    self._sgu_t = 0.0
    self._alert: int | None = None
    self._alert_t = 0.0
    self._last_log_t: float | None = None

  def observe(self, packets: Iterable[Any], now: float) -> None:
    for p in packets:
      if not radar_bus(p.src):
        continue
      if p.address == ADDR_SGU:
        self._sgu, self._sgu_t = bytes(p.dat), now
      elif p.address == ADDR_ALERT:
        self._alert, self._alert_t = int.from_bytes(bytes(p.dat)[:8], "little"), now

  def line(self, points: Iterable[Any], v_ego: float, now: float) -> str | None:
    if self._last_log_t is not None and now - self._last_log_t < self.period_s:
      return None
    self._last_log_t = now
    rec: dict[str, Any] = {}
    if self._sgu is not None:
      rec.update(decode_sgu(self._sgu))
    rec["sa"] = None if self._sgu is None else round(now - self._sgu_t, 1)
    rec["al"] = None if self._alert is None else f"{self._alert:016x}"
    rec["aln"] = [] if self._alert is None else alert_names(self._alert)
    rec["aa"] = None if self._alert is None else round(now - self._alert_t, 1)
    rec.update(track_summary(points, v_ego))
    rec["v"] = round(float(v_ego), 1)
    return "radarstat " + json.dumps(rec, separators=(",", ":"))

  def update(self, packets: Iterable[Any], points: Iterable[Any] | None, v_ego: float, now: float | None = None) -> str | None:
    """Call every card cycle. `points` is None between radar frames (no line then)."""
    try:
      t = time.monotonic() if now is None else now
      self.observe(packets, t)
      if points is None:
        return None
      return self.line(points, v_ego, t)
    except Exception:
      return None
