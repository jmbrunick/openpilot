"""Pre-AP Bosch radar `RADC_SensorDirty`: degrade, do not soft-disable.

Route 1c95345a3286a5db|00000156--eb2236cd87, I-40 Pigeon River Gorge
construction zone (single narrow lane between concrete walls), Oct 4 2026:
at 03:03:59 CT the radar set `RADC_SensorDirty` in its own SguInfo frame
(0x301, bit 44). opendbc maps that to `radarErrors.radarUnavailableTemporary`,
which selfdrived turns into SOFT_DISABLE + NO_ENTRY. The car dropped out of
control and could not re-engage for 3 min 50 s, although CAN was valid and the
radar kept delivering live lead tracks the whole time (leadOne radar=1,
17-62 m). The flag is the radar's own blindness/obstruction self-report, not a
CAN fault or a parse error; it is not proof that the radar sees nothing.

Policy (param `NAPRadarIgnoreSensorDirty`, default ON; OFF = old behavior):

  * flag set, live measured tracks present -> DEGRADED. radard publishes
    `radarUnavailableTemporary=False` (so no soft-disable, no NO_ENTRY, and
    the radar lead / prefer path stays on), tags `radarPreferReason` with
    "sensorDirty" and raises `radarPreferFallback`, which selfdrived already
    maps to the non-disabling "Radar Sensor Dirty" warning (events.py picks
    the text from the reason). No accel cap: the planner is untouched.
  * flag set AND no live measured track for SENSOR_DIRTY_PERSIST_S in a row
    -> ESCALATED. The flag is passed through untouched, which is exactly the
    old soft-disable + NO_ENTRY.
  * Live tracks come back for SENSOR_DIRTY_LIVE_RECOVER_S -> the no-track
    clock restarts and the policy is DEGRADED again (re-engage is allowed).

Only the SensorDirty flag is touched. canError, radarFault and wrongConfig
always pass through. This is main-repo policy only: the opendbc radar
interface is unchanged and still reports the raw flag.
"""
from __future__ import annotations

from typing import Any

SENSOR_DIRTY_PARAM = "NAPRadarIgnoreSensorDirty"
# Token in radarState.radarPreferReason (no capnp change). "+"-joined with the
# reliability reason, e.g. "sensorDirty+dropout".
SENSOR_DIRTY_REASON = "sensorDirty"
# Flag set AND no live measured track for this long, continuously, brings back
# the old soft-disable. Live tracks reset it.
SENSOR_DIRTY_PERSIST_S = 10.0
# Live tracks must be back this long before the no-track clock restarts.
SENSOR_DIRTY_LIVE_RECOVER_S = 1.0


def sensor_dirty_ignore_enabled(params: Any) -> bool:
  """NAPRadarIgnoreSensorDirty. Default ON; any read failure is the new behavior."""
  if params is None:
    return True
  try:
    if hasattr(params, "get_bool"):
      return bool(params.get_bool(SENSOR_DIRTY_PARAM))
    val = params.get(SENSOR_DIRTY_PARAM, return_default=True)
    if isinstance(val, (bytes, bytearray)):
      val = val.decode("utf-8", errors="ignore")
    if val is None or val == "":
      return True
    return bool(int(val)) if not isinstance(val, bool) else bool(val)
  except Exception:
    return True


class SensorDirtyPolicy:
  """Per-frame decision: pass the flag through, or mask it and degrade."""

  def __init__(self) -> None:
    self.degraded = False
    self.escalated = False
    self.no_track_s = 0.0
    self._live_s = 0.0

  def reset(self) -> None:
    self.degraded = False
    self.escalated = False
    self.no_track_s = 0.0
    self._live_s = 0.0

  def update(self, flag: bool, live_tracks: bool, dt: float, ignore: bool = True) -> bool:
    """Returns True when radard must mask `radarUnavailableTemporary`."""
    if not ignore or not flag:
      self.reset()
      return False
    if live_tracks:
      self._live_s += dt
      if self._live_s >= SENSOR_DIRTY_LIVE_RECOVER_S:
        self.no_track_s = 0.0
    else:
      self._live_s = 0.0
      self.no_track_s += dt
    self.escalated = self.no_track_s >= SENSOR_DIRTY_PERSIST_S
    self.degraded = not self.escalated
    return self.degraded


def reason_token(reliability_reason: str, degraded: bool) -> str:
  """radarPreferReason text: the sensorDirty token first, then the usual reason."""
  base = str(reliability_reason or "")
  if not degraded:
    return base
  return f"{SENSOR_DIRTY_REASON}+{base}" if base else SENSOR_DIRTY_REASON


def sensor_dirty_degraded(reason: Any) -> bool:
  """True when a radarState.radarPreferReason carries the degraded token."""
  if not isinstance(reason, str):
    return False
  return SENSOR_DIRTY_REASON in reason.split("+")
