"""Pre-AP pedal-long helpers used by selfdrived.

RegenDemandCheck reads the unclamped plan and prompts when planned
deceleration sits below the regen envelope floor. The carstate
pedalMaxRegen flag covers the other failure shape: a request inside the
envelope that weak battery regen fails to deliver.

update_preap_chimes edge-detects lat and long engage/disengage. Long
follows FSM intent (enableLongControl), not interceptor handshake, so a
stalk engage chimes immediately. Gas override is not an engagement edge:
enableLongControl stays true while the driver is on the pedal, so press
and release are silent.

A brake or driver-turn long drop that keeps cruiseEnabled is a silent
pause: do not fire pedalCruiseDisabled / AudibleAlert.disengage. One SET
that only restores long from that pause is also quiet. Full cancel
(session down) still chimes disengage.
"""
import math
from typing import NamedTuple

from opendbc.car.tesla.preap.interface import get_preap_accel_limits

# Match openpilot.common.realtime.DT_CTRL without importing hardware.
DT_CTRL = 0.01

# Evidence accumulates in a saturating up/down counter so a single MPC sample
# cannot flash a driver prompt, while brief dropouts do not restart the clock.
REGEN_DEMAND_EVIDENCE_COUNT = int(0.3 / DT_CTRL)
REGEN_DEMAND_TRIGGER_MARGIN = 0.2  # m/s² below the envelope floor
REGEN_DEMAND_CLEAR_MARGIN = 0.05  # m/s²
REGEN_DEMAND_MIN_SPEED = 2.0  # m/s; do not prompt for a stopped/settling car
REGEN_DEMAND_CLEAR_SPEED = 1.0  # m/s


class PreAPChimeState(NamedTuple):
  lat_engaged: bool = False
  long_engaged: bool = False
  long_paused: bool = False


class PreAPChimes(NamedTuple):
  lat_engage: bool = False
  lat_disengage: bool = False
  long_engage: bool = False
  long_disengage: bool = False


def update_preap_chimes(*, lat_engaged: bool, long_engaged: bool,
                        prev: PreAPChimeState) -> tuple[PreAPChimes, PreAPChimeState]:
  """Rising/falling edges for Pre-AP lat and long driver prompts.

  Long-only drop while the session stays up (brake / driver-turn pause)
  is silent. Long-only resume from that pause is silent. Initial second
  pull and full re-engage still chime long-engage. Session-down long drop
  still chimes long-disengage (pedalCruiseDisabled).
  """
  silent_pause = (
    prev.lat_engaged and lat_engaged
    and prev.long_engaged and not long_engaged
  )
  long_rising = long_engaged and not prev.long_engaged
  quiet_resume = long_rising and prev.long_paused and lat_engaged
  chimes = PreAPChimes(
    lat_engage=lat_engaged and not prev.lat_engaged,
    lat_disengage=(not lat_engaged) and prev.lat_engaged,
    long_engage=long_rising and not quiet_resume,
    long_disengage=(not long_engaged) and prev.long_engaged and not silent_pause,
  )
  if not lat_engaged:
    long_paused = False
  elif silent_pause:
    long_paused = True
  elif long_engaged:
    long_paused = False
  else:
    long_paused = prev.long_paused
  return chimes, PreAPChimeState(lat_engaged, long_engaged, long_paused)


class RegenDemandCheck:
  """Prompt when planned deceleration exceeds what the regen envelope allows."""

  def __init__(self):
    self.active = False
    self.evidence_updates = 0

  def reset(self):
    self.active = False
    self.evidence_updates = 0

  def update(self, *, pedal_long_active: bool, brake_pressed: bool,
             a_target: float, v_ego: float) -> bool:
    if not pedal_long_active or brake_pressed or not math.isfinite(a_target):
      self.reset()
      return False

    accel_floor, _ = get_preap_accel_limits(v_ego)

    if self.active:
      keep_prompting = (
        v_ego > REGEN_DEMAND_CLEAR_SPEED
        and a_target <= accel_floor - REGEN_DEMAND_CLEAR_MARGIN
      )
      if not keep_prompting:
        self.reset()
      return self.active

    demanding = (
      v_ego >= REGEN_DEMAND_MIN_SPEED
      and a_target <= accel_floor - REGEN_DEMAND_TRIGGER_MARGIN
    )
    if demanding:
      self.evidence_updates = min(self.evidence_updates + 1, REGEN_DEMAND_EVIDENCE_COUNT)
    else:
      self.evidence_updates = max(self.evidence_updates - 1, 0)
    self.active = self.evidence_updates >= REGEN_DEMAND_EVIDENCE_COUNT
    return self.active
