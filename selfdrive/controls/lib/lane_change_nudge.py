"""Tipped lane-change confirm vs emergency yank.

A quick stalk tip still arms ALC, and a latched stalk is still a driver
turn. Those splits are unchanged. While the tip's lane change is armed
or in progress (preLaneChange, laneChangeStarting, or finishing) —
including the ~1 s lamp flash-dark gap, because the lane-change
direction stays latched while the lamps blink off — driver torque in
the *same* direction is only the confirm nudge.

That confirm must not soft-yield, latch a driver turn, or
steering-disengage, at any torque inside the lines below.

An emergency yank, in either direction, still takes over immediately
(release lateral, cancel the lane change, allow panda to drop). It is
clearly past a nudge. Thresholds use existing constants:

* EPAS hands-on level 3, the maximum the car reports. Level 2 is
  ``HANDS_ON_DISENGAGE_LEVEL`` and is still only a confirm during this
  window. Outside a tipped lane change, level >= 2 disengages as today.
* |torsion| above ``2 * STEER_THRESHOLD`` (2.0 Nm). ``STEER_THRESHOLD``
  (1.0 Nm) is ``steeringPressed``. A firm confirm at 2× the soft-lat
  floor (1.1 Nm) is already past that override and must not release, so
  the hard line is twice the override threshold.
* Fast torque rise. There is no torsion-rate signal (``steeringRateDeg``
  is wheel rate). |torsion| reaching ``3 * SOFT_YIELD_TRIGGER_NM``
  (1.65 Nm) within 150 ms of being under the 0.55 Nm soft-lat floor.

Panda firmware must be reflashed. ``tesla_preap_blinker.h`` keeps
``controls_allowed`` for hands-on level 2 during a tipped-ALC flash gap
and does **not** suppress hands-on level 3.
"""

from opendbc.car.tesla.values import STEER_THRESHOLD

# Same factor as driver_lateral_handoff.SOFT_YIELD_TRIGGER_NM. Kept here so
# this module does not import the handoff (the handoff imports us).
SOFT_YIELD_TRIGGER_NM = 0.55 * float(STEER_THRESHOLD)

# EPAS_handsOnLevel top value. Level 2 remains HANDS_ON_DISENGAGE_LEVEL.
EMERGENCY_HANDS_ON_LEVEL = 3

# Above the steeringPressed override, clearly past a 2× soft-lat confirm.
EMERGENCY_TORQUE_NM = 2.0 * float(STEER_THRESHOLD)

EMERGENCY_RISE_WINDOW_S = 0.150
EMERGENCY_RISE_NM = 3.0 * SOFT_YIELD_TRIGGER_NM


def torque_is_same_direction(torque_nm: float, direction: int) -> bool:
  """Positive torsion is left (1); negative is right (2). Matches DesireHelper."""
  torque = float(torque_nm)
  if direction == 1:
    return torque > 0.0
  if direction == 2:
    return torque < 0.0
  return False


def is_emergency_yank(*, torque_nm: float, hands_on_level: int, fast_rise: bool) -> bool:
  """True when the driver is past a lane-change nudge."""
  if int(hands_on_level or 0) >= EMERGENCY_HANDS_ON_LEVEL:
    return True
  if abs(float(torque_nm)) > EMERGENCY_TORQUE_NM:
    return True
  return bool(fast_rise)


def steer_disengage_this_frame(*, confirm: bool, release: bool, release_prev: bool,
                              disengage_edge: bool, blocks: bool) -> bool:
  """True when car_specific should add EventName.steerDisengage.

  A same-direction confirm never does, including a lamp flash-dark gap.
  An emergency yank (``release``) does, on its rising edge. Otherwise the
  existing hands-on edge fires only when the blinker gate is open.
  """
  if confirm:
    return False
  return (bool(release) and not release_prev) or (bool(disengage_edge) and not blocks)


def tipped_lane_change_driver_release(*, same_direction: bool, emergency: bool) -> bool:
  """Whether torque during a tipped lane change should release lateral.

  Emergency yank always releases, including a same-direction yank during ALC.
  Same-direction nudge during tipped ALC is the lane-change confirm, not a takeover.
  Opposite torque is not a confirm; the caller keeps today's yield / cancel.
  """
  # Emergency yank always releases, including a same-direction yank during ALC.
  if emergency:
    return True
  # Same-direction nudge during tipped ALC is the lane-change confirm, not a takeover.
  if same_direction:
    return False
  return False


class EmergencyYankTracker:
  """Detect a rise from below the soft-lat floor to 3× that floor within 150 ms."""

  def __init__(self):
    self._t = 0.0
    self._samples: list[tuple[float, float]] = []

  def reset(self):
    self._t = 0.0
    self._samples.clear()

  def update(self, torque_nm: float, dt: float) -> bool:
    self._t += max(float(dt), 0.0)
    self._samples.append((self._t, abs(float(torque_nm))))
    cutoff = self._t - EMERGENCY_RISE_WINDOW_S
    if len(self._samples) > 8:
      self._samples = [(t, mag) for t, mag in self._samples if t >= cutoff - 1e-9]
    return self.fast_rise()

  def fast_rise(self) -> bool:
    if not self._samples:
      return False
    now_t, now_mag = self._samples[-1]
    if now_mag < EMERGENCY_RISE_NM:
      return False
    for t, mag in self._samples:
      if (now_t - t) <= EMERGENCY_RISE_WINDOW_S + 1e-9 and mag < SOFT_YIELD_TRIGGER_NM:
        return True
    return False
