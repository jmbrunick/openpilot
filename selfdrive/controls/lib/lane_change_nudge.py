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
clearly past a nudge:

* EPAS hands-on level 3, the maximum the car reports. Level 2 is
  ``HANDS_ON_DISENGAGE_LEVEL`` and is still only a confirm during this
  window. Outside a tipped lane change, level >= 2 disengages as today.
  During an armed / tipped lane change, level 3 with torque in the
  lane-change direction is not an emergency by itself (firm confirms at
  ~1.9-2.3 Nm reach level 3); it still needs one of the torque lines.
  Opposite-direction level 3 is still an emergency.
* |torsion| above ``EMERGENCY_TORQUE_NM`` (2.5 Nm) held for
  ``EMERGENCY_SUSTAIN_S`` (0.10 s). Logged confirms peak 1.0-2.3 Nm;
  hard takeovers reach 2.6-4 Nm and stay there.
* Fast torque rise. There is no torsion-rate signal (``steeringRateDeg``
  is wheel rate). |torsion| reaching ``EMERGENCY_RISE_NM`` (2.2 Nm, 4x the
  0.55 Nm soft-lat floor) within ``EMERGENCY_RISE_WINDOW_S`` (0.25 s) of
  being under the floor, and staying above 2.2 Nm for 0.10 s. The Sep 28
  E1 (-1.88 Nm in ~100 ms) and E3 (+1.77 Nm in ~100-200 ms) confirms
  tripped the old 1.65 Nm / 150 ms single-frame line.

A genuine hard yank (0 -> 3.5 Nm in 50 ms) releases ~0.13 s after onset.

Soft confirm (armed only). While ``preLaneChange`` is armed after a stalk
tip and v_ego >= 20 mph, same-direction torsion >= ``SOFT_CONFIRM_NM``
(0.65 Nm) sustained ``SOFT_CONFIRM_SUSTAIN_S`` (0.15 s) confirms even when
``steeringPressed`` (1.0 Nm, debounced) is still false. Logged first
attempts that did not reach steeringPressed sat at 0.7-1.0 Nm for
0.2-0.3 s; hands-off lane-keep torsion is |tq| p99.9 ~0.5 Nm and 0.65 Nm
held that long is ~0.6/h per direction. The global steeringPressed
threshold is unchanged.

Panda firmware must be reflashed. ``tesla_preap_blinker.h`` keeps
``controls_allowed`` for hands-on level 2 during a tipped-ALC flash gap
and does **not** suppress hands-on level 3 (unchanged here: a level-3
same-direction confirm inside the flash-dark gap can still let panda drop
controls; the lamp-lit phase blocks it).
"""

from opendbc.car.tesla.values import STEER_THRESHOLD

# Same factor as driver_lateral_handoff.SOFT_YIELD_TRIGGER_NM. Kept here so
# this module does not import the handoff (the handoff imports us).
SOFT_YIELD_TRIGGER_NM = 0.55 * float(STEER_THRESHOLD)

# EPAS_handsOnLevel top value. Level 2 remains HANDS_ON_DISENGAGE_LEVEL.
EMERGENCY_HANDS_ON_LEVEL = 3

# Clearly past a firm confirm (logged confirm peaks 1.0-2.3 Nm).
EMERGENCY_TORQUE_NM = 2.5 * float(STEER_THRESHOLD)

# From under the soft-lat floor to 4x that floor within 0.25 s.
EMERGENCY_RISE_WINDOW_S = 0.250
EMERGENCY_RISE_NM = 4.0 * SOFT_YIELD_TRIGGER_NM

# |torsion| must stay above the emergency line this long (not one frame).
EMERGENCY_SUSTAIN_S = 0.10

# Armed-only soft confirm (preLaneChange, v_ego >= 20 mph, same direction).
SOFT_CONFIRM_NM = 0.65 * float(STEER_THRESHOLD)
SOFT_CONFIRM_SUSTAIN_S = 0.15


def torque_is_same_direction(torque_nm: float, direction: int) -> bool:
  """Positive torsion is left (1); negative is right (2). Matches DesireHelper."""
  torque = float(torque_nm)
  if direction == 1:
    return torque > 0.0
  if direction == 2:
    return torque < 0.0
  return False


def is_emergency_yank(*, torque_nm: float, hands_on_level: int, fast_rise: bool,
                      over_torque: bool | None = None, alc_direction: int = 0) -> bool:
  """True when the driver is past a lane-change nudge.

  ``over_torque`` is ``EmergencyYankTracker.over_torque`` (sustained). None
  falls back to the instantaneous line. ``alc_direction`` (1 left, 2 right)
  is set only while a tipped lane change is armed or in progress; then a
  same-direction push at hands-on level 3 needs a torque line too.
  """
  if int(hands_on_level or 0) >= EMERGENCY_HANDS_ON_LEVEL:
    if not torque_is_same_direction(torque_nm, alc_direction):
      return True
  if over_torque is None:
    over_torque = abs(float(torque_nm)) > EMERGENCY_TORQUE_NM
  if over_torque:
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
  """Sustained fast rise / sustained over-torque detector.

  ``update`` returns ``fast_rise``: |torsion| crossed ``EMERGENCY_RISE_NM``
  within ``EMERGENCY_RISE_WINDOW_S`` of the last sample under the soft-lat
  floor and has stayed at or above that line for ``EMERGENCY_SUSTAIN_S``.
  ``over_torque``: |torsion| above ``EMERGENCY_TORQUE_NM`` for
  ``EMERGENCY_SUSTAIN_S``.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self._t = 0.0
    self._last_low_t: float | None = None
    self._rise_cross_t: float | None = None
    self._rise_ok = False
    self._over_t: float | None = None
    self.fast_rise = False
    self.over_torque = False

  def update(self, torque_nm: float, dt: float) -> bool:
    self._t += max(float(dt), 0.0)
    t = self._t
    mag = abs(float(torque_nm))
    if mag < SOFT_YIELD_TRIGGER_NM:
      self._last_low_t = t
    if mag >= EMERGENCY_RISE_NM:
      if self._rise_cross_t is None:
        self._rise_cross_t = t
        self._rise_ok = (self._last_low_t is not None and
                         (t - self._last_low_t) <= EMERGENCY_RISE_WINDOW_S + 1e-9)
    else:
      self._rise_cross_t = None
      self._rise_ok = False
    if mag > EMERGENCY_TORQUE_NM:
      if self._over_t is None:
        self._over_t = t
    else:
      self._over_t = None
    self.fast_rise = (self._rise_cross_t is not None and self._rise_ok and
                      (t - self._rise_cross_t) >= EMERGENCY_SUSTAIN_S - 1e-9)
    self.over_torque = (self._over_t is not None and
                        (t - self._over_t) >= EMERGENCY_SUSTAIN_S - 1e-9)
    return self.fast_rise


class SoftConfirmTracker:
  """Armed-only same-direction confirm below steeringPressed.

  Counts continuous time with torsion >= ``SOFT_CONFIRM_NM`` in the
  lane-change direction. Anything else (disarmed, below speed, weaker or
  opposite torque) resets the timer.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self._held_s = 0.0

  def update(self, *, torque_nm: float, direction: int, armed: bool, dt: float) -> bool:
    torque = float(torque_nm)
    signed = torque if direction == 1 else (-torque if direction == 2 else 0.0)
    if not armed or signed < SOFT_CONFIRM_NM:
      self._held_s = 0.0
      return False
    self._held_s += max(float(dt), 0.0)
    return self._held_s >= SOFT_CONFIRM_SUSTAIN_S - 1e-9
