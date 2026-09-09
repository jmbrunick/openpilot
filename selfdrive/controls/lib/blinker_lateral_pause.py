"""Blinker-lamp lateral pause.

Any single lit indicator lamp means the driver is turning and wants NAP to
release steering while staying engaged. Tesla lamps stay on after the stalk
returns, so the pause follows leftBlinker / rightBlinker, not the stalk enum.

Those lamp bits flash: they go dark between blinks. Latch turn-active as
soon as one lamp is seen, and keep it through flash gaps until both lamps
have been dark for longer than one flash (~1s). Stalk returning to center
does not end the turn.

After the turn is complete, keep lateral released until the wheel is no
longer held (steeringPressed). Longitudinal is unchanged by this helper.

Hazards (both lamps) do not pause and do not start a lane change.
No speed threshold, map, or OSM junction check.
"""

from openpilot.common.realtime import DT_CTRL

# Longer than one Tesla indicator off-period (~0.3s at 90 flashes/min).
LAMP_OFF_DEBOUNCE_S = 1.0


def blinker_pauses_lateral(left_blinker, right_blinker) -> bool:
  return bool(left_blinker) != bool(right_blinker)


class BlinkerLateralHold:
  """Process-local latch: one lamp starts a pause; flash gaps stay paused;
  hand-on keeps it after the turn is complete."""

  def __init__(self):
    self.holding = False
    self.turn_active = False
    self._dark_s = 0.0

  def _reset(self):
    self.holding = False
    self.turn_active = False
    self._dark_s = 0.0

  def update(self, left_blinker, right_blinker, steering_pressed, *,
             engaged=True, dt=None) -> bool:
    if dt is None:
      dt = DT_CTRL

    if not engaged:
      self._reset()
      return False

    one_lamp = blinker_pauses_lateral(left_blinker, right_blinker)
    both_dark = (not left_blinker) and (not right_blinker)

    if one_lamp:
      self.turn_active = True
      self._dark_s = 0.0
    elif self.turn_active:
      if both_dark:
        self._dark_s += dt
        if self._dark_s >= LAMP_OFF_DEBOUNCE_S:
          self.turn_active = False
          self._dark_s = 0.0
      else:
        # Hazards (both lamps) are not a turn.
        self.turn_active = False
        self._dark_s = 0.0

    if self.turn_active:
      self.holding = True
      return True
    if self.holding and steering_pressed:
      return True
    self.holding = False
    return False


def lat_active_with_blinker_pause(*, active, steer_fault_temporary, steer_fault_permanent,
                                  standstill, steer_at_standstill,
                                  left_blinker, right_blinker,
                                  steering_pressed=False, hold=None, dt=None) -> bool:
  lat_active = bool(active) and not steer_fault_temporary and not steer_fault_permanent and \
               (not standstill or steer_at_standstill)
  if hold is not None:
    paused = hold.update(left_blinker, right_blinker, steering_pressed,
                         engaged=bool(active), dt=dt)
  else:
    paused = blinker_pauses_lateral(left_blinker, right_blinker)
  if paused:
    return False
  return lat_active
