"""Blinker-lamp lateral pause.

Any single lit indicator lamp means the driver is turning and wants NAP to
release steering while staying engaged. Tesla lamps stay on after the stalk
returns, so the pause follows leftBlinker / rightBlinker, not the stalk enum.

After the lamp clears, keep lateral released until the wheel is no longer
held (steeringPressed). Longitudinal is unchanged by this helper.

Hazards (both lamps) do not pause and do not start a lane change.
No speed threshold, map, or OSM junction check.
"""


def blinker_pauses_lateral(left_blinker, right_blinker) -> bool:
  return bool(left_blinker) != bool(right_blinker)


class BlinkerLateralHold:
  """Process-local latch: one lamp starts a pause; hand-on keeps it after lamp-off."""

  def __init__(self):
    self.holding = False

  def update(self, left_blinker, right_blinker, steering_pressed, *, engaged=True) -> bool:
    if not engaged:
      self.holding = False
      return False
    if blinker_pauses_lateral(left_blinker, right_blinker):
      self.holding = True
      return True
    if self.holding and steering_pressed:
      return True
    self.holding = False
    return False


def lat_active_with_blinker_pause(*, active, steer_fault_temporary, steer_fault_permanent,
                                  standstill, steer_at_standstill,
                                  left_blinker, right_blinker,
                                  steering_pressed=False, hold=None) -> bool:
  lat_active = bool(active) and not steer_fault_temporary and not steer_fault_permanent and \
               (not standstill or steer_at_standstill)
  if hold is not None:
    paused = hold.update(left_blinker, right_blinker, steering_pressed, engaged=bool(active))
  else:
    paused = blinker_pauses_lateral(left_blinker, right_blinker)
  if paused:
    return False
  return lat_active
