"""Blinker-lamp lateral pause.

Approaching a turn, the driver leaves one indicator lamp on. Tesla lamps stay
lit after the stalk returns, so the pause follows leftBlinker / rightBlinker,
not the stalk enum. While exactly one lamp is on, NAP releases lateral torque
and stays engaged. Hazards (both lamps) do not pause.

A blinker here means "I am turning, let go of the wheel", not "change lanes".
"""


def blinker_pauses_lateral(left_blinker, right_blinker) -> bool:
  return bool(left_blinker) != bool(right_blinker)


def lat_active_with_blinker_pause(*, active, steer_fault_temporary, steer_fault_permanent,
                                  standstill, steer_at_standstill,
                                  left_blinker, right_blinker) -> bool:
  lat_active = bool(active) and not steer_fault_temporary and not steer_fault_permanent and \
               (not standstill or steer_at_standstill)
  if blinker_pauses_lateral(left_blinker, right_blinker):
    return False
  return lat_active
