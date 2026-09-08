"""Blinker-lamp lateral pause.

Any single lit indicator lamp means the driver is turning and wants NAP to
release steering while staying engaged. Tesla lamps stay on after the stalk
returns, so the pause follows leftBlinker / rightBlinker, not the stalk enum.

Hazards (both lamps) do not pause and do not start a lane change.
No speed threshold, map, or OSM junction check.
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
