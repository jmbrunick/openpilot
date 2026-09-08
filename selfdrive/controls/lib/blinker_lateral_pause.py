"""Blinker-lamp lateral pause vs highway ALC.

Tesla lamps stay lit after the stalk returns, so the pause follows
leftBlinker / rightBlinker, not the stalk enum. Hazards (both lamps) never
pause.

Rule, verified against this tree:
- Below LANE_CHANGE_SPEED_MIN (20 mph): a blinker is a turn. Pause lateral.
  Map junctions are often missing, so speed is the turn detector.
- At or above that speed: keep today's automatic lane change. Pause only if
  a mapped junction is actually flagged on that lamp side.
- liveMapDataNAP has no junction / turn-side fields today (speed limits
  only). Until one exists, highway blinkers never pause.

Do not use OSM to yaw a path. Longitudinal stays engaged during a pause.
"""

from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN


def blinker_pauses_lateral(left_blinker, right_blinker, v_ego=0.0,
                           junction_on_blinker_side=False) -> bool:
  if bool(left_blinker) == bool(right_blinker):
    return False
  if v_ego < LANE_CHANGE_SPEED_MIN:
    return True
  return bool(junction_on_blinker_side)


def lat_active_with_blinker_pause(*, active, steer_fault_temporary, steer_fault_permanent,
                                  standstill, steer_at_standstill,
                                  left_blinker, right_blinker, v_ego=0.0,
                                  junction_on_blinker_side=False) -> bool:
  lat_active = bool(active) and not steer_fault_temporary and not steer_fault_permanent and \
               (not standstill or steer_at_standstill)
  if blinker_pauses_lateral(left_blinker, right_blinker, v_ego, junction_on_blinker_side):
    return False
  return lat_active
