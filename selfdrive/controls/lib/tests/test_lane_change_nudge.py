"""Same-direction lane-change confirm is not a takeover. Emergency yank is."""

from cereal import log

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.blinker_lateral_pause import (
  BlinkerLateralHold,
  blinker_turn_blocks_steering_disengage,
  lat_active_with_blinker_pause,
)
from openpilot.selfdrive.controls.lib.desire_helper import (
  DT_MDL,
  DesireHelper,
  LaneChangeDirection,
  LaneChangeState,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  SOFT_YIELD_DEBOUNCE_FRAMES,
  SOFT_YIELD_TRIGGER_NM as HANDOFF_SOFT_NM,
  DriverLateralHandoff,
  lat_active_after_handoff,
  required_press_frames,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  EMERGENCY_HANDS_ON_LEVEL,
  EMERGENCY_RISE_NM,
  EMERGENCY_RISE_WINDOW_S,
  EMERGENCY_SUSTAIN_S,
  EMERGENCY_TORQUE_NM,
  SOFT_YIELD_TRIGGER_NM,
  EmergencyYankTracker,
  is_emergency_yank,
  steer_disengage_this_frame,
  tipped_lane_change_driver_release,
)
from openpilot.selfdrive.controls.lib.stalk_tip_turn import STALK_TIP_HOLD_S

FIRM_NUDGE_NM = 2.0 * SOFT_YIELD_TRIGGER_NM  # 1.1 Nm, hands-on 1–2 still a confirm
MPH_45 = 45.0 * CV.MPH_TO_MS
MPH_12 = 12.0 * CV.MPH_TO_MS


class _CS:
  def __init__(self, v_ego=MPH_45, left=False, right=False,
               steering_pressed=False, steering_torque=0.0,
               left_blindspot=False, right_blindspot=False, lever=0,
               hands_on=0):
    self.vEgo = v_ego
    self.leftBlinker = left
    self.rightBlinker = right
    self.steeringPressed = steering_pressed
    self.steeringTorque = steering_torque
    self.leftBlindspot = left_blindspot
    self.rightBlindspot = right_blindspot
    self.turnSignalStalkState = lever
    self.handsOnLevel = hands_on


def _arm_left(dh, v_ego=MPH_45):
  dh.update(_CS(v_ego=v_ego), True, 0.0)
  dh.update(_CS(v_ego=v_ego, left=True, lever=1), True, 0.0)
  dh.update(_CS(v_ego=v_ego, left=True, lever=0), True, 0.0)


def _tip_alc_hold():
  """Stalk tip so BlinkerLateralHold enters ALC keep-alive."""
  hold = BlinkerLateralHold()
  lat_active_with_blinker_pause(
    active=True, steer_fault_temporary=False, steer_fault_permanent=False,
    standstill=False, steer_at_standstill=False, left_blinker=True, right_blinker=False,
    hold=hold, engaged=True, stalk_state=1, v_ego=MPH_45, dt=0.01)
  lat_active_with_blinker_pause(
    active=True, steer_fault_temporary=False, steer_fault_permanent=False,
    standstill=False, steer_at_standstill=False, left_blinker=True, right_blinker=False,
    hold=hold, engaged=True, stalk_state=0, v_ego=MPH_45, dt=0.01)
  assert hold._alc_keep
  assert not hold.turn_active
  return hold


def test_thresholds_sit_above_a_firm_confirm():
  assert SOFT_YIELD_TRIGGER_NM == HANDOFF_SOFT_NM
  assert FIRM_NUDGE_NM == 1.1
  assert FIRM_NUDGE_NM < EMERGENCY_TORQUE_NM
  assert EMERGENCY_TORQUE_NM == 2.5
  assert EMERGENCY_RISE_NM == 4.0 * SOFT_YIELD_TRIGGER_NM == 2.2
  assert EMERGENCY_RISE_WINDOW_S == 0.250
  assert EMERGENCY_SUSTAIN_S == 0.10
  assert EMERGENCY_HANDS_ON_LEVEL == 3
  assert not is_emergency_yank(torque_nm=FIRM_NUDGE_NM, hands_on_level=2, fast_rise=False)
  assert is_emergency_yank(torque_nm=0.2, hands_on_level=3, fast_rise=False)
  assert is_emergency_yank(torque_nm=1.0, hands_on_level=1, fast_rise=False, over_torque=True)
  assert not is_emergency_yank(torque_nm=EMERGENCY_TORQUE_NM + 0.1, hands_on_level=1, fast_rise=False,
                               over_torque=False)
  # Legacy callers without a tracker keep the instantaneous line.
  assert is_emergency_yank(torque_nm=EMERGENCY_TORQUE_NM + 0.1, hands_on_level=1, fast_rise=False)
  assert not tipped_lane_change_driver_release(same_direction=True, emergency=False)
  assert tipped_lane_change_driver_release(same_direction=True, emergency=True)
  assert not tipped_lane_change_driver_release(same_direction=False, emergency=False)


def test_tip_at_45_same_direction_nudge_confirms_through_flash_gap():
  """Firm same-direction nudge starts the change and is not a takeover."""
  dh = DesireHelper()
  _arm_left(dh, MPH_45)
  assert dh.lane_change_state == LaneChangeState.preLaneChange

  nudge = _CS(v_ego=MPH_45, left=True, steering_pressed=True,
              steering_torque=FIRM_NUDGE_NM, hands_on=2)
  dh.update(nudge, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.lane_change_direction == LaneChangeDirection.left
  assert dh.desire == log.Desire.laneChangeLeft

  # Lamp flash-dark gap while the confirm torque is still on the wheel.
  dark = _CS(v_ego=MPH_45, left=False, steering_pressed=True,
             steering_torque=FIRM_NUDGE_NM, hands_on=2)
  for _ in range(int(0.40 / DT_MDL)):
    dh.update(dark, True, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.lane_change_direction == LaneChangeDirection.left

  h = DriverLateralHandoff(enabled=True)
  out = None
  for _ in range(max(SOFT_YIELD_DEBOUNCE_FRAMES, required_press_frames(FIRM_NUDGE_NM)) + 5):
    out = h.update(
      engaged=True, lat_would_be_active=True, steering_torque=FIRM_NUDGE_NM,
      steering_rate_deg=0.0, alc_active=True, hands_on_level=2,
      lane_change_confirm=True, emergency_yank=False, v_ego=MPH_45)
  assert out is not None
  assert not out.yielded
  assert out.authority == 1.0
  assert lat_active_after_handoff(True, out.yielded)

  hold = BlinkerLateralHold()
  assert lat_active_with_blinker_pause(
    active=True, steer_fault_temporary=False, steer_fault_permanent=False,
    standstill=False, steer_at_standstill=False, left_blinker=True, right_blinker=False,
    steering_pressed=True, steering_disengage=True, hold=hold, alc_active=True,
    v_ego=MPH_45, stalk_state=0, soft_lat_on=True, engaged=True, dt=0.01)
  assert lat_active_with_blinker_pause(
    active=True, steer_fault_temporary=False, steer_fault_permanent=False,
    standstill=False, steer_at_standstill=False, left_blinker=False, right_blinker=False,
    steering_pressed=True, steering_disengage=True, hold=hold, alc_active=True,
    v_ego=MPH_45, stalk_state=0, soft_lat_on=True, engaged=True, dt=0.40)
  assert not hold.turn_active
  assert hold._alc_keep
  # Flash-dark gap, hands-on level 2: still not a steering disengage.
  assert blinker_turn_blocks_steering_disengage(
    False, False, 0, hold, emergency=False, alc_confirm=True)
  assert not steer_disengage_this_frame(
    confirm=True, release=False, release_prev=False,
    disengage_edge=True, blocks=False)

  # Maneuver still completes after the gap.
  for _ in range(int(0.6 / DT_MDL)):
    dh.update(dark, True, 0.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
  for _ in range(int(1.5 / DT_MDL)):
    dh.update(_CS(v_ego=MPH_45), True, 0.0)
    if dh.lane_change_state != LaneChangeState.laneChangeFinishing:
      break
  assert dh.lane_change_state == LaneChangeState.off


def test_opposite_direction_nudge_does_not_confirm():
  """Opposite torque does not start the change and still soft-yields as today."""
  dh = DesireHelper()
  _arm_left(dh, MPH_45)
  dh.update(_CS(v_ego=MPH_45, left=True, steering_pressed=True,
                steering_torque=-FIRM_NUDGE_NM, hands_on=2), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.desire == log.Desire.none

  # Armed ALC still suppresses soft-lat in both directions (today).
  h = DriverLateralHandoff(enabled=True)
  out = None
  for _ in range(SOFT_YIELD_DEBOUNCE_FRAMES + 2):
    out = h.update(
      engaged=True, lat_would_be_active=True, steering_torque=-FIRM_NUDGE_NM,
      steering_rate_deg=30.0, alc_active=True, hands_on_level=1, v_ego=MPH_45)
  assert out is not None
  assert not out.yielded
  assert out.authority == 1.0

  # Not a same-direction confirm: a firm push still yields.
  h = DriverLateralHandoff(enabled=True)
  out = None
  frames = max(SOFT_YIELD_DEBOUNCE_FRAMES, required_press_frames(FIRM_NUDGE_NM))
  for _ in range(frames):
    out = h.update(
      engaged=True, lat_would_be_active=True, steering_torque=-FIRM_NUDGE_NM,
      steering_rate_deg=30.0, alc_active=False, hands_on_level=1, v_ego=MPH_45)
  assert out is not None
  assert out.yielded
  assert not lat_active_after_handoff(True, out.yielded)


def test_latched_stalk_is_still_a_driver_turn():
  dh = DesireHelper()
  dh.update(_CS(v_ego=MPH_45), True, 0.0)
  n = int(round(STALK_TIP_HOLD_S / DT_MDL)) + 2
  for _ in range(n):
    dh.update(_CS(v_ego=MPH_45, left=True, lever=1), True, 1.0)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh._tip_turn.is_turn

  hold = BlinkerLateralHold()
  dt = 0.01
  steps = int(round(STALK_TIP_HOLD_S / dt)) + 2
  lat = True
  for _ in range(steps):
    lat = lat_active_with_blinker_pause(
      active=True, steer_fault_temporary=False, steer_fault_permanent=False,
      standstill=False, steer_at_standstill=False, left_blinker=True, right_blinker=False,
      hold=hold, v_ego=MPH_45, stalk_state=1, soft_lat_on=False, engaged=True, dt=dt)
  assert not lat
  assert hold.turn_active


def test_tip_at_low_speed_still_arms_and_does_not_start():
  dh = DesireHelper()
  _arm_left(dh, MPH_12)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.lane_change_direction == LaneChangeDirection.left
  dh.update(_CS(v_ego=MPH_12, left=True, steering_pressed=True,
                steering_torque=FIRM_NUDGE_NM), True, 0.0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert dh.desire == log.Desire.none


def test_emergency_yank_same_direction_releases_and_cancels():
  dh = DesireHelper()
  _arm_left(dh, MPH_45)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  # Hard same-direction yank held past the sustain time.
  yank = _CS(v_ego=MPH_45, left=True, steering_pressed=True,
             steering_torque=EMERGENCY_TORQUE_NM + 0.25, hands_on=2)
  for _ in range(int(round(EMERGENCY_SUSTAIN_S / DT_MDL)) + 1):
    if dh.lane_change_state == LaneChangeState.off:
      break
    dh.update(yank, True, 0.0)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.queued_changes == 0

  h = DriverLateralHandoff(enabled=True)
  out = h.update(
    engaged=True, lat_would_be_active=True, steering_torque=FIRM_NUDGE_NM,
    steering_rate_deg=0.0, alc_active=True, hands_on_level=EMERGENCY_HANDS_ON_LEVEL,
    emergency_yank=True, lane_change_confirm=False, v_ego=MPH_45)
  assert out.yielded
  assert out.authority == 0.0
  assert not lat_active_after_handoff(True, out.yielded)

  hold = _tip_alc_hold()
  assert not blinker_turn_blocks_steering_disengage(
    False, False, 0, hold, emergency=True, alc_confirm=True)
  assert steer_disengage_this_frame(
    confirm=False, release=True, release_prev=False,
    disengage_edge=True, blocks=True)


def test_fast_torque_rise_is_an_emergency_yank():
  tracker = EmergencyYankTracker()
  assert not tracker.update(0.0, 0.01)
  # One frame above the line is not enough; it must be held 0.10 s.
  assert not tracker.update(EMERGENCY_RISE_NM, 0.05)
  assert not tracker.update(EMERGENCY_RISE_NM, EMERGENCY_SUSTAIN_S - 0.001)
  assert tracker.update(EMERGENCY_RISE_NM, 0.001)
  assert is_emergency_yank(torque_nm=EMERGENCY_RISE_NM, hands_on_level=1, fast_rise=True)

  # A slow climb through 4x soft-lat is not the fast-rise case.
  slow = EmergencyYankTracker()
  slow.update(0.0, 0.01)
  assert not slow.update(SOFT_YIELD_TRIGGER_NM, 0.30)
  for _ in range(5):
    assert not slow.update(EMERGENCY_RISE_NM, 0.10)

  dh = DesireHelper()
  _arm_left(dh, MPH_45)
  dh.update(_CS(v_ego=MPH_45, left=True, steering_torque=0.0), True, 1.0)
  for _ in range(int(round(EMERGENCY_SUSTAIN_S / DT_MDL)) + 1):
    dh.update(_CS(v_ego=MPH_45, left=True, steering_pressed=True,
                  steering_torque=EMERGENCY_RISE_NM, hands_on=1), True, 1.0)
  assert dh.lane_change_state == LaneChangeState.off
