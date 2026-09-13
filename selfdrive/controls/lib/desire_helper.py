import math

from cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.stalk_tip_turn import StalkTipTurn

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# Min speed to *start* the lane-change maneuver (preLaneChange → starting)
# after a wheel nudge. Tip vs turn is classified from stalk hold time, not
# this. Stock comma / prior NAP used the same 20 mph floor for the change
# itself.
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# Match openpilot.common.realtime.DT_MDL. Kept local so this helper does not
# import the hardware stack.
DT_MDL = 0.05

# Time the lane change stays armed waiting for steering input.
LANE_CHANGE_ARM_TIME = 7.0

# Cap on how many same-direction lane changes can be queued from repeated taps.
MAX_QUEUED_LANE_CHANGES = 3

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    self.arm_timer = 0.0
    self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)

    # Stalk tip vs turn. Independent of lane-change state so a held lever
    # is not re-read as a new tip after a reset.
    self._tip_turn = StalkTipTurn()
    self._suppress_next_tip = False

    # Includes the lane change currently armed or in progress.
    self.queued_changes = 0
    self.lane_changes_remaining = 0

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  @staticmethod
  def lane_change_keep_blinker(state, direction):
    """Request the turn indicator while ALC is armed or in progress.

    controlsd copies this onto CC.leftBlinker / rightBlinker so Pre-AP
    DAS_bodyControls keeps flashing until laneChangeState returns to off.
    Stalk returning to IDLE is not a cancel.
    """
    if state == LaneChangeState.off or direction == LaneChangeDirection.none:
      return False, False
    return (direction == LaneChangeDirection.left,
            direction == LaneChangeDirection.right)

  def _reset(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.arm_timer = 0.0
    self.queued_changes = 0
    self.lane_changes_remaining = 0

  def update(self, carstate, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Physical lever: IDLE=0, LEFT=1, RIGHT=2, SNA=3. No tip/latch bit —
    # a tip is LEFT/RIGHT then IDLE within STALK_TIP_HOLD_S; held longer
    # from idle is a driver turn. While ALC is armed/in progress, a
    # same-direction stalk must stay on past STALK_ALC_TURN_HOLD_S (1.0s)
    # to cancel ALC as a turn. Classification uses the stalk, not lamps.
    self._tip_turn.update(carstate.turnSignalStalkState, DT_MDL)
    left_press = self._tip_turn.left_press
    right_press = self._tip_turn.right_press
    tip_event = self._tip_turn.tip_event and not self._suppress_next_tip
    if not self._tip_turn.is_pending and self._tip_turn.direction == 0:
      self._suppress_next_tip = False

    if self.lane_change_direction == LaneChangeDirection.left:
      alc_stalk_dir = 1
      same_direction_tip, opposite_press = (
        tip_event and self._tip_turn.tip_direction == 1, right_press)
    elif self.lane_change_direction == LaneChangeDirection.right:
      alc_stalk_dir = 2
      same_direction_tip, opposite_press = (
        tip_event and self._tip_turn.tip_direction == 2, left_press)
    else:
      alc_stalk_dir = 0
      same_direction_tip, opposite_press = False, False

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self._reset()
    else:
      just_cancelled = False
      if self.lane_change_state != LaneChangeState.off:
        if opposite_press:
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif self._tip_turn.is_driver_turn(alc_latched=True,
                                           alc_direction=alc_stalk_dir):
          # Same-direction stalk held >1s during ALC: driver turn, not
          # another lane change. Opposite already canceled above.
          self._reset()
          just_cancelled = True
        elif same_direction_tip:
          self.queued_changes = min(self.queued_changes + 1, MAX_QUEUED_LANE_CHANGES)

      # LaneChangeState.off — tip-to-ALC. LEFT/RIGHT then IDLE within
      # STALK_TIP_HOLD_S arms (preLaneChange) at any speed. Desire stays
      # none; the car does not leave the lane until a wheel nudge
      # (steeringPressed + torque in that direction) enters
      # laneChangeStarting, and only then if v_ego >= LANE_CHANGE_SPEED_MIN.
      # Stalk IDLE alone does not cancel. A held LEFT/RIGHT past the tip
      # window does not arm. Hazards do not arm. Lamp edges without a
      # lever tip do not arm (OP keep-alive flashes).
      hazards = bool(carstate.leftBlinker) and bool(carstate.rightBlinker)
      if (not just_cancelled and self.lane_change_state == LaneChangeState.off and
          tip_event and not hazards):
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_direction = (LaneChangeDirection.left
                                      if self._tip_turn.tip_direction == 1
                                      else LaneChangeDirection.right)
        self.arm_timer = 0.0
        self.queued_changes = 1

      # LaneChangeState.preLaneChange — wait for a wheel nudge. Tap alone
      # must not start the maneuver.
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        self.arm_timer += DT_MDL

        if torque_applied and not blindspot_detected and not below_lane_change_speed:
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif self.arm_timer > LANE_CHANGE_ARM_TIME:
          # Window expired with no wheel nudge — cancel everything.
          self._reset()

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          # One change just completed.
          self.queued_changes = max(self.queued_changes - 1, 0)
          if self.queued_changes > 0:
            # More queued: keep the same direction and signal on, re-arm a fresh
            # window and wait for the next wheel nudge.
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.arm_timer = 0.0
          else:
            # Nothing left — full reset so the toast clears and no ALC re-arms.
            self._reset()

    # Retain the existing metadata even though the stock alert text no longer
    # displays the countdown or queue depth.
    if self.lane_change_state == LaneChangeState.preLaneChange:
      self.signals_remaining = max(math.ceil(LANE_CHANGE_ARM_TIME - self.arm_timer), 0)
    else:
      self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)
    self.lane_changes_remaining = max(self.queued_changes - 1, 0)

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
