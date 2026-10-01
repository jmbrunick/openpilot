import math

from cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import cs_hands_on_level
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  SOFT_YIELD_TRIGGER_NM,
  EmergencyYankTracker,
  SoftConfirmTracker,
  is_emergency_yank,
  tipped_lane_change_driver_release,
  torque_is_same_direction,
)
from openpilot.selfdrive.controls.lib.lane_change_target import (
  RECROSS_MARGIN_M,
  STALL_PROGRESS_LANES,
  STALL_REPULSE_S,
  LANE_LINES_UNCLEAR_SIGNAL,
  LOCK_RETRY_S,
  LaneChangeTarget,
  lane_line_offsets,
)
from openpilot.selfdrive.controls.lib.lane_change_turn import is_lane_change_turn
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

# Target-locked change: an opposite pull cancels only when it is a real
# pull (steeringPressed, >= the soft-yield floor against the change) held
# OPPOSITE_PULL_SUSTAIN_S. The spring-back after a confirm push is one
# 10 Hz sample: 54 of 145 logged starts (Sep 16-28) had a steeringPressed
# opposite-sign sample within [-0.6, +2.0] s of the start (up to 1.22 Nm,
# e.g. -2.52 -> +1.22 Nm), and with the armed soft confirm the start now
# comes before that spring-back. Held 0.20 s at >= 0.55 Nm only matched
# the 2 real opposite pulls (>= 4 Nm). Hard pulls still hit the emergency
# lines.
OPPOSITE_PULL_NM = SOFT_YIELD_TRIGGER_NM
OPPOSITE_PULL_SUSTAIN_S = 0.20

# "Lane lines unclear" stays up this long after a lock retry runs out.
LANE_LINES_UNCLEAR_SHOW_S = 2.0

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
    # Set by modeld from NAPRoundaboutLatched (roundabout ring assist latched): a tip is not a lane change on a ring.
    self.suppress_tips = False
    self._yank = EmergencyYankTracker()
    self._soft_confirm = SoftConfirmTracker()
    self._opposite_s = 0.0

    # Includes the lane change currently armed or in progress.
    self.queued_changes = 0
    self.lane_changes_remaining = 0

    # Target lock (see lane_change_target). None when no tipped lane
    # change is in progress, or when no modelV2 is passed (legacy path).
    self.target: LaneChangeTarget | None = None
    self.target_suspended = False
    self._best_progress = 0.0
    self._stall_s = 0.0

    # Confirmed but not yet locked (crossing line not seen): the lock is
    # retried for LOCK_RETRY_S instead of cancelling the change.
    self._lock_pending = False
    self._lock_gave_up = False
    self._lock_wait_s = 0.0
    self._confirm_prev = False
    self._unclear_s = 0.0
    self._last_lane_width: float | None = None

  @property
  def lane_lines_unclear(self) -> bool:
    """Armed, confirmed, and the lock retry ran out (crossing line weak)."""
    return self._unclear_s > 0.0

  @property
  def target_locked(self) -> bool:
    return self.target is not None

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

  @staticmethod
  def _opposite_pull(carstate, direction: int) -> bool:
    """Driver override (steeringPressed) against the lane-change direction."""
    if not bool(getattr(carstate, "steeringPressed", False)):
      return False
    torque = float(getattr(carstate, "steeringTorque", 0.0) or 0.0)
    if direction == 1:
      return torque <= -OPPOSITE_PULL_NM
    if direction == 2:
      return torque >= OPPOSITE_PULL_NM
    return False

  def _opposite_pull_held(self, carstate, direction: int) -> bool:
    """``_opposite_pull`` held OPPOSITE_PULL_SUSTAIN_S (spring-back is not)."""
    if self._opposite_pull(carstate, direction):
      self._opposite_s += DT_MDL
    else:
      self._opposite_s = 0.0
    return self._opposite_s >= OPPOSITE_PULL_SUSTAIN_S - 1e-9

  def _complete_one(self):
    self._clear_target()
    self.queued_changes = max(self.queued_changes - 1, 0)
    if self.queued_changes > 0:
      self.lane_change_state = LaneChangeState.preLaneChange
      self.lane_change_ll_prob = 1.0
      self.arm_timer = 0.0
    else:
      self._reset()

  def _update_locked(self, suspended: bool) -> bool:
    """Advance a target-locked change. Returns True for a one-frame desire gap."""
    target = self.target
    if target.reached:
      self._complete_one()
      return False
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
      if target.crossed:
        self.lane_change_state = LaneChangeState.laneChangeFinishing
    else:
      self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
      if target.dist_to_line_m > RECROSS_MARGIN_M:
        self.lane_change_state = LaneChangeState.laneChangeStarting
    # Stall: lat up but no progress toward the target for a while (the
    # model's desire pulse may have run out). Re-pulse the desire.
    if suspended:
      return False
    if target.progress > self._best_progress + STALL_PROGRESS_LANES:
      self._best_progress = target.progress
      self._stall_s = 0.0
      return False
    self._stall_s += DT_MDL
    if self._stall_s + 1e-9 >= STALL_REPULSE_S:
      self._stall_s = 0.0
      return True
    return False

  def _reset(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.arm_timer = 0.0
    self.queued_changes = 0
    self.lane_changes_remaining = 0
    self._clear_target()

  def _clear_target(self):
    self.target = None
    self.target_suspended = False
    self._best_progress = 0.0
    self._stall_s = 0.0
    self._lock_pending = False
    self._lock_gave_up = False
    self._lock_wait_s = 0.0
    self._confirm_prev = False
    self._unclear_s = 0.0

  def _try_lock(self, offsets) -> None:
    direction = 1 if self.lane_change_direction == LaneChangeDirection.left else 2
    self.target = LaneChangeTarget.lock(direction, offsets, self._last_lane_width)
    if self.target is not None:
      self._lock_pending = False
      self._lock_gave_up = False
      self._unclear_s = 0.0
      self.lane_change_state = LaneChangeState.laneChangeStarting
      self._best_progress = self.target.progress
      self._stall_s = 0.0

  def update(self, carstate, lateral_active, lane_change_prob, model=None, engaged=None):
    """``model`` is this frame's modelV2 (laneLines / laneLineProbs).

    With a model, a tipped lane change locks a target on start and runs
    to the target lane center (see lane_change_target). Without one the
    legacy lane_change_prob completion is used. ``engaged`` is
    carControl.enabled; None means "same as lateral_active".
    """
    v_ego = carstate.vEgo
    engaged = bool(lateral_active) if engaged is None else bool(engaged)
    offsets = lane_line_offsets(model)
    good_width = LaneChangeTarget.good_width(offsets)
    if good_width is not None:
      self._last_lane_width = good_width
    self._unclear_s = max(self._unclear_s - DT_MDL, 0.0)
    pulse_gap = False
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Physical lever: IDLE=0, LEFT=1, RIGHT=2, SNA=3. No tip/latch bit —
    # a tip is LEFT/RIGHT then IDLE within STALK_TIP_HOLD_S; held longer
    # from idle is a driver turn. While ALC is armed/in progress, a
    # same-direction stalk must stay on past STALK_ALC_TURN_HOLD_S (1.0s)
    # to cancel ALC as a turn. Classification uses the stalk, not lamps.
    self._tip_turn.update(carstate.turnSignalStalkState, DT_MDL)
    torque_nm = float(getattr(carstate, "steeringTorque", 0.0) or 0.0)
    fast_rise = self._yank.update(torque_nm, DT_MDL)
    if self.lane_change_state == LaneChangeState.off:
      lc_dir = 0
    elif self.lane_change_direction == LaneChangeDirection.left:
      lc_dir = 1
    elif self.lane_change_direction == LaneChangeDirection.right:
      lc_dir = 2
    else:
      lc_dir = 0
    emergency_yank = is_emergency_yank(
      torque_nm=torque_nm,
      hands_on_level=cs_hands_on_level(carstate),
      fast_rise=fast_rise,
      over_torque=self._yank.over_torque,
      alc_direction=lc_dir,
    )
    # Driver turning in the lane-change direction (wheel angle well past
    # any lane change): end the change and drop the target for good.
    # controlsd keeps the blinker and pauses lat like a manual turn.
    driver_turn = is_lane_change_turn(
      direction=lc_dir, steering_angle_deg=float(getattr(carstate, "steeringAngleDeg", 0.0) or 0.0),
      torque_nm=torque_nm, lat_active=bool(lateral_active))
    # Counted every frame so a spring-back never carries over.
    opposite_pull = self._opposite_pull_held(carstate, lc_dir)
    # Armed-only soft confirm: preLaneChange, >= 20 mph, same direction.
    soft_confirm = self._soft_confirm.update(
      torque_nm=torque_nm, direction=lc_dir,
      armed=(self.lane_change_state == LaneChangeState.preLaneChange and not below_lane_change_speed),
      dt=DT_MDL)
    left_press = self._tip_turn.left_press
    right_press = self._tip_turn.right_press
    tip_event = self._tip_turn.tip_event and not self._suppress_next_tip and not self.suppress_tips
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

    locked = self.target is not None
    if locked:
      self.target.update(offsets, DT_MDL)
    # An ordinary lateral release (not an emergency yank, not a cancel)
    # keeps a target-locked change: desire drops while lat is down and is
    # re-asserted (fresh rising edge for the model's desire pulse) when
    # lat resumes. The legacy time cap is replaced by the lock's own
    # TARGET_LOCK_TIMEOUT_S, which also counts released time.
    suspended = locked and not lateral_active
    if not engaged or (not lateral_active and not locked) or \
       (not locked and self.lane_change_timer > LANE_CHANGE_TIME_MAX):
      self._reset()
    elif locked and (self.target.timed_out or self.target.low_confidence):
      # Stale lane change, or cannot tell which lane the car is in.
      self._reset()
      self._suppress_next_tip = True
    else:
      if locked and self.target_suspended and not suspended:
        # Lateral just came back: re-assert toward the locked target.
        self._stall_s = 0.0
      self.target_suspended = suspended
      just_cancelled = False
      if self.lane_change_state != LaneChangeState.off:
        if self.lane_change_direction == LaneChangeDirection.left:
          nudge_dir = 1
        elif self.lane_change_direction == LaneChangeDirection.right:
          nudge_dir = 2
        else:
          nudge_dir = 0
        # Emergency yank cancels. A same-direction confirm does not.
        if tipped_lane_change_driver_release(
            same_direction=torque_is_same_direction(torque_nm, nudge_dir),
            emergency=emergency_yank):
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif driver_turn:
          # Turning at the intersection before the change finished.
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif below_lane_change_speed and self.lane_change_state in (
            LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
          # Slowed under 20 mph mid-change (e.g. into an intersection): end it.
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif locked and opposite_pull:
          # Target-locked change: an opposite-direction pull cancels.
          # Same-direction bumps never pause, slow, or cancel it.
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif opposite_press:
          self._reset()
          just_cancelled = True
          self._suppress_next_tip = True
        elif self._tip_turn.is_driver_turn(alc_latched=True,
                                           alc_direction=alc_stalk_dir):
          # Same-direction stalk held >1s during ALC: driver turn, not
          # another lane change. Opposite already cancelled above.
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
        # A lighter same-direction nudge held SOFT_CONFIRM_SUSTAIN_S also confirms.
        torque_applied = torque_applied or soft_confirm

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        self.arm_timer += DT_MDL

        eligible = bool(torque_applied) and not blindspot_detected and not below_lane_change_speed
        # A new eligible confirm (rising edge) starts a fresh lock retry window.
        if eligible and not self._confirm_prev:
          self._lock_pending = True
          self._lock_wait_s = 0.0
          self._lock_gave_up = False
          self._unclear_s = 0.0
        self._confirm_prev = eligible
        if blindspot_detected or below_lane_change_speed:
          self._lock_pending = False
          self._lock_gave_up = False

        if model is None and eligible:
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif model is not None and (self._lock_pending or (self._lock_gave_up and eligible)):
          self._try_lock(offsets)
          if self.target is None:
            # Crossing line not confident (a brief probability dip, or the
            # line is really not seen): stay armed and retry rather than
            # cancelling. After LOCK_RETRY_S "Lane lines unclear" shows and
            # the lock keeps being retried while the nudge is held, until
            # the 7 s arm window ends.
            if self._lock_pending:
              self._lock_wait_s += DT_MDL
              if self._lock_wait_s >= LOCK_RETRY_S - 1e-9:
                self._lock_pending = False
                self._lock_gave_up = True
                self._unclear_s = LANE_LINES_UNCLEAR_SHOW_S
            else:
              self._unclear_s = LANE_LINES_UNCLEAR_SHOW_S
            if self.arm_timer > LANE_CHANGE_ARM_TIME:
              self._reset()
        elif self.arm_timer > LANE_CHANGE_ARM_TIME:
          # Window expired with no wheel nudge — cancel everything.
          self._reset()

      # Target-locked: geometry decides, not lane_change_prob or a timer.
      elif self.target is not None and self.lane_change_state in (
          LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
        pulse_gap = self._update_locked(suspended)

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
      if self.lane_lines_unclear:
        # No capnp change: this otherwise-unused count flags the alert.
        self.signals_remaining = LANE_LINES_UNCLEAR_SIGNAL
    else:
      self.signals_remaining = math.ceil(LANE_CHANGE_ARM_TIME)
    self.lane_changes_remaining = max(self.queued_changes - 1, 0)

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    if self.target is not None and (self.target_suspended or pulse_gap):
      # Lat down: no desire. The next frame with lat back up is a fresh
      # rising edge, which is what the model's desire pulse keys on.
      self.desire = log.Desire.none

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
