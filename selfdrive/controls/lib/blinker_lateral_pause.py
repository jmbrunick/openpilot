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

Automatic lane change is the other blinker user. OP drives the lamps while
ALC is armed or in progress, so those flashes must not pause lat or latch a
driver turn. Gate pause off when ALC is active, and on a stalk *tip*
(LEFT/RIGHT then IDLE within STALK_TIP_HOLD_S=0.40s) so the driver's own
lamps cannot kill latActive before DesireHelper arms. A held LEFT/RIGHT
past that window is a driver turn at any speed when ALC is not latched.

While ALC is armed / in progress, or leftover keep-alive is still
flashing, a same-direction physical stalk (turnSignalStalkState, not the
bulb bit) must stay LEFT/RIGHT past STALK_ALC_TURN_HOLD_S (1.0s) to cancel
ALC and pause lat as a turn. A shorter same-direction hold does not force
a turn. Opposite uses the 0.40s window. After that turn, lamps dark ~1s
then hand release resumes lat. Leftover keep-alive flashes after ALC stay
ungated until ~1s of continuous dark, unless the stalk hold takes over.

Hazards (both lamps) do not pause and do not start a lane change.
No map or OSM junction check.

Panda tesla_preap still drops controls_allowed on hands-on >= 2 unless
the matching flash-latch is in tesla_preap.h. selfdrived must not turn
that expected disagreement into controlsMismatch / full cancel.
"""

from openpilot.selfdrive.controls.lib.stalk_tip_turn import StalkTipTurn

# Match controlsd / card (openpilot.common.realtime.DT_CTRL).
DT_CTRL = 0.01

# Longer than one Tesla indicator off-period (~0.3s at 90 flashes/min).
LAMP_OFF_DEBOUNCE_S = 1.0


def blinker_pauses_lateral(left_blinker, right_blinker) -> bool:
  return bool(left_blinker) != bool(right_blinker)


def preap_blinker_pause_hides_controls_mismatch(*, brand, fingerprint,
                                                blocks_steer_disengage) -> bool:
  """True when panda !controlsAllowed is the expected blinker-turn override.

  tesla_preap drops controls_allowed on hands-on >= 2 (and EPAS 6-9). During
  a lat-paused driver turn that is the driver steering, not a real cancel.
  Hiding that disagreement prevents EventName.controlsMismatch (2.0s) from
  fully disabling while the lamps are still flashing.
  """
  return (brand == "tesla"
          and fingerprint == "TESLA_MODEL_S_PREAP"
          and bool(blocks_steer_disengage))


def pause_gated_by_alc(*, alc_active, stalk_is_turn=False, **_unused) -> bool:
  """True when lamps are ALC, not a driver turn.

  stalk_is_turn is the already-classified driver-turn predicate (0.40s
  from idle; 1.0s same-direction during ALC/keep-alive). Speed is not
  part of this decision.
  """
  if stalk_is_turn:
    return False
  return bool(alc_active)


class BlinkerLateralHold:
  """Process-local latch: one lamp starts a pause; flash gaps stay paused;
  hand-on keeps it after the turn is complete. ALC keep-alive is not a turn."""

  def __init__(self):
    self.holding = False
    self.turn_active = False
    self._dark_s = 0.0
    self._alc_keep = False
    self._alc_dark_s = 0.0
    self._alc_direction = 0
    self._tip_turn = StalkTipTurn()

  def _reset(self):
    self.holding = False
    self.turn_active = False
    self._dark_s = 0.0
    self._alc_keep = False
    self._alc_dark_s = 0.0
    self._alc_direction = 0
    self._tip_turn.reset()

  @property
  def blocks_steer_disengage(self) -> bool:
    # Driver-turn pause, post-turn hand-on, ALC keep-alive (including
    # leftover flashes), or the unclassified tip window must not
    # USER_DISABLE cruise.
    return self.holding or self.turn_active or self._alc_keep or self._tip_turn.is_pending

  def _enter_alc_keep(self, direction=0):
    self.turn_active = False
    self.holding = False
    self._dark_s = 0.0
    self._alc_keep = True
    self._alc_dark_s = 0.0
    if direction in (1, 2) and self._alc_direction not in (1, 2):
      self._alc_direction = direction

  def update(self, left_blinker, right_blinker, steering_pressed, *,
             engaged=True, dt=None, alc_active=False, v_ego=0.0,
             stalk_state=0) -> bool:
    if dt is None:
      dt = DT_CTRL

    if not engaged:
      self._reset()
      return False

    self._tip_turn.update(stalk_state, dt)
    one_lamp = blinker_pauses_lateral(left_blinker, right_blinker)
    both_dark = (not left_blinker) and (not right_blinker)
    lamp_dir = 1 if (left_blinker and not right_blinker) else (
      2 if (right_blinker and not left_blinker) else 0)
    if ((alc_active or self._alc_keep) and lamp_dir in (1, 2) and
        self._alc_direction not in (1, 2)):
      self._alc_direction = lamp_dir
    alc_latched = bool(alc_active) or self._alc_keep
    stalk_is_turn = self._tip_turn.is_driver_turn(
      alc_latched=alc_latched, alc_direction=self._alc_direction)

    # Physical stalk held long enough: driver turn. Pause even if
    # DesireHelper still has ALC armed for a frame. While the stalk is
    # still LEFT/RIGHT, do not count flash-gap dark toward turn-end.
    if stalk_is_turn:
      self._alc_keep = False
      self._alc_dark_s = 0.0
      self._alc_direction = 0
      self.turn_active = True
      self._dark_s = 0.0
    elif (not (self.turn_active or self.holding) and
          (pause_gated_by_alc(alc_active=alc_active) or self._tip_turn.tip_event)):
      direction = self._tip_turn.tip_direction or lamp_dir
      self._enter_alc_keep(direction)
      return False
    elif self._tip_turn.is_pending:
      # Not yet tip vs turn. Do not pause and do not latch a turn from
      # the driver's lamps — a tip will arm ALC on IDLE.
      if self.turn_active or self.holding:
        return True
      return False

    if self._alc_keep:
      if both_dark:
        self._alc_dark_s += dt
        if self._alc_dark_s >= LAMP_OFF_DEBOUNCE_S:
          self._alc_keep = False
          self._alc_dark_s = 0.0
          self._alc_direction = 0
      else:
        # Still the ALC keep-alive (or its leftover flashes), including gaps.
        self._alc_dark_s = 0.0
        if lamp_dir in (1, 2) and self._alc_direction not in (1, 2):
          self._alc_direction = lamp_dir
      if self._alc_keep:
        self.turn_active = False
        self.holding = False
        return False

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
                                  steering_pressed=False, hold=None, dt=None,
                                  alc_active=False, v_ego=0.0, stalk_state=0) -> bool:
  lat_active = bool(active) and not steer_fault_temporary and not steer_fault_permanent and \
               (not standstill or steer_at_standstill)
  if hold is not None:
    paused = hold.update(left_blinker, right_blinker, steering_pressed,
                         engaged=bool(active), dt=dt, alc_active=alc_active,
                         v_ego=v_ego, stalk_state=stalk_state)
  else:
    gated = pause_gated_by_alc(alc_active=alc_active)
    paused = blinker_pauses_lateral(left_blinker, right_blinker) and not gated
  if paused:
    return False
  return lat_active
