"""Classify Pre-AP turn-stalk as a tip (ALC) vs a held driver turn.

TurnIndLvr_Stat / turnSignalStalkState is only IDLE=0, LEFT=1, RIGHT=2,
SNA=3 — there is no tip vs latch bit. Approximate from hold time:

- Tip (partial): LEFT or RIGHT then back to IDLE within STALK_TIP_HOLD_S
  → automatic lane change (arm ALC). A typical tip-blink is under ~0.3–0.5s.
- Full / latched: LEFT or RIGHT held past STALK_TIP_HOLD_S → driver turn
  (lateral pause). Speed is not used for this split.

While ALC is armed / in progress (or leftover ALC keep-alive is still
flashing), a *same-direction* physical stalk must be held past
STALK_ALC_TURN_HOLD_S (1.0s) to steal ALC as a driver turn. A shorter
same-direction press is not a turn (a tip still queues; 0.40–1.0s is
ignored). Opposite uses the 0.40s window.

Use the stalk enum (non-flashing), not the bulb flash bit, for hold time.
SNA is treated as IDLE. Hazards are not classified here.
"""

# Held this long (or longer) is a latched turn, not a tip-blink.
# 0.40s sits in the 0.3–0.5s tip-blink range: a real tap springs back sooner;
# a detented hold is still LEFT/RIGHT well past this.
STALK_TIP_HOLD_S = 0.40

# During ALC / keep-alive, same-direction stalk must stay LEFT/RIGHT this
# long before we cancel ALC and pause lat as a driver turn. Justin's
# sequence: tip into a turn lane, then hold the stalk for the actual turn.
STALK_ALC_TURN_HOLD_S = 1.0


def _normalize_stalk(stalk_state) -> int:
  stalk = int(stalk_state or 0)
  if stalk in (1, 2):
    return stalk
  return 0  # IDLE or SNA


class StalkTipTurn:
  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.held_s = 0.0
    self.is_turn = False
    self.tip_event = False
    self.tip_direction = 0
    self.left_press = False
    self.right_press = False

  @property
  def is_pending(self) -> bool:
    return self.direction in (1, 2) and not self.is_turn

  @property
  def is_alc_turn(self) -> bool:
    """Held long enough during ALC/keep-alive to take over as a driver turn."""
    return self.direction in (1, 2) and self.held_s + 1e-6 >= STALK_ALC_TURN_HOLD_S

  def is_driver_turn(self, *, alc_latched=False, alc_direction=0) -> bool:
    return stalk_hold_is_driver_turn(self, alc_latched=alc_latched,
                                     alc_direction=alc_direction)

  def update(self, stalk_state, dt: float):
    stalk = _normalize_stalk(stalk_state)
    self.tip_event = False
    self.tip_direction = 0
    self.left_press = False
    self.right_press = False

    if stalk in (1, 2):
      if self.direction != stalk:
        self.direction = stalk
        self.held_s = 0.0
        self.is_turn = False
        self.left_press = stalk == 1
        self.right_press = stalk == 2
      self.held_s += dt
      if not self.is_turn and self.held_s + 1e-6 >= STALK_TIP_HOLD_S:
        self.is_turn = True
    else:
      if self.direction in (1, 2) and not self.is_turn:
        self.tip_event = True
        self.tip_direction = self.direction
      self.direction = 0
      self.held_s = 0.0
      self.is_turn = False


def stalk_hold_is_driver_turn(tip_turn: StalkTipTurn, *, alc_latched=False,
                              alc_direction=0) -> bool:
  """True when the physical stalk is a driver turn (pause lat, cancel ALC).

  No ALC: LEFT/RIGHT held past STALK_TIP_HOLD_S (0.40s).
  ALC armed / in progress / leftover keep-alive:
    same direction (or unknown) → STALK_ALC_TURN_HOLD_S (1.0s);
    opposite → 0.40s (tap still cancels ALC elsewhere without pausing).
  """
  if tip_turn.direction not in (1, 2):
    return False
  if not alc_latched:
    return tip_turn.is_turn
  if alc_direction in (1, 2) and tip_turn.direction != alc_direction:
    return tip_turn.is_turn
  return tip_turn.is_alc_turn
