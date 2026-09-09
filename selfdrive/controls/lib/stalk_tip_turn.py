"""Classify Pre-AP turn-stalk as a tip (ALC) vs a held driver turn.

TurnIndLvr_Stat / turnSignalStalkState is only IDLE=0, LEFT=1, RIGHT=2,
SNA=3 — there is no tip vs latch bit. Approximate from hold time:

- Tip (partial): LEFT or RIGHT then back to IDLE within STALK_TIP_HOLD_S
  → automatic lane change (arm ALC). A typical tip-blink is under ~0.3–0.5s.
- Full / latched: LEFT or RIGHT held past STALK_TIP_HOLD_S → driver turn
  (lateral pause). Speed is not used for this split.

SNA is treated as IDLE. Hazards are not classified here.
"""

# Held this long (or longer) is a latched turn, not a tip-blink.
# 0.40s sits in the 0.3–0.5s tip-blink range: a real tap springs back sooner;
# a detented hold is still LEFT/RIGHT well past this.
STALK_TIP_HOLD_S = 0.40


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
