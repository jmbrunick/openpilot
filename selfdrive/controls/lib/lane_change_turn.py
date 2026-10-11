"""Driver turns during a tipped lane change: blinker hold + turn mode.

Scenario: tip into a turn lane, the change is in progress (target
locked), and the driver turns at the intersection before it finishes.

Turn vs confirm bump is classified by wheel angle, not torque. Logged
tipped lane changes (Sep 16-28, 180 started) peak at 3.5 / 6.3 / 15.7 deg
wheel angle in the lane-change direction (p50 / p90 / p99, max 19.2;
|angle| max 24.5). Driver blinker turns under 25 mph peak at a median
174 deg; 88% exceed 45 deg. ``TURN_ENTER_ANGLE_DEG`` (45 deg) is ~2x the
largest logged lane change. It must be driver steering: lateral already
released, or driver torque in the turn direction >= the soft-lat floor.
openpilot following a curve with no driver torque is not a turn (lat
active >= 20 mph reaches 55 deg at p99.99).

Turn mode (``DesireHelper`` and controlsd each classify from the same
carState, so no cereal change): the lane change ends and its target is
dropped for good (no resume toward the old lane when lat returns), but
controlsd keeps requesting the blinker in the lane-change direction and
feeds BlinkerLateralHold a driver turn, so lateral pauses exactly like a
manual blinker turn (same soft handoff) while openpilot and longitudinal
stay engaged. Only a sudden torque spike is an emergency.

The blinker hold ends when the turn completes (the wheel came back under
``TURN_EXIT_ANGLE_DEG`` for ``TURN_EXIT_HOLD_S``, like the stock
self-cancel), on an opposite stalk press, on disengage, or after
``TURN_HOLD_TIMEOUT_S``. Lane-line confidence does not matter once in
turn mode.

Slowing under 20 mph mid-change ends the change the same way but first
enters a pre-turn blinker hold (the driver is usually about to turn). It
becomes turn mode on a turn, and otherwise also ends once back above
``HOLD_RESUME_SPEED`` (25 mph), plus the exits above.
"""

from cereal import log

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.lane_change_nudge import SOFT_YIELD_TRIGGER_NM

LaneChangeState = log.LaneChangeState

TURN_ENTER_ANGLE_DEG = 45.0
TURN_DRIVER_TORQUE_NM = SOFT_YIELD_TRIGGER_NM
TURN_EXIT_ANGLE_DEG = 10.0
TURN_EXIT_HOLD_S = 0.5
TURN_HOLD_TIMEOUT_S = 25.0
# Same floor as desire_helper.LANE_CHANGE_SPEED_MIN (kept local: DH imports us).
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
HOLD_RESUME_SPEED = 25 * CV.MPH_TO_MS

MODE_NONE = 0
MODE_HOLD = 1   # change ended by slowing down; waiting for the turn
MODE_TURN = 2   # driver is turning


def _signed(value: float, direction: int) -> float:
  """Positive in the lane-change direction (left = +angle / +torque)."""
  return float(value) if direction == 1 else -float(value)


def is_lane_change_turn(*, direction: int, steering_angle_deg: float,
                        torque_nm: float, lat_active: bool) -> bool:
  """Driver wheel angle well beyond any lane change, in its direction."""
  if direction not in (1, 2):
    return False
  if _signed(steering_angle_deg, direction) < TURN_ENTER_ANGLE_DEG:
    return False
  return (not lat_active) or _signed(torque_nm, direction) >= TURN_DRIVER_TORQUE_NM


class LaneChangeTurnHold:
  """controlsd: blinker hold after a lane change turns into a driver turn."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.mode = MODE_NONE
    self._t = 0.0
    self._peaked = False
    self._exit_s = 0.0
    self._prev_stalk = 0

  @property
  def turning(self) -> bool:
    return self.mode == MODE_TURN

  def update(self, *, lc_state, lc_direction: int, steering_angle_deg: float,
             torque_nm: float, lat_active: bool, v_ego: float, stalk_state: int,
             engaged: bool, dt: float) -> int:
    """Returns the blinker direction to hold (1 left, 2 right, 0 none)."""
    stalk = int(stalk_state or 0)
    prev_stalk, self._prev_stalk = self._prev_stalk, stalk
    if not engaged:
      self.reset()
      return 0
    lc_dir = lc_direction if (lc_state != LaneChangeState.off and lc_direction in (1, 2)) else 0

    if self.mode == MODE_NONE:
      if lc_dir and is_lane_change_turn(direction=lc_dir, steering_angle_deg=steering_angle_deg,
                                        torque_nm=torque_nm, lat_active=lat_active):
        self._enter(MODE_TURN, lc_dir)
      elif lc_dir and float(v_ego) < LANE_CHANGE_SPEED_MIN and lc_state in (
          LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
        self._enter(MODE_HOLD, lc_dir)
      if self.mode == MODE_NONE:
        return 0
      self._prev_stalk = stalk
      return self.direction

    self._t += max(float(dt), 0.0)
    opposite = 2 if self.direction == 1 else 1
    if stalk == opposite and prev_stalk != opposite:
      self.reset()
      self._prev_stalk = stalk
      return 0
    if self._t >= TURN_HOLD_TIMEOUT_S - 1e-9:
      self.reset()
      self._prev_stalk = stalk
      return 0

    if self.mode == MODE_HOLD:
      if is_lane_change_turn(direction=self.direction, steering_angle_deg=steering_angle_deg,
                             torque_nm=torque_nm, lat_active=lat_active):
        self.mode = MODE_TURN
        self._peaked = True
      elif float(v_ego) >= HOLD_RESUME_SPEED:
        self.reset()
        self._prev_stalk = stalk
        return 0

    if self.mode == MODE_TURN:
      if _signed(steering_angle_deg, self.direction) >= TURN_ENTER_ANGLE_DEG:
        self._peaked = True
      if self._peaked and abs(float(steering_angle_deg)) < TURN_EXIT_ANGLE_DEG:
        self._exit_s += max(float(dt), 0.0)
        if self._exit_s >= TURN_EXIT_HOLD_S - 1e-9:
          self.reset()
          self._prev_stalk = stalk
          return 0
      else:
        self._exit_s = 0.0
    return self.direction

  def _enter(self, mode: int, direction: int):
    self.mode = mode
    self.direction = direction
    self._t = 0.0
    # Entering turn mode means the wheel is already past the turn angle.
    self._peaked = mode == MODE_TURN
    self._exit_s = 0.0


def blinker_with_turn_hold(lc_blinker: tuple[bool, bool], hold_direction: int) -> tuple[bool, bool]:
  """CC.leftBlinker / rightBlinker: lane change keep-alive OR the turn hold."""
  left, right = bool(lc_blinker[0]), bool(lc_blinker[1])
  if left or right:
    return left, right
  return hold_direction == 1, hold_direction == 2
