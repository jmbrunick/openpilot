"""Target lock for a tipped lane change.

When a tipped lane change starts (preLaneChange -> laneChangeStarting on
the confirm nudge), DesireHelper locks a target: the direction and the
lane line the car must cross. The line is taken from modelV2 laneLines:
the ego left line (laneLines[1]) for a left change, the ego right line
(laneLines[2]) for a right change.

Frame: modelV2 laneLines y is in the device frame, +y to the right, so
the ego left line is negative and the ego right line positive. y[0]
(the closest point ahead) is the lateral offset from the car.

The locked line is followed by continuity. The four laneLines slots
re-index as the car crosses a line, so each model frame the tracked line
is the confident line closest to where it was last frame (within
LINE_MATCH_GATE_M). From it:

* ``dist_to_line_m``: signed distance still to travel to the line, in
  the change direction. Positive before the crossing, negative after.
* lane width: ego right - ego left when both are confident and sane,
  otherwise the last good width.
* ``progress`` in lane widths from the original lane center, measured
  toward the target: 0.0 = centered in the original lane, 0.5 = on the
  line, 1.0 = centered in the target lane.

The change is complete when the line has been crossed and the car is
within CENTERED_TOL_M of the target lane center for CENTERED_HOLD_S.
Completion is geometry, not a timer.

If the tracked line is not confidently seen for LOW_CONFIDENCE_CANCEL_S,
the lock reports low confidence and the caller cancels rather than
guessing which lane the car is in. Lock itself is refused when the ego
lines are not confident.
"""

# Min laneLineProbs for a line to count as seen.
LANE_LINE_MIN_PROB = 0.4
# Tracked line unseen this long -> cancel. Short dropouts are tolerated.
LOW_CONFIDENCE_CANCEL_S = 1.0
# Total time from lock (including released / suspended time).
TARGET_LOCK_TIMEOUT_S = 12.0
# Within this of the target lane center counts as centered.
CENTERED_TOL_M = 0.30
CENTERED_HOLD_S = 0.5
# Past the line by this much after a crossing before drifting back
# re-enters laneChangeStarting.
RECROSS_MARGIN_M = 0.20
# Continuity gate for the tracked line between model frames.
LINE_MATCH_GATE_M = 1.2

# Lat up with no gain in progress for this long -> re-pulse the desire.
STALL_REPULSE_S = 3.0
STALL_PROGRESS_LANES = 0.05

DEFAULT_LANE_WIDTH_M = 3.7
MIN_LANE_WIDTH_M = 2.4
MAX_LANE_WIDTH_M = 5.0

LEFT = 1
RIGHT = 2


def lane_line_offsets(model):
  """(ys, probs) from a modelV2-like object, or None if unusable.

  ys[i] is laneLines[i].y[0] (m, +right); probs[i] is laneLineProbs[i].
  """
  if model is None:
    return None
  try:
    lines = list(model.laneLines)
    probs = [float(p) for p in model.laneLineProbs]
    ys = [float(line.y[0]) if len(line.y) else 0.0 for line in lines]
  except Exception:
    return None
  if len(ys) < 4 or len(probs) < 4:
    return None
  return ys[:4], probs[:4]


def _sane_width(w: float) -> bool:
  return MIN_LANE_WIDTH_M <= w <= MAX_LANE_WIDTH_M


class LaneChangeTarget:
  def __init__(self, direction: int, line_y: float, lane_width: float):
    self.direction = int(direction)
    self.line_y = float(line_y)
    self.lane_width = float(lane_width)
    self.low_conf_s = 0.0
    self.centered_s = 0.0
    self.age_s = 0.0

  @classmethod
  def lock(cls, direction: int, offsets):
    """Lock the ego line in ``direction``; None if not confident."""
    if offsets is None or direction not in (LEFT, RIGHT):
      return None
    ys, probs = offsets
    if probs[1] < LANE_LINE_MIN_PROB or probs[2] < LANE_LINE_MIN_PROB:
      return None
    left_y, right_y = ys[1], ys[2]
    if not (left_y < 0.0 < right_y):
      return None
    width = right_y - left_y
    if not _sane_width(width):
      width = DEFAULT_LANE_WIDTH_M
    line_y = left_y if direction == LEFT else right_y
    return cls(direction, line_y, width)

  @property
  def dist_to_line_m(self) -> float:
    return -self.line_y if self.direction == LEFT else self.line_y

  @property
  def crossed(self) -> bool:
    return self.dist_to_line_m < 0.0

  @property
  def progress(self) -> float:
    return 0.5 - self.dist_to_line_m / self.lane_width

  @property
  def center_error_m(self) -> float:
    """Distance from the target lane center (0 when centered in it)."""
    return abs(self.dist_to_line_m + 0.5 * self.lane_width)

  @property
  def low_confidence(self) -> bool:
    return self.low_conf_s + 1e-9 >= LOW_CONFIDENCE_CANCEL_S

  @property
  def timed_out(self) -> bool:
    return self.age_s > TARGET_LOCK_TIMEOUT_S

  @property
  def reached(self) -> bool:
    return self.crossed and self.centered_s + 1e-9 >= CENTERED_HOLD_S

  def update(self, offsets, dt: float) -> None:
    self.age_s += dt
    if offsets is None:
      self.low_conf_s += dt
      self.centered_s = 0.0
      return
    ys, probs = offsets
    best = None
    for y, p in zip(ys, probs, strict=True):
      if p < LANE_LINE_MIN_PROB:
        continue
      err = abs(y - self.line_y)
      if err <= LINE_MATCH_GATE_M and (best is None or err < best[0]):
        best = (err, y)
    if best is None:
      self.low_conf_s += dt
      self.centered_s = 0.0
      return
    self.low_conf_s = 0.0
    self.line_y = best[1]
    if probs[1] >= LANE_LINE_MIN_PROB and probs[2] >= LANE_LINE_MIN_PROB:
      width = ys[2] - ys[1]
      if _sane_width(width):
        self.lane_width = width
    if self.crossed and self.center_error_m <= CENTERED_TOL_M:
      self.centered_s += dt
    else:
      self.centered_s = 0.0
