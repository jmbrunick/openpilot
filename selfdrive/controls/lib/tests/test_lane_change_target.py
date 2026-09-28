"""Target-locked tipped lane change.

A tipped lane change locks its direction and the lane line to cross when
it starts. It runs until the car is centered in the target lane. Only an
opposite pull, an emergency yank, an opposite tip, the lock timeout, or
low-confidence lane lines end it early. An ordinary lateral release
pauses it and it resumes toward the locked target.
"""

import pytest

from cereal import log

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.desire_helper import (
  DT_MDL,
  OPPOSITE_PULL_NM,
  OPPOSITE_PULL_SUSTAIN_S,
  DesireHelper,
  LaneChangeDirection,
  LaneChangeState,
)
from openpilot.selfdrive.controls.lib.driver_lateral_handoff import (
  DriverLateralHandoff,
  lat_active_after_handoff,
)
from openpilot.selfdrive.controls.lib.lane_change_nudge import (
  EMERGENCY_SUSTAIN_S,
  EMERGENCY_TORQUE_NM,
  SOFT_YIELD_TRIGGER_NM,
)
from openpilot.selfdrive.controls.lib.lane_change_target import (
  CENTERED_HOLD_S,
  CENTERED_TOL_M,
  LANE_LINE_MIN_PROB,
  LOW_CONFIDENCE_CANCEL_S,
  TARGET_LOCK_TIMEOUT_S,
  STALL_REPULSE_S,
  LaneChangeTarget,
  lane_line_offsets,
)

FIRM_NUDGE_NM = 2.0 * SOFT_YIELD_TRIGGER_NM  # 1.1 Nm
MPH_45 = 45.0 * CV.MPH_TO_MS
LANE_W = 3.7
# World lane lines, +y right. Car starts centered in the lane between
# -1.85 and +1.85. Left target lane center is -3.7, right is +3.7.
WORLD_LINES = [-1.85 - k * LANE_W for k in range(4, -1, -1)] + [1.85 + k * LANE_W for k in range(5)]


class _CS:
  def __init__(self, left=False, right=False, steering_pressed=False,
               steering_torque=0.0, lever=0, hands_on=0, v_ego=MPH_45):
    self.vEgo = v_ego
    self.leftBlinker = left
    self.rightBlinker = right
    self.steeringPressed = steering_pressed
    self.steeringTorque = steering_torque
    self.leftBlindspot = False
    self.rightBlindspot = False
    self.turnSignalStalkState = lever
    self.handsOnLevel = hands_on


class _Line:
  def __init__(self, y):
    self.y = [y, y, y]


class _Model:
  """modelV2-like laneLines / laneLineProbs for a car at world y = ``car_y``."""

  def __init__(self, car_y, prob=0.9):
    rel = sorted(w - car_y for w in WORLD_LINES)
    left = [y for y in rel if y < 0.0]
    right = [y for y in rel if y >= 0.0]
    ys = [left[-2], left[-1], right[0], right[1]]
    self.laneLines = [_Line(y) for y in ys]
    self.laneLineProbs = [prob] * 4


def _arm(dh, car_y=0.0, direction=1):
  lever = 1 if direction == 1 else 2
  left, right = direction == 1, direction == 2
  m = _Model(car_y)
  dh.update(_CS(), True, 0.0, model=m, engaged=True)
  dh.update(_CS(left=left, right=right, lever=lever), True, 0.0, model=m, engaged=True)
  dh.update(_CS(left=left, right=right, lever=0), True, 0.0, model=m, engaged=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange


def _start(dh, car_y=0.0, direction=1):
  """Tip then confirm nudge: target locked, laneChangeStarting."""
  _arm(dh, car_y, direction)
  torque = FIRM_NUDGE_NM if direction == 1 else -FIRM_NUDGE_NM
  dh.update(_CS(left=direction == 1, right=direction == 2, steering_pressed=True,
                steering_torque=torque, hands_on=2), True, 0.0,
            model=_Model(car_y), engaged=True)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target_locked
  want = log.Desire.laneChangeLeft if direction == 1 else log.Desire.laneChangeRight
  assert dh.desire == want


def _step(dh, car_y, cs=None, lat=True, prob=0.9, engaged=True):
  dh.update(cs or _CS(), lat, 0.0, model=_Model(car_y, prob), engaged=engaged)


def _drive_to(dh, start_y, end_y, speed=1.0, cs=None):
  """Move the car laterally at ``speed`` m/s; returns desires seen."""
  desires = []
  y = start_y
  step = speed * DT_MDL * (1 if end_y >= start_y else -1)
  while (end_y - y) * step > 0 and dh.lane_change_state != LaneChangeState.off:
    y += step
    if (end_y - y) * step < 0:
      y = end_y
    _step(dh, y, cs)
    desires.append(dh.desire)
  return y, desires


def _hold(dh, car_y, seconds, cs=None, lat=True):
  desires = []
  for _ in range(int(round(seconds / DT_MDL))):
    _step(dh, car_y, cs, lat=lat)
    desires.append(dh.desire)
  return desires


def _assert_asserted_with_only_repulse_gaps(desires, want):
  """Desire held; the only gaps are single-frame stall re-pulses."""
  for i, d in enumerate(desires):
    if d != want:
      assert d == log.Desire.none
      assert i + 1 >= int(STALL_REPULSE_S / DT_MDL) - 5
      assert i + 1 == len(desires) or desires[i + 1] == want
      assert i == 0 or desires[i - 1] == want


def test_lock_records_direction_and_ego_line():
  t = LaneChangeTarget.lock(1, lane_line_offsets(_Model(0.0)))
  assert t.direction == 1
  assert t.line_y == pytest.approx(-1.85)
  assert t.lane_width == pytest.approx(LANE_W)
  assert t.progress == pytest.approx(0.0)
  t = LaneChangeTarget.lock(2, lane_line_offsets(_Model(0.0)))
  assert t.line_y == pytest.approx(1.85)
  # Line followed through the laneLines re-index while crossing.
  for car_y in (1.0, 1.8, 1.9, 2.5, 3.1, 3.7):
    t.update(lane_line_offsets(_Model(car_y)), DT_MDL)
    assert t.line_y == pytest.approx(1.85 - car_y)
  assert t.crossed
  assert t.progress == pytest.approx(1.0)
  assert t.center_error_m == pytest.approx(0.0)


def test_completes_when_centered_in_target_lane_not_on_a_timer():
  dh = DesireHelper()
  _start(dh)
  # Just past the line: crossed (finishing) but not centered.
  y, desires = _drive_to(dh, 0.0, -2.3)
  assert all(d == log.Desire.laneChangeLeft for d in desires)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
  # lane_change_prob 0 and long enough for the legacy fade: still going.
  desires = _hold(dh, y, 5.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
  _assert_asserted_with_only_repulse_gaps(desires, log.Desire.laneChangeLeft)
  # The stalled car got a fresh desire pulse (one-frame gap) to finish.
  assert log.Desire.none in desires
  y, _ = _drive_to(dh, y, -3.7)
  _hold(dh, y, CENTERED_HOLD_S + 2 * DT_MDL)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.desire == log.Desire.none
  assert not dh.target_locked


def test_same_direction_bump_mid_change_continues_with_no_pause():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.0)
  bump = _CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=2)
  state_before = dh.lane_change_state
  # Repeated same-direction bumps while the car keeps moving.
  y, desires = _drive_to(dh, y, -2.6, cs=bump)
  assert all(d == log.Desire.laneChangeLeft for d in desires)
  assert dh.lane_change_direction == LaneChangeDirection.left
  assert dh.target_locked and not dh.target_suspended
  assert state_before == LaneChangeState.laneChangeStarting
  # Held bump with no progress for a moment: still asserted.
  desires = _hold(dh, y, 1.0, cs=bump)
  assert all(d == log.Desire.laneChangeLeft for d in desires)
  # Lateral authority stays full on a same-direction confirm.
  h = DriverLateralHandoff(enabled=True)
  for _ in range(50):
    out = h.update(engaged=True, lat_would_be_active=True, steering_torque=FIRM_NUDGE_NM,
                   steering_rate_deg=0.0, alc_active=True, hands_on_level=2,
                   lane_change_confirm=True, emergency_yank=False, v_ego=MPH_45)
    assert not out.yielded and out.authority == 1.0
    assert lat_active_after_handoff(True, out.yielded)
  y, _ = _drive_to(dh, y, -3.7, cs=bump)
  _hold(dh, y, CENTERED_HOLD_S + 2 * DT_MDL)
  assert dh.lane_change_state == LaneChangeState.off


@pytest.mark.parametrize("pull_at", [-0.5, -1.6, -2.4])
def test_opposite_pull_cancels(pull_at):
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, pull_at)
  assert dh.target_locked
  pull = _CS(left=True, steering_pressed=True, steering_torque=-FIRM_NUDGE_NM, hands_on=1)
  frames = int(round(OPPOSITE_PULL_SUSTAIN_S / DT_MDL))
  for _ in range(frames - 1):
    _step(dh, y, pull)
  assert dh.lane_change_state != LaneChangeState.off, "cancelled before the sustain time"
  _step(dh, y, pull)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.desire == log.Desire.none
  assert not dh.target_locked
  # Does not come back afterwards.
  desires = _hold(dh, y, 3.0)
  assert all(d == log.Desire.none for d in desires)
  assert dh.lane_change_state == LaneChangeState.off


def test_confirm_spring_back_does_not_cancel():
  # Logged confirm pushes spring back one 10 Hz sample against the change
  # while steeringPressed is still set (e.g. -2.52 -> +1.22 Nm).
  assert OPPOSITE_PULL_NM == SOFT_YIELD_TRIGGER_NM
  assert OPPOSITE_PULL_SUSTAIN_S == 0.20
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -0.5)
  assert dh.target_locked
  back = _CS(left=True, steering_pressed=True, steering_torque=-1.3, hands_on=1)
  light = _CS(left=True, steering_pressed=True, steering_torque=-(OPPOSITE_PULL_NM - 0.05), hands_on=1)
  for _ in range(3):
    _step(dh, y, back)
    _step(dh, y, back)
    _step(dh, y)
  for _ in range(20):
    _step(dh, y, light)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target_locked


def test_same_direction_emergency_yank_cancels_and_does_not_resume():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.2)
  yank = _CS(left=True, steering_pressed=True,
             steering_torque=EMERGENCY_TORQUE_NM + 0.25, hands_on=3)
  # Same-direction level 3 alone is a firm confirm; the hard torque must
  # be held EMERGENCY_SUSTAIN_S (0.10 s = 3 model frames incl. the first).
  for _ in range(int(round(EMERGENCY_SUSTAIN_S / DT_MDL)) + 1):
    _step(dh, y, yank, lat=False)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.queued_changes == 0
  assert not dh.target_locked
  # Handoff releases immediately on the yank.
  h = DriverLateralHandoff(enabled=True)
  out = h.update(engaged=True, lat_would_be_active=True, steering_torque=EMERGENCY_TORQUE_NM + 0.25,
                 steering_rate_deg=0.0, alc_active=True, hands_on_level=3,
                 emergency_yank=True, lane_change_confirm=False, v_ego=MPH_45)
  assert out.yielded and not lat_active_after_handoff(True, out.yielded)
  # Hands still on, lat down, then hands off and lat back: no resume.
  _hold(dh, y, 1.0, cs=_CS(steering_pressed=True, steering_torque=1.2, hands_on=1), lat=False)
  desires = _hold(dh, y, 4.0)
  assert all(d == log.Desire.none for d in desires)
  assert dh.lane_change_state == LaneChangeState.off


def test_ordinary_release_pauses_then_resumes_to_target():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.0)
  desires = _hold(dh, y, 1.0, lat=False)
  assert all(d == log.Desire.none for d in desires)
  assert dh.target_locked and dh.target_suspended
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  # Lat back: desire re-asserted on the very next frame (fresh rising edge).
  _step(dh, y)
  assert dh.desire == log.Desire.laneChangeLeft
  assert not dh.target_suspended
  y, desires = _drive_to(dh, y, -3.7)
  assert all(d == log.Desire.laneChangeLeft for d in desires)
  _hold(dh, y, CENTERED_HOLD_S + 2 * DT_MDL)
  assert dh.lane_change_state == LaneChangeState.off


@pytest.mark.parametrize("resume_at", [-0.4, -1.3, -1.8, -2.0, -2.8, -3.2])
def test_resume_from_partially_over_finishes_centered_in_target_lane(resume_at):
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, resume_at)
  # Driver/other release moves the car with lat down; target kept.
  _hold(dh, y, 0.5, lat=False)
  assert dh.target_locked
  _step(dh, y)
  assert dh.desire == log.Desire.laneChangeLeft
  assert dh.lane_change_state != LaneChangeState.off
  # Not complete short of center, however far over it already was.
  y, desires = _drive_to(dh, y, -3.7 + CENTERED_TOL_M + 0.15)
  assert all(d == log.Desire.laneChangeLeft for d in desires)
  _hold(dh, y, 1.0)
  assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
  y, _ = _drive_to(dh, y, -3.7)
  _hold(dh, y, CENTERED_HOLD_S + 2 * DT_MDL)
  assert dh.lane_change_state == LaneChangeState.off


def test_resume_right_change_from_partially_over():
  dh = DesireHelper()
  _start(dh, direction=2)
  y, _ = _drive_to(dh, 0.0, 2.2)
  _hold(dh, y, 0.5, lat=False)
  _step(dh, y)
  assert dh.desire == log.Desire.laneChangeRight
  y, _ = _drive_to(dh, y, 3.7)
  _hold(dh, y, CENTERED_HOLD_S + 2 * DT_MDL)
  assert dh.lane_change_state == LaneChangeState.off


def test_opposite_tip_cancels():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.0)
  _step(dh, y, _CS(right=True, lever=2))
  _step(dh, y, _CS(right=True, lever=0))
  assert dh.lane_change_state == LaneChangeState.off
  assert not dh.target_locked
  desires = _hold(dh, y, 2.0)
  assert all(d == log.Desire.none for d in desires)


def test_timeout_cancels_including_released_time():
  assert 10.0 <= TARGET_LOCK_TIMEOUT_S <= 12.0
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -0.8)
  _hold(dh, y, 3.0, lat=False)
  # Stuck with no completion: still alive just before the timeout...
  remaining = TARGET_LOCK_TIMEOUT_S - dh.target.age_s
  _hold(dh, y, remaining - 0.5)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  # ...and cancelled after it, and a stale change never restarts.
  _hold(dh, y, 1.0)
  assert dh.lane_change_state == LaneChangeState.off
  assert not dh.target_locked
  desires = _hold(dh, y, 3.0)
  assert all(d == log.Desire.none for d in desires)


def test_low_lane_line_confidence_cancels():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.0)
  low = LANE_LINE_MIN_PROB * 0.5
  # A short dropout is tolerated.
  for _ in range(int(0.3 / DT_MDL)):
    _step(dh, y, prob=low)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  _step(dh, y)
  for _ in range(int(round(LOW_CONFIDENCE_CANCEL_S / DT_MDL)) + 1):
    _step(dh, y, prob=low)
  assert dh.lane_change_state == LaneChangeState.off
  assert not dh.target_locked
  assert dh.desire == log.Desire.none


def test_low_confidence_at_start_does_not_guess():
  dh = DesireHelper()
  _arm(dh)
  dh.update(_CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=2),
            True, 0.0, model=_Model(0.0, prob=0.1), engaged=True)
  assert dh.lane_change_state == LaneChangeState.off
  assert dh.desire == log.Desire.none
  assert not dh.target_locked


def test_disengage_drops_the_lock():
  dh = DesireHelper()
  _start(dh)
  y, _ = _drive_to(dh, 0.0, -1.0)
  _step(dh, y, lat=False, engaged=False)
  assert dh.lane_change_state == LaneChangeState.off
  assert not dh.target_locked
  desires = _hold(dh, y, 2.0)
  assert all(d == log.Desire.none for d in desires)
