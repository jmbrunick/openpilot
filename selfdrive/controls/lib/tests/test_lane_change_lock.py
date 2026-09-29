"""Lane-change lock robustness (Sep 29 2026, route 0000012f--6b1fe62a0c, build c54b4c2).

12:00:48 and 12:00:51 CT, left tips at ~40 mph. The ego RIGHT line prob was 0.20 / 0.13
(< 0.4) with the left line at 0.96 / 0.94, so lock() refused and DesireHelper cancelled
("lane lines too weak"). Only the line being crossed has to be seen; the far line may be
weak. A refused lock keeps the change armed and retries, then says "Lane lines unclear".
"""

import pytest

from cereal import log

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.desire_helper import (
  DT_MDL,
  DesireHelper,
  LaneChangeState,
)
from openpilot.selfdrive.controls.lib.lane_change_target import (
  LANE_LINES_UNCLEAR_SIGNAL,
  LOCK_RETRY_S,
  LaneChangeTarget,
)
from openpilot.selfdrive.controls.lib.tests.test_lane_change_target import (
  FIRM_NUDGE_NM,
  LANE_W,
  _CS,
  _Line,
)


class _ModelLines:
  """modelV2-like model with explicit ego line offsets / probs (+y right)."""

  def __init__(self, left_y, right_y, left_p, right_p):
    ys = [left_y - LANE_W, left_y, right_y, right_y + LANE_W]
    self.laneLines = [_Line(y) for y in ys]
    self.laneLineProbs = [0.5, left_p, right_p, 0.5]


def _tip_and_confirm(dh, model_arm, model_confirm=None, direction=1, torque=FIRM_NUDGE_NM, pressed=True, hands_on=2):
  lever = 1 if direction == 1 else 2
  left, right = direction == 1, direction == 2
  dh.update(_CS(), True, 0.0, model=model_arm, engaged=True)
  dh.update(_CS(left=left, right=right, lever=lever), True, 0.0, model=model_arm, engaged=True)
  dh.update(_CS(left=left, right=right, lever=0), True, 0.0, model=model_arm, engaged=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  sign = 1.0 if direction == 1 else -1.0
  dh.update(_CS(left=left, right=right, steering_pressed=pressed, steering_torque=sign * torque, hands_on=hands_on),
            True, 0.0, model=model_confirm or model_arm, engaged=True)


def _nudge_frames(dh, model, seconds, direction=1, torque=FIRM_NUDGE_NM, pressed=True):
  left, right = direction == 1, direction == 2
  sign = 1.0 if direction == 1 else -1.0
  for _ in range(int(round(seconds / DT_MDL))):
    dh.update(_CS(left=left, right=right, steering_pressed=pressed, steering_torque=sign * torque, hands_on=1 if pressed else 0),
              True, 0.0, model=model, engaged=True)


def test_sep29_1200_48_left_right_line_020_locks_on_left_line():
  dh = DesireHelper()
  m = _ModelLines(-1.59, 1.95, 0.96, 0.20)
  _tip_and_confirm(dh, m, torque=1.59)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target_locked
  assert dh.target.line_y == pytest.approx(-1.59)
  assert dh.target.lane_width == pytest.approx(3.7)
  assert dh.desire == log.Desire.laneChangeLeft


def test_sep29_1200_51_left_right_line_013_soft_confirm_locks():
  dh = DesireHelper()
  m = _ModelLines(-1.50, 1.97, 0.94, 0.13)
  # Soft confirm only (0.7 Nm, steeringPressed still false), held >= 0.15 s.
  _tip_and_confirm(dh, m, torque=0.7, pressed=False, hands_on=0)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  _nudge_frames(dh, m, 0.3, torque=0.7, pressed=False)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target_locked
  assert dh.target.line_y == pytest.approx(-1.50)


def test_far_line_weak_uses_last_good_lane_width():
  dh = DesireHelper()
  good = _ModelLines(-1.6, 1.8, 0.9, 0.9)  # width 3.4
  dh.update(_CS(), True, 0.0, model=good, engaged=True)
  weak = _ModelLines(-1.6, 1.8, 0.9, 0.1)
  _tip_and_confirm(dh, weak)
  assert dh.target_locked
  assert dh.target.lane_width == pytest.approx(3.4)


def test_right_change_needs_only_the_right_line():
  dh = DesireHelper()
  m = _ModelLines(-1.9, 1.7, 0.1, 0.95)
  _tip_and_confirm(dh, m, direction=2)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target.line_y == pytest.approx(1.7)
  assert dh.target.direction == 2


def test_lock_refuses_when_the_crossing_line_is_on_the_wrong_side():
  assert LaneChangeTarget.lock(1, ([0.0, 0.4, 1.9, 5.6], [0.5, 0.9, 0.1, 0.5])) is None
  assert LaneChangeTarget.lock(2, ([0.0, -1.9, -0.3, 3.4], [0.5, 0.1, 0.9, 0.5])) is None


def test_brief_crossing_line_dip_retries_and_locks():
  dh = DesireHelper()
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  dip = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  _tip_and_confirm(dh, dip)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh.target_locked
  _nudge_frames(dh, dip, 0.4)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh.lane_lines_unclear
  _nudge_frames(dh, good, DT_MDL)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert dh.target_locked


def test_weak_crossing_line_waits_shows_unclear_and_never_silently_cancels():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  _tip_and_confirm(dh, weak)
  _nudge_frames(dh, weak, LOCK_RETRY_S + 0.2)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh.target_locked
  assert dh.desire == log.Desire.none
  assert dh.lane_lines_unclear
  assert dh.signals_remaining == LANE_LINES_UNCLEAR_SIGNAL
  # Stays armed (not cancelled, next tip not suppressed) until the 7 s arm window.
  _nudge_frames(dh, weak, 3.0, torque=0.0, pressed=False)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh._suppress_next_tip
  assert not dh.lane_lines_unclear
  assert dh.signals_remaining < LANE_LINES_UNCLEAR_SIGNAL
  _nudge_frames(dh, weak, 4.0, torque=0.0, pressed=False)
  assert dh.lane_change_state == LaneChangeState.off


def test_line_returns_after_retry_ran_out_locks_while_nudge_held():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  _tip_and_confirm(dh, weak)
  _nudge_frames(dh, weak, LOCK_RETRY_S + 0.5)
  assert dh.lane_lines_unclear
  _nudge_frames(dh, good, DT_MDL)
  assert dh.target_locked
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  assert not dh.lane_lines_unclear


def test_released_nudge_after_retry_ran_out_does_not_lock_later():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  _tip_and_confirm(dh, weak)
  _nudge_frames(dh, weak, LOCK_RETRY_S + 0.2)
  _nudge_frames(dh, weak, 0.5, torque=0.0, pressed=False)
  _nudge_frames(dh, good, 0.5, torque=0.0, pressed=False)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh.target_locked


def test_emergency_yank_still_cancels_a_pending_lock():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  _tip_and_confirm(dh, weak)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  # Opposite-direction yank (emergency) cancels even while the lock retries.
  for _ in range(10):
    dh.update(_CS(left=True, steering_pressed=True, steering_torque=-3.5, hands_on=3), True, 0.0, model=weak, engaged=True)
  assert dh.lane_change_state == LaneChangeState.off


def test_below_speed_confirm_does_not_lock_or_retry():
  dh = DesireHelper()
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  slow = 10 * CV.MPH_TO_MS
  dh.update(_CS(v_ego=slow), True, 0.0, model=good, engaged=True)
  dh.update(_CS(left=True, lever=1, v_ego=slow), True, 0.0, model=good, engaged=True)
  dh.update(_CS(left=True, lever=0, v_ego=slow), True, 0.0, model=good, engaged=True)
  _nudge_frames_slow = [_CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=1, v_ego=slow)] * 20
  for cs in _nudge_frames_slow:
    dh.update(cs, True, 0.0, model=good, engaged=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  assert not dh.target_locked


def test_unclear_alert_text_only_when_flagged():
  from openpilot.selfdrive.selfdrived.lane_change_alerts import pre_lane_change_alert

  class _Meta:
    def __init__(self, n):
      self.laneChangeSignalsRemaining = n

  class _M:
    def __init__(self, n):
      self.meta = _Meta(n)

  for left, mici in ((True, False), (False, False), (True, True)):
    cb = pre_lane_change_alert(left, mici)
    normal = cb(None, None, {'modelV2': _M(5)}, False, 1, None)
    unclear = cb(None, None, {'modelV2': _M(LANE_LINES_UNCLEAR_SIGNAL)}, False, 1, None)
    assert "Steer" in normal.alert_text_1
    assert unclear.alert_text_1 == "Lane lines unclear"


def test_pending_lock_is_dropped_if_speed_falls_under_20_mph_before_the_line_returns():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  slow = 10 * CV.MPH_TO_MS
  _tip_and_confirm(dh, weak)
  _nudge_frames(dh, weak, 0.3)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  for _ in range(6):
    dh.update(_CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=1, v_ego=slow),
              True, 0.0, model=weak, engaged=True)
  for _ in range(10):
    dh.update(_CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=1, v_ego=slow),
              True, 0.0, model=good, engaged=True)
  assert not dh.target_locked
  assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_pending_lock_is_dropped_on_blindspot_before_the_line_returns():
  dh = DesireHelper()
  weak = _ModelLines(-1.8, 1.8, 0.1, 0.9)
  good = _ModelLines(-1.8, 1.8, 0.9, 0.9)
  _tip_and_confirm(dh, weak)
  cs = _CS(left=True, steering_pressed=True, steering_torque=FIRM_NUDGE_NM, hands_on=1)
  cs.leftBlindspot = True
  for _ in range(6):
    dh.update(cs, True, 0.0, model=weak, engaged=True)
  for _ in range(10):
    dh.update(cs, True, 0.0, model=good, engaged=True)
  assert not dh.target_locked
  assert dh.lane_change_state == LaneChangeState.preLaneChange


def test_unclear_alert_is_installed_over_the_stock_prompts():
  from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Alert
  from openpilot.selfdrive.selfdrived.lane_change_alerts import install_lane_change_alerts

  events = {log.OnroadEvent.EventName.preLaneChangeLeft: {ET.WARNING: Alert("x", "", 0, 0, 0, 0, 0, 0)},
            log.OnroadEvent.EventName.preLaneChangeRight: {ET.WARNING: Alert("x", "", 0, 0, 0, 0, 0, 0)}}
  install_lane_change_alerts(events, mici=False)
  assert all(callable(v[ET.WARNING]) for v in events.values())
  assert log.OnroadEvent.EventName.preLaneChangeLeft in EVENTS


def test_selfdrived_installs_the_unclear_alert():
  from pathlib import Path
  src = (Path(__file__).resolve().parents[3] / "selfdrived/helpers.py").read_text()
  assert "\ninstall_lane_change_alerts()\n" in src
