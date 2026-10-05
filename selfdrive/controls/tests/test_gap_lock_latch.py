"""Gap-lock latch, clear, and actuator-guard slack."""

from unittest.mock import MagicMock

import pytest

from openpilot.selfdrive.controls.lib.gap_lock import (
  HUD_LOST,
  HUD_UNAVAILABLE,
  GapLockLatch,
  gap_lock_param_enabled,
  slack_for_guard,
)
from openpilot.selfdrive.controls.lib.lead_approach import (
  LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2,
  guard_follow_actuator_regen,
  lead_follow_slack_m,
)


def _step(latch, **kw):
  base = dict(enabled=True, long_on=True, seq=0, status=True, radar=True,
              track_id=4, d_rel=28.0, stalk_exit=False)
  base.update(kw)
  latch.update(0.05, **base)


def _arm(latch, d_rel=28.0, track_id=4, seq=1):
  for _ in range(8):
    _step(latch, seq=0, d_rel=d_rel, track_id=track_id)
  _step(latch, seq=seq, d_rel=d_rel, track_id=track_id)


def test_median_in_range_latches_on_seq_change():
  latch = GapLockLatch()
  samples = [27, 29, 28, 30, 26, 28, 29, 27]
  for d in samples:
    _step(latch, seq=0, d_rel=d)
  assert latch.gap_m is None
  _step(latch, seq=1, d_rel=28)
  assert latch.track_id == 4
  assert latch.gap_m == pytest.approx(28.0)
  assert latch.hud == 0


def test_vision_only_no_lead_and_out_of_range_do_not_latch():
  for radar, status, dist in (
    (False, True, 28.0),
    (True, False, 28.0),
    (True, True, 5.0),
    (True, True, 100.0),
  ):
    latch = GapLockLatch()
    for _ in range(4):
      _step(latch, seq=0, radar=radar, status=status, d_rel=dist, track_id=-1 if not radar else 4)
    _step(latch, seq=1, radar=radar, status=status, d_rel=dist, track_id=-1 if not radar else 4)
    assert latch.gap_m is None
    assert latch.hud == HUD_UNAVAILABLE


def test_failed_arm_keeps_an_existing_lock():
  latch = GapLockLatch()
  _arm(latch, d_rel=28.0)
  for _ in range(10):
    _step(latch, seq=1, d_rel=100.0)
  _step(latch, seq=2, d_rel=100.0)
  assert latch.gap_m == pytest.approx(28.0)
  assert latch.hud == HUD_UNAVAILABLE


def test_flicker_under_half_second_holds_then_clears():
  latch = GapLockLatch()
  _arm(latch)
  for _ in range(9):
    _step(latch, seq=1, status=False, radar=False, d_rel=None)
  assert latch.gap_m == pytest.approx(28.0)
  _step(latch, seq=1, status=True, radar=True, d_rel=27.0)
  assert latch.gap_m == pytest.approx(28.0)
  for _ in range(9):
    _step(latch, seq=1, status=False, radar=False, d_rel=None)
  assert latch.gap_m == pytest.approx(28.0)
  for _ in range(2):
    _step(latch, seq=1, status=False, radar=False, d_rel=None)
  assert latch.gap_m is None
  assert latch.hud == HUD_LOST


def test_new_track_clears_same_frame_and_recapture_updates():
  latch = GapLockLatch()
  _arm(latch, track_id=4)
  _step(latch, seq=1, track_id=9, d_rel=30.0)
  assert latch.gap_m is None
  assert latch.hud == HUD_LOST
  for _ in range(8):
    _step(latch, seq=1, track_id=9, d_rel=40.0)
  _step(latch, seq=2, track_id=9, d_rel=40.0)
  assert latch.track_id == 9
  assert latch.gap_m == pytest.approx(40.0)
  assert latch.hud == 0


def test_disengage_brake_stalk_and_param_clear_without_lost_toast():
  for kw in (
    dict(long_on=False),
    dict(enabled=False),
    dict(stalk_exit=True),
  ):
    latch = GapLockLatch()
    _arm(latch)
    _step(latch, seq=1, **kw)
    assert latch.gap_m is None
    assert latch.hud == 0


def test_param_reader_ignores_mocks_and_missing_keys():
  assert gap_lock_param_enabled(MagicMock()) is False

  class Missing:
    def get_bool(self, _key):
      raise KeyError("NAPGapLock")

  class Off:
    def get_bool(self, _key):
      return False

  class On:
    def get_bool(self, _key):
      return True

  assert gap_lock_param_enabled(Missing()) is False
  assert gap_lock_param_enabled(Off()) is False
  assert gap_lock_param_enabled(On()) is True


def test_guard_slack_uses_locked_meters_and_keeps_mild_floor():
  time_gap = lead_follow_slack_m(30.0, 20.0, 1.3)
  assert slack_for_guard(30.0, 20.0, 1.3, 0) == time_gap
  assert slack_for_guard(30.0, 20.0, 1.3, 0.0) == time_gap
  assert slack_for_guard(30.0, 20.0, 1.3, 28.0) == pytest.approx(2.0)
  clipped = guard_follow_actuator_regen(-1.23, 0.0, slack=0.0)
  assert clipped == pytest.approx(LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2)
  assert LEAD_FOLLOW_ACT_REGEN_FLOOR_MS2 == pytest.approx(-0.22)
