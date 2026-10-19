"""Gap lock stalk bump: a tip steps the locked meters 1 m; a full press clears."""

import pytest

from openpilot.selfdrive.controls.lib.gap_lock import (
  GAP_LOCK_ABSENT_S,
  GAP_LOCK_MAX_M,
  GAP_LOCK_MIN_M,
  GapLockLatch,
  gap_lock_hud_chip,
)
from openpilot.selfdrive.controls.lib.gap_lock_bump import (
  BUMP_CLOSER,
  BUMP_FARTHER,
  BUMP_FULL,
  BUMP_NONE,
  GAP_LOCK_BUMP_M,
  GAP_LOCK_BUMP_TIMEOUT_S,
  PARAM_GAP_LOCK_ACTIVE,
  PARAM_GAP_LOCK_BUMP,
  GapLockBumpCard,
  GapLockBumpRouter,
  bump_code,
  bumped_gap_m,
  card_gap_lock_stalk,
  decode_bump,
  encode_bump,
  planner_gap_lock_step,
)
from openpilot.selfdrive.controls.lib.hypermile import (
  CRUISE_STALK_DN_1ST,
  CRUISE_STALK_IDLE,
  CRUISE_STALK_UP_1ST,
  CRUISE_STALK_UP_2ND,
  FollowStalkGesture,
)

DT = 0.05


class FakeParams:
  def __init__(self, **init):
    self.d = dict(init)
    self.writes = []

  def get(self, key, block=False, return_default=False):
    return self.d.get(key)

  def get_bool(self, key, block=False):
    return bool(self.d.get(key, False))

  def put(self, key, val, block=False):
    self.writes.append((key, val))
    self.d[key] = val

  def put_bool(self, key, val, block=False):
    self.writes.append((key, bool(val)))
    self.d[key] = bool(val)


def _latch_kw(**kw):
  # seq 1 is the arm that _locked_latch already consumed.
  base = dict(enabled=True, long_on=True, seq=1, status=True, radar=True,
              track_id=4, d_rel=28.0, holding=False, y_rel=0.0)
  base.update(kw)
  return base


def _locked_latch(d_rel=28.0):
  latch = GapLockLatch()
  for _ in range(10):
    latch.update(DT, stalk_exit=False, **_latch_kw(seq=0, d_rel=d_rel))
  latch.update(DT, stalk_exit=False, **_latch_kw(seq=1, d_rel=d_rel))
  assert latch.gap_m == pytest.approx(d_rel)
  return latch


# --- encoding / clamp -------------------------------------------------------

def test_encode_decode_roundtrip():
  for seq in (1, 2, 57, 9999):
    for code in (BUMP_CLOSER, BUMP_FARTHER, BUMP_FULL):
      assert decode_bump(encode_bump(seq, code)) == (seq, code)


@pytest.mark.parametrize("raw", [None, True, "junk", -5, 0, 17])
def test_decode_junk_is_none(raw):
  assert decode_bump(raw)[1] == BUMP_NONE


def test_bump_code_direction():
  assert bump_code(True) == BUMP_CLOSER
  assert bump_code(False) == BUMP_FARTHER
  assert bump_code(None) == BUMP_NONE


def test_bump_steps_one_meter_up_is_closer():
  assert GAP_LOCK_BUMP_M == 1.0
  assert bumped_gap_m(28.0, BUMP_CLOSER) == 27.0
  assert bumped_gap_m(28.0, BUMP_FARTHER) == 29.0
  assert bumped_gap_m(28.4, BUMP_CLOSER) == pytest.approx(27.4)


def test_bump_clamps_to_gap_lock_limits():
  assert (GAP_LOCK_MIN_M, GAP_LOCK_MAX_M) == (8.0, 80.0)
  assert bumped_gap_m(8.0, BUMP_CLOSER) == 8.0
  assert bumped_gap_m(8.5, BUMP_CLOSER) == 8.0
  assert bumped_gap_m(80.0, BUMP_FARTHER) == 80.0
  assert bumped_gap_m(79.6, BUMP_FARTHER) == 80.0


def test_bump_without_lock_is_none():
  assert bumped_gap_m(None, BUMP_CLOSER) is None
  assert bumped_gap_m(0.0, BUMP_CLOSER) is None
  assert bumped_gap_m(20.0, BUMP_NONE) is None
  assert bumped_gap_m(20.0, BUMP_FULL) is None


# --- latch ------------------------------------------------------------------

def test_latch_nudge_moves_meters_and_chip():
  latch = _locked_latch(28.0)
  assert latch.nudge(True) == 27.0
  assert gap_lock_hud_chip(latch.gap_m) == "LOCK 27m"
  assert latch.nudge(False) == 28.0
  assert latch.nudge(False) == 29.0
  assert gap_lock_hud_chip(latch.gap_m) == "LOCK 29m"
  assert latch.track_id == 4


def test_latch_nudge_without_lock_does_nothing():
  latch = GapLockLatch()
  assert latch.nudge(True) is None
  assert latch.gap_m is None


def test_latch_nudge_clamps():
  latch = _locked_latch(9.0)
  assert latch.nudge(True) == 8.0
  assert latch.nudge(True) == 8.0


def test_radar_id_swap_after_bumps_keeps_the_lock():
  """Bumped 8 m closer while the car is still at 28 m: same-car id swap is not lost."""
  latch = _locked_latch(28.0)
  for _ in range(8):
    latch.nudge(True)
  assert latch.gap_m == 20.0
  latch.update(DT, stalk_exit=False, **_latch_kw(d_rel=27.5))
  latch.update(DT, stalk_exit=False, **_latch_kw(track_id=9, d_rel=27.0))
  assert latch.track_id == 9
  assert latch.gap_m == 20.0


def test_other_car_after_bump_still_lost():
  latch = _locked_latch(28.0)
  latch.nudge(True)
  t = 0.0
  while t < GAP_LOCK_ABSENT_S + 0.2:
    latch.update(DT, stalk_exit=False, **_latch_kw(track_id=9, d_rel=55.0))
    t += DT
  assert latch.gap_m is None


def test_fresh_lock_clears_nudged_state():
  latch = _locked_latch(28.0)
  latch.nudge(True)
  latch.update(DT, stalk_exit=True, **_latch_kw())
  assert latch.gap_m is None
  assert latch._nudged is False


# --- planner router ---------------------------------------------------------

def test_router_unlocked_press_is_legacy_exit():
  r = GapLockBumpRouter()
  assert r.update(DT, locked=False, stalk_press=True, verdict=BUMP_NONE) == (True, BUMP_NONE)
  assert r.update(DT, locked=False, stalk_press=False, verdict=BUMP_CLOSER) == (False, BUMP_NONE)


def test_router_locked_press_waits_for_verdict():
  r = GapLockBumpRouter()
  assert r.update(DT, locked=True, stalk_press=True, verdict=BUMP_NONE) == (False, BUMP_NONE)
  assert r.update(DT, locked=True, stalk_press=False, verdict=BUMP_NONE) == (False, BUMP_NONE)
  assert r.update(DT, locked=True, stalk_press=False, verdict=BUMP_FARTHER) == (False, BUMP_FARTHER)


def test_router_full_press_clears():
  r = GapLockBumpRouter()
  r.update(DT, locked=True, stalk_press=True, verdict=BUMP_NONE)
  assert r.update(DT, locked=True, stalk_press=False, verdict=BUMP_FULL) == (True, BUMP_NONE)


def test_router_no_verdict_times_out_to_legacy_clear():
  r = GapLockBumpRouter()
  r.update(DT, locked=True, stalk_press=True, verdict=BUMP_NONE)
  t = 0.0
  out = (False, BUMP_NONE)
  while t < GAP_LOCK_BUMP_TIMEOUT_S + 0.1:
    out = r.update(DT, locked=True, stalk_press=False, verdict=BUMP_NONE)
    t += DT
    if out[0]:
      break
  assert out == (True, BUMP_NONE)


def test_router_ignores_stale_bump_at_start():
  p = FakeParams(**{PARAM_GAP_LOCK_BUMP: encode_bump(5, BUMP_CLOSER)})
  r = GapLockBumpRouter()
  assert r.read_verdict(p) == BUMP_NONE
  assert r.read_verdict(p) == BUMP_NONE
  p.d[PARAM_GAP_LOCK_BUMP] = encode_bump(6, BUMP_FARTHER)
  assert r.read_verdict(p) == BUMP_FARTHER
  assert r.read_verdict(p) == BUMP_NONE


def test_router_first_write_after_empty_param_counts():
  p = FakeParams()
  r = GapLockBumpRouter()
  assert r.read_verdict(p) == BUMP_NONE
  p.d[PARAM_GAP_LOCK_BUMP] = encode_bump(1, BUMP_CLOSER)
  assert r.read_verdict(p) == BUMP_CLOSER


def test_router_publishes_active_only_on_change():
  p = FakeParams()
  r = GapLockBumpRouter()
  r.publish_active(p, False)
  r.publish_active(p, False)
  r.publish_active(p, True)
  r.publish_active(p, True)
  r.publish_active(p, False)
  assert [w for w in p.writes if w[0] == PARAM_GAP_LOCK_ACTIVE] == [
    (PARAM_GAP_LOCK_ACTIVE, False), (PARAM_GAP_LOCK_ACTIVE, True), (PARAM_GAP_LOCK_ACTIVE, False)]


# --- planner hook ------------------------------------------------------------

def _planner_frame(router, latch, p, *, stalk_press=False, **kw):
  return planner_gap_lock_step(
    router, latch, p, DT, stalk_press=stalk_press,
    update_latch=lambda stalk_exit: latch.update(DT, stalk_exit=stalk_exit, **_latch_kw(**kw)))


def test_planner_step_applies_tip_and_keeps_lock():
  p = FakeParams()
  router = GapLockBumpRouter()
  latch = _locked_latch(28.0)
  assert _planner_frame(router, latch, p) is None
  assert p.d[PARAM_GAP_LOCK_ACTIVE] is True
  # Press edge reaches the planner first: the lock must not clear.
  _planner_frame(router, latch, p, stalk_press=True)
  assert latch.gap_m == 28.0
  p.d[PARAM_GAP_LOCK_BUMP] = encode_bump(1, BUMP_CLOSER)
  assert _planner_frame(router, latch, p) == 27.0
  assert latch.gap_m == 27.0
  p.d[PARAM_GAP_LOCK_BUMP] = encode_bump(2, BUMP_FARTHER)
  _planner_frame(router, latch, p, stalk_press=True)
  assert latch.gap_m == 28.0


def test_planner_step_full_press_clears_lock():
  p = FakeParams()
  router = GapLockBumpRouter()
  latch = _locked_latch(28.0)
  _planner_frame(router, latch, p)
  _planner_frame(router, latch, p, stalk_press=True)
  p.d[PARAM_GAP_LOCK_BUMP] = encode_bump(1, BUMP_FULL)
  _planner_frame(router, latch, p)
  assert latch.gap_m is None
  assert p.d[PARAM_GAP_LOCK_ACTIVE] is False


def test_planner_step_unlocked_press_unchanged():
  """No lock: the press still feeds stalk_exit (blocks the in-hold preview, as before)."""
  p = FakeParams()
  router = GapLockBumpRouter()
  latch = GapLockLatch()
  seen = []
  planner_gap_lock_step(router, latch, p, DT, stalk_press=True, update_latch=seen.append)
  assert seen == [True]


# --- card hook -----------------------------------------------------------------

def _card_gesture(detents, *, locked, p=None):
  """Drive FollowStalkGesture + card hook through a detent sequence."""
  p = p if p is not None else FakeParams(**{PARAM_GAP_LOCK_ACTIVE: locked})
  bump = GapLockBumpCard()
  g = FollowStalkGesture()
  routed_to_lock = []
  follow_commits = []
  for d in detents:
    gap_locked = bump.refresh(p, d, False)
    before = g.saw_hold
    commit, closer, _undo = g.update(has_lead=False or gap_locked, detent=d)
    took = card_gap_lock_stalk(bump, p, gap_locked=gap_locked, saw_hold_before=before,
                               saw_hold_now=g.saw_hold, commit=commit, closer=closer)
    routed_to_lock.append(took)
    if commit and not took:
      follow_commits.append(closer)
  return p, routed_to_lock, follow_commits


def test_card_tip_up_while_locked_reports_closer_not_follow():
  p, took, follow = _card_gesture(
    [CRUISE_STALK_IDLE, CRUISE_STALK_UP_1ST, CRUISE_STALK_UP_1ST, CRUISE_STALK_IDLE], locked=True)
  assert any(took)
  assert follow == []
  assert decode_bump(p.d[PARAM_GAP_LOCK_BUMP]) == (1, BUMP_CLOSER)


def test_card_tip_down_while_locked_reports_farther():
  p, _took, follow = _card_gesture(
    [CRUISE_STALK_IDLE, CRUISE_STALK_DN_1ST, CRUISE_STALK_IDLE], locked=True)
  assert follow == []
  assert decode_bump(p.d[PARAM_GAP_LOCK_BUMP]) == (1, BUMP_FARTHER)


def test_card_full_press_while_locked_reports_full():
  p, took, follow = _card_gesture(
    [CRUISE_STALK_IDLE, CRUISE_STALK_UP_1ST, CRUISE_STALK_UP_2ND, CRUISE_STALK_UP_1ST, CRUISE_STALK_IDLE],
    locked=True)
  assert not any(took)
  assert follow == []
  assert decode_bump(p.d[PARAM_GAP_LOCK_BUMP]) == (1, BUMP_FULL)


def test_card_not_locked_leaves_follow_path_alone():
  p = FakeParams(**{PARAM_GAP_LOCK_ACTIVE: False})
  bump = GapLockBumpCard()
  g = FollowStalkGesture()
  commits = []
  for d in (CRUISE_STALK_IDLE, CRUISE_STALK_UP_1ST, CRUISE_STALK_IDLE):
    locked = bump.refresh(p, d, False)
    before = g.saw_hold
    commit, closer, _ = g.update(has_lead=True, detent=d)
    assert not card_gap_lock_stalk(bump, p, gap_locked=locked, saw_hold_before=before,
                                   saw_hold_now=g.saw_hold, commit=commit, closer=closer)
    if commit:
      commits.append(closer)
  assert commits == [True]
  assert PARAM_GAP_LOCK_BUMP not in p.d


def test_card_reads_active_once_per_gesture():
  class CountingParams(FakeParams):
    reads = 0

    def get_bool(self, key, block=False):
      if key == PARAM_GAP_LOCK_ACTIVE:
        CountingParams.reads += 1
      return super().get_bool(key, block)

  p = CountingParams(**{PARAM_GAP_LOCK_ACTIVE: True})
  bump = GapLockBumpCard()
  for d in [CRUISE_STALK_IDLE] * 50 + [CRUISE_STALK_UP_1ST] * 5 + [CRUISE_STALK_IDLE] * 50:
    bump.refresh(p, d, False)
  assert CountingParams.reads == 1


def test_card_without_detent_reads_on_button_press():
  p = FakeParams(**{PARAM_GAP_LOCK_ACTIVE: True})
  bump = GapLockBumpCard()
  assert bump.refresh(p, None, False) is False
  assert bump.refresh(p, None, True) is True


def test_card_to_planner_end_to_end():
  """Two up tips and one down tip from card land as 28 → 27 → 26 → 27 m."""
  p = FakeParams()
  router = GapLockBumpRouter()
  latch = _locked_latch(28.0)
  _planner_frame(router, latch, p)
  assert p.d[PARAM_GAP_LOCK_ACTIVE] is True
  bump = GapLockBumpCard()
  g = FollowStalkGesture()
  seen = []
  for gesture in ([CRUISE_STALK_UP_1ST, CRUISE_STALK_IDLE],
                  [CRUISE_STALK_UP_1ST, CRUISE_STALK_IDLE],
                  [CRUISE_STALK_DN_1ST, CRUISE_STALK_IDLE]):
    for d in gesture:
      locked = bump.refresh(p, d, False)
      before = g.saw_hold
      commit, closer, _ = g.update(has_lead=locked, detent=d)
      card_gap_lock_stalk(bump, p, gap_locked=locked, saw_hold_before=before,
                          saw_hold_now=g.saw_hold, commit=commit, closer=closer)
      _planner_frame(router, latch, p, stalk_press=d != CRUISE_STALK_IDLE)
    seen.append(latch.gap_m)
  assert seen == [27.0, 26.0, 27.0]
  assert latch.track_id == 4


def test_decode_non_param_object_is_none():
  """A mock Params returns an arbitrary object, which int() could coerce."""
  class Coercible:
    def __int__(self):
      return 11
  assert decode_bump(Coercible()) == (0, BUMP_NONE)
