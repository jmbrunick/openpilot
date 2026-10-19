"""Planner hook: _refresh_gap_lock routes a locked tip to the meters, not a clear."""
import types
from types import SimpleNamespace

from cereal import car
from openpilot.selfdrive.controls.lib.gap_lock import GapLockLatch
from openpilot.selfdrive.controls.lib.gap_lock_bump import (
  BUMP_CLOSER,
  BUMP_FARTHER,
  BUMP_FULL,
  PARAM_GAP_LOCK_ACTIVE,
  PARAM_GAP_LOCK_BUMP,
  GapLockBumpRouter,
  encode_bump,
)
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

BE = car.CarState.ButtonEvent.Type


class FakeParams:
  def __init__(self):
    self.d = {}

  def get(self, key, block=False, return_default=False):
    return self.d.get(key)

  def get_bool(self, key, block=False):
    return bool(self.d.get(key, False))

  def put(self, key, val, block=False):
    self.d[key] = val

  def put_bool(self, key, val, block=False):
    self.d[key] = bool(val)


def _planner():
  p = SimpleNamespace(
    dt=0.05,
    _params=FakeParams(),
    _gap_lock=GapLockLatch(),
    _gap_bump=GapLockBumpRouter(),
    _gap_lock_enabled=True,
  )
  p._refresh_gap_lock = types.MethodType(LongitudinalPlanner._refresh_gap_lock, p)
  return p


def _sm(seq, press=None):
  events = [] if press is None else [SimpleNamespace(pressed=True, type=int(press))]
  cs = SimpleNamespace(enableLongControl=True, cruiseState=SimpleNamespace(enabled=True),
                       gapLockArmSeq=seq, gapLockHold=False, buttonEvents=events)
  return {"carState": cs}


LEAD = SimpleNamespace(status=True, radar=True, radarTrackId=7, dRel=30.0, yRel=0.0)


def _lock(p):
  for _ in range(10):
    p._refresh_gap_lock(_sm(0), LEAD)
  p._refresh_gap_lock(_sm(1), LEAD)
  assert p._gap_lock.gap_m == 30.0
  assert p._params.d[PARAM_GAP_LOCK_ACTIVE] is True


def test_planner_tip_steps_meters_and_keeps_lock():
  p = _planner()
  _lock(p)
  p._refresh_gap_lock(_sm(1, BE.accelCruise), LEAD)
  assert p._gap_lock.gap_m == 30.0
  p._params.d[PARAM_GAP_LOCK_BUMP] = encode_bump(1, BUMP_CLOSER)
  p._refresh_gap_lock(_sm(1), LEAD)
  assert p._gap_lock.gap_m == 29.0
  p._refresh_gap_lock(_sm(1, BE.decelCruise), LEAD)
  p._params.d[PARAM_GAP_LOCK_BUMP] = encode_bump(2, BUMP_FARTHER)
  p._refresh_gap_lock(_sm(1), LEAD)
  p._params.d[PARAM_GAP_LOCK_BUMP] = encode_bump(3, BUMP_FARTHER)
  p._refresh_gap_lock(_sm(1), LEAD)
  assert p._gap_lock.gap_m == 31.0


def test_planner_full_press_still_clears():
  p = _planner()
  _lock(p)
  p._refresh_gap_lock(_sm(1, BE.accelCruise), LEAD)
  p._params.d[PARAM_GAP_LOCK_BUMP] = encode_bump(1, BUMP_FULL)
  p._refresh_gap_lock(_sm(1), LEAD)
  assert p._gap_lock.gap_m is None
  assert p._params.d[PARAM_GAP_LOCK_ACTIVE] is False
