"""card: with gap lock engaged, an up/down tip goes to the lock, not MAX or Follow Distance."""
import types
from types import SimpleNamespace

from openpilot.common.constants import CV
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.lib.gap_lock_bump import (
  BUMP_CLOSER,
  BUMP_FARTHER,
  BUMP_FULL,
  PARAM_GAP_LOCK_ACTIVE,
  PARAM_GAP_LOCK_BUMP,
  GapLockBumpCard,
  decode_bump,
)
from openpilot.selfdrive.controls.lib.hypermile import (
  CRUISE_STALK_DN_1ST,
  CRUISE_STALK_IDLE,
  CRUISE_STALK_UP_1ST,
  CRUISE_STALK_UP_2ND,
  FollowStalkGesture,
)

MAX_MPH = 60.0


class FakeParams:
  def __init__(self, **init):
    self.d = dict(init)

  def get(self, key, block=False, return_default=False):
    return self.d.get(key, 4 if return_default else None)

  def get_bool(self, key, block=False):
    return bool(self.d.get(key, False))

  def put(self, key, val, block=False):
    self.d[key] = val

  def put_bool(self, key, val, block=False):
    self.d[key] = bool(val)


class _Sm:
  def __init__(self, lead: bool):
    self.valid = {"radarState": True}
    self._rs = SimpleNamespace(leadOne=SimpleNamespace(status=lead))

  def __getitem__(self, key):
    assert key == "radarState"
    return self._rs


def _harness(*, locked: bool, lead: bool):
  inner = SimpleNamespace(cruise_buttons=CRUISE_STALK_IDLE, pedal_speed_kph=MAX_MPH * CV.MPH_TO_KPH)
  h = SimpleNamespace(
    sm=_Sm(lead),
    CS_prev=SimpleNamespace(cruiseState=SimpleNamespace(speed=MAX_MPH * CV.MPH_TO_MS)),
    CI=SimpleNamespace(CS=inner),
    params=FakeParams(**{PARAM_GAP_LOCK_ACTIVE: locked}),
    _follow_gesture=FollowStalkGesture(),
    _gap_bump=GapLockBumpCard(),
    _follow_stalk_mono=0.0,
    _last_pedal_kph=None,
    _pedal_self_write_kph=None,
    v_cruise_helper=SimpleNamespace(v_cruise_kph=MAX_MPH * CV.MPH_TO_KPH),
  )
  for name in ("_maybe_follow_stalk", "_preap_cruise_detent", "_write_preap_pedal_speed"):
    setattr(h, name, types.MethodType(getattr(Car, name), h))
  return h


def _frame(h, detent, raw_mph):
  h.CI.CS.cruise_buttons = detent
  cs = SimpleNamespace(
    enableLongControl=True,
    buttonEvents=[],
    vEgo=MAX_MPH * CV.MPH_TO_MS,
    cruiseState=SimpleNamespace(speed=raw_mph * CV.MPH_TO_MS),
  )
  out_kph, routed = h._maybe_follow_stalk(cs, raw_mph * CV.MPH_TO_KPH)
  h.CS_prev.cruiseState.speed = out_kph * CV.KPH_TO_MS
  return out_kph * CV.KPH_TO_MPH, routed


def _follow_keys(params):
  return {k: v for k, v in params.d.items() if k.startswith("NAPFollow")}


def test_locked_tip_up_undoes_max_and_bumps_closer():
  h = _harness(locked=True, lead=True)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  mph, routed = _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
  assert routed
  assert round(mph, 3) == MAX_MPH
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  assert decode_bump(h.params.d[PARAM_GAP_LOCK_BUMP])[1] == BUMP_CLOSER
  assert _follow_keys(h.params) == {}


def test_locked_tip_down_bumps_farther():
  h = _harness(locked=True, lead=True)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  mph, _ = _frame(h, CRUISE_STALK_DN_1ST, MAX_MPH - 1)
  assert round(mph, 3) == MAX_MPH
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  assert decode_bump(h.params.d[PARAM_GAP_LOCK_BUMP])[1] == BUMP_FARTHER
  assert _follow_keys(h.params) == {}


def test_locked_tip_without_radar_lead_still_leaves_max():
  """Lock is holding through a brief radar dropout: the tip still is not a MAX step."""
  h = _harness(locked=True, lead=False)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  mph, _ = _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
  assert round(mph, 3) == MAX_MPH
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  assert decode_bump(h.params.d[PARAM_GAP_LOCK_BUMP])[1] == BUMP_CLOSER


def test_locked_full_press_reports_full_and_keeps_plus_five():
  h = _harness(locked=True, lead=True)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
  mph, _ = _frame(h, CRUISE_STALK_UP_2ND, MAX_MPH + 5)
  assert round(mph, 3) == MAX_MPH + 5
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH + 5)
  assert decode_bump(h.params.d[PARAM_GAP_LOCK_BUMP])[1] == BUMP_FULL
  assert _follow_keys(h.params) == {}


def test_unlocked_tip_with_lead_still_steps_follow_distance():
  h = _harness(locked=False, lead=True)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  assert PARAM_GAP_LOCK_BUMP not in h.params.d
  assert h.params.d.get("NAPFollowDistanceHwy") == 3


def test_unlocked_tip_without_lead_still_steps_max():
  h = _harness(locked=False, lead=False)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  mph, routed = _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
  assert not routed
  assert round(mph, 3) == MAX_MPH + 1
  assert PARAM_GAP_LOCK_BUMP not in h.params.d


def test_locked_quick_double_tip_is_two_bumps():
  h = _harness(locked=True, lead=True)
  _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
  seqs = []
  for _ in range(2):
    _frame(h, CRUISE_STALK_UP_1ST, MAX_MPH + 1)
    _frame(h, CRUISE_STALK_IDLE, MAX_MPH)
    seqs.append(decode_bump(h.params.d[PARAM_GAP_LOCK_BUMP]))
  assert seqs == [(1, BUMP_CLOSER), (2, BUMP_CLOSER)]
