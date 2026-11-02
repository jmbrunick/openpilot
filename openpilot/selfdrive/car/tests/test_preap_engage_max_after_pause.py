"""MAX on the first frame of a one-SET resume after a One-Pedal gas pause.

Sep 25 10:43:32–10:43:59 CT: MAX 70, gas takeover paused long, the driver
took a highway exit and a sharp low-speed turn on gas (map dropped out),
then accelerated to ~39 mph and pressed SET with the gas still down.
The temporary curve cap had pulled HUD MAX to 14.4 mph during the pause
and was ramping back at the restore rate. On the resume frame that ramp
value overrode the held-MAX seed and was written onto the pedal set
speed: MAX published 16.7 mph at 39 mph.

The curve cap exists to slow OP's own long control through a bend. While
long is paused the driver owns speed, so the cap must not rewrite MAX,
and a resume must publish the held MAX (dropout holds MAX) on its first
frame.
"""
import types
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL
from openpilot.common.constants import CV
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.curve_max_hold import CurveMaxHold
from openpilot.selfdrive.mapd.constants import ACCEL_DEFAULT, LOOKAHEAD_NORMAL, MODE_FOLLOW
from openpilot.selfdrive.mapd.map_speed_policy import MapCruiseHold

MPH = CV.MPH_TO_KPH


class _Cruise:
  def __init__(self):
    self.v_cruise_kph = float(V_CRUISE_UNSET)
    self.v_cruise_kph_last = 0.0
    self.v_cruise_cluster_kph = float(V_CRUISE_UNSET)


class _MapSm:
  def __init__(self, md):
    self.valid = {"liveMapDataNAP": True}
    self._md = md

  def __getitem__(self, key):
    if key == "liveMapDataNAP":
      return self._md
    raise KeyError(key)


def _harness(limit_mph: float):
  md = SimpleNamespace(
    speedLimit=float(limit_mph) * CV.MPH_TO_MS,
    speedLimitValid=True,
    nextSpeedLimit=0.0,
    nextSpeedLimitDistance=0.0,
  )
  v = float(limit_mph) * CV.MPH_TO_MS
  eng = SimpleNamespace(
    _nap_set_resume_long=False, _nap_set_take_speed_now=False,
    _nap_held_max_kph=None, pedal_speed_kph=0.0,
  )
  inner = SimpleNamespace(engagement=eng, pedal_speed_kph=0.0)
  cs = SimpleNamespace(
    cruiseState=SimpleNamespace(speed=v, enabled=True),
    pedalLongActive=True, enableLongControl=True,
    vEgo=v, steeringAngleDeg=0.0, buttonEvents=[],
  )
  h = SimpleNamespace(
    CS_prev=SimpleNamespace(pedalLongActive=False, cruiseState=SimpleNamespace(speed=v)),
    CI=SimpleNamespace(CS=inner),
    CP=SimpleNamespace(steerRatio=15.0, wheelbase=2.98),
    sm=_MapSm(md),
    _map_hold=MapCruiseHold(),
    _curve_max=CurveMaxHold(),
    _map_slew_ms=None,
    _last_pedal_kph=None,
    _pedal_self_write_kph=None,
    _map_speed_mode=MODE_FOLLOW,
    _map_speed_lookahead=LOOKAHEAD_NORMAL,
    _map_speed_accel=ACCEL_DEFAULT,
    _map_speed_user_offset_kph=0.0,
    _map_hypermile_on=False,
    _map_step_down_on=False,
    v_cruise_helper=_Cruise(),
    _maybe_follow_stalk=lambda _cs, raw_kph: (raw_kph, False),
  )
  for name in (
    "_preap_engagement", "_preap_set_events", "_adopt_preap_fsm_held_max",
    "_publish_preap_held_max", "_curve_cornering", "_live_map_offset_kph",
    "_write_preap_pedal_speed",
  ):
    setattr(h, name, types.MethodType(getattr(Car, name), h))
  h._preap_stalk_set_pressed = Car._preap_stalk_set_pressed
  return h, cs, md, eng


def _frame(h, cs):
  Car._update_preap_map_cruise(h, cs)
  h.CS_prev.pedalLongActive = cs.pedalLongActive
  h.CS_prev.cruiseState.speed = cs.cruiseState.speed


def _run(h, cs, seconds, v_from_mph=None, v_to_mph=None, steer=None):
  n = max(1, int(round(seconds / DT_CTRL)))
  for i in range(n):
    if v_from_mph is not None:
      cs.vEgo = (v_from_mph + (v_to_mph - v_from_mph) * (i + 1) / n) * CV.MPH_TO_MS
    if steer is not None:
      cs.steeringAngleDeg = steer(i / n)
    if not cs.enableLongControl:
      # Pause publishes cruiseState.speed as 0 / ego, never MAX.
      cs.cruiseState.speed = 0.0
    _frame(h, cs)


def _pause_turn_and_resume_with_gas(h, cs, md, eng):
  # Engaged at a 70 mph posted limit (Follow): MAX 70.
  _run(h, cs, 2.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)
  # Gas takeover: One-Pedal pause (software long off, session and lat on).
  cs.enableLongControl = False
  cs.pedalLongActive = False
  _run(h, cs, 1.0, 48.0, 45.0)
  # Exit ramp bend on gas, then decelerating into a sharp turn.
  _run(h, cs, 2.0, 45.0, 30.0, steer=lambda _x: 20.0)
  _run(h, cs, 2.0, 30.0, 12.0, steer=lambda _x: 0.0)
  # Map drops out in the sharp low-speed turn (steer ~130°).
  md.speedLimitValid = False
  h.sm.valid["liveMapDataNAP"] = False
  _run(h, cs, 5.0, 11.0, 13.0, steer=lambda _x: 135.0)
  # Straighten and accelerate on gas to ~39 mph.
  _run(h, cs, 1.5, 13.0, 22.0, steer=lambda x: 135.0 * (1.0 - x))
  _run(h, cs, 4.5, 22.0, 39.0, steer=lambda _x: 0.0)
  # One SET with gas still pressed: the FSM restores its held MAX onto the
  # pedal and flags resume-held for card on this frame.
  held = eng._nap_held_max_kph
  eng._nap_set_resume_long = True
  if held is not None:
    eng.pedal_speed_kph = float(held)
  cs.enableLongControl = True
  cs.pedalLongActive = False  # authority acquires on gas lift
  cs.cruiseState.speed = float(eng.pedal_speed_kph) * CV.KPH_TO_MS
  _frame(h, cs)
  return held


def test_resume_with_gas_after_pause_turn_publishes_held_max_first_frame():
  h, cs, md, eng = _harness(70.0)
  held = _pause_turn_and_resume_with_gas(h, cs, md, eng)
  # Map dropout holds MAX: the held 70 comes back, on the first frame.
  assert held == pytest.approx(70.0 * MPH, abs=0.3)
  first = h.v_cruise_helper.v_cruise_kph
  assert first == pytest.approx(70.0 * MPH, abs=0.3)
  assert cs.cruiseState.speed * CV.MS_TO_KPH == pytest.approx(70.0 * MPH, abs=0.3)
  assert eng._nap_held_max_kph == pytest.approx(70.0 * MPH, abs=0.3)
  # And it stays there (no stale seed a frame later).
  cs.pedalLongActive = False
  _run(h, cs, 0.5)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)


def test_paused_long_curve_does_not_rewrite_hud_max():
  """The driver owns speed in a pause. A bend does not drag HUD MAX down."""
  h, cs, md, eng = _harness(70.0)
  _run(h, cs, 2.0)
  cs.enableLongControl = False
  cs.pedalLongActive = False
  lowest = float("inf")
  n = int(round(3.0 / DT_CTRL))
  for _ in range(n):
    cs.vEgo = 12.0 * CV.MPH_TO_MS
    cs.steeringAngleDeg = 135.0
    cs.cruiseState.speed = 0.0
    _frame(h, cs)
    lowest = min(lowest, h.v_cruise_helper.v_cruise_kph)
  assert lowest == pytest.approx(70.0 * MPH, abs=0.3)
  assert eng._nap_held_max_kph == pytest.approx(70.0 * MPH, abs=0.3)


def test_engaged_curve_still_caps_and_restores():
  """Under OP long the temporary bend cap still lowers MAX and restores it."""
  h, cs, md, eng = _harness(70.0)
  _run(h, cs, 2.0)
  _run(h, cs, 2.0, 40.0, 40.0, steer=lambda _x: 30.0)
  assert h.v_cruise_helper.v_cruise_kph < 60.0 * MPH
  _run(h, cs, 40.0, 40.0, 40.0, steer=lambda _x: 0.0)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(70.0 * MPH, abs=0.3)
