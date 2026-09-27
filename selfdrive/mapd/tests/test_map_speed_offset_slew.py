"""Map-speed offset is applied once. Slew state stays in raw-limit units.

Engage / posted-raise seeds are posted+offset on the pedal, but `_map_slew_ms`
is the raw OSM limit because `apply_map_speed_kph` adds the offset again.
A self-written pedal step must not be read as a driver stalk press.
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


def _harness(offset_kph: float, limit_kph: float, v_ego_kph: float):
  md = SimpleNamespace(
    speedLimit=float(limit_kph) * CV.KPH_TO_MS,
    speedLimitValid=True,
    nextSpeedLimit=0.0,
    nextSpeedLimitDistance=0.0,
  )
  v_ego_ms = float(v_ego_kph) * CV.KPH_TO_MS
  cs = SimpleNamespace(
    cruiseState=SimpleNamespace(speed=v_ego_ms, enabled=True),
    pedalLongActive=True,
    enableLongControl=True,
    vEgo=v_ego_ms,
    steeringAngleDeg=0.0,
    buttonEvents=[],
  )
  h = SimpleNamespace(
    CS_prev=SimpleNamespace(pedalLongActive=False, cruiseState=SimpleNamespace(speed=v_ego_ms)),
    CI=None,
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
    _map_speed_user_offset_kph=float(offset_kph),
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
  return h, cs, md


def _step(h, cs, *, first: bool) -> None:
  h.CS_prev.pedalLongActive = False if first else True
  Car._update_preap_map_cruise(h, cs)


def _run_s(h, cs, seconds: float, *, first: bool = True) -> None:
  n = max(1, int(round(float(seconds) / DT_CTRL)))
  for i in range(n):
    _step(h, cs, first=first and i == 0)


def _expect_max(h, expected_kph: float) -> None:
  assert h._map_hold.sticky_set_kph is None
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(expected_kph, abs=0.05)


@pytest.mark.parametrize("offset_mph", range(0, 11))
def test_engage_mph_offset_is_limit_plus_offset_after_1s(offset_mph):
  """55 mph engage: MAX is exactly limit+offset after 1 s, and not a sticky stalk set."""
  limit_kph = 55.0 * CV.MPH_TO_KPH
  offset_kph = float(offset_mph) * CV.MPH_TO_KPH
  h, cs, _md = _harness(offset_kph, limit_kph, limit_kph)
  # 0.2 s is before a double-added 1 mph/1 kph offset can slew back to the limit.
  _run_s(h, cs, 0.2)
  _expect_max(h, limit_kph + offset_kph)
  _run_s(h, cs, 0.8, first=False)
  _expect_max(h, limit_kph + offset_kph)


@pytest.mark.parametrize("offset_kph", [0.0, 1.0, 5.0, 10.0])
def test_engage_kph_offset_is_limit_plus_offset_after_1s(offset_kph):
  """Same contract in kph. 1 and 5 kph are stalk steps and must not latch sticky."""
  limit_kph = 80.0
  h, cs, _md = _harness(float(offset_kph), limit_kph, limit_kph)
  _run_s(h, cs, 0.2)
  _expect_max(h, limit_kph + float(offset_kph))
  _run_s(h, cs, 0.8, first=False)
  _expect_max(h, limit_kph + float(offset_kph))


def test_posted_raise_keeps_single_offset():
  """55 → 65 with +5 mph lands on 70, not 75."""
  offset_kph = 5.0 * CV.MPH_TO_KPH
  limit_55 = 55.0 * CV.MPH_TO_KPH
  limit_65 = 65.0 * CV.MPH_TO_KPH
  h, cs, md = _harness(offset_kph, limit_55, limit_55)
  _run_s(h, cs, 1.0)
  _expect_max(h, limit_55 + offset_kph)
  md.speedLimit = limit_65 * CV.KPH_TO_MS
  _run_s(h, cs, 1.0, first=False)
  _expect_max(h, limit_65 + offset_kph)


def test_self_written_pedal_step_is_not_a_stalk():
  """A pedal jump this code just wrote is not a driver stalk press."""
  offset_kph = 5.0 * CV.MPH_TO_KPH
  limit_kph = 55.0 * CV.MPH_TO_KPH
  h, cs, _md = _harness(offset_kph, limit_kph, limit_kph)
  _run_s(h, cs, 1.0)
  held = float(h.v_cruise_helper.v_cruise_kph)
  jumped = held + 5.0 * CV.MPH_TO_KPH
  h._pedal_self_write_kph = jumped
  cs.cruiseState.speed = jumped * CV.KPH_TO_MS
  _step(h, cs, first=False)
  assert h._map_hold.sticky_set_kph is None
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(held, abs=0.05)


def test_driver_stalk_step_still_latches_follow():
  """A real 5 mph stalk step, not a self-write, still latches Follow."""
  offset_kph = 5.0 * CV.MPH_TO_KPH
  limit_kph = 55.0 * CV.MPH_TO_KPH
  h, cs, _md = _harness(offset_kph, limit_kph, limit_kph)
  _run_s(h, cs, 1.0)
  held = float(h.v_cruise_helper.v_cruise_kph)
  stalk = held + 5.0 * CV.MPH_TO_KPH
  h._pedal_self_write_kph = None
  cs.cruiseState.speed = stalk * CV.KPH_TO_MS
  _step(h, cs, first=False)
  assert h._map_hold.sticky_set_kph is not None
  assert h._map_hold.sticky_set_kph == pytest.approx(stalk, abs=0.05)
  assert h.v_cruise_helper.v_cruise_kph == pytest.approx(stalk, abs=0.05)
