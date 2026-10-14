"""Oct 14 path-object brake: LOG-ONLY wiring in the longitudinal planner."""
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]


class _Params:
  def __init__(self, on=False):
    self.on = on

  def get_bool(self, key):
    assert key == "NAPObstacleBrake"
    return self.on


class _SM(dict):
  def __init__(self, obs, v=20.0, gas=False, age_s=0.1):
    super().__init__(carState=SimpleNamespace(vEgo=v, gasPressed=gas), pathObstacleVisionNAP=obs)
    self.seen = {"pathObstacleVisionNAP": obs is not None}
    self.logMonoTime = {"carState": int(10e9), "pathObstacleVisionNAP": int(10e9 - age_s * 1e9)}


def _obs(lat=0.2, agree=True, cls="animal", rng=35.0):
  return SimpleNamespace(agree=agree, visionEvaluated=True, rangeM=rng, lateral=lat, spanM=0.6,
                         objectClass=cls, vRel=-20.0)


def _planner(on=False):
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  p = LongitudinalPlanner.__new__(LongitudinalPlanner)
  p._params = _Params(on)
  p._frame = 0
  return p


def _log(sm, on=False):
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  return LongitudinalPlanner._obstacle_brake_log(_planner(on), sm)


def test_brake_zone_logs_strong_decel_up_to_the_stack_floor():
  from opendbc.car.interfaces import ACCEL_MIN
  would, decel, mode = _log(_SM(_obs()))
  assert would is True and decel == pytest.approx(ACCEL_MIN) and mode == 0


def test_gates():
  assert _log(_SM(_obs(agree=False)))[0] is False      # radar AND vision
  assert _log(_SM(_obs(), v=5.0))[0] is False           # >= 15 mph
  assert _log(_SM(_obs(), gas=True))[0] is False        # gas overrides
  assert _log(_SM(_obs(), age_s=2.0))[0] is False       # stale
  assert _log(_SM(_obs(lat=2.0)))[0] is False           # steer zone never brakes
  assert _log(_SM(None))[0] is False


def test_toggle_defaults_log_only_and_is_reported():
  keys = (ROOT / "common/params_keys.h").read_text()
  assert '{"NAPObstacleBrake", {PERSISTENT, BOOL, "0"}}' in keys
  assert _log(_SM(_obs()), on=True)[2] == 1


def test_never_commanded():
  src = (ROOT / "selfdrive/controls/lib/longitudinal_planner.py").read_text()
  body = src[src.index("def _obstacle_brake_log"):src.index("  def publish")]
  for word in ("a_desired", "output_a_target", "accel_clip", "mpc", "should_stop", "v_cruise"):
    assert word not in body
  pub = src[src.index("  def publish"):]
  assert "obstacleBrakeDecel = float(decel)" in pub
  log = (ROOT / "cereal/log.capnp").read_text()
  for f in ("obstacleWouldBrake @45 :Bool;", "obstacleBrakeDecel @46 :Float32;", "obstacleBrakeMode @47 :UInt8;"):
    assert f in log
