"""Skip ONNX while OP is engaged; allow it while driving manually (moving OK)."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import (
  detect_if_allowed,
  engaged_from_sm,
  op_engaged,
  should_run_onnx_detect,
  should_run_speed_sign_log,
)


class _RaiseIfDetect:
  def detect(self, y, min_conf=None, rgb=None):
    raise AssertionError("ONNX detect must not run while OP is engaged")


class _HitDetect:
  def __init__(self):
    self.calls = 0

  def detect(self, y, min_conf=None, rgb=None):
    self.calls += 1
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 8, 8))]


class _FakeParams:
  def __init__(self, enabled=False):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled


def test_logger_on_still_starts_process_when_onroad():
  assert should_run_speed_sign_log(True, _FakeParams(True))
  assert not should_run_speed_sign_log(True, _FakeParams(False))


def test_no_onnx_while_op_engaged():
  assert not should_run_onnx_detect(True)
  assert should_run_onnx_detect(False)


def test_unknown_engagement_is_fail_safe_engaged():
  assert op_engaged(None) is True
  assert not should_run_onnx_detect(op_engaged(None))


def test_only_selfdrive_enabled_is_the_gate():
  assert op_engaged(True) is True
  assert op_engaged(False) is False


def test_detect_if_allowed_skips_onnx_and_jsonl_when_engaged(tmp_path):
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, _RaiseIfDetect(), log, now=1.0, engaged=True,
  )
  assert signs == [] and written == []
  assert not (tmp_path / "out.jsonl").exists()


def test_detect_if_allowed_runs_when_disengaged_moving(tmp_path):
  """Product: log signs while driving manually. Moving / not parked is OK."""
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=1.0, engaged=False,
  )
  assert det.calls == 1
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55


class _Sm:
  def __init__(self, recv: dict[str, int], services: dict[str, object]):
    self.recv_frame = recv
    self._services = services

  def __getitem__(self, name: str):
    return self._services[name]


def test_engaged_from_sm_fail_safe_until_cereal():
  sm = _Sm({}, {})
  assert engaged_from_sm(sm) is True
  assert not should_run_onnx_detect(engaged_from_sm(sm))


def test_engaged_from_sm_stale_or_invalid_is_fail_safe():
  sm = _Sm({"selfdriveState": 9}, {"selfdriveState": SimpleNamespace(enabled=False)})
  sm.alive = {"selfdriveState": False}
  assert engaged_from_sm(sm) is True
  sm = _Sm({"selfdriveState": 9}, {"selfdriveState": SimpleNamespace(enabled=False)})
  sm.valid = {"selfdriveState": False}
  assert engaged_from_sm(sm) is True


def test_engaged_from_sm_selfdrive_enabled():
  sm = _Sm({"selfdriveState": 3}, {"selfdriveState": SimpleNamespace(enabled=True)})
  assert engaged_from_sm(sm) is True
  sm = _Sm({"selfdriveState": 3}, {"selfdriveState": SimpleNamespace(enabled=False)})
  assert engaged_from_sm(sm) is False
  assert should_run_onnx_detect(engaged_from_sm(sm))


def test_stock_cruise_or_moving_does_not_block_detect():
  """Manual driving (stock CC on, moving) still runs YOLO when OP is off."""
  cruise_on = SimpleNamespace(cruiseState=SimpleNamespace(enabled=True), vEgo=22.0)
  sm = _Sm(
    {"selfdriveState": 4, "carState": 2},
    {"selfdriveState": SimpleNamespace(enabled=False), "carState": cruise_on},
  )
  assert engaged_from_sm(sm) is False
  assert should_run_onnx_detect(engaged_from_sm(sm))
  # carState-only (no selfdriveState yet) stays fail-safe skipped.
  sm = _Sm({"carState": 2}, {"carState": cruise_on})
  assert engaged_from_sm(sm) is True


def test_detect_gate_is_not_parked_or_force_offroad():
  from pathlib import Path
  src = Path(__file__).resolve().parents[1] / "speedsignd.py"
  text = src.read_text(encoding="utf-8")
  assert "NAPForceOffroad" not in text
  assert "standstill" not in text
  assert "carState" not in text
  assert "selfdriveState" in text
