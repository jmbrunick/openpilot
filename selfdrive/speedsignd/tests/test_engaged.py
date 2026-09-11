"""Logger On must not run YOLO while the driving stack is engaged."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import (
  detect_if_allowed,
  driving_controls_engaged,
  engaged_from_sm,
  should_run_onnx_detect,
  should_run_speed_sign_log,
)


class _RaiseIfDetect:
  def detect(self, y, min_conf=None, rgb=None):
    raise AssertionError("ONNX detect must not run while engaged")


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


def test_no_onnx_while_engaged():
  assert not should_run_onnx_detect(True)
  assert should_run_onnx_detect(False)


def test_unknown_engagement_is_fail_safe_engaged():
  assert driving_controls_engaged(selfdrive_enabled=None, cruise_enabled=None) is True
  assert not should_run_onnx_detect(driving_controls_engaged(selfdrive_enabled=None, cruise_enabled=None))


def test_selfdrive_or_cruise_counts_as_engaged():
  assert driving_controls_engaged(selfdrive_enabled=True, cruise_enabled=False) is True
  assert driving_controls_engaged(selfdrive_enabled=False, cruise_enabled=True) is True
  assert driving_controls_engaged(selfdrive_enabled=True, cruise_enabled=True) is True
  assert driving_controls_engaged(selfdrive_enabled=False, cruise_enabled=False) is False
  assert driving_controls_engaged(selfdrive_enabled=False, cruise_enabled=None) is False
  assert driving_controls_engaged(selfdrive_enabled=None, cruise_enabled=False) is False


def test_detect_if_allowed_skips_onnx_and_jsonl_when_engaged(tmp_path):
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, _RaiseIfDetect(), log, now=1.0, engaged=True,
  )
  assert signs == [] and written == []
  assert not (tmp_path / "out.jsonl").exists()


def test_detect_if_allowed_runs_when_disengaged(tmp_path):
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


def test_engaged_from_sm_cruise_enabled():
  cruise_on = SimpleNamespace(cruiseState=SimpleNamespace(enabled=True))
  cruise_off = SimpleNamespace(cruiseState=SimpleNamespace(enabled=False))
  sm = _Sm({"carState": 2}, {"carState": cruise_on})
  assert engaged_from_sm(sm) is True
  sm = _Sm(
    {"selfdriveState": 4, "carState": 2},
    {"selfdriveState": SimpleNamespace(enabled=False), "carState": cruise_on},
  )
  assert engaged_from_sm(sm) is True
  sm = _Sm(
    {"selfdriveState": 4, "carState": 2},
    {"selfdriveState": SimpleNamespace(enabled=False), "carState": cruise_off},
  )
  assert engaged_from_sm(sm) is False
  assert should_run_onnx_detect(engaged_from_sm(sm))
