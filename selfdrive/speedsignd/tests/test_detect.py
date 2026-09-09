"""Tests for MUTCD numeric speed-sign detection (no modelV2 head)."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import (
  MUTCD_MPH,
  OnnxSpeedSignDetector,
  SpeedSignDetector,
  detect_mutcd_speed_signs,
  paint_mutcd_r2_1,
  y_plane_from_nv12,
)


def _scene(h=240, w=320, seed=0) -> np.ndarray:
  rng = np.random.default_rng(seed)
  y = np.full((h, w), 70, np.uint8)
  y += rng.integers(0, 12, size=(h, w), dtype=np.uint8)
  return y


def test_mutcd_numeric_sign_is_read():
  y = _scene()
  paint_mutcd_r2_1(y, 45, x=200, y=30, w=90, h=112)
  hits = detect_mutcd_speed_signs(y)
  assert hits, "expected a MUTCD 45 mph detection"
  assert hits[0].mph == 45
  assert hits[0].conf >= 0.42


def test_several_mutcd_speeds():
  for mph in (15, 25, 30, 40, 45, 55, 60, 70, 80):
    y = _scene(seed=mph)
    paint_mutcd_r2_1(y, mph, x=190, y=24, w=96, h=120)
    hits = detect_mutcd_speed_signs(y)
    assert hits, f"missed {mph}"
    assert hits[0].mph == mph


def test_empty_road_is_not_a_sign():
  y = _scene(seed=99)
  assert detect_mutcd_speed_signs(y) == []


def test_license_plate_aspect_is_rejected():
  y = _scene()
  # Wide white plate (license-like), not R2-1.
  y[160:190, 80:220] = 230
  y[168:182, 95:110] = 20
  y[168:182, 120:140] = 20
  y[168:182, 150:170] = 20
  assert detect_mutcd_speed_signs(y) == []


def test_detector_falls_back_without_onnx(tmp_path, monkeypatch):
  missing = str(tmp_path / "nope.onnx")
  monkeypatch.setenv("NAP_SPEED_SIGN_ONNX", missing)
  det = SpeedSignDetector(onnx_path=missing)
  assert det.onnx is None
  y = _scene()
  paint_mutcd_r2_1(y, 35, x=200, y=30, w=90, h=112)
  hits = det.detect(y)
  assert hits and hits[0].mph == 35


def test_onnx_parse_and_mutcd_filter():
  class Sess:
    def get_inputs(self):
      return [type("I", (), {"name": "image", "shape": [1, 1, 64, 64]})()]

    def run(self, _out, _feed):
      return [np.array([[10.0, 10.0, 40.0, 50.0, 45.0, 0.9]], dtype=np.float32)]

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Sess())
  y = np.zeros((64, 64), np.uint8)
  hits = det.detect(y)
  assert len(hits) == 1
  assert hits[0].mph == 45
  assert hits[0].conf == pytest.approx(0.9, abs=1e-5)


def test_onnx_rejects_non_mutcd_mph():
  class Sess:
    def get_inputs(self):
      return [type("I", (), {"name": "image", "shape": [1, 1, 32, 32]})()]

    def run(self, _out, _feed):
      return [np.array([[0.0, 0.0, 10.0, 10.0, 47.0, 0.99]], dtype=np.float32)]

  hits = OnnxSpeedSignDetector("x", Sess()).detect(np.zeros((32, 32), np.uint8))
  assert hits == []


def test_y_plane_from_nv12():
  class Buf:
    width = 4
    height = 2
    stride = 4
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8]) + bytes(8)

  y = y_plane_from_nv12(Buf())
  assert y is not None
  assert y.shape == (2, 4)
  assert int(y[0, 0]) == 1


def test_mutcd_set_is_us_r2_1():
  assert 15 in MUTCD_MPH and 25 in MUTCD_MPH and 70 in MUTCD_MPH
  assert 47 not in MUTCD_MPH
