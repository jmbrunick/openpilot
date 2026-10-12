"""YOLOv8 ONNX interface: class map, letterbox, decode, no numpy fallback."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector, SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene
from openpilot.selfdrive.speedsignd.weights_manifest import YOLO_CLASS_NAMES, YOLO_MIN_CONF
from openpilot.selfdrive.speedsignd.yolo import class_to_mph, decode_yolov8, letterbox_rgb, nms_xyxy, refine_mph


def test_class_to_mph_reads_mutcd_r2_1_only():
  assert class_to_mph("speedLimit55") == 55
  assert class_to_mph("speedLimit60") == 60
  assert class_to_mph("speedLimit25") == 25
  assert class_to_mph("stop") is None
  assert class_to_mph("yield") is None
  assert class_to_mph("speedLimit47") is None
  assert class_to_mph("doNotEnter") is None


def test_yolo_class_list_covers_highway_speeds():
  names = set(YOLO_CLASS_NAMES)
  for mph in (15, 25, 30, 45, 55, 60, 70, 80):
    assert f"speedLimit{mph}" in names
  assert YOLO_CLASS_NAMES[12] == "speedLimit55"
  assert YOLO_CLASS_NAMES[13] == "speedLimit60"


def test_letterbox_keeps_aspect_and_pads():
  rgb = np.zeros((120, 320, 3), np.uint8)
  rgb[:, :] = (10, 20, 30)
  boxed, scale, pad_x, pad_y = letterbox_rgb(rgb, 320)
  assert boxed.shape == (320, 320, 3)
  assert scale == pytest.approx(1.0)
  assert pad_x == 0
  assert pad_y == 100
  assert boxed[pad_y, 10, 0] == 10


def test_nms_keeps_highest_score():
  boxes = np.array([
    [0, 0, 10, 10],
    [1, 1, 11, 11],
    [50, 50, 60, 60],
  ], dtype=np.float32)
  scores = np.array([0.4, 0.9, 0.8], dtype=np.float32)
  keep = nms_xyxy(boxes, scores, iou_thr=0.3, max_det=3)
  assert keep[0] == 1
  assert 2 in keep
  assert 0 not in keep


def test_decode_yolov8_speed_limit_55():
  raw = np.zeros((1, 25, 8), np.float32)
  raw[0, 0, 0] = 160.0
  raw[0, 1, 0] = 120.0
  raw[0, 2, 0] = 80.0
  raw[0, 3, 0] = 100.0
  raw[0, 4 + 12, 0] = 0.91  # speedLimit55
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1
  assert hits[0].mph == 55
  assert hits[0].conf == pytest.approx(0.91, abs=1e-5)
  x, y, w, h = hits[0].bbox
  assert x == 120 and y == 70
  assert w == 80 and h == 100


def test_decode_yolov8_ignores_stop_and_low_conf():
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, :4, 0] = [80, 80, 40, 40]
  raw[0, 4 + 19, 0] = 0.99  # stop
  raw[0, :4, 1] = [200, 80, 40, 40]
  raw[0, 4 + 13, 1] = 0.2  # speedLimit60 below threshold
  assert decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4) == []


class _YoloSess:
  def get_inputs(self):
    return [type("I", (), {"name": "images", "shape": [1, 3, 320, 320]})()]

  def run(self, _out, feed):
    blob = next(iter(feed.values()))
    assert blob.shape == (1, 3, 320, 320)
    raw = np.zeros((1, 25, 4), np.float32)
    raw[0, 0, 0] = 200.0
    raw[0, 1, 0] = 80.0
    raw[0, 2, 0] = 60.0
    raw[0, 3, 0] = 80.0
    raw[0, 4 + 13, 0] = 0.88  # speedLimit60
    return [raw]


def test_onnx_yolo_session_returns_60():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess())
  y = np.zeros((320, 320), np.uint8)
  hits = det.detect(y, min_conf=0.4)
  assert hits and hits[0].mph == 60
  assert hits[0].conf == pytest.approx(0.88, abs=1e-5)


def test_loaded_onnx_does_not_fall_back_to_numpy(tmp_path):
  class EmptyYolo(_YoloSess):
    def run(self, _out, _feed):
      return [np.zeros((1, 25, 4), np.float32)]

  y = _scene()
  paint_mutcd_r2_1(y, 45, x=200, y=30, w=90, h=112)
  det = SpeedSignDetector(onnx=OnnxSpeedSignDetector(str(tmp_path / "x.onnx"), EmptyYolo()))
  assert det.onnx is not None
  assert det.detect(y) == []


def test_refine_overrides_wrong_yolo_class_on_clear_crop():
  from openpilot.selfdrive.speedsignd.detect import _read_mph
  y = _scene()
  paint_mutcd_r2_1(y, 55, x=200, y=30, w=90, h=112)
  wrong = SpeedSign(mph=65, conf=0.57, bbox=(200, 30, 90, 112))
  out = refine_mph(wrong, y, _read_mph)
  assert out.mph == 55


def test_yolo_min_conf_constant():
  assert YOLO_MIN_CONF == 0.40


@pytest.mark.skipif(
  not __import__("os").path.isfile("/tmp/speedsign/best.onnx"),
  reason="exported ONNX not in this environment",
)
def test_real_onnx_empty_road_is_quiet():
  ort = pytest.importorskip("onnxruntime")
  from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector
  sess = ort.InferenceSession("/tmp/speedsign/best.onnx", providers=["CPUExecutionProvider"])
  det = OnnxSpeedSignDetector("/tmp/speedsign/best.onnx", sess)
  rgb = np.zeros((240, 320, 3), np.uint8)
  rgb[:80] = (140, 170, 200)
  rgb[80:] = (45, 45, 40)
  assert det.detect(None, min_conf=0.40, rgb=rgb) == []
