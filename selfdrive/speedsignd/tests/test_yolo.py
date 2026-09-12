"""YOLOv8 ONNX interface: class map, letterbox, decode, no numpy fallback."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import OnnxSpeedSignDetector, SpeedSign, SpeedSignDetector, paint_mutcd_r2_1
from openpilot.selfdrive.speedsignd.tests.test_detect import _scene
from openpilot.selfdrive.speedsignd.weights_manifest import YOLO_CLASS_NAMES, YOLO_IMGSZ, YOLO_MIN_CONF
from openpilot.selfdrive.speedsignd.yolo import (
  _resize_rgb,
  class_to_mph,
  decode_yolov8,
  letterbox_rgb,
  nms_xyxy,
  refine_mph,
  road_detect_crop,
  yolo_peak,
)


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


def test_resize_rgb_is_bilinear_not_nearest():
  """Nearest index-sample of 2×2→1×1 picks a corner; bilinear averages."""
  img = np.zeros((2, 2, 3), np.uint8)
  img[0, 0] = 0
  img[0, 1] = 100
  img[1, 0] = 0
  img[1, 1] = 100
  out = _resize_rgb(img, 1, 1)
  assert 40 <= int(out[0, 0, 0]) <= 60


def test_road_detect_crop_keeps_center_and_right():
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  crop, box = road_detect_crop(rgb)
  x, y, w, h = box
  assert crop.shape == (1208, 1208, 3)
  assert w == h == 1208
  assert x == 720
  assert x <= 964 < x + w  # 3X ROAD optical center stays in-frame
  # Scale vs full-frame letterbox: 320/1208 vs 320/1928.
  assert (YOLO_IMGSZ / float(h)) > (YOLO_IMGSZ / 1928.0) * 1.5


def test_yolo_peak_reports_below_threshold_class():
  raw = np.zeros((1, 25, 4), np.float32)
  raw[0, 4 + 19, 0] = 0.33  # stop
  shape, conf, name, n_over = yolo_peak(raw)
  assert shape == (1, 25, 4)
  assert conf == pytest.approx(0.33, abs=1e-5)
  assert name == "stop"
  assert n_over == 0


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


def test_decode_yolov8_accepts_extra_singleton_dim():
  """tinygrad OnnxRunner has returned [1,1,25,N]; old _as_cn dropped that as empty."""
  raw = np.zeros((1, 1, 25, 8), np.float32)
  raw[0, 0, 0, 0] = 160.0
  raw[0, 0, 1, 0] = 120.0
  raw[0, 0, 2, 0] = 80.0
  raw[0, 0, 3, 0] = 100.0
  raw[0, 0, 4 + 12, 0] = 0.91
  hits = decode_yolov8(raw, scale=1.0, pad_x=0, pad_y=0, src_hw=(320, 320), min_conf=0.4)
  assert len(hits) == 1 and hits[0].mph == 55


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


def test_yolo_crop_offsets_bbox_to_full_road_frame():
  det = OnnxSpeedSignDetector("/tmp/fake.onnx", _YoloSess(), backend="tinygrad")
  rgb = np.zeros((1208, 1928, 3), np.uint8)
  hits = det.detect(None, min_conf=0.4, rgb=rgb)
  assert hits and hits[0].mph == 60
  x, _y, _w, _h = hits[0].bbox
  assert x >= 720
  d = det.diag_dict()
  assert d["frame_w"] == 1928 and d["frame_h"] == 1208
  assert d["crop"] == (720, 0, 1208, 1208)
  assert d["backend"] == "tinygrad"


def test_onnx_records_peak_when_decode_empty():
  class NearMiss(_YoloSess):
    def run(self, _out, _feed):
      raw = np.zeros((1, 25, 4), np.float32)
      raw[0, :4, 0] = [80, 80, 40, 40]
      raw[0, 4 + 19, 0] = 0.33  # stop, below 0.40
      return [raw]

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", NearMiss(), backend="tinygrad")
  y = np.zeros((320, 320), np.uint8)
  assert det.detect(y, min_conf=0.4) == []
  d = det.diag_dict()
  assert d["backend"] == "tinygrad"
  assert d["frame_w"] == 320 and d["frame_h"] == 320
  assert d["letterbox"] == 320
  assert d["peak_name"] == "stop"
  assert d["peak_conf"] == pytest.approx(0.33, abs=1e-5)
  assert d["n_over"] == 0
  assert d["error"] == ""


def test_onnx_detect_surfaces_session_error():
  class Boom(_YoloSess):
    def run(self, _out, _feed):
      raise RuntimeError("tinygrad layout")

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Boom(), backend="tinygrad")
  assert det.detect(np.zeros((64, 64), np.uint8)) == []
  assert "tinygrad layout" in det.diag_dict()["error"]


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
